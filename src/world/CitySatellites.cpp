#include "world/CitySatellites.h"

#include <algorithm>
#include <cmath>

#include "world/CityComposition.h"
#include "world/CityStreets.h"

namespace city {
namespace {

constexpr float kQuarterTurn = 1.5707964f;
constexpr float kDegree = 0.017453292f;

constexpr float kMinDistrictSize = 480.0f;  // ring road centre line to centre line
constexpr float kMaxDistrictSize = 640.0f;
// The streets that join a district to the core leave each ring road straight for a while, then
// bend between those two stretches.
constexpr float kMinStub = 40.0f;
constexpr float kMaxStub = 70.0f;
constexpr float kMinMiddle = 90.0f;
constexpr float kMaxMiddle = 240.0f;
// The bend where the stub meets the middle stretch, at both ends. Smaller bends are not worth a
// node, and larger ones make the street look like a zigzag.
constexpr float kMinBend = 12.0f * kDegree;
constexpr float kMaxBend = 45.0f * kDegree;
// A district is turned against the grid of the core by an angle between these (plus whole
// quarter turns), so that it reads as turned rather than almost straight.
constexpr float kMinRelativeTurn = 20.0f * kDegree;
constexpr float kMaxRelativeTurn = 70.0f * kDegree;

constexpr float kDistrictClearance = 60.0f;  // open land kept between districts, and around the core
constexpr float kWaterClearance = 40.0f;     // kept between anything new and the river and canal
constexpr float kRoadClearance = 40.0f;      // kept between two joining streets
constexpr float kLandMargin = 150.0f;        // kept from the edge of the open land
constexpr float kSampleSpacing = 5.0f;       // spacing of the points tested along a joining street
constexpr int kAttemptsPerDistrict = 80;

// A T junction of a ring road, where a joining street can be attached without moving any edge.
struct RingJunction {
    uint32_t node;
    glm::vec2 position;
    glm::vec2 outward;  // unit vector pointing away from the middle of the ring
};

std::vector<RingJunction> FindRingJunctions(const StreetGraph& streets, float halfSize) {
    constexpr float kTolerance = 0.05f;
    std::vector<RingJunction> junctions;
    for (uint32_t node = 0; node < streets.nodes.size(); ++node) {
        if (streets.edgesAtNode[node].size() != 3) {
            continue;  // a T junction has the two ring edges and the side street
        }
        const glm::vec2 position = streets.nodes[node];
        const bool east = std::abs(position.x - halfSize) < kTolerance;
        const bool west = std::abs(position.x + halfSize) < kTolerance;
        const bool north = std::abs(position.y - halfSize) < kTolerance;
        const bool south = std::abs(position.y + halfSize) < kTolerance;
        if (static_cast<int>(east) + static_cast<int>(west) + static_cast<int>(north) + static_cast<int>(south) != 1) {
            continue;  // a corner, or not on the ring road
        }
        const glm::vec2 outward = east ? glm::vec2(1.0f, 0.0f)
                                       : west ? glm::vec2(-1.0f, 0.0f) : north ? glm::vec2(0.0f, 1.0f) : glm::vec2(0.0f, -1.0f);
        junctions.push_back({node, position, outward});
    }
    return junctions;
}

float SignedTurn(glm::vec2 before, glm::vec2 after) {
    return std::atan2(before.x * after.y - before.y * after.x, glm::dot(before, after));
}

float DistanceToSegment(glm::vec2 point, glm::vec2 a, glm::vec2 b) {
    const glm::vec2 span = b - a;
    const float t = std::clamp(glm::dot(point - a, span) / glm::dot(span, span), 0.0f, 1.0f);
    return glm::distance(point, a + span * t);
}

class SatelliteBuilder {
public:
    SatelliteBuilder(CityLayout& cityLayout, const CitySettings& citySettings, const CityPlan& cityPlan)
        : city(cityLayout), settings(citySettings), plan(cityPlan),
          coreJunctions(FindRingJunctions(cityLayout.streets, cityPlan.halfSize)),
          landLimit(cityPlan.farDistance - kLandMargin) {
        const float coreEdge = plan.halfSize + plan.streetHalfWidth;
        core = Rect{glm::vec2(-coreEdge), glm::vec2(coreEdge)};
        for (const OrientedRect& water : city.water) {
            waterZones.push_back(water.Expanded(kWaterClearance));
        }
    }

    void Run() {
        if (coreJunctions.empty()) {
            return;
        }
        for (int index = 0; index < settings.satelliteCount; ++index) {
            Random random(MixSeed(settings.seed, 0x53415400u + static_cast<uint32_t>(index)));
            for (int attempt = 0; attempt < kAttemptsPerDistrict; ++attempt) {
                const uint32_t districtSeed = MixSeed(settings.seed, 0x44495300u + static_cast<uint32_t>(index * 256 + attempt));
                if (TryPlace(random, districtSeed, static_cast<uint32_t>(index))) {
                    break;
                }
            }
        }
    }

private:
    // Plans the layout of one district and its joining street. Returns false, leaving the city
    // untouched, when it does not fit.
    bool TryPlace(Random& random, uint32_t districtSeed, uint32_t index) {
        // The street: out of the core at right angles, a bend, a straight stretch, a second bend
        // and into the district at right angles.
        const RingJunction& coreJunction = coreJunctions[static_cast<size_t>(random.Int(0, static_cast<int>(coreJunctions.size()) - 1))];
        const float firstStub = random.Range(kMinStub, kMaxStub);
        const float lastStub = random.Range(kMinStub, kMaxStub);
        const float firstBend = random.Range(kMinBend, kMaxBend) * random.Sign();
        const glm::vec2 firstBendAt = coreJunction.position + coreJunction.outward * firstStub;
        const glm::vec2 middleDirection = Placement{{0.0f, 0.0f}, firstBend}.Turn(coreJunction.outward);
        const glm::vec2 secondBendAt = firstBendAt + middleDirection * random.Range(kMinMiddle, kMaxMiddle);

        // The district, turned so that one of its sides faces back along the middle stretch.
        CitySettings districtSettings = settings;
        districtSettings.districtOnly = true;
        districtSettings.satelliteCount = 0;
        districtSettings.citySize = random.Range(kMinDistrictSize, kMaxDistrictSize);
        districtSettings.seed = districtSeed;
        const float districtHalf = districtSettings.citySize * 0.5f;
        const float turn = random.Range(kMinRelativeTurn, kMaxRelativeTurn) + static_cast<float>(random.Int(0, 3)) * kQuarterTurn;
        const CityLayout district = GenerateCity(districtSettings);
        const std::vector<RingJunction> districtJunctions = FindRingJunctions(district.streets, districtHalf);

        const Placement turnOnly{{0.0f, 0.0f}, turn};
        const glm::vec2 sideNormals[] = {{1.0f, 0.0f}, {0.0f, 1.0f}, {-1.0f, 0.0f}, {0.0f, -1.0f}};
        glm::vec2 facingLocal = sideNormals[0];
        for (glm::vec2 normal : sideNormals) {
            if (glm::dot(turnOnly.Turn(normal), -middleDirection) > glm::dot(turnOnly.Turn(facingLocal), -middleDirection)) {
                facingLocal = normal;
            }
        }
        const glm::vec2 facing = turnOnly.Turn(facingLocal);
        const glm::vec2 intoDistrict = -facing;
        const float secondBend = SignedTurn(middleDirection, intoDistrict);
        if (std::abs(secondBend) < kMinBend || std::abs(secondBend) > kMaxBend) {
            return false;
        }

        // The T junction in the middle of that side is where the street enters.
        const RingJunction* entry = nullptr;
        for (const RingJunction& junction : districtJunctions) {
            if (junction.outward != facingLocal) {
                continue;
            }
            const float lateral = facingLocal.x != 0.0f ? junction.position.y : junction.position.x;
            const float best = entry == nullptr ? 0.0f : (facingLocal.x != 0.0f ? entry->position.y : entry->position.x);
            if (entry == nullptr || std::abs(lateral) < std::abs(best)) {
                entry = &junction;
            }
        }
        if (entry == nullptr) {
            return false;
        }
        const glm::vec2 entryAt = secondBendAt + intoDistrict * lastStub;
        const Placement placement{entryAt - turnOnly.Turn(entry->position), turn};

        const std::vector<glm::vec2> path = {coreJunction.position, firstBendAt, secondBendAt, entryAt};
        const OrientedRect square = placement.Apply(OrientedRect(district.bounds));
        if (!Fits(square, path)) {
            return false;
        }

        Commit(district, placement, coreJunction, *entry, firstBendAt, secondBendAt, index);
        return true;
    }

    bool Fits(const OrientedRect& square, const std::vector<glm::vec2>& path) const {
        for (glm::vec2 corner : square.Corners()) {
            if (std::abs(corner.x) > landLimit || std::abs(corner.y) > landLimit) {
                return false;
            }
        }
        const OrientedRect wanted = square.Expanded(kDistrictClearance);
        if (wanted.Overlaps(OrientedRect(core))) {
            return false;
        }
        for (const OrientedRect& other : districts) {
            if (wanted.Overlaps(other)) {
                return false;
            }
        }
        for (const OrientedRect& water : waterZones) {
            if (square.Overlaps(water)) {
                return false;
            }
        }

        // Points along the street. Its two ends lie on ring roads, so only the stretch between
        // the bends is tested against the core and against the district it leads to.
        for (size_t leg = 0; leg + 1 < path.size(); ++leg) {
            const float length = glm::distance(path[leg], path[leg + 1]);
            for (float along = 0.0f; along <= length; along += kSampleSpacing) {
                const glm::vec2 point = path[leg] + (path[leg + 1] - path[leg]) * (along / length);
                for (const OrientedRect& water : waterZones) {
                    if (water.Contains(point)) {
                        return false;
                    }
                }
                for (const OrientedRect& other : districts) {
                    if (other.Expanded(kRoadClearance * 0.5f).Contains(point)) {
                        return false;
                    }
                }
                if (leg == 1 && (OrientedRect(core).Expanded(kRoadClearance * 0.5f).Contains(point) ||
                                 square.Expanded(kRoadClearance * 0.5f).Contains(point))) {
                    return false;
                }
                for (const std::vector<glm::vec2>& road : roads) {
                    for (size_t i = 0; i + 1 < road.size(); ++i) {
                        if (DistanceToSegment(point, road[i], road[i + 1]) < kRoadClearance) {
                            return false;
                        }
                    }
                }
            }
        }
        return true;
    }

    void Commit(const CityLayout& district, const Placement& placement, const RingJunction& coreJunction,
                const RingJunction& districtJunction, glm::vec2 firstBendAt, glm::vec2 secondBendAt, uint32_t index) {
        const uint32_t districtNodeOffset = static_cast<uint32_t>(city.streets.nodes.size());
        AppendPart(city, district, placement);
        districts.push_back(placement.Apply(OrientedRect(district.bounds)));

        // Two new nodes for the bends and three edges joining the two ring roads.
        StreetGraph& streets = city.streets;
        const uint32_t entryNode = districtNodeOffset + districtJunction.node;
        const glm::vec2 entryAt = streets.nodes[entryNode];
        const uint32_t firstBend = static_cast<uint32_t>(streets.nodes.size());
        streets.nodes.push_back(firstBendAt);
        streets.nodes.push_back(secondBendAt);
        streets.edgesAtNode.resize(streets.nodes.size());
        const uint32_t secondBend = firstBend + 1;
        const uint32_t joins[][2] = {{coreJunction.node, firstBend}, {firstBend, secondBend}, {secondBend, entryNode}};
        for (const auto& join : joins) {
            const uint32_t edge = static_cast<uint32_t>(streets.edges.size());
            streets.edges.push_back({join[0], join[1], glm::distance(streets.nodes[join[0]], streets.nodes[join[1]])});
            streets.edgesAtNode[join[0]].push_back(edge);
            streets.edgesAtNode[join[1]].push_back(edge);
        }

        const std::vector<glm::vec2> path = {coreJunction.position, firstBendAt, secondBendAt, entryAt};
        roads.push_back(path);
        for (glm::vec2 bend : {firstBendAt, secondBendAt}) {
            city.bounds.min = glm::min(city.bounds.min, bend - plan.streetHalfWidth);
            city.bounds.max = glm::max(city.bounds.max, bend + plan.streetHalfWidth);
        }

        BoxWriter boxes(city);
        Random random(MixSeed(settings.seed, 0x434F4E4Eu + index));
        BuildConnectorStreet(path, plan.roadHalfWidth, plan.streetHalfWidth, random, boxes, city);
    }

    CityLayout& city;
    const CitySettings& settings;
    const CityPlan& plan;
    const std::vector<RingJunction> coreJunctions;
    const float landLimit;
    Rect core;                                  // the core inside its outer pavement
    std::vector<OrientedRect> waterZones;       // the river and canal, widened by the clearance
    std::vector<OrientedRect> districts;        // the districts placed so far
    std::vector<std::vector<glm::vec2>> roads;  // the joining streets placed so far
};

}  // namespace

void AddSatellites(CityLayout& city, const CitySettings& settings, const CityPlan& plan) {
    SatelliteBuilder(city, settings, plan).Run();
}

}  // namespace city
