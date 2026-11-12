#include "world/CitySatellites.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

// The first district tries to stand across the river from the core, joined by a street that crosses
// it on a bridge. The street may meet the river at an angle, but not too shallow a one.
constexpr int kAttemptsAcrossRiver = 60;
constexpr float kMinRiverDistance = 90.0f;   // a junction this far or further from the river's line is used
constexpr float kMaxRiverDistance = 480.0f;
constexpr float kMinCrossingSine = 0.64f;    // sine of the smallest angle between the street and the river, about 40 degrees
constexpr float kMaxCrossingBend = 55.0f * kDegree;  // the first bend may be sharper when the street heads for the river
constexpr float kFarBank = 70.0f;            // the second bend lies this far past the levee on the far bank
constexpr float kBridgeApproach = 12.0f;     // a bridge reaches this far past the levee, along the street's perpendicular
constexpr float kLandBeforeBridge = 30.0f;   // the street keeps this much land between a bend and the bridge
constexpr size_t kConnectorBoxAllowance = 1000;  // room kept for the street that joins a district

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

// The turn about the vertical axis that carries the unit vector `local` onto the unit vector `wanted`
// (the sign convention of Placement).
float TurnOnto(glm::vec2 local, glm::vec2 wanted) {
    return std::atan2(local.y * wanted.x - local.x * wanted.y, local.x * wanted.x + local.y * wanted.y);
}

float SignedTurn(glm::vec2 before, glm::vec2 after) {
    return std::atan2(before.x * after.y - before.y * after.x, glm::dot(before, after));
}

float DistanceToSegment(glm::vec2 point, glm::vec2 a, glm::vec2 b) {
    const glm::vec2 span = b - a;
    const float t = std::clamp(glm::dot(point - a, span) / glm::dot(span, span), 0.0f, 1.0f);
    return glm::distance(point, a + span * t);
}

// The outer pavement of a ring road runs unbroken past every T junction, because nothing used to
// leave a ring road outwards. Where a joining street leaves, the pavement is opened across its
// road, or it would cross the street as a kerb. The pavement box that holds the point just
// outside `junction` is cut in two, leaving a gap as wide as the road. `lift` is how high the
// ring road stands above the level of the city. If no pavement lies there
// nothing is cut; CityTests' road-clear check reports a street that still has a kerb across it.
void OpenPavementAt(CityLayout& city, glm::vec2 junction, glm::vec2 outward, float roadHalfWidth, float streetHalfWidth,
                    float lift) {
    const glm::vec2 probe = junction + outward * ((roadHalfWidth + streetHalfWidth) * 0.5f);
    for (size_t i = 0; i < city.boxes.size(); ++i) {
        const BoxInstance original = city.boxes[i];
        const bool isPavementSlab = city.boxParts[i] == CityPart::Pavement &&
                                    std::abs(original.center.y - original.halfSize.y - lift) < 1e-3f &&
                                    std::abs(original.center.y + original.halfSize.y - kPavementHeight - lift) < 1e-3f;
        if (!isPavementSlab) {
            continue;
        }
        const OrientedRect footprint(Rect{{original.center.x - original.halfSize.x, original.center.z - original.halfSize.z},
                                          {original.center.x + original.halfSize.x, original.center.z + original.halfSize.z}},
                                     original.yaw);
        if (!footprint.Contains(probe)) {
            continue;
        }

        // Work in the box's own frame. The ring road runs across `outward`, so the cut is made along
        // whichever of the box's axes is further from it.
        const glm::vec2 here = footprint.ToFrame(probe);
        const glm::vec2 outwardInFrame = footprint.ToFrame(probe + outward) - here;
        const bool alongX = std::abs(outwardInFrame.y) > std::abs(outwardInFrame.x);
        const float at = alongX ? here.x : here.y;
        const float low = alongX ? footprint.area.min.x : footprint.area.min.y;
        const float high = alongX ? footprint.area.max.x : footprint.area.max.y;
        const glm::vec2 middle = footprint.Center();
        const float pieces[2][2] = {{low, at - roadHalfWidth}, {at + roadHalfWidth, high}};
        bool reused = false;
        for (const auto& piece : pieces) {
            if (piece[1] - piece[0] <= 0.05f) {
                continue;
            }
            const float center = (piece[0] + piece[1]) * 0.5f;
            const glm::vec2 world = footprint.FromFrame(alongX ? glm::vec2(center, middle.y) : glm::vec2(middle.x, center));
            BoxInstance cut = original;
            cut.center.x = world.x;
            cut.center.z = world.y;
            (alongX ? cut.halfSize.x : cut.halfSize.z) = (piece[1] - piece[0]) * 0.5f;
            if (!reused) {
                city.boxes[i] = cut;  // the first piece takes the old slot
                reused = true;
            } else {
                city.boxes.push_back(cut);
                city.boxParts.push_back(CityPart::Pavement);
            }
        }
        return;
    }
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
        // The water of the reaches was added to the layout last, one rectangle per leg.
        size_t reachLegs = 0;
        for (const std::vector<glm::vec2>& reach : city.riverReaches) {
            reachLegs += reach.size() - 1;
        }
        firstReachZone = waterZones.size() - reachLegs;
    }

    void Run() {
        if (coreJunctions.empty()) {
            return;
        }
        for (int index = 0; index < settings.satelliteCount; ++index) {
            // The first district tries to stand across the river. It draws from a stream of its own, so
            // that when no crossing fits, the ordinary attempts below are what they would have been.
            if (index == 0 && !city.riverReaches.empty()) {
                Random crossingRandom(MixSeed(settings.seed, 0x42524447u));
                bool crossed = false;
                for (int attempt = 0; attempt < kAttemptsAcrossRiver && !crossed && !budgetReached; ++attempt) {
                    const uint32_t districtSeed = MixSeed(settings.seed, 0x42524400u + static_cast<uint32_t>(attempt));
                    crossed = TryPlace(crossingRandom, districtSeed, 0, true);
                }
                if (crossed || budgetReached) {
                    if (budgetReached) {
                        return;
                    }
                    continue;
                }
            }
            Random random(MixSeed(settings.seed, 0x53415400u + static_cast<uint32_t>(index)));
            for (int attempt = 0; attempt < kAttemptsPerDistrict; ++attempt) {
                const uint32_t districtSeed = MixSeed(settings.seed, 0x44495300u + static_cast<uint32_t>(index * 256 + attempt));
                if (TryPlace(random, districtSeed, static_cast<uint32_t>(index), false) || budgetReached) {
                    break;
                }
            }
            if (budgetReached) {
                return;
            }
        }
    }

private:
    // Plans the layout of one district and its joining street. Returns false, leaving the city
    // untouched, when it does not fit.
    bool TryPlace(Random& random, uint32_t districtSeed, uint32_t index, bool acrossRiver) {
        // The street: out of the core at right angles, a bend, a straight stretch, a second bend
        // and into the district at right angles. Across the river, the straight stretch is aimed
        // at the far bank.
        const RingJunction* source = nullptr;
        float lastStub = 0.0f;
        glm::vec2 firstBendAt{0.0f};
        glm::vec2 middleDirection{0.0f};
        glm::vec2 secondBendAt{0.0f};
        glm::vec2 towardsRiver{0.0f};  // across the river: the way the near side of the district must face
        if (acrossRiver) {
            if (!PlanCrossingStreet(random, source, lastStub, firstBendAt, middleDirection, secondBendAt, towardsRiver)) {
                return false;
            }
        } else {
            source = &coreJunctions[static_cast<size_t>(random.Int(0, static_cast<int>(coreJunctions.size()) - 1))];
            const float firstStub = random.Range(kMinStub, kMaxStub);
            lastStub = random.Range(kMinStub, kMaxStub);
            const float bendSize = random.Range(kMinBend, kMaxBend);
            const float firstBend = bendSize * random.Sign();
            firstBendAt = source->position + source->outward * firstStub;
            middleDirection = Placement{{0.0f, 0.0f}, firstBend}.Turn(source->outward);
            secondBendAt = firstBendAt + middleDirection * random.Range(kMinMiddle, kMaxMiddle);
        }
        const RingJunction& coreJunction = *source;

        // The district, turned so that one of its sides faces back along the middle stretch.
        CitySettings districtSettings = settings;
        districtSettings.districtOnly = true;
        districtSettings.satelliteCount = 0;
        districtSettings.citySize = random.Range(kMinDistrictSize, kMaxDistrictSize);
        districtSettings.seed = districtSeed;
        const float districtHalf = districtSettings.citySize * 0.5f;
        const glm::vec2 sideNormals[] = {{1.0f, 0.0f}, {0.0f, 1.0f}, {-1.0f, 0.0f}, {0.0f, -1.0f}};
        float turn = 0.0f;
        int riverSide = 0;
        if (acrossRiver) {
            // One of the sides of the district runs along the bank, so that a square fits beside the
            // river instead of reaching back over it.
            riverSide = random.Int(0, 3);
            turn = TurnOnto(sideNormals[riverSide], towardsRiver);
        } else {
            const float relativeTurn = random.Range(kMinRelativeTurn, kMaxRelativeTurn);
            const float quarterTurns = static_cast<float>(random.Int(0, 3));
            turn = relativeTurn + quarterTurns * kQuarterTurn;
        }
        const CityLayout district = GenerateCity(districtSettings);
        if (city.boxes.size() + district.boxes.size() + kConnectorBoxAllowance > kMaxCityBoxes) {
            budgetReached = true;
            return false;
        }
        const std::vector<RingJunction> districtJunctions = FindRingJunctions(district.streets, districtHalf);

        const Placement turnOnly{{0.0f, 0.0f}, turn};
        glm::vec2 facingLocal = sideNormals[acrossRiver ? riverSide : 0];
        if (!acrossRiver) {
            for (glm::vec2 normal : sideNormals) {
                if (glm::dot(turnOnly.Turn(normal), -middleDirection) > glm::dot(turnOnly.Turn(facingLocal), -middleDirection)) {
                    facingLocal = normal;
                }
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
        Crossing crossing;
        if (acrossRiver && !FindCrossing(path, crossing)) {
            return false;
        }
        if (!Fits(square, path, acrossRiver)) {
            return false;
        }

        Commit(district, placement, coreJunction, *entry, firstBendAt, secondBendAt, index, acrossRiver ? &crossing : nullptr);
        return true;
    }

    // Aims a street at the far bank of a reach of the river. It leaves a T junction on the side of the
    // core where the reach leaves, not too far from the river, and its first bend points it at a spot
    // beyond the levee on the other bank. Returns false, planning nothing, when this try has no street.
    bool PlanCrossingStreet(Random& random, const RingJunction*& source, float& lastStub, glm::vec2& firstBendAt,
                            glm::vec2& middleDirection, glm::vec2& secondBendAt, glm::vec2& towardsRiver) const {
        const size_t reachIndex = static_cast<size_t>(random.Int(0, static_cast<int>(city.riverReaches.size()) - 1));
        const std::vector<glm::vec2>& reach = city.riverReaches[reachIndex];
        const float outwardX = reach.front().x > 0.0f ? 1.0f : -1.0f;
        const float riverZ = reach.front().y;

        std::vector<const RingJunction*> candidates;
        for (const RingJunction& junction : coreJunctions) {
            const float distance = std::abs(junction.position.y - riverZ);
            if (junction.outward.x == outwardX && distance >= kMinRiverDistance && distance <= kMaxRiverDistance) {
                candidates.push_back(&junction);
            }
        }
        if (candidates.empty()) {
            return false;
        }
        source = candidates[static_cast<size_t>(random.Int(0, static_cast<int>(candidates.size()) - 1))];
        const float firstStub = random.Range(kMinStub, kMaxStub);
        lastStub = random.Range(kMinStub, kMaxStub);
        firstBendAt = source->position + source->outward * firstStub;

        // A leg of the reach a little way out from the first bend, whose midpoint is the aim.
        std::vector<size_t> legs;
        for (size_t leg = 0; leg + 1 < reach.size(); ++leg) {
            const float out = ((reach[leg].x + reach[leg + 1].x) * 0.5f - firstBendAt.x) * outwardX;
            if (out >= 80.0f && out <= 380.0f) {
                legs.push_back(leg);
            }
        }
        if (legs.empty()) {
            return false;
        }
        const size_t leg = legs[static_cast<size_t>(random.Int(0, static_cast<int>(legs.size()) - 1))];
        const glm::vec2 aim = (reach[leg] + reach[leg + 1]) * 0.5f;
        const glm::vec2 along = glm::normalize(reach[leg + 1] - reach[leg]);
        const glm::vec2 left{-along.y, along.x};
        const float sourceSide = glm::dot(source->position - aim, left) >= 0.0f ? 1.0f : -1.0f;
        secondBendAt = aim - left * sourceSide * (city.riverReachHalfWidth + kReachWallThickness + kFarBank);
        towardsRiver = left * sourceSide;  // from the far bank back to the river

        middleDirection = glm::normalize(secondBendAt - firstBendAt);
        const float bend = SignedTurn(source->outward, middleDirection);
        return std::abs(bend) >= kMinBend && std::abs(bend) <= kMaxCrossingBend;
    }

    // Where the street's straight stretch crosses the river, as distances along it.
    struct Crossing {
        float start = 0.0f;  // the bridge begins this far from the first bend
        float end = 0.0f;
    };

    // Finds the one stretch of the straight stretch that is over the river, with its levees and room for
    // the ends of a bridge, as a Crossing. There must be exactly one, with land before and after it, and
    // the street must meet the river at a good angle. Anything else is not a crossing.
    bool FindCrossing(const std::vector<glm::vec2>& path, Crossing& crossing) const {
        const glm::vec2 from = path[1];
        const float length = glm::distance(from, path[2]);
        const glm::vec2 direction = (path[2] - from) / length;
        const float reachRadius = city.riverReachHalfWidth + kReachWallThickness + kBridgeApproach;

        int stretches = 0;
        bool inside = false;
        float first = 0.0f;
        float last = 0.0f;
        float smallestSine = 1.0f;  // the sine of the smallest angle between the street and the river it crosses
        for (float along = 0.0f; along <= length; along += 1.0f) {
            const glm::vec2 point = from + direction * along;
            float nearest = std::numeric_limits<float>::max();
            float shallowest = 1.0f;  // the smallest sine to any leg of the river within reach of this point
            for (const std::vector<glm::vec2>& reach : city.riverReaches) {
                for (size_t i = 0; i + 1 < reach.size(); ++i) {
                    const float distance = DistanceToSegment(point, reach[i], reach[i + 1]);
                    nearest = std::min(nearest, distance);
                    if (distance < reachRadius) {
                        const glm::vec2 river = glm::normalize(reach[i + 1] - reach[i]);
                        shallowest = std::min(shallowest, std::abs(direction.x * river.y - direction.y * river.x));
                    }
                }
            }
            if (nearest < reachRadius) {
                if (!inside) {
                    ++stretches;
                    first = along;
                    inside = true;
                }
                last = along;
                // The street meets every leg of the river it passes over, including a neighbour that bends
                // the other way; the shallowest angle counts.
                smallestSine = std::min(smallestSine, shallowest);
            } else {
                inside = false;
            }
        }
        if (stretches != 1 || first < kLandBeforeBridge || last > length - kLandBeforeBridge ||
            smallestSine < kMinCrossingSine) {
            return false;
        }
        crossing.start = first;
        crossing.end = last;
        return true;
    }

    bool Fits(const OrientedRect& square, const std::vector<glm::vec2>& path, bool allowCrossing) const {
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
        const OrientedRect roadClear = square.Expanded(kRoadClearance * 0.5f);
        for (const std::vector<glm::vec2>& road : roads) {
            for (size_t leg = 0; leg + 1 < road.size(); ++leg) {
                const float length = glm::distance(road[leg], road[leg + 1]);
                for (float along = 0.0f; along <= length; along += kSampleSpacing) {
                    if (roadClear.Contains(road[leg] + (road[leg + 1] - road[leg]) * (along / length))) {
                        return false;
                    }
                }
            }
        }

        // Points along the street. Its two ends lie on ring roads, so only the stretch between
        // the bends is tested against the core and against the district it leads to.
        for (size_t leg = 0; leg + 1 < path.size(); ++leg) {
            const float length = glm::distance(path[leg], path[leg + 1]);
            for (float along = 0.0f; along <= length; along += kSampleSpacing) {
                const glm::vec2 point = path[leg] + (path[leg + 1] - path[leg]) * (along / length);
                for (size_t zone = 0; zone < waterZones.size(); ++zone) {
                    // A street that crosses the river is allowed over the reaches on its straight
                    // stretch; FindCrossing has already checked that it does so cleanly.
                    const bool crossesHere = allowCrossing && leg == 1 && zone >= firstReachZone;
                    if (!crossesHere && waterZones[zone].Contains(point)) {
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
                const RingJunction& districtJunction, glm::vec2 firstBendAt, glm::vec2 secondBendAt, uint32_t index,
                const Crossing* crossing) {
        const uint32_t districtNodeOffset = static_cast<uint32_t>(city.streets.nodes.size());
        AppendPart(city, district, placement);
        districts.push_back(placement.Apply(OrientedRect(district.bounds)));
        OpenPavementAt(city, coreJunction.position, coreJunction.outward, plan.roadHalfWidth, plan.streetHalfWidth, 0.0f);
        OpenPavementAt(city, city.streets.nodes[districtNodeOffset + districtJunction.node],
                       placement.Turn(districtJunction.outward), plan.roadHalfWidth, plan.streetHalfWidth, placement.lift);

        // Two new nodes for the bends and three edges joining the two ring roads.
        StreetGraph& streets = city.streets;
        const uint32_t entryNode = districtNodeOffset + districtJunction.node;
        const glm::vec2 entryAt = streets.nodes[entryNode];
        const uint32_t firstBend = static_cast<uint32_t>(streets.nodes.size());
        streets.nodes.push_back(firstBendAt);
        streets.nodes.push_back(secondBendAt);
        streets.edgesAtNode.resize(streets.nodes.size());
        const uint32_t secondBend = firstBend + 1;
        const uint32_t firstEdge = static_cast<uint32_t>(streets.edges.size());
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
        if (crossing != nullptr) {
            const glm::vec2 direction = glm::normalize(secondBendAt - firstBendAt);
            BuildRiverBridge(firstBendAt, direction, crossing->start, crossing->end, firstEdge + 1, plan.roadHalfWidth,
                             plan.streetHalfWidth, random, boxes, city);
        }
    }

    CityLayout& city;
    const CitySettings& settings;
    const CityPlan& plan;
    const std::vector<RingJunction> coreJunctions;
    const float landLimit;
    Rect core;                                  // the core inside its outer pavement
    std::vector<OrientedRect> waterZones;       // the river and canal, widened by the clearance
    size_t firstReachZone = 0;                  // the zones from here on are legs of the reaches of the river
    std::vector<OrientedRect> districts;        // the districts placed so far
    std::vector<std::vector<glm::vec2>> roads;  // the joining streets placed so far
    bool budgetReached = false;                 // another district would exceed kMaxCityBoxes
};

}  // namespace

void AddSatellites(CityLayout& city, const CitySettings& settings, const CityPlan& plan) {
    SatelliteBuilder(city, settings, plan).Run();
}

}  // namespace city
