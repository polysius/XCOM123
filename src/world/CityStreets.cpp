#include "world/CityStreets.h"

#include <algorithm>
#include <cmath>

#include "world/CityNature.h"

namespace city {
namespace {

constexpr float kRiverBed = -12.0f;
constexpr float kWallFoot = -8.0f;      // quay walls reach this far down into the water
constexpr float kDeckBottom = -1.2f;    // underside of the bridge decks
constexpr float kParapetWidth = 0.5f;
constexpr float kParapetTop = 1.05f;
constexpr float kArchSpan = 22.0f;      // preferred distance between bridge piers
constexpr float kLampSpacing = 30.0f;
constexpr float kLampFromJunction = 9.0f;  // keeps lamps out of the pavements of crossing streets
constexpr float kLampHeight = 4.3f;        // light centre above the pavement

const glm::vec3 kQuayStone{0.55f, 0.51f, 0.45f};
const glm::vec3 kBridgeStone{0.62f, 0.58f, 0.51f};
const glm::vec3 kLampIron{0.055f, 0.06f, 0.065f};
const glm::vec3 kLanternGlass{0.95f, 0.78f, 0.52f};

bool IsAlongX(glm::vec2 a, glm::vec2 b) {
    return std::abs(b.x - a.x) >= std::abs(b.y - a.y);
}

bool ContainsPoint(const std::vector<Rect>& rects, glm::vec2 point) {
    for (const Rect& rect : rects) {
        if (rect.Contains(point)) {
            return true;
        }
    }
    return false;
}

// A lamp post standing on the pavement at `position`, and its light in the layout. The square
// post is turned by `yaw` so that it lines up with a street that does not run along an axis.
void AddLamp(BoxWriter& boxes, CityLayout& city, glm::vec2 position, float yaw = 0.0f) {
    const Look iron{kLampIron, SurfaceMaterial::Metal, 0.45f, 0.0f};
    const Look lantern{kLanternGlass, SurfaceMaterial::Lamp, 0.2f, 0.0f};
    const float ground = kPavementHeight;
    auto post = [&](float halfWidth, float bottom, float top, const Look& look) {
        if (yaw == 0.0f) {
            boxes.Box(glm::vec3(position.x - halfWidth, ground + bottom, position.y - halfWidth),
                      glm::vec3(position.x + halfWidth, ground + top, position.y + halfWidth), look);
        } else {
            boxes.TurnedBox({position.x, ground + (bottom + top) * 0.5f, position.y},
                            {halfWidth, (top - bottom) * 0.5f, halfWidth}, yaw, look);
        }
    };
    post(0.18f, 0.0f, 0.55f, iron);
    post(0.07f, 0.55f, 4.0f, iron);
    post(0.22f, 4.0f, 4.6f, lantern);
    post(0.28f, 4.6f, 4.72f, iron);
    city.streetLights.push_back(AtHeight(position, ground + kLampHeight));
}

class StreetBuilder {
public:
    StreetBuilder(const CityPlan& cityPlan, uint32_t seed, BoxWriter& writer, CityLayout& layout)
        : plan(cityPlan), boxes(writer), city(layout), random(MixSeed(seed, 0x53545254u)) {
        for (const Waterway& waterway : plan.waterways) {
            waterZones.push_back(WaterZone(waterway));
        }
        for (const StreetGraph::Edge& edge : city.streets.edges) {
            roadBands.push_back(SegmentRect(city.streets.nodes[edge.from], city.streets.nodes[edge.to], plan.roadHalfWidth));
        }
    }

    void Run() {
        LayGround();
        BuildBridges();
        BuildWaterways();
        LayPromenades();
        LayBlockPavements();
        LayRingPavements();
        PlaceStreetLamps();
    }

private:
    // Asphalt inside the ring road, grass beyond it, and holes where the water is. A district has
    // no grass, and its asphalt is a centimetre higher so that it does not fight with the grass of
    // the city it is placed in.
    void LayGround() {
        PartScope scope(boxes, CityPart::Ground);
        const float paved = plan.halfSize + plan.roadHalfWidth;
        const Rect streets{glm::vec2(-paved), glm::vec2(paved)};
        const float asphaltTop = plan.districtOnly ? kDistrictAsphaltRaise : 0.0f;
        for (const Rect& piece : Subtract(streets, waterZones)) {
            boxes.Footprint(piece, -1.0f, asphaltTop, AsphaltLook());
        }
        if (plan.districtOnly) {
            return;
        }
        std::vector<Rect> cuts = waterZones;
        cuts.push_back(streets);
        const Rect land{glm::vec2(-plan.farDistance), glm::vec2(plan.farDistance)};
        for (const Rect& piece : Subtract(land, cuts)) {
            boxes.Footprint(piece, -1.0f, 0.0f, GrassLook(0.3f));
        }
        for (const Waterway& waterway : plan.waterways) {
            city.water.push_back(waterway.water);
        }
    }

    // Every street that passes over water gets a bridge for that stretch.
    void BuildBridges() {
        for (uint32_t i = 0; i < city.streets.edges.size(); ++i) {
            const StreetGraph::Edge& edge = city.streets.edges[i];
            const glm::vec2 a = city.streets.nodes[edge.from];
            const glm::vec2 b = city.streets.nodes[edge.to];
            const bool alongX = IsAlongX(a, b);
            const Rect street = SegmentRect(a, b, plan.streetHalfWidth);
            for (const Rect& zone : waterZones) {
                const Rect overWater = Intersection(street, zone);
                if (!overWater.IsEmpty()) {
                    BuildBridge(i, alongX, alongX ? a.y : a.x, AlongMin(overWater, alongX), AlongMax(overWater, alongX));
                }
            }
        }
    }

    void BuildBridge(uint32_t edge, bool alongX, float centerLine, float start, float end) {
        PartScope scope(boxes, CityPart::Bridge);
        const float road = plan.roadHalfWidth;
        const float street = plan.streetHalfWidth;
        const float outer = street + kParapetWidth;
        const Look stone = StoneLook(kBridgeStone, random.Unit());
        auto strip = [&](float alongMin, float alongMax, float acrossMin, float acrossMax, float bottom, float top,
                         const Look& look) {
            boxes.Footprint(FromRanges(alongX, alongMin, alongMax, centerLine + acrossMin, centerLine + acrossMax),
                            bottom, top, look);
        };

        // The deck: a stone body carrying the road, the pavements and a parapet on each side.
        strip(start, end, -outer, outer, kDeckBottom, -0.05f, stone);
        strip(start, end, -road, road, -0.05f, 0.0f, AsphaltLook());
        strip(start, end, -street, -road, -0.05f, kPavementHeight, PavingLook());
        strip(start, end, road, street, -0.05f, kPavementHeight, PavingLook());
        strip(start, end, -outer, -street, -0.05f, kParapetTop, stone);
        strip(start, end, street, outer, -0.05f, kParapetTop, stone);

        // Abutments fill the gap cut into the quay walls, piers stand in the water, and stepped
        // haunches beside every support make the openings read as arches.
        const float length = end - start;
        const int arches = std::max(1, static_cast<int>(std::lround(length / kArchSpan)));
        strip(start, start + kQuayWallThickness + 1.0f, -outer, outer, kWallFoot, kDeckBottom, stone);
        strip(end - kQuayWallThickness - 1.0f, end, -outer, outer, kWallFoot, kDeckBottom, stone);
        for (int support = 0; support <= arches; ++support) {
            const float at = start + length * static_cast<float>(support) / static_cast<float>(arches);
            if (support > 0 && support < arches) {
                strip(at - 1.5f, at + 1.5f, -(street + 1.0f), street + 1.0f, kRiverBed, kDeckBottom, stone);
                strip(at - 0.8f, at + 0.8f, -(street + 3.0f), street + 3.0f, kRiverBed, -2.2f, stone);
            }
            strip(std::max(start, at - 3.7f), std::min(end, at + 3.7f), -(street + 0.35f), street + 0.35f, -2.6f,
                  kDeckBottom, stone);
            strip(std::max(start, at - 6.0f), std::min(end, at + 6.0f), -(street + 0.35f), street + 0.35f, -1.7f,
                  kDeckBottom, stone);
        }

        bridgeBands.push_back(FromRanges(alongX, start, end, centerLine - outer, centerLine + outer));
        city.bridges.push_back({edge, FromRanges(alongX, start, end, centerLine - street, centerLine + street)});
        const float middle = (start + end) * 0.5f;
        city.pointsOfInterest.push_back({AtHeight(OnLine(alongX, middle, centerLine + street - 0.6f), kPavementHeight),
                                         OnLine(alongX, 0.0f, 1.0f), PointOfInterestKind::BridgeView});
    }

    // Water, with quay walls along its edges that stop where bridges or other water meet them.
    void BuildWaterways() {
        for (size_t i = 0; i < plan.waterways.size(); ++i) {
            const Waterway& waterway = plan.waterways[i];
            {
                PartScope scope(boxes, CityPart::Water);
                boxes.Footprint(waterway.water, kRiverBed, kWaterLevel, WaterLook());
            }
            std::vector<Rect> cuts = bridgeBands;
            for (size_t j = 0; j < plan.waterways.size(); ++j) {
                if (j < i) {
                    cuts.push_back(plan.waterways[j].water);
                } else if (j > i) {
                    cuts.push_back(waterZones[j]);
                }
            }
            BuildQuayWalls(waterway, cuts);
        }
    }

    void BuildQuayWalls(const Waterway& waterway, const std::vector<Rect>& cuts) {
        PartScope scope(boxes, CityPart::Embankment);
        const Rect& water = waterway.water;
        const float t = kQuayWallThickness;
        const struct {
            Rect wall;
            bool alongX;
        } walls[] = {
            {{{water.min.x - t, water.min.y - t}, {water.max.x + t, water.min.y}}, true},
            {{{water.min.x - t, water.max.y}, {water.max.x + t, water.max.y + t}}, true},
            {{{water.min.x - t, water.min.y}, {water.min.x, water.max.y}}, false},
            {{{water.max.x, water.min.y}, {water.max.x + t, water.max.y}}, false},
        };
        const Look stone = StoneLook(kQuayStone, 0.3f);
        const Look coping = StoneLook(kBridgeStone, 0.8f);
        for (const auto& side : walls) {
            for (const Rect& piece : Subtract(side.wall, cuts)) {
                boxes.Footprint(piece, kWallFoot, waterway.wallTop - 0.2f, stone);
                // A capstone slightly wider than the wall, overhanging both faces.
                const Rect cap = side.alongX ? InsetRect(piece, 0.0f, 0.0f, -0.12f, -0.12f)
                                             : InsetRect(piece, -0.12f, -0.12f, 0.0f, 0.0f);
                boxes.Footprint(cap, waterway.wallTop - 0.2f, waterway.wallTop + 0.06f, coping);
            }
        }
    }

    // The quays between the streets along the water and the walls, planted with trees.
    void LayPromenades() {
        std::vector<Rect> cuts = roadBands;
        cuts.insert(cuts.end(), waterZones.begin(), waterZones.end());
        for (const Gap& gap : plan.gaps) {
            if (gap.kind == GapKind::Median) {
                continue;
            }
            const bool alongX = gap.alongX;
            const float start = AlongMin(gap.area, alongX);
            const float end = AlongMax(gap.area, alongX);
            const float low = AcrossMin(gap.area, alongX);
            const float high = AcrossMax(gap.area, alongX);
            const Rect bands[] = {
                FromRanges(alongX, start, end, low + plan.roadHalfWidth, low + plan.streetHalfWidth),
                FromRanges(alongX, start, end, low + plan.streetHalfWidth, high - plan.streetHalfWidth),
                FromRanges(alongX, start, end, high - plan.streetHalfWidth, high - plan.roadHalfWidth),
            };
            PartScope scope(boxes, CityPart::Pavement);
            for (int band = 0; band < 3; ++band) {
                const Look look = band == 1 ? CobbleLook(0.4f) : PavingLook();
                for (const Rect& piece : Subtract(bands[band], cuts)) {
                    boxes.Footprint(piece, 0.0f, kPavementHeight, look);
                }
            }
            PlantQuay(gap, low + plan.streetHalfWidth, 1.0f);
            PlantQuay(gap, high - plan.streetHalfWidth, -1.0f);
        }
    }

    // A row of trees along one quay, with benches looking out over the water between them.
    void PlantQuay(const Gap& gap, float quayEdge, float towardWater) {
        const bool alongX = gap.alongX;
        const bool river = gap.kind == GapKind::River;
        const float treeOffset = river ? 3.6f : 1.6f;
        const float spacing = river ? 11.0f : 14.0f;
        std::vector<Rect> keepClear;
        for (const Rect& band : roadBands) {
            keepClear.push_back(band.Expanded(3.5f));
        }
        for (const Rect& zone : waterZones) {
            keepClear.push_back(zone.Expanded(1.0f));
        }
        const float start = AlongMin(gap.area, alongX);
        const float end = AlongMax(gap.area, alongX);
        int count = 0;
        for (float along = start + spacing * 0.5f; along < end; along += spacing, ++count) {
            const glm::vec2 tree = OnLine(alongX, along, quayEdge + towardWater * treeOffset);
            if (!ContainsPoint(keepClear, tree)) {
                AddTree(boxes, city, tree, kPavementHeight, random.Range(9.0f, 13.0f), random);
            }
            const glm::vec2 bench = OnLine(alongX, along + spacing * 0.5f, quayEdge + towardWater * 6.2f);
            if (river && count % 3 == 1 && !ContainsPoint(keepClear, bench)) {
                AddBench(boxes, city, bench, kPavementHeight, OnLine(alongX, 0.0f, towardWater));
            }
        }
    }

    // Each block is paved out to the kerb; its land is covered later by buildings, squares or parks.
    void LayBlockPavements() {
        PartScope scope(boxes, CityPart::Pavement);
        for (const PlannedBlock& block : plan.blocks) {
            boxes.Footprint(block.area.Expanded(-plan.roadHalfWidth), 0.0f, kPavementHeight, PavingLook());
        }
    }

    // The outer pavement of the ring road.
    void LayRingPavements() {
        PartScope scope(boxes, CityPart::Pavement);
        const float kerb = plan.halfSize + plan.roadHalfWidth;
        const float edge = plan.halfSize + plan.streetHalfWidth;
        const Rect strips[] = {
            {{-edge, -edge}, {edge, -kerb}},
            {{-edge, kerb}, {edge, edge}},
            {{-edge, -kerb}, {-kerb, kerb}},
            {{kerb, -kerb}, {edge, kerb}},
        };
        for (const Rect& strip : strips) {
            for (const Rect& piece : Subtract(strip, waterZones)) {
                boxes.Footprint(piece, 0.0f, kPavementHeight, PavingLook());
            }
        }
    }

    // Lamp posts at the kerb, alternating between the two sides of every street.
    void PlaceStreetLamps() {
        PartScope scope(boxes, CityPart::StreetLamp);
        const float kerbOffset = plan.roadHalfWidth + 0.32f;
        for (const StreetGraph::Edge& edge : city.streets.edges) {
            if (edge.length < kLampFromJunction * 2.0f) {
                continue;
            }
            const glm::vec2 a = city.streets.nodes[edge.from];
            const glm::vec2 direction = (city.streets.nodes[edge.to] - a) / edge.length;
            const glm::vec2 side(-direction.y, direction.x);
            const float usable = edge.length - kLampFromJunction * 2.0f;
            const int count = static_cast<int>(usable / kLampSpacing) + 1;
            float sideSign = random.Sign();
            for (int i = 0; i < count; ++i) {
                const float along = count == 1 ? edge.length * 0.5f
                                               : kLampFromJunction + usable * static_cast<float>(i) / static_cast<float>(count - 1);
                AddStreetLamp(a + direction * along + side * (kerbOffset * sideSign));
                sideSign = -sideSign;
            }
        }
    }

    void AddStreetLamp(glm::vec2 position) { AddLamp(boxes, city, position); }

    const CityPlan& plan;
    BoxWriter& boxes;
    CityLayout& city;
    Random random;
    std::vector<Rect> waterZones;   // water plus quay walls, one per waterway
    std::vector<Rect> roadBands;    // the carriageway of every street
    std::vector<Rect> bridgeBands;  // bridge decks including their parapets
};

}  // namespace

void BuildStreetsAndWater(const CityPlan& plan, uint32_t seed, BoxWriter& boxes, CityLayout& city) {
    StreetBuilder(plan, seed, boxes, city).Run();
}

namespace {

glm::vec2 LeftOf(glm::vec2 direction) {
    return {-direction.y, direction.x};
}

// The yaw of a box whose own X axis runs along `direction` on the ground.
float YawAlong(glm::vec2 direction) {
    return std::atan2(-direction.y, direction.x);
}

// The turn from one direction to the next, positive when the street bends to the left.
float SignedTurn(glm::vec2 before, glm::vec2 after) {
    return std::atan2(before.x * after.y - before.y * after.x, glm::dot(before, after));
}

// A box whose floor and roof are at heights `bottom` and `top`, lying along a street leg: it
// spans `alongFrom` to `alongTo` metres from the leg's start and `lateralLow` to `lateralHigh`
// to the left of its centre line.
void LegBox(BoxWriter& boxes, glm::vec2 start, glm::vec2 direction, float alongFrom, float alongTo, float lateralLow,
            float lateralHigh, float bottom, float top, const Look& look) {
    const glm::vec2 center = start + direction * ((alongFrom + alongTo) * 0.5f) +
                             LeftOf(direction) * ((lateralLow + lateralHigh) * 0.5f);
    boxes.TurnedBox({center.x, (bottom + top) * 0.5f, center.y},
                    {(alongTo - alongFrom) * 0.5f, (top - bottom) * 0.5f, (lateralHigh - lateralLow) * 0.5f},
                    YawAlong(direction), look);
}

}  // namespace

void BuildConnectorStreet(const std::vector<glm::vec2>& path, float roadHalfWidth, float streetHalfWidth,
                          Random& random, BoxWriter& boxes, CityLayout& city) {
    const size_t legCount = path.size() - 1;
    std::vector<glm::vec2> directions(legCount);
    std::vector<float> lengths(legCount);
    for (size_t leg = 0; leg < legCount; ++leg) {
        lengths[leg] = glm::distance(path[leg], path[leg + 1]);
        directions[leg] = (path[leg + 1] - path[leg]) / lengths[leg];
    }
    // turns[leg] is the turn at the start of the leg, where it meets the previous one.
    std::vector<float> turns(legCount + 1, 0.0f);
    for (size_t leg = 1; leg < legCount; ++leg) {
        turns[leg] = SignedTurn(directions[leg - 1], directions[leg]);
    }
    auto mitre = [](float turn, float halfWidth) { return halfWidth * std::tan(std::abs(turn) * 0.5f); };

    for (size_t leg = 0; leg < legCount; ++leg) {
        const glm::vec2 start = path[leg];
        const glm::vec2 direction = directions[leg];
        const float length = lengths[leg];
        const float startTurn = turns[leg];
        const float endTurn = turns[leg + 1];
        const bool endsAtStreet = leg + 1 == legCount;

        // The road reaches past each bend, so that the outside of the bend has no gap. Where it
        // starts or ends at another street it simply meets that street's road.
        {
            PartScope scope(boxes, CityPart::Ground);
            LegBox(boxes, start, direction, -mitre(startTurn, roadHalfWidth), length + mitre(endTurn, roadHalfWidth),
                   -roadHalfWidth, roadHalfWidth, -1.0f, kDistrictAsphaltRaise, AsphaltLook());
        }

        // A pavement on each side. Next to another street the pavement begins where that street's
        // own pavement ends. At a bend the pavement on the outside reaches past it like the road;
        // the one on the inside stops at the kerb, since reaching further would put it in the
        // carriageway of the next leg.
        PartScope scope(boxes, CityPart::Pavement);
        for (float side : {-1.0f, 1.0f}) {
            const bool insideAtStart = side * startTurn > 0.0f;
            const bool insideAtEnd = side * endTurn > 0.0f;
            const float from = leg == 0 ? streetHalfWidth
                                        : (insideAtStart ? mitre(startTurn, roadHalfWidth) : -mitre(startTurn, streetHalfWidth));
            const float to = endsAtStreet ? length - streetHalfWidth
                                          : (insideAtEnd ? length - mitre(endTurn, roadHalfWidth)
                                                         : length + mitre(endTurn, streetHalfWidth));
            if (to - from > 0.05f) {
                const float low = side > 0.0f ? roadHalfWidth : -streetHalfWidth;
                const float high = side > 0.0f ? streetHalfWidth : -roadHalfWidth;
                LegBox(boxes, start, direction, from, to, low, high, 0.0f, kPavementHeight, PavingLook());
            }
        }

        // Lamps at the kerb, alternating between the two sides, as along every other street.
        if (length >= kLampFromJunction * 2.0f) {
            PartScope lampScope(boxes, CityPart::StreetLamp);
            const float usable = length - kLampFromJunction * 2.0f;
            const int count = static_cast<int>(usable / kLampSpacing) + 1;
            float sideSign = random.Sign();
            for (int i = 0; i < count; ++i) {
                const float along = count == 1 ? length * 0.5f
                                               : kLampFromJunction + usable * static_cast<float>(i) / static_cast<float>(count - 1);
                AddLamp(boxes, city, start + direction * along + LeftOf(direction) * ((roadHalfWidth + 0.32f) * sideSign),
                        YawAlong(direction));
                sideSign = -sideSign;
            }
        }
    }
}

}  // namespace city
