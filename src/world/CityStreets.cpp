#include "world/CityStreets.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

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
constexpr float kBankGrassCell = 2.0f;     // grass beside a bend of the river is laid in cells this wide
constexpr float kBankRegionMargin = 12.0f; // the cells reach this far beyond the levees
constexpr float kLeveeOverlap = 6.0f;      // a levee starts this far back over the wall of the straight river
constexpr float kSillTop = -0.02f;         // the sill a bridge crosses stands a hair below the grass, which hides its edges
constexpr float kSillMargin = 2.0f;        // the sill reaches this far past the ends of a bridge
constexpr float kSillCutMargin = 1.5f;     // and this far past the stretch of levee the street crosses
constexpr float kLeveeCapRise = 0.10f;     // a levee capstone stands this far above the quay wall caps, so they never fight
constexpr float kPlinthFoot = -1.0f;       // the plinth and the steps start at the underside of the grass
constexpr float kStairCheek = 0.5f;        // the walls beside a stair are this thick
constexpr float kStairParapet = 0.9f;      // and stand this far above the step beside them

const glm::vec3 kQuayStone{0.55f, 0.51f, 0.45f};
const glm::vec3 kBridgeStone{0.62f, 0.58f, 0.51f};
const glm::vec3 kPlinthStone{0.50f, 0.47f, 0.42f};
const glm::vec3 kLampIron{0.055f, 0.06f, 0.065f};
const glm::vec3 kLanternGlass{0.95f, 0.78f, 0.52f};

bool IsAlongX(glm::vec2 a, glm::vec2 b) {
    return std::abs(b.x - a.x) >= std::abs(b.y - a.y);
}

float DistanceToSegment(glm::vec2 point, glm::vec2 a, glm::vec2 b) {
    const glm::vec2 span = b - a;
    const float t = std::clamp(glm::dot(point - a, span) / glm::dot(span, span), 0.0f, 1.0f);
    return glm::distance(point, a + span * t);
}

float DistanceToPolyline(glm::vec2 point, const std::vector<glm::vec2>& points) {
    float nearest = std::numeric_limits<float>::max();
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        nearest = std::min(nearest, DistanceToSegment(point, points[i], points[i + 1]));
    }
    return nearest;
}

bool ContainsPoint(const std::vector<Rect>& rects, glm::vec2 point) {
    for (const Rect& rect : rects) {
        if (rect.Contains(point)) {
            return true;
        }
    }
    return false;
}

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

// Draws one box of a bridge: it spans `alongMin` to `alongMax` along the street and `acrossMin` to
// `acrossMax` to either side of its centre line, between heights `bottom` and `top`.
using BridgeStrip = std::function<void(float alongMin, float alongMax, float acrossMin, float acrossMax, float bottom,
                                       float top, const Look& look)>;

// The body of a bridge from `start` to `end` along its street: a stone deck with a parapet on each
// side, abutments that fill the gap cut into the walls of the banks (`wallThickness` thick), piers in
// the water, and stepped haunches beside every support that make the openings read as arches. With
// `layRoad` the deck also carries its own road and pavements; a street laid across open land has
// them already.
void BuildBridgeBody(const BridgeStrip& strip, float start, float end, float road, float street, float wallThickness,
                     bool layRoad, const Look& stone) {
    const float outer = street + kParapetWidth;
    strip(start, end, -outer, outer, kDeckBottom, -0.05f, stone);
    if (layRoad) {
        strip(start, end, -road, road, -0.05f, 0.0f, AsphaltLook());
        strip(start, end, -street, -road, -0.05f, kPavementHeight, PavingLook());
        strip(start, end, road, street, -0.05f, kPavementHeight, PavingLook());
    }
    strip(start, end, -outer, -street, -0.05f, kParapetTop, stone);
    strip(start, end, street, outer, -0.05f, kParapetTop, stone);

    const float length = end - start;
    const int arches = std::max(1, static_cast<int>(std::lround(length / kArchSpan)));
    strip(start, start + wallThickness + 1.0f, -outer, outer, kWallFoot, kDeckBottom, stone);
    strip(end - wallThickness - 1.0f, end, -outer, outer, kWallFoot, kDeckBottom, stone);
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
        BuildRiverReaches();
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
        std::vector<Rect> bankRegions;  // where the river bends; the grass there is laid in cells
        for (const RiverReach& reach : plan.reaches) {
            bankRegions.push_back(BankRegion(reach, land));
        }
        cuts.insert(cuts.end(), bankRegions.begin(), bankRegions.end());
        for (const Rect& piece : Subtract(land, cuts)) {
            boxes.Footprint(piece, -1.0f, 0.0f, GrassLook(0.3f));
        }
        for (size_t i = 0; i < bankRegions.size(); ++i) {
            LayRiverBankGrass(plan.reaches[i], bankRegions[i]);
        }
        for (const Waterway& waterway : plan.waterways) {
            city.water.push_back(waterway.water);
        }
    }

    // The land around a reach of the river, from the end of the straight river to the edge of the land.
    Rect BankRegion(const RiverReach& reach, const Rect& land) const {
        glm::vec2 low = reach.points.front();
        glm::vec2 high = reach.points.front();
        for (glm::vec2 point : reach.points) {
            low = glm::min(low, point);
            high = glm::max(high, point);
        }
        const float margin = reach.halfWidth + kReachWallThickness + kBankRegionMargin;
        Rect region{low - margin, high + margin};
        const Rect& straightRiver = waterZones.front();  // the river is the first waterway
        if (reach.points.front().x > 0.0f) {
            region.min.x = std::max(region.min.x, straightRiver.max.x);
        } else {
            region.max.x = std::min(region.max.x, straightRiver.min.x);
        }
        return Intersection(region, land);
    }

    // Grass beside a reach, in cells of kBankGrassCell merged into runs along each row. A cell is left
    // out when its centre is closer to the centre line of the river than the middle of the levee, so
    // the jagged edge of the cells stays under the levee: the most a centre can be off is half a cell
    // diagonal, which is less than half the width of the levee.
    void LayRiverBankGrass(const RiverReach& reach, const Rect& region) {
        static_assert(kBankGrassCell * 0.70710678f < kReachWallThickness * 0.5f,
                      "the levee must be wider than the diagonal of the grass cells");
        const float holeRadius = reach.halfWidth + kReachWallThickness * 0.5f;
        for (float z0 = region.min.y; z0 < region.max.y; z0 += kBankGrassCell) {
            const float z1 = std::min(z0 + kBankGrassCell, region.max.y);
            bool inRun = false;
            float runStart = region.min.x;
            for (float x0 = region.min.x; x0 < region.max.x; x0 += kBankGrassCell) {
                const float x1 = std::min(x0 + kBankGrassCell, region.max.x);
                const glm::vec2 center((x0 + x1) * 0.5f, (z0 + z1) * 0.5f);
                const bool isGrass = DistanceToPolyline(center, reach.points) >= holeRadius;
                if (isGrass && !inRun) {
                    runStart = x0;
                    inRun = true;
                } else if (!isGrass && inRun) {
                    boxes.Footprint({{runStart, z0}, {x0, z1}}, -1.0f, 0.0f, GrassLook(0.3f));
                    inRun = false;
                }
            }
            if (inRun) {
                boxes.Footprint({{runStart, z0}, {region.max.x, z1}}, -1.0f, 0.0f, GrassLook(0.3f));
            }
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

        BuildBridgeBody(strip, start, end, road, street, kQuayWallThickness, true, stone);

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
            if (i == 0 && !plan.reaches.empty()) {
                // The river goes on beyond the straight part, so nothing closes its two ends.
                const Rect& water = waterway.water;
                const float t = kQuayWallThickness;
                cuts.push_back({{water.min.x - t, water.min.y - t}, {water.min.x, water.max.y + t}});
                cuts.push_back({{water.max.x, water.min.y - t}, {water.max.x + t, water.max.y + t}});
            }
            BuildQuayWalls(waterway, cuts);
        }
    }

    // The river beyond its straight part, leg by leg along each reach: the water, and a levee on each
    // bank. At a bend the water and the levee on the outside reach past the vertex so that the outside
    // has no gap, and the levee on the inside stops at the edge of the water, as the pavements of a
    // joining street do.
    void BuildRiverReaches() {
        for (const RiverReach& reach : plan.reaches) {
            BuildRiverReach(reach);
            city.riverReaches.push_back(reach.points);
        }
        if (!plan.reaches.empty()) {
            city.riverReachHalfWidth = plan.reaches.front().halfWidth;
        }
    }

    void BuildRiverReach(const RiverReach& reach) {
        const size_t legCount = reach.points.size() - 1;
        std::vector<glm::vec2> directions(legCount);
        std::vector<float> lengths(legCount);
        for (size_t leg = 0; leg < legCount; ++leg) {
            lengths[leg] = glm::distance(reach.points[leg], reach.points[leg + 1]);
            directions[leg] = (reach.points[leg + 1] - reach.points[leg]) / lengths[leg];
        }
        std::vector<float> turns(legCount + 1, 0.0f);  // turns[leg] is the turn at the start of the leg
        for (size_t leg = 1; leg < legCount; ++leg) {
            turns[leg] = SignedTurn(directions[leg - 1], directions[leg]);
        }
        auto mitre = [](float turn, float halfWidth) { return halfWidth * std::tan(std::abs(turn) * 0.5f); };

        const float water = reach.halfWidth;
        const float levee = kReachWallThickness;
        const Look stone = StoneLook(kQuayStone, 0.3f);
        const Look coping = StoneLook(kBridgeStone, 0.8f);
        for (size_t leg = 0; leg < legCount; ++leg) {
            const glm::vec2 start = reach.points[leg];
            const glm::vec2 direction = directions[leg];
            const float length = lengths[leg];
            const float startTurn = turns[leg];
            const float endTurn = turns[leg + 1];
            {
                PartScope scope(boxes, CityPart::Water);
                LegBox(boxes, start, direction, -mitre(startTurn, water), length + mitre(endTurn, water), -water, water,
                       kRiverBed, kWaterLevel, WaterLook());
                const glm::vec2 center = start + direction * (length * 0.5f);
                const glm::vec2 half(length * 0.5f, water);
                city.water.push_back(OrientedRect(Rect{center - half, center + half}, YawAlong(direction)));
            }

            PartScope scope(boxes, CityPart::Embankment);
            for (float side : {-1.0f, 1.0f}) {
                const bool insideAtStart = side * startTurn > 0.0f;
                const bool insideAtEnd = side * endTurn > 0.0f;
                const float from = leg == 0 ? -kLeveeOverlap
                                            : (insideAtStart ? mitre(startTurn, water) : -mitre(startTurn, water + levee));
                const float to = insideAtEnd ? length - mitre(endTurn, water) : length + mitre(endTurn, water + levee);
                const float low = side > 0.0f ? water : -(water + levee);
                const float high = side > 0.0f ? water + levee : -water;
                LegBox(boxes, start, direction, from, to, low, high, kWallFoot, reach.wallTop - 0.2f, stone);
                // A capstone slightly wider than the levee, overhanging both faces.
                LegBox(boxes, start, direction, from, to, low - 0.12f, high + 0.12f, reach.wallTop - 0.2f,
                       reach.wallTop + kLeveeCapRise, coping);
            }
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

// `polygon` cut to the half plane where coordinate `axis` is at least `bound` (or at most it).
std::vector<glm::vec2> ClipToHalfPlane(const std::vector<glm::vec2>& polygon, int axis, float bound, bool keepAbove) {
    std::vector<glm::vec2> clipped;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const glm::vec2 a = polygon[i];
        const glm::vec2 b = polygon[(i + 1) % polygon.size()];
        const float inA = keepAbove ? a[axis] - bound : bound - a[axis];
        const float inB = keepAbove ? b[axis] - bound : bound - b[axis];
        if (inA >= 0.0f) {
            clipped.push_back(a);
        }
        if ((inA >= 0.0f) != (inB >= 0.0f)) {
            clipped.push_back(a + (b - a) * (inA / (inA - inB)));
        }
    }
    return clipped;
}

// Lowers the levees where the strip of a street crosses them. Each levee box that the strip touches
// is cut at both ends of the stretch the strip covers, and the middle is a solid stone sill from the
// box's foot up to ground level, so nothing is left open below the street. A capstone has no middle.
// The footprint of every stretch of levee lowered is added to the layout's river crossings.
void LowerLevees(CityLayout& city, const OrientedRect& strip) {
    const size_t boxCount = city.boxes.size();  // pieces added below are not looked at again
    for (size_t i = 0; i < boxCount; ++i) {
        const BoxInstance original = city.boxes[i];
        if (city.boxParts[i] != CityPart::Embankment || original.halfSize.x <= original.halfSize.z) {
            continue;  // only the long levee boxes along the reaches
        }
        const OrientedRect footprint(Rect{{original.center.x - original.halfSize.x, original.center.z - original.halfSize.z},
                                          {original.center.x + original.halfSize.x, original.center.z + original.halfSize.z}},
                                     original.yaw);
        if (!footprint.Overlaps(strip)) {
            continue;
        }

        // The part of the strip over this box, in the box's own frame, gives the stretch along it.
        std::vector<glm::vec2> covered;
        for (glm::vec2 corner : strip.Corners()) {
            covered.push_back(footprint.ToFrame(corner));
        }
        covered = ClipToHalfPlane(covered, 0, footprint.area.min.x, true);
        covered = ClipToHalfPlane(covered, 0, footprint.area.max.x, false);
        covered = ClipToHalfPlane(covered, 1, footprint.area.min.y, true);
        covered = ClipToHalfPlane(covered, 1, footprint.area.max.y, false);
        if (covered.empty()) {
            continue;
        }
        float low = covered.front().x;
        float high = covered.front().x;
        for (glm::vec2 point : covered) {
            low = std::min(low, point.x);
            high = std::max(high, point.x);
        }
        low = std::max(low - kSillCutMargin, footprint.area.min.x);
        high = std::min(high + kSillCutMargin, footprint.area.max.x);

        const float bottom = original.center.y - original.halfSize.y;
        const bool isBody = bottom < -4.0f;
        const glm::vec2 middle = footprint.Center();
        struct Piece {
            float from;
            float to;
            bool sill;
        };
        const Piece pieces[] = {{footprint.area.min.x, low, false}, {low, high, true}, {high, footprint.area.max.x, false}};
        bool reused = false;
        for (const Piece& piece : pieces) {
            if (piece.to - piece.from <= 0.05f || (piece.sill && !isBody)) {
                continue;
            }
            const glm::vec2 world = footprint.FromFrame({(piece.from + piece.to) * 0.5f, middle.y});
            BoxInstance part = original;
            part.center.x = world.x;
            part.center.z = world.y;
            part.halfSize.x = (piece.to - piece.from) * 0.5f;
            if (piece.sill) {
                part.center.y = (bottom + kSillTop) * 0.5f;
                part.halfSize.y = (kSillTop - bottom) * 0.5f;
                city.riverCrossings.push_back(OrientedRect(
                    Rect{world - glm::vec2(part.halfSize.x, part.halfSize.z), world + glm::vec2(part.halfSize.x, part.halfSize.z)},
                    original.yaw));
            }
            if (!reused) {
                city.boxes[i] = part;  // the first piece takes the old slot
                reused = true;
            } else {
                city.boxes.push_back(part);
                city.boxParts.push_back(CityPart::Embankment);
            }
        }
    }
}

}  // namespace

void BuildPlinth(const OrientedRect& square, float lift, BoxWriter& boxes) {
    PartScope scope(boxes, CityPart::Terrace);
    const glm::vec2 center = square.Center();
    const glm::vec2 half = square.Size() * 0.5f;
    boxes.TurnedBox({center.x, (kPlinthFoot + lift) * 0.5f, center.y}, {half.x, (lift - kPlinthFoot) * 0.5f, half.y},
                    square.yaw, StoneLook(kPlinthStone, 0.4f));
}

void BuildStair(glm::vec2 start, glm::vec2 direction, int steps, float rise, float tread, float streetHalfWidth,
                BoxWriter& boxes) {
    PartScope scope(boxes, CityPart::Stair);
    const Look paving = PavingLook();
    const Look stone = StoneLook(kPlinthStone, 0.4f);
    for (int step = 0; step < steps; ++step) {
        const float from = static_cast<float>(step) * tread;
        const float to = from + tread;
        // The top of the step is the walking surface, kPavementHeight above the ground that step stands for.
        const float top = kPavementHeight + static_cast<float>(step + 1) * rise;
        LegBox(boxes, start, direction, from, to, -streetHalfWidth, streetHalfWidth, kPlinthFoot, top, paving);
        for (float side : {-1.0f, 1.0f}) {
            const float low = side > 0.0f ? streetHalfWidth : -(streetHalfWidth + kStairCheek);
            const float high = side > 0.0f ? streetHalfWidth + kStairCheek : -streetHalfWidth;
            LegBox(boxes, start, direction, from, to, low, high, kPlinthFoot, top + kStairParapet, stone);
        }
    }
}

void BuildRiverBridge(glm::vec2 origin, glm::vec2 direction, float start, float end, uint32_t edge, float roadHalfWidth,
                      float streetHalfWidth, Random& random, BoxWriter& boxes, CityLayout& city) {
    const float outer = streetHalfWidth + kParapetWidth;
    {
        PartScope scope(boxes, CityPart::Bridge);
        const Look stone = StoneLook(kBridgeStone, random.Unit());
        auto strip = [&](float alongMin, float alongMax, float acrossMin, float acrossMax, float bottom, float top,
                         const Look& look) {
            LegBox(boxes, origin, direction, alongMin, alongMax, acrossMin, acrossMax, bottom, top, look);
        };
        BuildBridgeBody(strip, start, end, roadHalfWidth, streetHalfWidth, kReachWallThickness, false, stone);
    }

    // The street, the parapets and a little beyond them stand on the sill.
    const float sillFrom = start - kSillMargin;
    const float sillTo = end + kSillMargin;
    const float yaw = YawAlong(direction);
    auto stripAround = [&](float from, float to, float halfWidth) {
        const glm::vec2 center = origin + direction * ((from + to) * 0.5f);
        const glm::vec2 half((to - from) * 0.5f, halfWidth);
        return OrientedRect(Rect{center - half, center + half}, yaw);
    };
    const OrientedRect sillZone = stripAround(sillFrom, sillTo, outer + 1.0f);
    LowerLevees(city, sillZone);

    city.bridges.push_back({edge, stripAround(start, end, streetHalfWidth)});
    const glm::vec2 middle = origin + direction * ((start + end) * 0.5f) + LeftOf(direction) * (streetHalfWidth - 0.6f);
    city.pointsOfInterest.push_back({AtHeight(middle, kPavementHeight), LeftOf(direction), PointOfInterestKind::BridgeView});
}

void BuildConnectorStreet(const std::vector<glm::vec2>& path, float roadHalfWidth, float streetHalfWidth,
                          Random& random, BoxWriter& boxes, CityLayout& city, bool endsAtStair) {
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
            const float to = endsAtStreet ? (endsAtStair ? length : length - streetHalfWidth)
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
