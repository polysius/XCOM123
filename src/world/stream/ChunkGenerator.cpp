#include "world/stream/ChunkGenerator.h"

#include <algorithm>
#include <cmath>

#include "world/CityBuild.h"
#include "world/CityBuildings.h"
#include "world/CityLandmarks.h"
#include "world/CityNature.h"
#include "world/CityPlan.h"

namespace world {
namespace {

using city::AsphaltLook;
using city::PartScope;
using city::PavingLook;
using city::Random;

constexpr uint32_t kBlockSalt = 0x424C4B53u;
constexpr uint32_t kRiverSalt = 0x52495645u;
constexpr uint32_t kCrowdSalt = 0x43524F57u;

// Blocks this narrow are left as plain paving, as in the fixed city.
constexpr float kMinUsableBlock = 9.0f;

// Street props: the chance that a block edge has a stack of crates, and how far from the corners it stands.
constexpr float kPropChancePerEdge = 0.4f;
constexpr float kPropEdgeMargin = 14.0f;

constexpr float kLampSpacing = 32.0f;
constexpr float kLampFromCorner = 10.0f;
constexpr float kLampKerbOffset = kRoadHalfWidth + 0.32f;
constexpr float kLampHeight = 4.3f;  // light centre above the pavement

// River geometry, in metres.
constexpr float kRiverBed = -12.0f;
constexpr float kBankFoot = -8.0f;
constexpr float kBankTop = -0.05f;      // just under the lawn, so a street over the bank needs no step
constexpr float kRiverSampleStep = 6.0f;
constexpr float kWaterOverlap = 1.2f;   // neighbouring river boxes overlap, which hides the wedge at a bend
constexpr float kBankOverlap = 2.0f;
constexpr float kLawnStrip = 2.0f;      // the lawn beside the river is laid in strips this wide
constexpr float kLawnHoleMargin = 1.0f; // the lawn stops this far from the water, under the bank's width
constexpr float kDeckBottom = -1.2f;
constexpr float kParapetWidth = 0.5f;
constexpr float kParapetTop = 1.05f;
constexpr float kQuayTreeSpacing = 10.0f;

const glm::vec3 kQuayStone{0.55f, 0.51f, 0.45f};
const glm::vec3 kBridgeStone{0.62f, 0.58f, 0.51f};
const glm::vec3 kLampIron{0.055f, 0.06f, 0.065f};
const glm::vec3 kLanternGlass{0.95f, 0.78f, 0.52f};

glm::vec2 LeftOf(glm::vec2 direction) {
    return {-direction.y, direction.x};
}

// The yaw of a box whose own X axis runs along `direction` on the ground.
float YawAlong(glm::vec2 direction) {
    return std::atan2(-direction.y, direction.x);
}

float BoxReach(const BoxInstance& box, int axis) {
    const float cosine = std::abs(std::cos(box.yaw));
    const float sine = std::abs(std::sin(box.yaw));
    switch (axis) {
    case 0: return cosine * box.halfSize.x + sine * box.halfSize.z;
    case 1: return box.halfSize.y;
    default: return sine * box.halfSize.x + cosine * box.halfSize.z;
    }
}

Aabb BoundsOfBoxes(const std::vector<BoxInstance>& boxes, const Rect& fallback) {
    if (boxes.empty()) {
        return {{fallback.min.x, 0.0f, fallback.min.y}, {fallback.max.x, 0.0f, fallback.max.y}};
    }
    Aabb bounds{glm::vec3(INFINITY), glm::vec3(-INFINITY)};
    for (const BoxInstance& box : boxes) {
        const glm::vec3 reach(BoxReach(box, 0), BoxReach(box, 1), BoxReach(box, 2));
        bounds.min = glm::min(bounds.min, box.center - reach);
        bounds.max = glm::max(bounds.max, box.center + reach);
    }
    return bounds;
}

// The rectangle of a street of the lattice: a strip along Z at `x`, or along X at `z`.
Rect RoadAlongZ(float x, float z0, float z1) {
    return {{x - kRoadHalfWidth, z0}, {x + kRoadHalfWidth, z1}};
}

Rect RoadAlongX(float z, float x0, float x1) {
    return {{x0, z - kRoadHalfWidth}, {x1, z + kRoadHalfWidth}};
}

class ChunkBuilder {
public:
    ChunkBuilder(uint32_t worldSeed, ChunkData& chunk) : seed(worldSeed), data(chunk), layout(chunk.layout), boxes(chunk.layout) {}

    void Build() {
        PrepareLayout();
        const int firstColumn = data.coord.x * kCellsPerChunk;
        const int firstRow = data.coord.z * kCellsPerChunk;
        for (int row = firstRow; row < firstRow + kCellsPerChunk; ++row) {
            for (int column = firstColumn; column < firstColumn + kCellsPerChunk; ++column) {
                BuildCell(PlanCell(seed, column, row), column == firstColumn + kCellsPerChunk - 1,
                          row == firstRow + kCellsPerChunk - 1);
            }
        }
        layout.streets = city::BuildStreetGraph(segments);
    }

private:
    void PrepareLayout() {
        layout.roadWidth = kRoadHalfWidth * 2.0f;
        layout.sidewalkWidth = kSidewalkWidth;
        layout.sidewalkCenterOffset = kRoadHalfWidth + kSidewalkWidth * 0.5f;
        layout.walkHeight = city::kPavementHeight;
        layout.waterLevel = city::kWaterLevel;
        const Rect first = CellArea(seed, data.coord.x * kCellsPerChunk, data.coord.z * kCellsPerChunk);
        const Rect last = CellArea(seed, data.coord.x * kCellsPerChunk + kCellsPerChunk - 1,
                                   data.coord.z * kCellsPerChunk + kCellsPerChunk - 1);
        layout.bounds = {first.min, last.max};
        layout.coreBounds = layout.bounds;
    }

    // --- Cells -----------------------------------------------------------------------------------------------

    // `lastColumn` and `lastRow` say that the cell is on the east or north edge of the chunk, which is where the
    // chunk's street graph needs the arterial that the next chunk would otherwise own.
    void BuildCell(const CellPlan& plan, bool lastColumn, bool lastRow) {
        AddStreetSegments(plan, lastColumn, lastRow);
        if (plan.river) {
            BuildRiverCell(plan);
        } else {
            BuildLandCell(plan);
        }
    }

    void AddStreetSegments(const CellPlan& plan, bool lastColumn, bool lastRow) {
        const Rect& area = plan.area;
        segments.push_back({{area.min.x, area.min.y}, {area.min.x, area.max.y}});
        segments.push_back({{area.min.x, area.min.y}, {area.max.x, area.min.y}});
        if (lastColumn) {
            segments.push_back({{area.max.x, area.min.y}, {area.max.x, area.max.y}});
        }
        if (lastRow) {
            segments.push_back({{area.min.x, area.max.y}, {area.max.x, area.max.y}});
        }
        for (const float x : plan.xSplits) {
            segments.push_back({{x, area.min.y}, {x, area.max.y}});
        }
        for (const float z : plan.zSplits) {
            segments.push_back({{area.min.x, z}, {area.max.x, z}});
        }
    }

    // --- Cells of buildings, squares and parks -----------------------------------------------------------------

    void BuildLandCell(const CellPlan& plan) {
        LayLandCellRoads(plan);
        for (size_t index = 0; index < plan.blocks.size(); ++index) {
            const bool landmark = plan.landmark && index == 0;
            BuildBlock(plan, index, landmark ? plan.landmark : std::nullopt);
            AddBlockLamps(plan.blocks[index]);
        }
    }

    // The asphalt of the streets this cell owns: its west and south arterials and its local streets. The east and north
    // arterials belong to the neighbouring cells, which lay them as their own west and south ones.
    void LayLandCellRoads(const CellPlan& plan) {
        const Rect& area = plan.area;
        PartScope scope(boxes, CityPart::Ground);
        std::vector<float> alongZ{area.min.x};
        alongZ.insert(alongZ.end(), plan.xSplits.begin(), plan.xSplits.end());
        std::vector<float> alongX{area.min.y};
        alongX.insert(alongX.end(), plan.zSplits.begin(), plan.zSplits.end());
        for (const float x : alongZ) {
            Asphalt(RoadAlongZ(x, area.min.y - kRoadHalfWidth, area.max.y + kRoadHalfWidth));
        }
        for (const float z : alongX) {
            Asphalt(RoadAlongX(z, area.min.x - kRoadHalfWidth, area.max.x + kRoadHalfWidth));
        }
    }

    void Asphalt(const Rect& rect) { boxes.Footprint(rect, -1.0f, 0.0f, AsphaltLook()); }

    void Paving(const Rect& rect) {
        PartScope scope(boxes, CityPart::Pavement);
        boxes.Footprint(rect, -1.0f, city::kPavementHeight, PavingLook());
    }

    static BlockUse ChooseBlockUse(const Rect& land, District district, Random& random) {
        const city::DistrictStyle& style = city::StyleOf(district);
        const float roll = random.Unit();
        if (roll < style.parkChance && std::min(land.Size().x, land.Size().y) >= 30.0f) {
            return BlockUse::Park;
        }
        if (roll < style.parkChance + style.plazaChance) {
            return BlockUse::Plaza;
        }
        return BlockUse::Buildings;
    }

    void BuildBlock(const CellPlan& plan, size_t index, std::optional<LandmarkKind> landmark) {
        const Rect& planned = plan.blocks[index];
        Paving(planned.Expanded(-kRoadHalfWidth));  // the whole block is paved to the kerb; its land is covered next
        const Rect land = planned.Expanded(-kStreetHalfWidth);
        if (std::min(land.Size().x, land.Size().y) < kMinUsableBlock) {
            return;
        }
        Random random(city::MixSeed(HashBits(seed, kBlockSalt, plan.column, plan.row), static_cast<uint32_t>(index)));
        const BlockUse use = landmark ? BlockUse::Landmark : ChooseBlockUse(land, plan.district, random);
        layout.blocks.push_back({land, plan.district, use});
        switch (use) {
        case BlockUse::Landmark:
            city::BuildLandmark(boxes, layout, *landmark, land, plan.district, random);
            break;
        case BlockUse::Park:
            city::BuildPark(boxes, layout, land, random);
            break;
        case BlockUse::Plaza:
            city::BuildPlaza(boxes, layout, land, plan.district, random);
            break;
        case BlockUse::Buildings:
        default:
            city::BuildBlockOfBuildings(boxes, layout, land, plan.district, random);
            AddStreetProps(land, plan.district, random);
            break;
        }
    }

    // Crates and barrels stacked against the walls along the street, in the districts of shops and workshops. They stand
    // in the strip of pavement next to the wall, clear of the lane people walk in. They come after the buildings in the block's
    // random stream, so they never change how the buildings turn out.
    void AddStreetProps(const Rect& land, District district, Random& random) {
        if (district != District::Market && district != District::OldQuarter) {
            return;
        }
        PartScope scope(boxes, CityPart::MarketStall);
        const city::Look crate{{0.50f, 0.36f, 0.20f}, SurfaceMaterial::Wood, 0.85f, 0.0f};
        const city::Look barrel{{0.30f, 0.22f, 0.14f}, SurfaceMaterial::Wood, 0.8f, 0.0f};
        struct Edge {
            glm::vec2 start;   // the end of the edge with the smaller coordinate
            glm::vec2 along;   // unit direction of the edge
            glm::vec2 outward; // towards the street
            float length;
        };
        const Edge edges[] = {
            {land.min, {1.0f, 0.0f}, {0.0f, -1.0f}, land.Size().x},
            {{land.min.x, land.max.y}, {1.0f, 0.0f}, {0.0f, 1.0f}, land.Size().x},
            {land.min, {0.0f, 1.0f}, {-1.0f, 0.0f}, land.Size().y},
            {{land.max.x, land.min.y}, {0.0f, 1.0f}, {1.0f, 0.0f}, land.Size().y},
        };
        for (const Edge& edge : edges) {
            if (edge.length < 2.0f * kPropEdgeMargin || !random.Chance(kPropChancePerEdge)) {
                continue;
            }
            const glm::vec2 spot = edge.start + edge.along * random.Range(kPropEdgeMargin, edge.length - kPropEdgeMargin) +
                                   edge.outward * 0.45f;
            const float ground = city::kPavementHeight;
            boxes.Box({spot.x - 0.4f, ground, spot.y - 0.4f}, {spot.x + 0.4f, ground + 0.8f, spot.y + 0.4f}, crate);
            if (random.Chance(0.5f)) {
                boxes.Box({spot.x - 0.35f, ground + 0.8f, spot.y - 0.35f}, {spot.x + 0.35f, ground + 1.5f, spot.y + 0.35f}, crate);
            } else {
                const glm::vec2 beside = spot + edge.along * 0.95f;
                boxes.Box({beside.x - 0.28f, ground, beside.y - 0.28f}, {beside.x + 0.28f, ground + 0.9f, beside.y + 0.28f}, barrel);
            }
        }
    }

    // Lamps along the south and west edges of a block, on its side of the street. The other two edges are lit from the
    // neighbouring blocks' side, so a street has a lamp row on one side of it only and a chunk never lights a street
    // that another chunk lights too.
    void AddBlockLamps(const Rect& planned) {
        PartScope scope(boxes, CityPart::StreetLamp);
        const float southZ = planned.min.y + kLampKerbOffset;
        for (const float x : LampPositions(planned.min.x, planned.max.x)) {
            AddLamp({x, southZ});
        }
        const float westX = planned.min.x + kLampKerbOffset;
        for (const float z : LampPositions(planned.min.y, planned.max.y)) {
            AddLamp({westX, z});
        }
    }

    static std::vector<float> LampPositions(float start, float end) {
        std::vector<float> positions;
        const float usable = end - start - kLampFromCorner * 2.0f;
        if (usable < 0.0f) {
            return positions;
        }
        const int count = static_cast<int>(usable / kLampSpacing) + 1;
        for (int i = 0; i < count; ++i) {
            const float along = count == 1 ? usable * 0.5f : usable * static_cast<float>(i) / static_cast<float>(count - 1);
            positions.push_back(start + kLampFromCorner + along);
        }
        return positions;
    }

    // A lamp post standing on the pavement at `position`, and its light in the layout.
    void AddLamp(glm::vec2 position) {
        const city::Look iron{kLampIron, SurfaceMaterial::Metal, 0.45f, 0.0f};
        const city::Look lantern{kLanternGlass, SurfaceMaterial::Lamp, 0.2f, 0.0f};
        const float ground = city::kPavementHeight;
        auto post = [&](float halfWidth, float bottom, float top, const city::Look& look) {
            boxes.Box({position.x - halfWidth, ground + bottom, position.y - halfWidth},
                      {position.x + halfWidth, ground + top, position.y + halfWidth}, look);
        };
        post(0.18f, 0.0f, 0.55f, iron);
        post(0.07f, 0.55f, 4.0f, iron);
        post(0.22f, 4.0f, 4.6f, lantern);
        post(0.28f, 4.6f, 4.72f, iron);
        layout.streetLights.push_back(city::AtHeight(position, ground + kLampHeight));
    }

    // --- Cells of river -------------------------------------------------------------------------------------

    // The river runs through the middle of the cell. The cell has no blocks: lawns and trees fill the land on either
    // side, and the arterial along Z that the cell owns crosses the river on a bridge.
    void BuildRiverCell(const CellPlan& plan) {
        BuildRiverWater(plan);
        LayRiverLawns(plan);
        LayRiverQuays(plan);
        LayBridgeStreet(plan);
        PlantRiverside(plan);
    }

    float CentreZ(const CellPlan& plan, float x) const { return RiverCentreZ(seed, plan.row, x); }

    // The river boxes of one stretch of the curve: water and a thick bank on each side, along the chord between two
    // points of the curve.
    void AddRiverStretch(glm::vec2 start, glm::vec2 end) {
        const float length = glm::distance(start, end);
        const glm::vec2 direction = (end - start) / length;
        const glm::vec2 middle = (start + end) * 0.5f;
        const float yaw = YawAlong(direction);
        {
            PartScope scope(boxes, CityPart::Water);
            const float halfLength = length * 0.5f + kWaterOverlap;
            boxes.TurnedBox({middle.x, (kRiverBed + city::kWaterLevel) * 0.5f, middle.y},
                            {halfLength, (city::kWaterLevel - kRiverBed) * 0.5f, kRiverHalfWidth}, yaw, city::WaterLook());
            layout.water.push_back(OrientedRect(Rect{middle - glm::vec2(halfLength, kRiverHalfWidth),
                                                     middle + glm::vec2(halfLength, kRiverHalfWidth)}, yaw));
        }
        PartScope scope(boxes, CityPart::Embankment);
        const city::Look stone = city::StoneLook(kQuayStone, 0.3f);
        for (const float side : {-1.0f, 1.0f}) {
            const glm::vec2 centre = middle + LeftOf(direction) * (side * (kRiverHalfWidth + kBankThickness * 0.5f));
            boxes.TurnedBox({centre.x, (kBankFoot + kBankTop) * 0.5f, centre.y},
                            {length * 0.5f + kBankOverlap, (kBankTop - kBankFoot) * 0.5f, kBankThickness * 0.5f}, yaw, stone);
        }
    }

    void BuildRiverWater(const CellPlan& plan) {
        const float start = plan.area.min.x;
        const float end = plan.area.max.x;
        const int stretches = std::max(1, static_cast<int>(std::ceil((end - start) / kRiverSampleStep)));
        glm::vec2 previous(start, CentreZ(plan, start));
        for (int i = 1; i <= stretches; ++i) {
            const float x = i == stretches ? end : start + (end - start) * static_cast<float>(i) / static_cast<float>(stretches);
            const glm::vec2 next(x, CentreZ(plan, x));
            AddRiverStretch(previous, next);
            previous = next;
        }
    }

    // The lowest and highest the river's centre line gets between two X, which bounds where water and banks can be.
    glm::vec2 CentreRange(const CellPlan& plan, float x0, float x1) const {
        float low = INFINITY;
        float high = -INFINITY;
        constexpr int kSamples = 5;
        for (int i = 0; i < kSamples; ++i) {
            const float z = CentreZ(plan, x0 + (x1 - x0) * static_cast<float>(i) / static_cast<float>(kSamples - 1));
            low = std::min(low, z);
            high = std::max(high, z);
        }
        return {low, high};
    }

    // Lawn on both sides of the river in strips along X. The strips stop a little way from the water, inside the width
    // of the bank, so that the bank hides the stepped edge they make against the curve.
    void LayRiverLawns(const CellPlan& plan) {
        PartScope scope(boxes, CityPart::Ground);
        const float from = plan.area.min.x + kStreetHalfWidth;
        const float to = plan.area.max.x - kStreetHalfWidth;
        const float lowLand = plan.area.min.y + kStreetHalfWidth;
        const float highLand = plan.area.max.y - kStreetHalfWidth;
        const int strips = std::max(1, static_cast<int>(std::ceil((to - from) / kLawnStrip)));
        for (int i = 0; i < strips; ++i) {
            const float x0 = from + (to - from) * static_cast<float>(i) / static_cast<float>(strips);
            const float x1 = from + (to - from) * static_cast<float>(i + 1) / static_cast<float>(strips);
            const glm::vec2 range = CentreRange(plan, x0, x1);
            const float holeLow = range.x - kRiverHalfWidth - kLawnHoleMargin;
            const float holeHigh = range.y + kRiverHalfWidth + kLawnHoleMargin;
            const city::Look grass = city::GrassLook(0.3f);
            if (holeLow > lowLand) {
                boxes.Footprint({{x0, lowLand}, {x1, holeLow}}, -1.0f, 0.0f, grass);
            }
            if (highLand > holeHigh) {
                boxes.Footprint({{x0, holeHigh}, {x1, highLand}}, -1.0f, 0.0f, grass);
            }
        }
    }

    // The pavements along the two arterials that bound the river's row, on the river's side. The asphalt of the south
    // arterial is the cell's own; the north one is laid by the row above, which also lays its far pavement.
    void LayRiverQuays(const CellPlan& plan) {
        const Rect& area = plan.area;
        const float x0 = area.min.x - kStreetHalfWidth;
        const float x1 = area.max.x + kStreetHalfWidth;
        {
            PartScope scope(boxes, CityPart::Ground);
            Asphalt(RoadAlongX(area.min.y, area.min.x - kRoadHalfWidth, area.max.x + kRoadHalfWidth));
        }
        Paving({{x0, area.min.y + kRoadHalfWidth}, {x1, area.min.y + kStreetHalfWidth}});
        Paving({{x0, area.max.y - kStreetHalfWidth}, {x1, area.max.y - kRoadHalfWidth}});
        PartScope scope(boxes, CityPart::StreetLamp);
        for (const float x : LampPositions(area.min.x, area.max.x)) {
            AddLamp({x, area.min.y + kLampKerbOffset});
            AddLamp({x, area.max.y - kLampKerbOffset});
        }
    }

    // The arterial along Z at the cell's west edge, with a bridge over the river and its banks.
    void LayBridgeStreet(const CellPlan& plan) {
        const Rect& area = plan.area;
        const float x = area.min.x;
        const glm::vec2 range = CentreRange(plan, x - kStreetHalfWidth, x + kStreetHalfWidth);
        const float reach = kRiverHalfWidth + kBankThickness + 2.0f;
        const float deckStart = range.x - reach;
        const float deckEnd = range.y + reach;
        const float streetStart = area.min.y + kRoadHalfWidth;
        const float streetEnd = area.max.y - kRoadHalfWidth;
        {
            PartScope scope(boxes, CityPart::Ground);
            Asphalt(RoadAlongZ(x, area.min.y - kRoadHalfWidth, deckStart));
            Asphalt(RoadAlongZ(x, deckEnd, area.max.y + kRoadHalfWidth));
        }
        for (const float side : {-1.0f, 1.0f}) {
            const float inner = x + side * kRoadHalfWidth;
            const float outer = x + side * kStreetHalfWidth;
            const Rect pavement{{std::min(inner, outer), streetStart}, {std::max(inner, outer), streetEnd}};
            Paving(pavement);
        }
        BuildDeck(x, deckStart, deckEnd);
    }

    void BuildDeck(float x, float start, float end) {
        PartScope scope(boxes, CityPart::Bridge);
        const city::Look stone = city::StoneLook(kBridgeStone, 0.4f);
        boxes.Footprint({{x - kStreetHalfWidth, start}, {x + kStreetHalfWidth, end}}, kDeckBottom, -0.2f, stone);
        boxes.Footprint({{x - kRoadHalfWidth, start}, {x + kRoadHalfWidth, end}}, -0.2f, 0.0f, AsphaltLook());
        for (const float side : {-1.0f, 1.0f}) {
            const float inner = x + side * kStreetHalfWidth;
            const float outer = x + side * (kStreetHalfWidth + kParapetWidth);
            boxes.Footprint({{std::min(inner, outer), start}, {std::max(inner, outer), end}}, -1.0f, kParapetTop, stone);
        }
        layout.bridges.push_back({0, OrientedRect(Rect{{x - kStreetHalfWidth, start}, {x + kStreetHalfWidth, end}})});
    }

    // Trees along the quays and scattered over the lawns, and a bench now and then looking over the water.
    void PlantRiverside(const CellPlan& plan) {
        Random random(HashBits(seed, kRiverSalt, plan.column, plan.row));
        const Rect& area = plan.area;
        const float southRow = area.min.y + kStreetHalfWidth + 3.5f;
        const float northRow = area.max.y - kStreetHalfWidth - 3.5f;
        int count = 0;
        for (float x = area.min.x + kStreetHalfWidth + 6.0f; x < area.max.x - kStreetHalfWidth - 4.0f;
             x += kQuayTreeSpacing, ++count) {
            const float jitter = random.Range(-2.0f, 2.0f);
            city::AddTree(boxes, layout, {x + jitter, southRow}, city::kPavementHeight, random.Range(9.0f, 13.0f), random);
            city::AddTree(boxes, layout, {x - jitter, northRow}, city::kPavementHeight, random.Range(9.0f, 13.0f), random);
            if (count % 4 == 2) {
                city::AddBench(boxes, layout, {x, area.min.y + kStreetHalfWidth + 8.0f}, city::kPavementHeight, {0.0f, 1.0f});
            }
        }
    }

    uint32_t seed;
    ChunkData& data;
    CityLayout& layout;
    city::BoxWriter boxes;
    std::vector<city::Segment> segments;  // street centre lines, for the street graph
};

}  // namespace

Rect ChunkRect(streaming::ChunkCoord coord) {
    const glm::vec2 min(static_cast<float>(coord.x) * kChunkSize, static_cast<float>(coord.z) * kChunkSize);
    return {min, min + glm::vec2(kChunkSize)};
}

streaming::ChunkCoord ChunkAt(glm::vec2 position) {
    return {static_cast<int32_t>(std::floor(position.x / kChunkSize)), static_cast<int32_t>(std::floor(position.y / kChunkSize))};
}

size_t ChunkData::ResidentBytes() const {
    constexpr size_t kBytesPerBoxEntry = sizeof(BoxInstance) + sizeof(CityPart);
    // The crowd keeps grids of obstacles and street lookups whose size follows the chunk's area, about 1.5 MB.
    constexpr size_t kBytesPerCrowd = 1536u * 1024u;
    return layout.boxes.size() * kBytesPerBoxEntry + (crowd ? kBytesPerCrowd : 0u);
}

uint32_t PeoplePerChunk(uint32_t peopleTotal, float loadRadius) {
    if (peopleTotal == 0 || loadRadius <= 0.0f) {
        return 0;
    }
    constexpr float kPi = 3.14159265f;
    const float perSquareMetre = static_cast<float>(peopleTotal) / (kPi * loadRadius * loadRadius);
    return std::max(1u, static_cast<uint32_t>(std::lround(perSquareMetre * kChunkSize * kChunkSize)));
}

std::shared_ptr<ChunkData> GenerateChunk(const ChunkSettings& settings, streaming::ChunkCoord coord) {
    auto chunk = std::make_shared<ChunkData>();
    chunk->coord = coord;
    ChunkBuilder(settings.seed, *chunk).Build();
    chunk->bounds = BoundsOfBoxes(chunk->layout.boxes, ChunkRect(coord));
    if (settings.agentCount > 0) {
        chunk->crowd = settings.crowdJobs != nullptr ? std::make_shared<Crowd>(*settings.crowdJobs) : std::make_shared<Crowd>();
        chunk->crowd->Spawn(chunk->layout, {.agentCount = settings.agentCount,
                                            .seed = HashBits(settings.seed, kCrowdSalt, coord.x, coord.z)});
    }
    return chunk;
}

}  // namespace world
