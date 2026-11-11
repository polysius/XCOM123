#include "world/CityGenerator.h"

#include <algorithm>

#include "world/CityBuild.h"
#include "world/CityBuildings.h"
#include "world/CityLandmarks.h"
#include "world/CityNature.h"
#include "world/CityPlan.h"
#include "world/CitySatellites.h"
#include "world/CityStreets.h"

// The generator runs in stages, each in its own file:
//   CityPlan       river, canal, boulevards, districts, landmark sites and the street layout
//   CityStreets    the ground, roads, pavements, quays, bridges and street lamps
//   CityBuildings  ordinary blocks of houses
//   CityLandmarks  cathedral, palace, market hall, clock tower and parish churches
//   CityNature     squares, parks, boulevard medians, courtyards, trees and benches

namespace {

// Blocks this narrow are left as plain paving.
constexpr float kMinUsableBlock = 9.0f;

BlockUse ChooseBlockUse(const city::PlannedBlock& planned, const Rect& land, District district, city::Random& random) {
    if (planned.use != BlockUse::Buildings) {
        return planned.use;  // medians and landmark blocks are decided by the planner
    }
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

void BuildBlock(city::BoxWriter& boxes, CityLayout& layout, const city::CityPlan& plan, const city::PlannedBlock& planned,
                uint32_t blockSeed) {
    const Rect land = planned.area.Expanded(-plan.streetHalfWidth);
    const glm::vec2 size = land.Size();
    if (std::min(size.x, size.y) < kMinUsableBlock && planned.use != BlockUse::Median) {
        return;
    }
    city::Random random(blockSeed);
    const District district = plan.districts.At(land.Center());
    const BlockUse use = ChooseBlockUse(planned, land, district, random);
    layout.blocks.push_back({land, district, use});
    switch (use) {
    case BlockUse::Median:
        city::BuildMedian(boxes, layout, land, random);
        break;
    case BlockUse::Landmark:
        city::BuildLandmark(boxes, layout, plan.sites[static_cast<size_t>(planned.site)].kind, land, district, random);
        break;
    case BlockUse::Park:
        city::BuildPark(boxes, layout, land, random);
        break;
    case BlockUse::Plaza:
        city::BuildPlaza(boxes, layout, land, district, random);
        break;
    case BlockUse::Buildings:
    default:
        city::BuildBlockOfBuildings(boxes, layout, land, district, random);
        break;
    }
}

}  // namespace

CityLayout GenerateCity(const CitySettings& settings) {
    const city::CityPlan plan = city::PlanCity(settings);

    CityLayout layout;
    layout.roadWidth = settings.roadWidth;
    layout.sidewalkWidth = settings.sidewalkWidth;
    layout.sidewalkCenterOffset = settings.roadWidth * 0.5f + settings.sidewalkWidth * 0.5f;
    layout.walkHeight = city::kPavementHeight;
    layout.waterLevel = city::kWaterLevel;
    const float outerEdge = plan.halfSize + plan.streetHalfWidth;
    layout.bounds = {glm::vec2(-outerEdge), glm::vec2(outerEdge)};
    layout.coreBounds = layout.bounds;
    layout.streets = city::BuildStreetGraph(plan.segments);

    city::BoxWriter boxes(layout);
    city::BuildStreetsAndWater(plan, settings.seed, boxes, layout);
    for (size_t i = 0; i < plan.blocks.size(); ++i) {
        BuildBlock(boxes, layout, plan, plan.blocks[i], city::MixSeed(settings.seed, 0x10000u + static_cast<uint32_t>(i)));
    }
    if (settings.satelliteCount > 0 && !settings.districtOnly) {
        city::AddSatellites(layout, settings, plan);
    }
    return layout;
}

const char* DistrictName(District district) {
    switch (district) {
    case District::OldQuarter: return "old quarter";
    case District::Grand: return "grand district";
    case District::Market: return "market district";
    case District::Residential: return "residential district";
    case District::Parkland: return "parkland";
    default: return "unknown district";
    }
}

const char* CityPartName(CityPart part) {
    switch (part) {
    case CityPart::Ground: return "ground";
    case CityPart::Pavement: return "pavement";
    case CityPart::Plaza: return "plaza";
    case CityPart::Park: return "park";
    case CityPart::Water: return "water";
    case CityPart::Embankment: return "embankment";
    case CityPart::Bridge: return "bridge";
    case CityPart::Building: return "building";
    case CityPart::Landmark: return "landmark";
    case CityPart::Tree: return "tree";
    case CityPart::StreetLamp: return "street lamp";
    case CityPart::Bench: return "bench";
    case CityPart::Fountain: return "fountain";
    case CityPart::MarketStall: return "market stall";
    default: return "unknown";
    }
}
