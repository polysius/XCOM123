#include "world/CityBuildings.h"

#include <algorithm>
#include <cmath>

#include "world/CityNature.h"
#include "world/CityPlan.h"

namespace city {
namespace {

// Lime-washed and painted plaster for the old quarter.
const glm::vec3 kPlaster[] = {
    {0.78f, 0.60f, 0.38f},  // ochre
    {0.84f, 0.78f, 0.62f},  // cream
    {0.80f, 0.62f, 0.55f},  // faded pink
    {0.62f, 0.66f, 0.58f},  // grey green
    {0.70f, 0.45f, 0.32f},  // terracotta
    {0.64f, 0.68f, 0.70f},  // blue grey
    {0.86f, 0.84f, 0.78f},  // white
    {0.66f, 0.48f, 0.34f},  // sienna
};
// Dressed limestone for the grand district and the better streets.
const glm::vec3 kLimestone[] = {
    {0.78f, 0.72f, 0.62f},
    {0.74f, 0.69f, 0.60f},
    {0.82f, 0.77f, 0.68f},
    {0.70f, 0.65f, 0.56f},
    {0.80f, 0.74f, 0.62f},
};
const glm::vec3 kBrick[] = {
    {0.45f, 0.26f, 0.19f},
    {0.52f, 0.31f, 0.22f},
    {0.40f, 0.24f, 0.18f},
};
const glm::vec3 kSlate[] = {
    {0.20f, 0.23f, 0.28f},
    {0.24f, 0.26f, 0.30f},
    {0.18f, 0.20f, 0.23f},
};
const glm::vec3 kClay[] = {
    {0.50f, 0.25f, 0.15f},
    {0.58f, 0.32f, 0.19f},
    {0.45f, 0.24f, 0.16f},
};
const glm::vec3 kAwnings[] = {
    {0.52f, 0.11f, 0.09f},
    {0.13f, 0.28f, 0.18f},
    {0.60f, 0.50f, 0.28f},
    {0.16f, 0.22f, 0.40f},
    {0.58f, 0.56f, 0.50f},
};
const glm::vec3 kChimneyBrick{0.42f, 0.27f, 0.21f};

constexpr float kLedgeBottom = kGroundFloorHeight - 0.05f;
constexpr float kAtticHeight = kUpperFloorHeight;
constexpr float kMinRoofWidth = 0.6f;  // narrowest top a stepped roof may shrink to
constexpr float kMansardTopInset = 2.6f;  // how far the flat top of a mansard roof is set in from the walls
constexpr float kMinCourtyard = 8.0f;
constexpr float kRearJitter = 2.5f;    // how far the back wall of a house may step in from the courtyard edge

uint8_t OppositeSides(uint8_t sides) {
    uint8_t opposite = 0;
    if (sides & kSideWest) opposite |= kSideEast;
    if (sides & kSideEast) opposite |= kSideWest;
    if (sides & kSideSouth) opposite |= kSideNorth;
    if (sides & kSideNorth) opposite |= kSideSouth;
    return opposite;
}

// Shrinks the given sides by `amount`, but never below kMinRoofWidth across.
Rect ShrinkClamped(const Rect& rect, uint8_t sides, float amount) {
    const glm::vec2 size = rect.Size();
    auto limit = [&](uint8_t low, uint8_t high, float extent) {
        const int count = ((sides & low) ? 1 : 0) + ((sides & high) ? 1 : 0);
        return count == 0 ? 0.0f : std::min(amount, std::max(0.0f, (extent - kMinRoofWidth) / static_cast<float>(count)));
    };
    const float x = limit(kSideWest, kSideEast, size.x);
    const float z = limit(kSideSouth, kSideNorth, size.y);
    return InsetRect(rect, (sides & kSideWest) ? x : 0.0f, (sides & kSideEast) ? x : 0.0f,
                     (sides & kSideSouth) ? z : 0.0f, (sides & kSideNorth) ? z : 0.0f);
}

// A strip along one side of `rect`, from `from` to `to` metres outside the edge (negative is
// inside), shortened by `trim` at both ends.
Rect EdgeBand(const Rect& rect, Side side, float from, float to, float trim) {
    switch (side) {
    case kSideWest:
        return {{rect.min.x - to, rect.min.y + trim}, {rect.min.x - from, rect.max.y - trim}};
    case kSideEast:
        return {{rect.max.x + from, rect.min.y + trim}, {rect.max.x + to, rect.max.y - trim}};
    case kSideSouth:
        return {{rect.min.x + trim, rect.min.y - to}, {rect.max.x - trim, rect.min.y - from}};
    case kSideNorth:
    default:
        return {{rect.min.x + trim, rect.max.y + from}, {rect.max.x - trim, rect.max.y + to}};
    }
}

constexpr Side kSides[] = {kSideWest, kSideEast, kSideSouth, kSideNorth};

// A repeatable yes or no for a lot, decided from where it stands. The architecture details use this
// instead of the random generator so that choosing them never changes the draws that follow, and
// the layout of the city stays exactly as it was.
bool LotHasFeature(const Lot& lot, uint32_t salt, float chance) {
    const auto quantised = [](float metres) { return static_cast<uint32_t>(static_cast<int32_t>(std::lround(metres * 4.0f))); };
    const uint32_t hash = MixSeed(MixSeed(salt, quantised(lot.area.min.x)), quantised(lot.area.min.y));
    return static_cast<float>(hash & 0xFFFFu) / 65536.0f < chance;
}

// Whether a lot gets a belvedere. Only a house with a single street front and a mansard roof is
// considered, since a corner lot has its chimney in the middle of the roof.
bool WantsBelvedere(District district, const Lot& lot) {
    constexpr uint32_t kBelvedereSalt = 0x42454C56u;
    const bool corner = (lot.streetSides & (lot.streetSides - 1)) != 0;
    if (lot.streetSides == 0 || corner) {
        return false;
    }
    switch (district) {
    case District::OldQuarter:
        return LotHasFeature(lot, kBelvedereSalt, 0.12f);
    case District::Market:
    case District::Residential:
        return LotHasFeature(lot, kBelvedereSalt, 0.08f);
    case District::Grand:
    case District::Parkland:
    default:
        return false;
    }
}

// Round-headed openings and troweled plaster, the softer building language of the hill towns that
// inspired the map. The grand district keeps its square-cut limestone.
uint32_t ArchitectureFlags(District district, const Lot& lot) {
    constexpr uint32_t kArchSalt = 0x41524348u;
    constexpr uint32_t kStuccoSalt = 0x53544B4Fu;
    switch (district) {
    case District::OldQuarter:
        return (LotHasFeature(lot, kArchSalt, 0.3f) ? kFacadeArched : 0u) |
               (LotHasFeature(lot, kStuccoSalt, 0.5f) ? kFacadeStucco : 0u);
    case District::Market:
        return LotHasFeature(lot, kArchSalt, 0.6f) ? kFacadeArched : 0u;
    case District::Residential:
        return (LotHasFeature(lot, kArchSalt, 0.2f) ? kFacadeArched : 0u) |
               (LotHasFeature(lot, kStuccoSalt, 0.3f) ? kFacadeStucco : 0u);
    case District::Grand:
    case District::Parkland:
    default:
        return 0u;
    }
}

// A slab the size of the lot that sticks out `overhang` over the street sides. It is pulled in
// slightly at the party walls so that neighbouring slabs never share a face and flicker.
Rect ProjectingSlab(const Lot& lot, float overhang) {
    const Rect pulledIn = ExpandSides(lot.area, static_cast<uint8_t>(kAllSides & ~lot.streetSides), -0.05f);
    return ExpandSides(pulledIn, lot.streetSides, overhang);
}

// The sides a roof slopes down towards: the street front and the back, or every side on a corner.
uint8_t SlopeSides(const Lot& lot) {
    if (lot.streetSides != 0) {
        return lot.streetSides | OppositeSides(lot.streetSides);
    }
    const glm::vec2 size = lot.area.Size();
    return size.x >= size.y ? static_cast<uint8_t>(kSideSouth | kSideNorth) : static_cast<uint8_t>(kSideWest | kSideEast);
}

void AddBalconies(BoxWriter& boxes, const Lot& lot, const BuildingDesign& design, const glm::vec3& stoneColor) {
    const Look slab{stoneColor, SurfaceMaterial::Plain, 0.75f, 0.0f};
    const Look railing{{0.05f, 0.05f, 0.055f}, SurfaceMaterial::Metal, 0.45f, PatternSeed(kMetalRailing, 0.5f)};
    // A continuous balcony on the first upper floor and, on taller houses, one under the cornice.
    std::vector<float> levels{kGroundFloorHeight + kUpperFloorHeight};
    if (design.upperFloors >= 4) {
        levels.push_back(kGroundFloorHeight + kUpperFloorHeight * static_cast<float>(design.upperFloors - 1));
    }
    for (Side side : kSides) {
        if ((lot.streetSides & side) == 0) {
            continue;
        }
        for (float level : levels) {
            boxes.Footprint(EdgeBand(lot.area, side, -0.1f, 0.9f, 0.9f), level - 0.18f, level, slab);
            boxes.Footprint(EdgeBand(lot.area, side, 0.84f, 0.9f, 0.95f), level, level + 1.0f, railing);
        }
    }
}

void AddAwnings(BoxWriter& boxes, const Lot& lot, Random& random) {
    const Look cloth{random.Pick(kAwnings), SurfaceMaterial::Plain, 0.95f, 0.0f};
    for (Side side : kSides) {
        if ((lot.streetSides & side) == 0) {
            continue;
        }
        // Two layers make the canvas read as sloping down towards the street.
        boxes.Footprint(EdgeBand(lot.area, side, -0.05f, 1.5f, 0.6f), kOverhangClearance + 0.3f,
                        kOverhangClearance + 0.42f, cloth);
        boxes.Footprint(EdgeBand(lot.area, side, -0.05f, 0.8f, 0.6f), kOverhangClearance + 0.42f,
                        kOverhangClearance + 0.58f, cloth);
    }
}

// Builds the roof on `footprint` starting at `base` and returns the height of its top.
float AddRoof(BoxWriter& boxes, const Rect& footprint, uint8_t slopeSides, float base, const BuildingDesign& design,
              Random& random) {
    const Look tiles{design.roofColor, SurfaceMaterial::RoofTile, 0.7f, PatternSeed(0, random.Unit())};
    switch (design.roof) {
    case RoofShape::Mansard: {
        // A steep lower slope carrying the dormers, then a shallow top behind it.
        const Look dormers{design.roofColor, SurfaceMaterial::RoofTile, 0.65f, PatternSeed(kRoofDormers, random.Unit())};
        boxes.Footprint(ShrinkClamped(footprint, slopeSides, 0.6f), base, base + 2.8f, dormers);
        boxes.Footprint(ShrinkClamped(footprint, slopeSides, kMansardTopInset), base + 2.8f, base + 3.9f, tiles);
        return base + 3.9f;
    }
    case RoofShape::Pitched: {
        const glm::vec2 size = footprint.Size();
        const bool slopesAlongZ = (slopeSides & (kSideSouth | kSideNorth)) != 0;
        const float depth = slopesAlongZ ? size.y : size.x;
        const float stepHeight = std::clamp(depth * 0.09f, 0.8f, 1.4f);
        const float fractions[] = {0.10f, 0.22f, 0.34f};
        float top = base;
        for (float fraction : fractions) {
            boxes.Footprint(ShrinkClamped(footprint, slopeSides, depth * fraction), top, top + stepHeight, tiles);
            top += stepHeight;
        }
        return top;
    }
    case RoofShape::Flat:
    default:
        // A lead-covered flat roof behind a low parapet, with a stair house on top.
        boxes.Footprint(footprint.Expanded(-0.3f), base, base + 0.6f, tiles);
        boxes.Footprint({footprint.Center() - glm::vec2(1.2f), footprint.Center() + glm::vec2(1.2f)}, base + 0.6f,
                        base + 2.8f, Look{design.wallColor * 0.9f, SurfaceMaterial::Plain, 0.85f, 0.0f});
        return base + 0.6f;
    }
}

// A small tower turned a quarter turn to the street, standing in the middle of a mansard roof,
// with a stepped copper spire. `top` is the flat top of the roof. Nothing is built when the flat
// part is too small for the tower to stand clear of the slopes.
void AddBelvedere(BoxWriter& boxes, const Rect& top, float base, const BuildingDesign& design) {
    constexpr float kHalfSide = 1.2f;
    constexpr float kClearance = 0.3f;
    constexpr float kQuarterTurn = 0.78539816f;
    const float halfDiagonal = kHalfSide * 1.41421356f;
    const glm::vec2 size = top.Size();
    if (std::min(size.x, size.y) < 2.0f * (halfDiagonal + kClearance)) {
        return;
    }
    const glm::vec2 middle = top.Center();
    const uint32_t flags = kFacadeUpperOnly | kFacadeNoShops | kFacadeNarrowBays | kFacadeArched;
    const Look walls{design.wallColor, SurfaceMaterial::Facade, 0.85f, PatternSeed(flags, 0.5f)};
    const Look trim{design.trimColor, SurfaceMaterial::Plain, 0.8f, 0.0f};
    const Look copper{{0.24f, 0.52f, 0.45f}, SurfaceMaterial::Metal, 0.55f, PatternSeed(kMetalVerdigris, 0.5f)};

    constexpr float kShaftHeight = kUpperFloorHeight;
    float height = base;
    boxes.TurnedBox(AtHeight(middle, height + kShaftHeight * 0.5f), {kHalfSide, kShaftHeight * 0.5f, kHalfSide}, kQuarterTurn, walls);
    height += kShaftHeight;
    boxes.TurnedBox(AtHeight(middle, height + 0.12f), {kHalfSide + 0.2f, 0.12f, kHalfSide + 0.2f}, kQuarterTurn, trim);
    height += 0.24f;
    // Each step is turned back square to the street and narrower than the last, so the spire
    // reads as eight sided from a distance.
    const float halfSides[] = {1.35f, 0.95f, 0.6f, 0.28f};
    for (float halfSide : halfSides) {
        constexpr float kStepHeight = 0.7f;
        boxes.TurnedBox(AtHeight(middle, height + kStepHeight * 0.5f), {halfSide, kStepHeight * 0.5f, halfSide}, 0.0f, copper);
        height += kStepHeight;
    }
    boxes.TurnedBox(AtHeight(middle, height + 0.5f), {0.06f, 0.5f, 0.06f}, 0.0f, copper);  // the finial
}

void AddChimneys(BoxWriter& boxes, const Lot& lot, uint8_t slopeSides, float base, float roofTop, Random& random) {
    const Look brick{kChimneyBrick * random.Range(0.85f, 1.1f), SurfaceMaterial::Plain, 0.9f, 0.0f};
    const glm::vec2 center = lot.area.Center();
    const glm::vec2 size = lot.area.Size();
    const bool partyWallsAlongZ = (slopeSides & (kSideSouth | kSideNorth)) != 0 && (slopeSides & kSideWest) == 0;
    const bool partyWallsAlongX = (slopeSides & (kSideWest | kSideEast)) != 0 && (slopeSides & kSideSouth) == 0;
    std::vector<glm::vec2> positions;
    if (partyWallsAlongZ && size.x > 2.0f) {
        // Stacks rise from the party walls, as fireplaces back onto them.
        positions.push_back({lot.area.min.x + 0.4f, center.y + random.Range(-0.15f, 0.15f) * size.y});
        if (random.Chance(0.5f)) {
            positions.push_back({lot.area.max.x - 0.4f, center.y + random.Range(-0.15f, 0.15f) * size.y});
        }
    } else if (partyWallsAlongX && size.y > 2.0f) {
        positions.push_back({center.x + random.Range(-0.15f, 0.15f) * size.x, lot.area.min.y + 0.4f});
    } else {
        positions.push_back(center + glm::vec2(random.Range(-0.2f, 0.2f), random.Range(-0.2f, 0.2f)) * size);
    }
    for (glm::vec2 position : positions) {
        const glm::vec2 half = partyWallsAlongZ ? glm::vec2(0.3f, 0.7f) : glm::vec2(0.7f, 0.3f);
        boxes.Footprint({position - half, position + half}, base + 0.4f, roofTop + random.Range(0.6f, 1.3f), brick);
    }
}

// The look shared by all buildings of one block, so grand blocks read as one composition.
struct BlockTheme {
    glm::vec3 wallColor;
    glm::vec3 roofColor;
    int upperFloors;
    bool balconies;
};

BlockTheme ChooseTheme(District district, Random& random) {
    const DistrictStyle& style = StyleOf(district);
    BlockTheme theme{random.Pick(kLimestone), random.Pick(kSlate),
                     random.Int(style.minUpperFloors, style.maxUpperFloors), false};
    if (district == District::Grand) {
        theme.balconies = true;
    } else if (district == District::Residential) {
        theme.balconies = random.Chance(0.45f);
    }
    return theme;
}

BuildingDesign DesignBuilding(District district, const BlockTheme& theme, const Lot& lot, Random& random) {
    const DistrictStyle& style = StyleOf(district);
    BuildingDesign design;
    design.trimColor = glm::vec3(0.86f, 0.82f, 0.74f);
    design.corniceDepth = random.Range(0.25f, 0.5f);
    const bool facesStreet = lot.streetSides != 0;
    const bool corner = lot.streetSides != 0 && (lot.streetSides & (lot.streetSides - 1)) != 0;
    switch (district) {
    case District::Grand:
        // Uniform heights and stone along the whole block, with a taller pavilion on the corners.
        design.wallColor = theme.wallColor * random.Range(0.97f, 1.03f);
        design.trimColor = theme.wallColor * 1.08f;
        design.roofColor = theme.roofColor;
        design.upperFloors = theme.upperFloors + (corner ? 1 : 0);
        design.facadeFlags = kFacadeWideBays | kFacadeRusticated;
        design.roof = RoofShape::Mansard;
        design.balconies = theme.balconies;
        design.corniceDepth = 0.5f;
        break;
    case District::OldQuarter:
        design.wallColor = random.Pick(kPlaster) * random.Range(0.92f, 1.04f);
        design.upperFloors = random.Int(style.minUpperFloors, style.maxUpperFloors);
        design.facadeFlags = (random.Chance(0.6f) ? kFacadeNarrowBays : 0u) | (random.Chance(0.5f) ? kFacadeShutters : 0u) |
                             (random.Chance(0.3f) ? kFacadeNoShops : 0u);
        design.roof = random.Chance(0.7f) ? RoofShape::Pitched : RoofShape::Mansard;
        design.roofColor = design.roof == RoofShape::Pitched ? random.Pick(kClay) : random.Pick(kSlate);
        design.awnings = random.Chance(0.25f);
        design.attic = design.roof == RoofShape::Pitched && random.Chance(0.12f);
        break;
    case District::Market:
        design.wallColor = random.Chance(0.55f) ? random.Pick(kBrick) : random.Pick(kPlaster);
        design.upperFloors = random.Int(style.minUpperFloors, style.maxUpperFloors);
        design.facadeFlags = random.Chance(0.4f) ? kFacadeShutters : 0u;
        design.roof = random.Chance(0.75f) ? RoofShape::Pitched : RoofShape::Flat;
        design.roofColor = random.Pick(kClay);
        design.awnings = random.Chance(0.7f);
        break;
    case District::Residential:
    case District::Parkland:
    default:
        design.wallColor = random.Chance(0.6f) ? theme.wallColor * random.Range(0.95f, 1.05f) : random.Pick(kPlaster);
        design.upperFloors = std::clamp(theme.upperFloors + random.Int(-1, 1), style.minUpperFloors, style.maxUpperFloors);
        design.facadeFlags = (random.Chance(0.6f) ? kFacadeNoShops : 0u) | (random.Chance(0.3f) ? kFacadeShutters : 0u);
        design.roof = random.Chance(0.8f) ? RoofShape::Mansard : RoofShape::Pitched;
        design.roofColor = design.roof == RoofShape::Mansard ? theme.roofColor : random.Pick(kClay);
        design.balconies = theme.balconies;
        design.attic = random.Chance(0.15f);
        break;
    }
    design.facadeFlags |= ArchitectureFlags(district, lot);
    design.belvedere = design.roof == RoofShape::Mansard && WantsBelvedere(district, lot);
    if (!facesStreet) {
        // Back buildings in courtyards have no shops and no balconies.
        design.facadeFlags |= kFacadeNoShops;
        design.balconies = false;
        design.awnings = false;
    }
    if (design.facadeFlags & kFacadeNoShops) {
        design.awnings = false;
    }
    return design;
}

// Cuts a row of buildings into lots of the district's widths.
void CutRow(const Rect& row, bool alongX, const DistrictStyle& style, Random& random, std::vector<Lot>& lots) {
    const float start = AlongMin(row, alongX);
    const float end = AlongMax(row, alongX);
    float position = start;
    while (position < end - 0.01f) {
        float width = random.Range(style.minLotWidth, style.maxLotWidth);
        if (end - (position + width) < style.minLotWidth * 0.6f) {
            width = end - position;  // the last lot takes up the remainder
        }
        lots.push_back({FromRanges(alongX, position, position + width, AcrossMin(row, alongX), AcrossMax(row, alongX)), 0});
        position += width;
    }
}

uint8_t SidesOnBoundary(const Rect& lot, const Rect& land) {
    constexpr float kTolerance = 0.01f;
    uint8_t sides = 0;
    if (lot.min.x <= land.min.x + kTolerance) sides |= kSideWest;
    if (lot.max.x >= land.max.x - kTolerance) sides |= kSideEast;
    if (lot.min.y <= land.min.y + kTolerance) sides |= kSideSouth;
    if (lot.max.y >= land.max.y - kTolerance) sides |= kSideNorth;
    return sides;
}

// Lays out the lots of a perimeter block around a courtyard and returns the open courtyard areas.
std::vector<Rect> LayOutPerimeterBlock(const Rect& land, float depth, const DistrictStyle& style, Random& random,
                                       std::vector<Lot>& lots) {
    const Rect south{land.min, {land.max.x, land.min.y + depth}};
    const Rect north{{land.min.x, land.max.y - depth}, land.max};
    const Rect west{{land.min.x, south.max.y}, {land.min.x + depth, north.min.y}};
    const Rect east{{land.max.x - depth, south.max.y}, {land.max.x, north.min.y}};
    CutRow(south, true, style, random, lots);
    CutRow(north, true, style, random, lots);
    CutRow(west, false, style, random, lots);
    CutRow(east, false, style, random, lots);

    const Rect courtyard = land.Expanded(-depth);
    std::vector<Rect> courtyards{courtyard};
    // A long courtyard gets a cross wing through its middle.
    const glm::vec2 size = courtyard.Size();
    const bool alongX = size.x >= size.y;
    const float length = alongX ? size.x : size.y;
    if (length > 70.0f) {
        const float middle = (AlongMin(courtyard, alongX) + AlongMax(courtyard, alongX)) * 0.5f;
        const float halfWidth = depth * 0.45f;
        const Rect wing = FromRanges(alongX, middle - halfWidth, middle + halfWidth, AcrossMin(courtyard, alongX),
                                     AcrossMax(courtyard, alongX));
        CutRow(wing, !alongX, style, random, lots);
        courtyards = Subtract(courtyard, {wing});
    }
    return courtyards;
}

// Adds low workshops and back buildings along the edges of a large courtyard, as old city blocks
// fill up over time. Each annex keeps clear of the courtyard corners, so annexes on neighbouring
// edges never overlap.
std::vector<Rect> PlanCourtyardAnnexes(const Rect& courtyard, Random& random) {
    constexpr float kMaxAnnexDepth = 8.0f;
    constexpr float kCornerMargin = kMaxAnnexDepth + 0.5f;
    std::vector<Rect> annexes;
    const glm::vec2 size = courtyard.Size();
    if (std::min(size.x, size.y) < 20.0f) {
        return annexes;
    }
    const float maxDepth = std::min(kMaxAnnexDepth, std::min(size.x, size.y) * 0.3f);
    for (Side side : {kSideWest, kSideEast, kSideSouth, kSideNorth}) {
        const bool alongX = side == kSideSouth || side == kSideNorth;
        const float edgeStart = AlongMin(courtyard, alongX) + kCornerMargin;
        const float edgeEnd = AlongMax(courtyard, alongX) - kCornerMargin;
        if (edgeEnd - edgeStart < 6.0f || !random.Chance(0.55f)) {
            continue;
        }
        const float length = std::max(6.0f, (edgeEnd - edgeStart) * random.Range(0.3f, 0.8f));
        const float start = random.Range(edgeStart, edgeEnd - length);
        const float depth = random.Range(5.0f, maxDepth);
        const float acrossLow = AcrossMin(courtyard, alongX);
        const float acrossHigh = AcrossMax(courtyard, alongX);
        const bool atLowEdge = side == kSideWest || side == kSideSouth;
        const Rect annex = atLowEdge ? FromRanges(alongX, start, start + length, acrossLow, acrossLow + depth)
                                     : FromRanges(alongX, start, start + length, acrossHigh - depth, acrossHigh);
        // Reach back into the houses, whose rear walls may stand a little behind the courtyard edge.
        annexes.push_back(ExpandSides(annex, side, kRearJitter));
    }
    return annexes;
}

// Lays out a block too small for a courtyard: two rows of houses back to back, or one row.
void LayOutRowBlock(const Rect& land, const DistrictStyle& style, Random& random, std::vector<Lot>& lots) {
    const glm::vec2 size = land.Size();
    const bool alongX = size.x >= size.y;
    const float across = alongX ? size.y : size.x;
    const float low = AcrossMin(land, alongX);
    const float high = AcrossMax(land, alongX);
    if (across < 12.0f) {
        CutRow(land, alongX, style, random, lots);
        return;
    }
    const float middle = low + across * random.Range(0.45f, 0.55f);
    CutRow(FromRanges(alongX, AlongMin(land, alongX), AlongMax(land, alongX), low, middle), alongX, style, random, lots);
    CutRow(FromRanges(alongX, AlongMin(land, alongX), AlongMax(land, alongX), middle, high), alongX, style, random, lots);
}

// A courtyard annex is a plain one or two storey back building without shops or balconies.
void AddCourtyardAnnex(BoxWriter& boxes, const Rect& area, District district, const BlockTheme& theme,
                       Random& random) {
    const Lot lot{area, 0};
    BuildingDesign design = DesignBuilding(district, theme, lot, random);
    design.upperFloors = random.Int(0, 1);
    design.roof = random.Chance(0.5f) ? RoofShape::Flat : RoofShape::Pitched;
    design.facadeFlags |= kFacadeNoShops;
    design.attic = false;
    design.balconies = false;
    design.awnings = false;
    AddBuilding(boxes, lot, design, random);
}

}  // namespace

float WallHeight(int upperFloors) {
    return kGroundFloorHeight + kUpperFloorHeight * static_cast<float>(upperFloors);
}

void AddBuilding(BoxWriter& boxes, const Lot& lot, const BuildingDesign& design, Random& random) {
    const float wallTop = WallHeight(design.upperFloors);
    const Look wall{design.wallColor, SurfaceMaterial::Facade, 0.85f, PatternSeed(design.facadeFlags, random.Unit())};
    // The walls start below the pavement so no gap shows at their foot.
    boxes.Footprint(lot.area, 0.0f, wallTop, wall);

    const Look trim{design.trimColor, SurfaceMaterial::Plain, 0.8f, 0.0f};
    if (lot.streetSides != 0) {
        boxes.Footprint(ProjectingSlab(lot, random.Range(0.12f, 0.2f)), kLedgeBottom, kLedgeBottom + 0.25f, trim);
        boxes.Footprint(ProjectingSlab(lot, design.corniceDepth), wallTop - 0.4f, wallTop + 0.1f, trim);
    } else {
        boxes.Footprint(lot.area.Expanded(0.15f), wallTop - 0.3f, wallTop + 0.1f, trim);
    }
    if (design.balconies) {
        AddBalconies(boxes, lot, design, design.trimColor);
    }
    if (design.awnings) {
        AddAwnings(boxes, lot, random);
    }

    Rect roofFootprint = lot.area;
    float roofBase = wallTop;
    if (design.attic) {
        // A set-back attic storey behind the cornice.
        roofFootprint = ShrinkClamped(lot.area, lot.streetSides, 1.5f);
        const uint32_t atticFlags = kFacadeUpperOnly | kFacadeNoShops | (design.facadeFlags & kFacadeNarrowBays);
        boxes.Footprint(roofFootprint, wallTop, wallTop + kAtticHeight,
                        Look{design.wallColor * 0.95f, SurfaceMaterial::Facade, 0.85f, PatternSeed(atticFlags, random.Unit())});
        roofBase = wallTop + kAtticHeight;
    }
    const uint8_t slopeSides = SlopeSides(lot);
    const float roofTop = AddRoof(boxes, roofFootprint, slopeSides, roofBase, design, random);
    AddChimneys(boxes, lot, slopeSides, roofBase, roofTop, random);
    if (design.belvedere) {
        AddBelvedere(boxes, ShrinkClamped(roofFootprint, slopeSides, kMansardTopInset), roofTop, design);
    }
}

void BuildBlockOfBuildings(BoxWriter& boxes, CityLayout& city, const Rect& land, District district, Random& random) {
    PartScope scope(boxes, CityPart::Building);
    const DistrictStyle& style = StyleOf(district);
    const BlockTheme theme = ChooseTheme(district, random);
    const float depth = random.Range(style.minDepth, style.maxDepth);
    const glm::vec2 size = land.Size();

    std::vector<Lot> lots;
    std::vector<Rect> annexes;
    const bool hasCourtyard = std::min(size.x, size.y) >= depth * 2.0f + kMinCourtyard;
    if (hasCourtyard) {
        for (const Rect& courtyard : LayOutPerimeterBlock(land, depth, style, random, lots)) {
            const std::vector<Rect> courtyardAnnexes = PlanCourtyardAnnexes(courtyard, random);
            PlantCourtyard(boxes, city, courtyard, courtyardAnnexes, random);
            annexes.insert(annexes.end(), courtyardAnnexes.begin(), courtyardAnnexes.end());
        }
    } else {
        LayOutRowBlock(land, style, random, lots);
    }

    for (Lot& lot : lots) {
        lot.streetSides = SidesOnBoundary(lot.area, land);
        // Houses with a single street front vary in depth at the back, breaking up the courtyard.
        const bool singleFront = lot.streetSides != 0 && (lot.streetSides & (lot.streetSides - 1)) == 0;
        if (singleFront && hasCourtyard) {
            lot.area = ShrinkClamped(lot.area, OppositeSides(lot.streetSides), random.Range(0.0f, kRearJitter));
        }
        AddBuilding(boxes, lot, DesignBuilding(district, theme, lot, random), random);
    }
    for (const Rect& annex : annexes) {
        AddCourtyardAnnex(boxes, annex, district, theme, random);
    }
}

}  // namespace city
