#include "world/CityLandmarks.h"

#include <algorithm>
#include <cmath>

#include "world/CityBuildings.h"
#include "world/CityFrame.h"
#include "world/CityInteriors.h"
#include "world/CityNature.h"

namespace city {
namespace {

const glm::vec3 kChurchStone{0.72f, 0.68f, 0.60f};
const glm::vec3 kPalaceStone{0.82f, 0.77f, 0.67f};
const glm::vec3 kTrimStone{0.86f, 0.82f, 0.74f};
const glm::vec3 kLead{0.30f, 0.32f, 0.34f};
const glm::vec3 kCopperPatina{0.30f, 0.48f, 0.40f};
const glm::vec3 kZinc{0.42f, 0.44f, 0.46f};
const glm::vec3 kSlateRoof{0.20f, 0.22f, 0.26f};
const glm::vec3 kDoorWood{0.22f, 0.13f, 0.08f};
const glm::vec3 kStainedGlass{0.20f, 0.18f, 0.30f};
const glm::vec3 kClearGlass{0.10f, 0.12f, 0.13f};
const glm::vec3 kDarkOpening{0.03f, 0.03f, 0.035f};
const glm::vec3 kClockFace{0.90f, 0.86f, 0.74f};
const glm::vec3 kIron{0.05f, 0.05f, 0.055f};

// Picks a front at random among the two long sides, or the two short sides, of `land`.
Side ChooseFront(const Rect& land, bool onLongSide, Random& random) {
    const glm::vec2 size = land.Size();
    const bool longAlongX = size.x >= size.y;
    // The long sides of a block that is long along X are its south and north sides.
    const bool facesSouthOrNorth = longAlongX == onLongSide;
    if (facesSouthOrNorth) {
        return random.Chance(0.5f) ? kSideSouth : kSideNorth;
    }
    return random.Chance(0.5f) ? kSideWest : kSideEast;
}

void Block(BoxWriter& boxes, const LocalFrame& frame, float u0, float u1, float v0, float v1, float bottom, float top,
           const Look& look) {
    boxes.Footprint(frame.Area(u0, u1, v0, v1), bottom, top, look);
}

void SquareBox(BoxWriter& boxes, glm::vec2 center, float halfSize, float bottom, float top, const Look& look) {
    boxes.Footprint({center - glm::vec2(halfSize), center + glm::vec2(halfSize)}, bottom, top, look);
}

// A pitched roof over a frame rectangle, approximated by boxes that narrow towards the ridge.
// The ridge runs along u when `ridgeAlongU`, otherwise along v.
void SteppedRoof(BoxWriter& boxes, const LocalFrame& frame, float u0, float u1, float v0, float v1, float base,
                 float height, int steps, bool ridgeAlongU, const Look& look) {
    const float uLow = std::min(u0, u1);
    const float uHigh = std::max(u0, u1);
    const float vLow = std::min(v0, v1);
    const float vHigh = std::max(v0, v1);
    const float halfSpan = 0.5f * (ridgeAlongU ? vHigh - vLow : uHigh - uLow);
    const float stepHeight = height / static_cast<float>(steps);
    for (int i = 0; i < steps; ++i) {
        const float inset = halfSpan * (0.06f + 0.86f * static_cast<float>(i) / static_cast<float>(steps));
        const float bottom = base + stepHeight * static_cast<float>(i);
        if (ridgeAlongU) {
            Block(boxes, frame, uLow, uHigh, vLow + inset, vHigh - inset, bottom, bottom + stepHeight, look);
        } else {
            Block(boxes, frame, uLow + inset, uHigh - inset, vLow, vHigh, bottom, bottom + stepHeight, look);
        }
    }
}

// A square spire or pyramid roof built from shrinking steps, with a thin finial on top.
void SteppedSpire(BoxWriter& boxes, glm::vec2 center, float halfBase, float base, float height, int steps,
                  const Look& look) {
    const float stepHeight = height / static_cast<float>(steps);
    for (int i = 0; i < steps; ++i) {
        const float half = halfBase * (1.0f - 0.85f * static_cast<float>(i) / static_cast<float>(steps));
        const float bottom = base + stepHeight * static_cast<float>(i);
        SquareBox(boxes, center, half, bottom, bottom + stepHeight, look);
    }
    const float top = base + height;
    SquareBox(boxes, center, std::max(0.15f, halfBase * 0.06f), top, top + height * 0.25f, look);
}

// Dark openings on the four faces of a square tower stage, such as belfry louvres or windows.
void TowerOpenings(BoxWriter& boxes, glm::vec2 center, float halfSize, float halfOpening, float bottom, float top,
                   const Look& look) {
    const float outer = halfSize + 0.08f;
    const float inner = halfSize - 0.05f;
    boxes.Footprint({center + glm::vec2(-halfOpening, inner), center + glm::vec2(halfOpening, outer)}, bottom, top, look);
    boxes.Footprint({center + glm::vec2(-halfOpening, -outer), center + glm::vec2(halfOpening, -inner)}, bottom, top, look);
    boxes.Footprint({center + glm::vec2(inner, -halfOpening), center + glm::vec2(outer, halfOpening)}, bottom, top, look);
    boxes.Footprint({center + glm::vec2(-outer, -halfOpening), center + glm::vec2(-inner, halfOpening)}, bottom, top, look);
}

void RecordLandmark(CityLayout& city, LandmarkKind kind, const Rect& footprint, const LocalFrame& frame, float entranceU) {
    const glm::vec2 entrance = frame.Point(entranceU, 0.0f);
    city.landmarks.push_back({kind, footprint, entrance});
    city.pointsOfInterest.push_back({AtHeight(entrance, kPavementHeight), -frame.forward, PointOfInterestKind::LandmarkEntrance});
}

Look StainedGlass(Random& random) {
    return {kStainedGlass, SurfaceMaterial::Glass, 0.1f, PatternSeed(kGlassStained, random.Unit())};
}

// A paved forecourt from u0 to u1 across the whole frame, exported as a square, with benches
// along its sides.
void BuildForecourt(BoxWriter& boxes, CityLayout& city, const LocalFrame& frame, float u0, float u1, Random& random) {
    const float halfWidth = frame.width * 0.5f;
    {
        PartScope scope(boxes, CityPart::Plaza);
        Block(boxes, frame, u0, u1, -halfWidth, halfWidth, 0.1f, kPlazaHeight, CobbleLook(random.Unit()));
    }
    city.plazas.push_back(frame.Area(u0 + 3.0f, u1 - 1.5f, -halfWidth + 3.0f, halfWidth - 3.0f));
    for (float side : {-1.0f, 1.0f}) {
        for (float t : {0.3f, 0.7f}) {
            AddBench(boxes, city, frame.Point(u0 + (u1 - u0) * t, side * (halfWidth - 1.8f)), kPlazaHeight,
                     frame.right * -side);
        }
    }
}

void BuildCathedral(BoxWriter& boxes, CityLayout& city, const Rect& land, Random& random) {
    const LocalFrame frame = FrameFacing(land, ChooseFront(land, false, random));
    const Look stone = StoneLook(kChurchStone, random.Unit());
    const Look trim{kTrimStone, SurfaceMaterial::Plain, 0.75f, 0.0f};
    const Look roof{kLead, SurfaceMaterial::RoofTile, 0.5f, PatternSeed(0, random.Unit())};
    const Look glass = StainedGlass(random);
    const Look door{kDoorWood, SurfaceMaterial::Plain, 0.7f, 0.0f};
    const Look louvres{kDarkOpening, SurfaceMaterial::Plain, 0.9f, 0.0f};
    const Look lead{kLead, SurfaceMaterial::Metal, 0.45f, 0.0f};

    const float halfWidth = frame.width * 0.5f;
    const float parvis = std::clamp(frame.length * 0.22f, 24.0f, 40.0f);
    const float front = parvis + 2.0f;           // the west front, behind a porch
    const float towerDepth = 14.0f;
    const float naveStart = front + towerDepth;
    const float naveEnd = frame.length - 16.0f;  // the apse fills the rest
    const float crossing = naveStart + (naveEnd - naveStart) * 0.62f;
    const float transeptHalf = std::min(26.0f, halfWidth - 4.0f);
    constexpr float kNaveHalf = 7.5f;
    constexpr float kAisleHalf = 15.0f;
    constexpr float kNaveHeight = 30.0f;
    constexpr float kAisleHeight = 14.0f;

    BuildForecourt(boxes, city, frame, 0.0f, front, random);
    PartScope scope(boxes, CityPart::Landmark);

    // The west front: two towers flanking a gabled centre with a rose window and a deep porch.
    for (float side : {-1.0f, 1.0f}) {
        const glm::vec2 tower = frame.Point(front + towerDepth * 0.5f, side * 10.0f);
        Block(boxes, frame, front, naveStart, side * 4.0f, side * 16.0f, 0.0f, 40.0f, stone);
        Block(boxes, frame, front + 0.8f, naveStart - 0.8f, side * 4.8f, side * 15.2f, 40.0f, 54.0f, stone);
        TowerOpenings(boxes, tower, 5.2f, 2.6f, 42.0f, 52.0f, louvres);
        Block(boxes, frame, front - 0.3f, naveStart + 0.3f, side * 3.7f, side * 16.3f, 54.0f, 55.4f, trim);
        for (float du : {-5.6f, 5.6f}) {
            for (float dv : {-5.6f, 5.6f}) {
                SquareBox(boxes, tower + frame.forward * du + frame.right * dv, 0.6f, 55.4f, 59.5f, stone);
            }
        }
        SteppedSpire(boxes, tower, 4.4f, 55.4f, 15.0f, 4, lead);
        Block(boxes, frame, front - 0.06f, front + 0.1f, side * 8.0f, side * 12.0f, 0.15f, 6.0f, door);
    }
    Block(boxes, frame, front, naveStart, -4.0f, 4.0f, 0.0f, 33.0f, stone);
    Block(boxes, frame, front, front + 2.0f, -3.0f, 3.0f, 33.0f, 36.0f, stone);
    Block(boxes, frame, front, front + 2.0f, -1.5f, 1.5f, 36.0f, 38.5f, stone);
    Block(boxes, frame, front - 0.08f, front + 0.1f, -3.2f, 3.2f, 20.0f, 26.4f, glass);  // the rose window
    Block(boxes, frame, front - 0.4f, front + 0.4f, -16.0f, 16.0f, 14.0f, 15.0f, trim);  // the gallery
    Block(boxes, frame, front - 1.2f, front, -3.4f, 3.4f, 0.0f, 9.6f, stone);           // the porch
    Block(boxes, frame, front - 1.2f, front, -2.4f, 2.4f, 9.6f, 11.4f, stone);
    Block(boxes, frame, front - 1.28f, front - 1.1f, -2.2f, 2.2f, 0.15f, 8.0f, door);

    // Nave, aisles and transept, each under a stepped lead roof.
    Block(boxes, frame, naveStart, naveEnd, -kNaveHalf, kNaveHalf, 0.0f, kNaveHeight, stone);
    SteppedRoof(boxes, frame, naveStart, naveEnd, -kNaveHalf, kNaveHalf, kNaveHeight, 9.0f, 4, true, roof);
    for (float side : {-1.0f, 1.0f}) {
        Block(boxes, frame, naveStart, naveEnd, side * kNaveHalf, side * kAisleHalf, 0.0f, kAisleHeight, stone);
        SteppedRoof(boxes, frame, naveStart, naveEnd, side * kNaveHalf, side * (kAisleHalf - 1.0f), kAisleHeight, 2.4f,
                    2, true, roof);
        Block(boxes, frame, crossing - 3.2f, crossing + 3.2f, side * (transeptHalf - 0.1f), side * (transeptHalf + 0.08f),
              15.0f, 24.0f, glass);
        Block(boxes, frame, crossing - 2.0f, crossing + 2.0f, side * (transeptHalf - 0.1f), side * (transeptHalf + 0.1f),
              0.15f, 6.0f, door);
    }
    Block(boxes, frame, crossing - 8.0f, crossing + 8.0f, -transeptHalf, transeptHalf, 0.0f, kNaveHeight, stone);
    SteppedRoof(boxes, frame, crossing - 8.0f, crossing + 8.0f, -transeptHalf, transeptHalf, kNaveHeight, 9.0f, 4, false,
                roof);

    // Bays of tall windows, with buttresses and flying buttresses between them.
    constexpr float kBay = 6.0f;
    for (float u = naveStart + kBay * 0.5f; u + kBay * 0.5f <= naveEnd; u += kBay) {
        if (std::abs(u - crossing) < 11.0f) {
            continue;  // the transept covers these bays
        }
        const float buttress = u + kBay * 0.5f;
        const bool hasButtress = buttress < naveEnd - 1.0f && std::abs(buttress - crossing) >= 11.0f;
        for (float side : {-1.0f, 1.0f}) {
            Block(boxes, frame, u - 1.3f, u + 1.3f, side * (kNaveHalf - 0.08f), side * (kNaveHalf + 0.08f), 17.5f, 27.0f, glass);
            Block(boxes, frame, u - 1.2f, u + 1.2f, side * (kAisleHalf - 0.08f), side * (kAisleHalf + 0.08f), 3.5f, 11.0f, glass);
            if (hasButtress) {
                Block(boxes, frame, buttress - 0.8f, buttress + 0.8f, side * kAisleHalf, side * 17.6f, 0.0f, 15.0f, stone);
                Block(boxes, frame, buttress - 0.5f, buttress + 0.5f, side * 16.0f, side * 17.0f, 15.0f, 18.5f, stone);
                Block(boxes, frame, buttress - 0.45f, buttress + 0.45f, side * kNaveHalf, side * 16.4f, 20.5f, 22.0f, stone);
            }
        }
    }

    // A lantern tower and spire over the crossing.
    const glm::vec2 crossingCenter = frame.Point(crossing, 0.0f);
    SquareBox(boxes, crossingCenter, 7.6f, kNaveHeight, 45.0f, stone);
    TowerOpenings(boxes, crossingCenter, 7.6f, 3.0f, 34.0f, 42.0f, Look{kClearGlass, SurfaceMaterial::Glass, 0.1f, glass.seed});
    SquareBox(boxes, crossingCenter, 8.0f, 45.0f, 46.0f, trim);
    SteppedSpire(boxes, crossingCenter, 6.0f, 46.0f, 32.0f, 6, lead);

    // A stepped apse with chapels around it.
    Block(boxes, frame, naveEnd, naveEnd + 6.0f, -6.5f, 6.5f, 0.0f, 27.0f, stone);
    SteppedRoof(boxes, frame, naveEnd, naveEnd + 6.0f, -6.5f, 6.5f, 27.0f, 6.0f, 3, true, roof);
    Block(boxes, frame, naveEnd + 6.0f, naveEnd + 10.0f, -4.5f, 4.5f, 0.0f, 23.0f, stone);
    SteppedRoof(boxes, frame, naveEnd + 6.0f, naveEnd + 10.0f, -4.5f, 4.5f, 23.0f, 4.5f, 2, true, roof);
    Block(boxes, frame, naveEnd + 10.0f, naveEnd + 12.0f, -2.5f, 2.5f, 0.0f, 19.0f, stone);
    Block(boxes, frame, naveEnd + 12.0f, naveEnd + 12.08f, -1.2f, 1.2f, 6.0f, 15.0f, glass);
    for (float side : {-1.0f, 1.0f}) {
        Block(boxes, frame, naveEnd, naveEnd + 8.0f, side * 6.5f, side * 12.0f, 0.0f, 11.0f, stone);
        SteppedRoof(boxes, frame, naveEnd, naveEnd + 8.0f, side * 6.5f, side * 12.0f, 11.0f, 2.5f, 2, false, roof);
        Block(boxes, frame, naveEnd + 1.0f, naveEnd + 5.0f, side * (6.42f), side * (6.58f), 14.0f, 24.0f, glass);
    }
    const float outerHalf = std::max(17.6f, transeptHalf);
    RecordLandmark(city, LandmarkKind::Cathedral, frame.Area(front - 1.3f, naveEnd + 12.1f, -outerHalf, outerHalf), frame,
                   front - 1.3f);

    // Lawns and trees in the close beside the church.
    if (halfWidth - 1.0f > 19.5f + 4.0f) {
        PartScope lawn(boxes, CityPart::Park);
        for (float side : {-1.0f, 1.0f}) {
            Block(boxes, frame, naveStart + 1.0f, frame.length - 1.0f, side * 19.5f, side * (halfWidth - 1.0f), 0.1f,
                  kLawnHeight, GrassLook(random.Unit()));
            for (float u = naveStart + 6.0f; u < frame.length - 5.0f; u += 12.0f) {
                if (std::abs(u - crossing) > 13.0f || transeptHalf + 4.5f < halfWidth - 4.0f) {
                    AddTree(boxes, city, frame.Point(u, side * (halfWidth - 4.0f)), kLawnHeight, random.Range(8.0f, 12.0f),
                            random);
                }
            }
        }
    }
}

void BuildPalace(BoxWriter& boxes, CityLayout& city, const Rect& land, Random& random) {
    const LocalFrame frame = FrameFacing(land, ChooseFront(land, true, random));
    const float halfWidth = frame.width * 0.5f;
    constexpr float kMargin = 4.0f;
    constexpr float kWingWidth = 16.0f;
    constexpr float kBodyDepth = 18.0f;
    constexpr float kPavilionHalf = 14.0f;
    const float court = std::clamp(frame.length * 0.4f, 40.0f, 70.0f);  // depth of the cour d'honneur
    const float courtHalf = halfWidth - kMargin - kWingWidth;

    // The cour d'honneur, closed by railings with a gate, with a fountain in the middle.
    {
        PartScope scope(boxes, CityPart::Plaza);
        Block(boxes, frame, 0.0f, court, -courtHalf, courtHalf, 0.1f, kPlazaHeight, CobbleLook(random.Unit()));
    }
    const Rect courtSquare = frame.Area(kMargin, court - 7.0f, -courtHalf + 3.0f, courtHalf - 3.0f);
    city.plazas.push_back(courtSquare);
    AddFountain(boxes, city, courtSquare.Center(), kPlazaHeight, 4.0f, random);

    PartScope scope(boxes, CityPart::Landmark);
    const Look stone = StoneLook(kPalaceStone, random.Unit());
    const Look railing{kIron, SurfaceMaterial::Metal, 0.4f, PatternSeed(kMetalRailing, 0.5f)};
    for (float side : {-1.0f, 1.0f}) {
        Block(boxes, frame, 1.0f, 1.3f, side * 5.0f, side * courtHalf, 0.1f, 2.6f, railing);
        SquareBox(boxes, frame.Point(1.15f, side * 5.0f), 0.6f, 0.0f, 3.4f, stone);
        SquareBox(boxes, frame.Point(1.15f, side * courtHalf), 0.6f, 0.0f, 3.4f, stone);
    }

    // Wings, end pavilions and the main range, built like grand town houses but all in one stone.
    BuildingDesign design;
    design.wallColor = kPalaceStone;
    design.trimColor = kTrimStone;
    design.roofColor = kSlateRoof;
    design.facadeFlags = kFacadeWideBays | kFacadeRusticated | kFacadeNoShops;
    design.upperFloors = 3;
    design.roof = RoofShape::Mansard;
    design.corniceDepth = 0.5f;
    BuildingDesign pavilionDesign = design;
    pavilionDesign.upperFloors = 4;
    for (float side : {-1.0f, 1.0f}) {
        const float inner = side * (halfWidth - kMargin - kWingWidth);
        const float outer = side * (halfWidth - kMargin);
        AddBuilding(boxes, {frame.Area(kMargin, kMargin + 18.0f, inner, outer), kAllSides}, pavilionDesign, random);
        AddBuilding(boxes, {frame.Area(kMargin + 18.0f, court, inner, outer), kAllSides}, design, random);
        AddBuilding(boxes, {frame.Area(court, court + kBodyDepth, side * kPavilionHalf, outer), kAllSides}, design, random);
    }

    // The domed central pavilion with a columned portico facing the court.
    const float pavilionFront = court - 3.0f;
    const float pavilionBack = court + kBodyDepth + 3.0f;
    const float pavilionTop = WallHeight(4);
    const Look pavilionWalls{kPalaceStone, SurfaceMaterial::Facade, 0.85f, PatternSeed(design.facadeFlags, random.Unit())};
    const Look trim{kTrimStone, SurfaceMaterial::Plain, 0.75f, 0.0f};
    Block(boxes, frame, pavilionFront, pavilionBack, -kPavilionHalf, kPavilionHalf, 0.0f, pavilionTop, pavilionWalls);
    Block(boxes, frame, pavilionFront - 0.5f, pavilionBack + 0.5f, -kPavilionHalf - 0.5f, kPavilionHalf + 0.5f,
          pavilionTop - 0.4f, pavilionTop + 0.1f, trim);
    const glm::vec2 domeCenter = frame.Point((pavilionFront + pavilionBack) * 0.5f, 0.0f);
    const Look dome{kCopperPatina, SurfaceMaterial::Metal, 0.5f, 0.0f};
    SquareBox(boxes, domeCenter, 9.0f, pavilionTop + 0.1f, pavilionTop + 3.4f, stone);
    const float domeSteps[][2] = {{8.6f, 2.4f}, {7.2f, 2.2f}, {5.3f, 1.8f}, {3.0f, 1.2f}};
    float domeBase = pavilionTop + 3.4f;
    for (const auto& step : domeSteps) {
        SquareBox(boxes, domeCenter, step[0], domeBase, domeBase + step[1], dome);
        domeBase += step[1];
    }
    SquareBox(boxes, domeCenter, 1.4f, domeBase, domeBase + 2.2f, Look{kClearGlass, SurfaceMaterial::Glass, 0.1f, 0.5f});
    SquareBox(boxes, domeCenter, 1.7f, domeBase + 2.2f, domeBase + 2.6f, dome);
    SquareBox(boxes, domeCenter, 0.25f, domeBase + 2.6f, domeBase + 5.0f, dome);
    for (float v = -7.5f; v <= 7.51f; v += 3.0f) {
        SquareBox(boxes, frame.Point(pavilionFront - 1.6f, v), 0.55f, 0.0f, 11.0f, stone);
    }
    Block(boxes, frame, pavilionFront - 2.4f, pavilionFront, -8.3f, 8.3f, 11.0f, 12.6f, trim);
    Block(boxes, frame, pavilionFront - 2.4f, pavilionFront, -6.5f, 6.5f, 12.6f, 13.8f, stone);
    Block(boxes, frame, pavilionFront - 2.4f, pavilionFront, -4.0f, 4.0f, 13.8f, 14.8f, stone);
    Block(boxes, frame, pavilionFront - 2.4f, pavilionFront, -1.5f, 1.5f, 14.8f, 15.5f, stone);
    RecordLandmark(city, LandmarkKind::Palace, frame.Area(kMargin, pavilionBack, -(halfWidth - kMargin), halfWidth - kMargin),
                   frame, pavilionFront - 2.4f);

    // A formal garden behind the palace.
    const float gardenStart = pavilionBack + 5.0f;
    const Rect garden = frame.Area(gardenStart, frame.length - 2.0f, -(halfWidth - 2.0f), halfWidth - 2.0f);
    if (frame.length - 2.0f - gardenStart >= 30.0f) {
        BuildPark(boxes, city, garden, random);
    }
}

void BuildMarketHall(BoxWriter& boxes, CityLayout& city, const Rect& land, Random& random) {
    {
        PartScope scope(boxes, CityPart::Plaza);
        boxes.Footprint(land, 0.1f, kPlazaHeight, CobbleLook(random.Unit()));
    }
    const float border = std::min(13.0f, std::min(land.Size().x, land.Size().y) * 0.18f);
    const Rect hall = land.Expanded(-border);

    // The squares around the hall.
    const float gap = 2.5f;
    city.plazas.push_back(Rect{land.min + glm::vec2(2.0f), {land.max.x - 2.0f, hall.min.y - gap}});
    city.plazas.push_back(Rect{{land.min.x + 2.0f, hall.max.y + gap}, land.max - glm::vec2(2.0f)});
    city.plazas.push_back(Rect{{land.min.x + 2.0f, hall.min.y - gap}, {hall.min.x - gap, hall.max.y + gap}});
    city.plazas.push_back(Rect{{hall.max.x + gap, hall.min.y - gap}, {land.max.x - 2.0f, hall.max.y + gap}});

    PartScope scope(boxes, CityPart::Landmark);
    const Look stone = StoneLook(kChurchStone, random.Unit());
    const Look zinc{kZinc, SurfaceMaterial::Metal, 0.45f, 0.0f};
    const Look glass{kClearGlass, SurfaceMaterial::Glass, 0.08f, random.Unit()};
    constexpr float kEaves = 7.5f;

    // Stone piers around the edge carry the roof; the sides stay open.
    const glm::vec2 corners[] = {hall.min, {hall.max.x, hall.min.y}, {hall.min.x, hall.max.y}, hall.max};
    for (glm::vec2 corner : corners) {
        SquareBox(boxes, corner, 1.0f, 0.0f, kEaves, stone);
    }
    const glm::vec2 size = hall.Size();
    const int piersAlongX = std::max(2, static_cast<int>(std::lround(size.x / 6.0f)));
    const int piersAlongZ = std::max(2, static_cast<int>(std::lround(size.y / 6.0f)));
    for (int i = 1; i < piersAlongX; ++i) {
        const float x = hall.min.x + size.x * static_cast<float>(i) / static_cast<float>(piersAlongX);
        SquareBox(boxes, {x, hall.min.y}, 0.45f, 0.0f, kEaves, stone);
        SquareBox(boxes, {x, hall.max.y}, 0.45f, 0.0f, kEaves, stone);
    }
    for (int i = 1; i < piersAlongZ; ++i) {
        const float z = hall.min.y + size.y * static_cast<float>(i) / static_cast<float>(piersAlongZ);
        SquareBox(boxes, {hall.min.x, z}, 0.45f, 0.0f, kEaves, stone);
        SquareBox(boxes, {hall.max.x, z}, 0.45f, 0.0f, kEaves, stone);
    }

    // Roof: zinc eaves, a glazed clerestory, an upper roof and a glass lantern along the ridge.
    boxes.Footprint(hall.Expanded(1.2f), kEaves, kEaves + 0.6f, zinc);
    boxes.Footprint(hall.Expanded(-5.0f), kEaves + 0.6f, kEaves + 2.9f, glass);
    boxes.Footprint(hall.Expanded(-4.4f), kEaves + 2.9f, kEaves + 3.5f, zinc);
    const bool alongX = size.x >= size.y;
    const float middle = alongX ? hall.Center().y : hall.Center().x;
    const Rect lantern = FromRanges(alongX, AlongMin(hall, alongX) + 8.0f, AlongMax(hall, alongX) - 8.0f, middle - 2.5f,
                                    middle + 2.5f);
    boxes.Footprint(lantern, kEaves + 3.5f, kEaves + 5.3f, glass);
    boxes.Footprint(lantern.Expanded(0.5f), kEaves + 5.3f, kEaves + 5.7f, zinc);

    // Two double rows of stalls inside.
    const float across = (alongX ? size.y : size.x) * 0.22f;
    for (float side : {-1.0f, 1.0f}) {
        const float row = middle + side * across;
        for (float along = AlongMin(hall, alongX) + 5.0f; along <= AlongMax(hall, alongX) - 5.0f; along += 4.2f) {
            for (float facing : {-1.0f, 1.0f}) {
                const glm::vec2 facingVector = OnLine(alongX, 0.0f, facing);
                AddMarketStall(boxes, city, OnLine(alongX, along, row) + facingVector * 1.3f, facingVector, kPlazaHeight,
                               random);
            }
        }
    }
    const LocalFrame frame = FrameFacing(land, alongX ? kSideSouth : kSideWest);
    // People shop between the piers; the stalls are obstacles on the floor.
    city.interiors.push_back({LandmarkKind::MarketHall, OrientedRect(hall.Expanded(-1.5f)), frame.Point(border + 1.6f, 0.0f)});
    RecordLandmark(city, LandmarkKind::MarketHall, hall.Expanded(1.2f), frame, border - 1.2f);
}

void BuildClockTower(BoxWriter& boxes, CityLayout& city, const Rect& land, District district, Random& random) {
    const LocalFrame frame = FrameFacing(land, ChooseFront(land, true, random));
    const float halfWidth = frame.width * 0.5f;
    const float squareDepth = frame.length - 16.0f;
    BuildPlaza(boxes, city, frame.Area(0.0f, squareDepth, -halfWidth, halfWidth), district, random);
    {
        PartScope scope(boxes, CityPart::Plaza);
        Block(boxes, frame, squareDepth, frame.length, -halfWidth, halfWidth, 0.1f, kPlazaHeight, CobbleLook(random.Unit()));
    }

    PartScope scope(boxes, CityPart::Landmark);
    const Look stone = StoneLook(kChurchStone * 0.95f, random.Unit());
    const Look trim{kTrimStone, SurfaceMaterial::Plain, 0.75f, 0.0f};
    const Look clock{kClockFace, SurfaceMaterial::Lamp, 0.4f, 0.0f};
    const Look louvres{kDarkOpening, SurfaceMaterial::Plain, 0.9f, 0.0f};
    const Look copper{kCopperPatina, SurfaceMaterial::Metal, 0.5f, 0.0f};
    const float towerU = frame.length - 9.0f;
    const glm::vec2 center = frame.Point(towerU, 0.0f);
    SquareBox(boxes, center, 5.5f, 0.0f, 4.5f, stone);
    SquareBox(boxes, center, 4.5f, 4.5f, 38.0f, stone);
    SquareBox(boxes, center, 4.8f, 14.0f, 14.6f, trim);
    SquareBox(boxes, center, 4.8f, 26.0f, 26.6f, trim);
    SquareBox(boxes, center, 5.0f, 38.0f, 46.0f, stone);
    TowerOpenings(boxes, center, 5.0f, 1.7f, 40.3f, 43.7f, clock);  // four clock faces that glow at night
    SquareBox(boxes, center, 5.6f, 46.0f, 46.4f, trim);
    SquareBox(boxes, center, 4.2f, 46.4f, 54.0f, stone);
    TowerOpenings(boxes, center, 4.2f, 1.6f, 47.5f, 53.0f, louvres);
    SquareBox(boxes, center, 5.0f, 54.0f, 54.6f, trim);
    SteppedSpire(boxes, center, 4.6f, 54.6f, 10.0f, 4, copper);
    Block(boxes, frame, towerU - 5.58f, towerU - 5.45f, -1.3f, 1.3f, 0.15f, 3.6f, Look{kDoorWood, SurfaceMaterial::Plain, 0.7f, 0.0f});
    RecordLandmark(city, LandmarkKind::ClockTower, frame.Area(towerU - 5.6f, towerU + 5.5f, -5.6f, 5.6f), frame, towerU - 5.6f);
}

void BuildParishChurch(BoxWriter& boxes, CityLayout& city, const Rect& land, District district, Random& random) {
    const LocalFrame frame = FrameFacing(land, ChooseFront(land, false, random));
    const float halfWidth = frame.width * 0.5f;
    constexpr float kForecourt = 20.0f;
    BuildPlaza(boxes, city, frame.Area(0.0f, kForecourt, -halfWidth, halfWidth), district, random);

    PartScope scope(boxes, CityPart::Landmark);
    const Look stone = StoneLook(kChurchStone * random.Range(0.9f, 1.05f), random.Unit());
    const Look roof{kSlateRoof, SurfaceMaterial::RoofTile, 0.55f, PatternSeed(0, random.Unit())};
    const Look spire{kLead, SurfaceMaterial::Metal, 0.45f, 0.0f};
    const Look glass = StainedGlass(random);
    const Look louvres{kDarkOpening, SurfaceMaterial::Plain, 0.9f, 0.0f};
    const Look floorStone = StoneLook(kTrimStone * 0.9f);  // no random draw, so the rest of the churchyard is unchanged
    const float towerFront = kForecourt + 2.0f;
    const float naveStart = towerFront + 9.0f;
    const float naveEnd = frame.length - 14.0f;

    // A west tower with a belfry and spire, then the nave and a small apse.
    const glm::vec2 tower = frame.Point(towerFront + 4.5f, 0.0f);
    // The tower is hollow at its foot: a vestibule behind the west door, open to the nave.
    constexpr float kVestibuleCeiling = 5.6f;  // above the top of the doorway into the nave
    const RoomSpec vestibule{towerFront, towerFront + 9.0f, -4.5f, 4.5f, 28.0f, 0.9f, Doorway{1.6f, 4.6f}, Doorway{2.0f, 5.5f}};
    BuildRoomWalls(boxes, frame, vestibule, stone);
    BuildRoomFloor(boxes, frame, vestibule, floorStone);
    boxes.Footprint(RoomFloor(frame, vestibule), kVestibuleCeiling, 28.0f, stone);  // solid above the ceiling
    TowerOpenings(boxes, tower, 4.5f, 1.4f, 21.0f, 26.0f, louvres);
    SquareBox(boxes, tower, 4.9f, 28.0f, 28.7f, Look{kTrimStone, SurfaceMaterial::Plain, 0.75f, 0.0f});
    SteppedSpire(boxes, tower, 4.0f, 28.7f, 17.0f, 5, spire);

    // The nave and the apse are hollow rooms: the nave has pews on either side of a central aisle and
    // opens through a wide chancel arch into the apse, where the altar stands.
    constexpr float kChancelArch = 3.5f;
    const RoomSpec nave{naveStart, naveEnd, -7.0f, 7.0f, 15.0f, 0.8f, Doorway{2.0f, 5.5f}, Doorway{kChancelArch, 9.0f}};
    const RoomSpec apse{naveEnd, naveEnd + 7.0f, -5.0f, 5.0f, 12.0f, 0.8f, Doorway{kChancelArch, 9.0f}, std::nullopt};
    const Look ceiling{kTrimStone, SurfaceMaterial::Plain, 0.9f, 0.0f};
    const Look wood{kDoorWood, SurfaceMaterial::Plain, 0.6f, 0.0f};
    for (const RoomSpec* room : {&nave, &apse}) {
        BuildRoomWalls(boxes, frame, *room, stone);
        BuildRoomFloor(boxes, frame, *room, floorStone);
        BuildRoomCeiling(boxes, frame, *room, ceiling);
    }
    AddPews(boxes, frame, nave, PewLayout{naveStart + 4.0f, naveEnd - 6.0f, 2.4f, 1.2f, 5}, wood);
    AddAltar(boxes, frame, naveEnd + 4.5f, floorStone);
    // People walk in from the forecourt along the axis of the church, through the west door, the
    // vestibule and the nave doorway, to the central aisle. The strip is narrower than every doorway.
    constexpr float kPassageStart = 13.0f;  // inside the forecourt square, which reaches at least this far
    constexpr float kPassageHalfWidth = 1.0f;
    Interior churchNave{LandmarkKind::ParishChurch, OrientedRect(RoomFloor(frame, nave)), frame.Point(naveStart + 0.4f, 0.0f)};
    churchNave.hasPassage = true;
    churchNave.passage = OrientedRect(frame.Area(kPassageStart, naveStart + 2.5f, -kPassageHalfWidth, kPassageHalfWidth));
    churchNave.outsideSpot = frame.Point(kPassageStart, 0.0f);
    churchNave.insideSpot = frame.Point(naveStart + 2.5f, 0.0f);
    city.interiors.push_back(churchNave);
    SteppedRoof(boxes, frame, naveStart, naveEnd, -7.0f, 7.0f, 15.0f, 6.0f, 3, true, roof);
    SteppedRoof(boxes, frame, naveEnd, naveEnd + 7.0f, -5.0f, 5.0f, 12.0f, 4.0f, 2, true, roof);
    for (float u = naveStart + 3.0f; u + 3.0f <= naveEnd; u += 6.0f) {
        const float buttress = u + 3.0f;
        for (float side : {-1.0f, 1.0f}) {
            Block(boxes, frame, u - 1.1f, u + 1.1f, side * 6.92f, side * 7.08f, 4.0f, 11.5f, glass);
            Block(boxes, frame, u - 1.1f, u + 1.1f, side * 6.1f, side * 6.2f, 4.0f, 11.5f, glass);  // seen from inside
            if (buttress < naveEnd - 1.0f) {
                Block(boxes, frame, buttress - 0.6f, buttress + 0.6f, side * 7.0f, side * 8.4f, 0.0f, 10.0f, stone);
            }
        }
    }
    RecordLandmark(city, LandmarkKind::ParishChurch, frame.Area(towerFront - 0.1f, naveEnd + 7.0f, -8.4f, 8.4f), frame,
                   towerFront - 0.1f);

    // A churchyard of grass and trees on both sides.
    if (halfWidth - 1.0f > 14.0f) {
        PartScope lawn(boxes, CityPart::Park);
        for (float side : {-1.0f, 1.0f}) {
            Block(boxes, frame, towerFront, frame.length - 2.0f, side * 10.0f, side * (halfWidth - 1.0f), 0.1f, kLawnHeight,
                  GrassLook(random.Unit()));
            for (float u = towerFront + 6.0f; u < frame.length - 6.0f; u += 12.0f) {
                AddTree(boxes, city, frame.Point(u, side * (halfWidth - 5.0f)), kLawnHeight, random.Range(8.0f, 12.0f), random);
            }
        }
    }
}

}  // namespace

void BuildLandmark(BoxWriter& boxes, CityLayout& city, LandmarkKind kind, const Rect& land, District district,
                   Random& random) {
    switch (kind) {
    case LandmarkKind::Cathedral:
        BuildCathedral(boxes, city, land, random);
        break;
    case LandmarkKind::Palace:
        BuildPalace(boxes, city, land, random);
        break;
    case LandmarkKind::MarketHall:
        BuildMarketHall(boxes, city, land, random);
        break;
    case LandmarkKind::ClockTower:
        BuildClockTower(boxes, city, land, district, random);
        break;
    case LandmarkKind::ParishChurch:
    default:
        BuildParishChurch(boxes, city, land, district, random);
        break;
    }
}

}  // namespace city
