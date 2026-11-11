#include "world/CityNature.h"

#include <algorithm>
#include <cmath>

namespace city {
namespace {

const glm::vec3 kBark{0.20f, 0.15f, 0.11f};
const glm::vec3 kLeafColors[] = {
    {0.12f, 0.21f, 0.07f},
    {0.16f, 0.25f, 0.08f},
    {0.10f, 0.18f, 0.08f},
    {0.19f, 0.26f, 0.09f},
    {0.24f, 0.25f, 0.09f},  // a tree turning early
};
const glm::vec3 kBenchWood{0.34f, 0.22f, 0.13f};
const glm::vec3 kDarkIron{0.05f, 0.055f, 0.06f};
const glm::vec3 kFountainStone{0.66f, 0.62f, 0.55f};
const glm::vec3 kBronzePatina{0.20f, 0.33f, 0.27f};
const glm::vec3 kAwningColors[] = {
    {0.55f, 0.12f, 0.10f},
    {0.15f, 0.30f, 0.20f},
    {0.62f, 0.52f, 0.30f},
    {0.18f, 0.24f, 0.42f},
    {0.60f, 0.58f, 0.52f},
};

constexpr float kPathHeight = 0.27f;  // gravel paths sit just above the lawn

Rect SquareAround(glm::vec2 center, float halfSize) {
    return {center - glm::vec2(halfSize), center + glm::vec2(halfSize)};
}

void Column(BoxWriter& boxes, glm::vec2 center, float halfSize, float bottom, float top, const Look& look) {
    boxes.Footprint(SquareAround(center, halfSize), bottom, top, look);
}

// The four sides of a frame of `width` lying just inside `outer`.
std::vector<Rect> FrameOf(const Rect& outer, float width) {
    return {
        {{outer.min.x, outer.min.y}, {outer.max.x, outer.min.y + width}},
        {{outer.min.x, outer.max.y - width}, {outer.max.x, outer.max.y}},
        {{outer.min.x, outer.min.y + width}, {outer.min.x + width, outer.max.y - width}},
        {{outer.max.x - width, outer.min.y + width}, {outer.max.x, outer.max.y - width}},
    };
}

bool TouchesAny(const Rect& area, const std::vector<Rect>& rects) {
    for (const Rect& rect : rects) {
        if (rect.Overlaps(area)) {
            return true;
        }
    }
    return false;
}

// Adds `rect` minus everything already laid, so path surfaces never overlap and flicker.
void LayPath(BoxWriter& boxes, std::vector<Rect>& laid, const Rect& rect, const Look& look) {
    for (const Rect& piece : Subtract(rect, laid)) {
        boxes.Footprint(piece, kLawnHeight - 0.06f, kPathHeight, look);
    }
    laid.push_back(rect);
}

}  // namespace

void AddTree(BoxWriter& boxes, CityLayout& city, glm::vec2 position, float ground, float height, Random& random) {
    PartScope scope(boxes, CityPart::Tree);
    const float crownRadius = height * random.Range(0.25f, 0.33f);
    const float crownBottom = ground + std::max(kOverhangClearance + 0.2f, height * random.Range(0.33f, 0.42f));
    const float top = ground + height;
    const float trunk = 0.16f + height * 0.012f;
    const Look bark{kBark, SurfaceMaterial::Plain, 0.9f, 0.0f};
    boxes.Box({position.x - trunk, ground - 0.05f, position.y - trunk},
              {position.x + trunk, crownBottom + 1.0f, position.y + trunk}, bark);

    // Two crossed boxes and a smaller top read as a rounded crown from every side.
    const glm::vec3 leaves = random.Pick(kLeafColors) * random.Range(0.85f, 1.1f);
    const Look foliage{leaves, SurfaceMaterial::Foliage, 0.8f, random.Unit()};
    const float wide = crownRadius;
    const float narrow = crownRadius * 0.62f;
    boxes.Box({position.x - wide, crownBottom + 0.6f, position.y - narrow},
              {position.x + wide, top - 1.0f, position.y + narrow}, foliage);
    boxes.Box({position.x - narrow, crownBottom, position.y - wide},
              {position.x + narrow, top - 0.6f, position.y + wide}, foliage);
    const float crest = narrow * 0.8f;
    boxes.Box({position.x - crest, top - std::min(2.2f, (top - crownBottom) * 0.4f), position.y - crest},
              {position.x + crest, top, position.y + crest}, foliage);
    city.trees.push_back({AtHeight(position, ground), height, crownRadius});
}

void AddBench(BoxWriter& boxes, CityLayout& city, glm::vec2 center, float ground, glm::vec2 facing) {
    PartScope scope(boxes, CityPart::Bench);
    const glm::vec2 along(std::abs(facing.x) > 0.5f ? glm::vec2(0.0f, 1.0f) : glm::vec2(1.0f, 0.0f));
    auto rect = [&](float halfLength, float front, float back) {
        const glm::vec2 a = center - along * halfLength + facing * front;
        const glm::vec2 b = center + along * halfLength + facing * back;
        return Rect{glm::min(a, b), glm::max(a, b)};
    };
    const Look wood{kBenchWood, SurfaceMaterial::Plain, 0.75f, 0.0f};
    const Look iron{kDarkIron, SurfaceMaterial::Metal, 0.5f, 0.0f};
    boxes.Footprint(rect(0.75f, 0.2f, -0.2f), ground - 0.02f, ground + 0.40f, iron);
    boxes.Footprint(rect(0.9f, 0.25f, -0.25f), ground + 0.40f, ground + 0.47f, wood);
    boxes.Footprint(rect(0.9f, -0.18f, -0.26f), ground + 0.47f, ground + 0.92f, wood);
    city.pointsOfInterest.push_back({AtHeight(center, ground + 0.45f), facing, PointOfInterestKind::Bench});
}

void AddFountain(BoxWriter& boxes, CityLayout& city, glm::vec2 center, float ground, float radius, Random& random) {
    PartScope scope(boxes, CityPart::Fountain);
    const Look stone = StoneLook(kFountainStone, random.Unit());
    const float rim = 0.4f;
    Column(boxes, center, radius, ground - 0.05f, ground + 0.25f, stone);
    for (const Rect& wall : FrameOf(SquareAround(center, radius), rim)) {
        boxes.Footprint(wall, ground + 0.25f, ground + 0.7f, stone);
    }
    Column(boxes, center, radius - rim, ground + 0.25f, ground + 0.55f, WaterLook());

    // A tiered centre piece: pedestal, lower bowl with water, column, upper bowl and finial.
    const float scale = glm::clamp(radius / 4.5f, 0.6f, 1.0f);
    Column(boxes, center, 0.6f * scale, ground + 0.55f, ground + 1.4f, stone);
    Column(boxes, center, 1.5f * scale, ground + 1.4f, ground + 1.62f, stone);
    Column(boxes, center, 1.3f * scale, ground + 1.62f, ground + 1.7f, WaterLook());
    Column(boxes, center, 0.25f * scale, ground + 1.7f, ground + 2.7f, stone);
    Column(boxes, center, 0.65f * scale, ground + 2.7f, ground + 2.85f, stone);
    Column(boxes, center, 0.15f * scale, ground + 2.85f, ground + 3.4f, stone);
    city.pointsOfInterest.push_back({AtHeight(center, ground), glm::vec2(0.0f, 1.0f), PointOfInterestKind::Fountain});
}

void AddStatue(BoxWriter& boxes, glm::vec2 center, float ground, Random& random) {
    PartScope scope(boxes, CityPart::Fountain);
    const Look stone = StoneLook(kFountainStone, random.Unit());
    const Look bronze{kBronzePatina, SurfaceMaterial::Metal, 0.55f, 0.0f};
    Column(boxes, center, 1.6f, ground - 0.05f, ground + 0.5f, stone);
    Column(boxes, center, 1.1f, ground + 0.5f, ground + 2.6f, stone);
    Column(boxes, center, 1.3f, ground + 2.6f, ground + 2.85f, stone);
    const bool alongX = random.Chance(0.5f);
    const glm::vec2 body = alongX ? glm::vec2(0.5f, 0.32f) : glm::vec2(0.32f, 0.5f);
    boxes.Footprint({center - body, center + body}, ground + 2.85f, ground + 4.9f, bronze);
    Column(boxes, center, 0.2f, ground + 4.9f, ground + 5.35f, bronze);
}

void AddMarketStall(BoxWriter& boxes, CityLayout& city, glm::vec2 center, glm::vec2 facing, float ground,
                    Random& random) {
    PartScope scope(boxes, CityPart::MarketStall);
    const glm::vec2 along(std::abs(facing.x) > 0.5f ? glm::vec2(0.0f, 1.0f) : glm::vec2(1.0f, 0.0f));
    auto rect = [&](float halfLength, float front, float back) {
        const glm::vec2 a = center - along * halfLength + facing * front;
        const glm::vec2 b = center + along * halfLength + facing * back;
        return Rect{glm::min(a, b), glm::max(a, b)};
    };
    const Look wood{kBenchWood * 1.2f, SurfaceMaterial::Plain, 0.8f, 0.0f};
    const Look cloth{random.Pick(kAwningColors), SurfaceMaterial::Plain, 0.9f, 0.0f};
    boxes.Footprint(rect(1.5f, 0.5f, -0.5f), ground - 0.05f, ground + 0.95f, wood);
    boxes.Footprint(rect(1.5f, -1.0f, -1.1f), ground - 0.05f, ground + 2.3f, wood);
    boxes.Footprint(rect(1.7f, 1.3f, -1.2f), ground + 2.3f, ground + 2.42f, cloth);
    city.pointsOfInterest.push_back({AtHeight(center + facing * 1.3f, ground), facing, PointOfInterestKind::MarketStall});
}

void BuildPlaza(BoxWriter& boxes, CityLayout& city, const Rect& land, District district, Random& random) {
    {
        PartScope scope(boxes, CityPart::Plaza);
        boxes.Footprint(land, 0.1f, kPlazaHeight, CobbleLook(random.Unit()));
    }
    const bool market = district == District::Market;
    const float ring = market ? 6.0f : 4.0f;
    const Rect walkable = land.Expanded(-ring);
    const glm::vec2 size = walkable.Size();
    if (size.x < 12.0f || size.y < 12.0f) {
        city.plazas.push_back(land.Expanded(-1.0f));
        return;
    }
    city.plazas.push_back(walkable);
    const glm::vec2 center = land.Center();
    if (std::min(size.x, size.y) >= 22.0f) {
        AddFountain(boxes, city, center, kPlazaHeight, std::min(4.5f, std::min(size.x, size.y) * 0.16f), random);
    } else {
        AddStatue(boxes, center, kPlazaHeight, random);
    }

    // Trees and benches, or market stalls, around the edge, leaving the middle of each side open.
    const struct {
        bool alongX;
        float edge;     // the land edge
        float inward;   // +1 when the square lies towards larger coordinates
    } sides[] = {
        {true, land.min.y, 1.0f},
        {true, land.max.y, -1.0f},
        {false, land.min.x, 1.0f},
        {false, land.max.x, -1.0f},
    };
    for (const auto& side : sides) {
        const float start = AlongMin(land, side.alongX) + 5.0f;
        const float end = AlongMax(land, side.alongX) - 5.0f;
        const float middle = (start + end) * 0.5f;
        const glm::vec2 facing = OnLine(side.alongX, 0.0f, side.inward);
        const float spacing = market ? 4.4f : 9.0f;
        int count = 0;
        for (float along = start; along <= end; along += spacing, ++count) {
            if (std::abs(along - middle) < 6.0f) {
                continue;  // an opening onto the square
            }
            if (market) {
                AddMarketStall(boxes, city, OnLine(side.alongX, along, side.edge + side.inward * 2.6f), facing,
                               kPlazaHeight, random);
            } else if (count % 2 == 0) {
                AddTree(boxes, city, OnLine(side.alongX, along, side.edge + side.inward * 2.0f), kPlazaHeight,
                        random.Range(7.0f, 10.0f), random);
            } else {
                AddBench(boxes, city, OnLine(side.alongX, along, side.edge + side.inward * 3.2f), kPlazaHeight, facing);
            }
        }
    }
}

void BuildPark(BoxWriter& boxes, CityLayout& city, const Rect& land, Random& random) {
    city.parks.push_back(land);
    PartScope scope(boxes, CityPart::Park);
    boxes.Footprint(land, 0.1f, kLawnHeight, GrassLook(random.Unit()));

    // Paths: a ring inside the hedge, a cross through the middle and a round place where they meet.
    const glm::vec2 center = land.Center();
    const glm::vec2 size = land.Size();
    const Look gravel = GravelLook();
    std::vector<Rect> paths;
    const float fountainRadius = std::min(4.5f, std::min(size.x, size.y) * 0.08f);
    LayPath(boxes, paths, SquareAround(center, fountainRadius + 3.0f), gravel);
    LayPath(boxes, paths, {{land.min.x, center.y - 2.0f}, {land.max.x, center.y + 2.0f}}, gravel);
    LayPath(boxes, paths, {{center.x - 2.0f, land.min.y}, {center.x + 2.0f, land.max.y}}, gravel);
    const Rect ring = land.Expanded(-3.0f);
    LayPath(boxes, paths, {{ring.min.x, ring.min.y}, {ring.max.x, ring.min.y + 3.0f}}, gravel);
    LayPath(boxes, paths, {{ring.min.x, ring.max.y - 3.0f}, {ring.max.x, ring.max.y}}, gravel);
    LayPath(boxes, paths, {{ring.min.x, ring.min.y}, {ring.min.x + 3.0f, ring.max.y}}, gravel);
    LayPath(boxes, paths, {{ring.max.x - 3.0f, ring.min.y}, {ring.max.x, ring.max.y}}, gravel);
    AddFountain(boxes, city, center, kPathHeight, fountainRadius, random);

    // A pond in one quarter of the larger parks.
    std::vector<Rect> keepClear = paths;
    if (size.x > 110.0f && size.y > 110.0f) {
        const glm::vec2 quarter = center + glm::vec2(random.Sign(), random.Sign()) * size * 0.25f;
        const glm::vec2 pondHalf = size * glm::vec2(random.Range(0.12f, 0.16f), random.Range(0.12f, 0.16f));
        const Rect pond{quarter - pondHalf, quarter + pondHalf};
        for (const Rect& kerb : FrameOf(pond.Expanded(0.6f), 0.6f)) {
            boxes.Footprint(kerb, kLawnHeight - 0.05f, kLawnHeight + 0.22f, StoneLook(kFountainStone, 0.2f));
        }
        boxes.Footprint(pond, kLawnHeight - 0.04f, kLawnHeight + 0.12f, WaterLook());
        keepClear.push_back(pond.Expanded(1.5f));
    }

    // Clipped hedges along the edge, open where the paths come in.
    const Look hedge{kLeafColors[2], SurfaceMaterial::Foliage, 0.85f, 0.5f};
    const std::vector<Rect> entrances{{{center.x - 3.0f, land.min.y}, {center.x + 3.0f, land.max.y}},
                                      {{land.min.x, center.y - 3.0f}, {land.max.x, center.y + 3.0f}}};
    for (const Rect& line : FrameOf(land.Expanded(-0.4f), 0.8f)) {
        for (const Rect& piece : Subtract(line, entrances)) {
            boxes.Footprint(piece, kLawnHeight - 0.05f, kLawnHeight + 1.1f, hedge);
        }
    }

    // Trees on a loose grid, kept off the paths.
    const float spacing = random.Range(11.0f, 14.0f);
    const Rect grove = land.Expanded(-6.5f);
    for (float z = grove.min.y; z <= grove.max.y; z += spacing) {
        for (float x = grove.min.x; x <= grove.max.x; x += spacing) {
            const glm::vec2 tree(x + random.Range(-2.5f, 2.5f), z + random.Range(-2.5f, 2.5f));
            if (grove.Contains(tree) && !TouchesAny(SquareAround(tree, 2.0f), keepClear)) {
                AddTree(boxes, city, tree, kLawnHeight, random.Range(8.0f, 15.0f), random);
            }
        }
    }

    // Benches along both sides of the east-west path.
    for (float x = land.min.x + 12.0f; x < land.max.x - 12.0f; x += 16.0f) {
        if (std::abs(x - center.x) < fountainRadius + 6.0f) {
            continue;
        }
        AddBench(boxes, city, {x, center.y - 2.6f}, kLawnHeight, {0.0f, 1.0f});
        AddBench(boxes, city, {x, center.y + 2.6f}, kLawnHeight, {0.0f, -1.0f});
    }
}

void BuildMedian(BoxWriter& boxes, CityLayout& city, const Rect& land, Random& random) {
    PartScope scope(boxes, CityPart::Park);
    boxes.Footprint(land, 0.1f, kPlazaHeight + 0.02f, GravelLook());
    const glm::vec2 size = land.Size();
    const bool alongX = size.x >= size.y;
    const float length = alongX ? size.x : size.y;
    const float width = alongX ? size.y : size.x;
    const float middle = alongX ? land.Center().y : land.Center().x;
    if (length < 30.0f) {
        if (width >= 8.0f) {
            AddStatue(boxes, land.Center(), kPlazaHeight + 0.02f, random);
        }
        return;
    }
    // Two rows of trees with benches on the walk between them.
    const float start = AlongMin(land, alongX) + 4.5f;
    const float end = AlongMax(land, alongX) - 4.5f;
    const int gaps = std::max(1, static_cast<int>((end - start) / 9.0f));
    const float spacing = (end - start) / static_cast<float>(gaps);
    for (int i = 0; i <= gaps; ++i) {
        const float along = start + spacing * static_cast<float>(i);
        for (float side : {-1.0f, 1.0f}) {
            AddTree(boxes, city, OnLine(alongX, along, middle + side * width * 0.27f), kPlazaHeight,
                    random.Range(10.0f, 13.0f), random);
        }
        if (i < gaps && i % 3 == 1) {
            AddBench(boxes, city, OnLine(alongX, along + spacing * 0.5f, middle), kPlazaHeight + 0.02f,
                     OnLine(alongX, 0.0f, random.Sign()));
        }
    }
}

void PlantCourtyard(BoxWriter& boxes, CityLayout& city, const Rect& courtyard, const std::vector<Rect>& annexes,
                    Random& random) {
    PartScope scope(boxes, CityPart::Park);
    const glm::vec2 size = courtyard.Size();
    const float shortSide = std::min(size.x, size.y);
    // Most courtyards are paved yards; only some are gardens with a tree or two. The surface
    // runs under the annexes, whose walls hide it.
    const float roll = random.Unit();
    if (shortSide < 7.0f || roll < 0.3f) {
        boxes.Footprint(courtyard, 0.1f, kPlazaHeight, GravelLook());
        return;
    }
    if (roll < 0.6f) {
        boxes.Footprint(courtyard, 0.1f, kPlazaHeight, CobbleLook(random.Unit()));
        return;
    }
    boxes.Footprint(courtyard, 0.1f, kLawnHeight, GrassLook(random.Unit()));
    if (shortSide < 11.0f) {
        return;
    }
    const bool alongX = size.x >= size.y;
    const float length = alongX ? size.x : size.y;
    const int trees = length > 34.0f ? 2 : 1;
    for (int i = 0; i < trees; ++i) {
        const float t = (static_cast<float>(i) + 1.0f) / static_cast<float>(trees + 1);
        const glm::vec2 offset = alongX ? glm::vec2((t - 0.5f) * size.x, 0.0f) : glm::vec2(0.0f, (t - 0.5f) * size.y);
        const glm::vec2 position = courtyard.Center() + offset;
        const float height = std::min(shortSide * 0.9f, 12.0f);
        if (!TouchesAny(SquareAround(position, height * 0.35f), annexes)) {
            AddTree(boxes, city, position, kLawnHeight, height, random);
        }
    }
}

}  // namespace city
