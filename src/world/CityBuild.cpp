#include "world/CityBuild.h"

#include <algorithm>

namespace city {

uint32_t MixSeed(uint32_t seed, uint32_t salt) {
    // A small integer hash (from the "lowbias32" family): every input bit affects every output bit.
    uint32_t x = seed ^ (salt * 0x9E3779B9u);
    x ^= x >> 16;
    x *= 0x7FEB352Du;
    x ^= x >> 15;
    x *= 0x846CA68Bu;
    x ^= x >> 16;
    return x;
}

Rect Intersection(const Rect& a, const Rect& b) {
    return {glm::max(a.min, b.min), glm::min(a.max, b.max)};
}

Rect ExpandSides(const Rect& rect, uint8_t sides, float amount) {
    Rect result = rect;
    if (sides & kSideWest) result.min.x -= amount;
    if (sides & kSideEast) result.max.x += amount;
    if (sides & kSideSouth) result.min.y -= amount;
    if (sides & kSideNorth) result.max.y += amount;
    return result;
}

Rect InsetRect(const Rect& rect, float west, float east, float south, float north) {
    return {{rect.min.x + west, rect.min.y + south}, {rect.max.x - east, rect.max.y - north}};
}

namespace {
// Adds the parts of `area` outside `cut`: full-width strips below and above the cut, then the
// pieces to its left and right.
void SubtractOne(const Rect& area, const Rect& cut, std::vector<Rect>& pieces) {
    if (!area.Overlaps(cut)) {
        pieces.push_back(area);
        return;
    }
    const Rect overlap = Intersection(area, cut);
    const Rect below{{area.min.x, area.min.y}, {area.max.x, overlap.min.y}};
    const Rect above{{area.min.x, overlap.max.y}, {area.max.x, area.max.y}};
    const Rect left{{area.min.x, overlap.min.y}, {overlap.min.x, overlap.max.y}};
    const Rect right{{overlap.max.x, overlap.min.y}, {area.max.x, overlap.max.y}};
    for (const Rect& piece : {below, above, left, right}) {
        if (!piece.IsEmpty()) {
            pieces.push_back(piece);
        }
    }
}
}  // namespace

std::vector<Rect> Subtract(const Rect& area, const std::vector<Rect>& cuts) {
    std::vector<Rect> pieces{area};
    std::vector<Rect> next;
    for (const Rect& cut : cuts) {
        next.clear();
        for (const Rect& piece : pieces) {
            SubtractOne(piece, cut, next);
        }
        pieces.swap(next);
    }
    return pieces;
}

Rect SegmentRect(glm::vec2 a, glm::vec2 b, float halfWidth, float extend) {
    const glm::vec2 low = glm::min(a, b);
    const glm::vec2 high = glm::max(a, b);
    const bool alongX = (high.x - low.x) >= (high.y - low.y);
    if (alongX) {
        return {{low.x - extend, low.y - halfWidth}, {high.x + extend, high.y + halfWidth}};
    }
    return {{low.x - halfWidth, low.y - extend}, {high.x + halfWidth, high.y + extend}};
}

void BoxWriter::Box(glm::vec3 min, glm::vec3 max, const Look& look) {
    TurnedBox((min + max) * 0.5f, (max - min) * 0.5f, 0.0f, look);
}

void BoxWriter::TurnedBox(glm::vec3 center, glm::vec3 halfSize, float yaw, const Look& look) {
    BoxInstance box;
    box.center = center;
    box.material = static_cast<float>(look.material);
    box.halfSize = halfSize;
    box.roughness = look.roughness;
    box.color = look.color;
    box.seed = look.seed;
    box.yaw = yaw;
    city.boxes.push_back(box);
    city.boxParts.push_back(part);
}

void BoxWriter::Footprint(const Rect& footprint, float bottom, float top, const Look& look) {
    Box({footprint.min.x, bottom, footprint.min.y}, {footprint.max.x, top, footprint.max.y}, look);
}

Look AsphaltLook() {
    return {{0.085f, 0.085f, 0.09f}, SurfaceMaterial::Ground, 0.9f, GroundSeed(GroundPattern::Asphalt)};
}

Look PavingLook() {
    return {{0.50f, 0.48f, 0.45f}, SurfaceMaterial::Ground, 0.8f, GroundSeed(GroundPattern::Slabs)};
}

Look CobbleLook(float random01) {
    const glm::vec3 color = glm::mix(glm::vec3(0.46f, 0.43f, 0.40f), glm::vec3(0.58f, 0.54f, 0.48f), random01);
    return {color, SurfaceMaterial::Ground, 0.75f, GroundSeed(GroundPattern::Cobbles, random01)};
}

Look GravelLook() {
    return {{0.62f, 0.57f, 0.48f}, SurfaceMaterial::Ground, 0.95f, GroundSeed(GroundPattern::Gravel)};
}

Look StoneLook(glm::vec3 color, float random01) {
    return {color, SurfaceMaterial::Stone, 0.7f, random01};
}

Look GrassLook(float random01) {
    const glm::vec3 color = glm::mix(glm::vec3(0.13f, 0.23f, 0.07f), glm::vec3(0.19f, 0.29f, 0.09f), random01);
    return {color, SurfaceMaterial::Grass, 0.9f, random01};
}

Look WaterLook() {
    return {{0.025f, 0.05f, 0.055f}, SurfaceMaterial::Water, 0.06f, 0.0f};
}

}  // namespace city
