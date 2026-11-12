#pragma once

// The frame landmarks and their interiors are designed in. Internal to src/world.

#include "world/CityBuild.h"

namespace city {

// A rectangle described from its front: u runs from the front edge (0) to the back (length),
// v across it from -width/2 to +width/2. Landmarks are designed in these coordinates so the same
// code can face any of the four directions.
struct LocalFrame {
    glm::vec2 origin;   // the middle of the front edge
    glm::vec2 forward;  // from the front towards the back
    glm::vec2 right;
    float length;
    float width;

    glm::vec2 Point(float u, float v) const { return origin + forward * u + right * v; }
    Rect Area(float u0, float u1, float v0, float v1) const {
        const glm::vec2 a = Point(u0, v0);
        const glm::vec2 b = Point(u1, v1);
        return {glm::min(a, b), glm::max(a, b)};
    }
};

inline LocalFrame FrameFacing(const Rect& land, Side front) {
    const glm::vec2 center = land.Center();
    const glm::vec2 size = land.Size();
    switch (front) {
    case kSideSouth:
        return {{center.x, land.min.y}, {0.0f, 1.0f}, {1.0f, 0.0f}, size.y, size.x};
    case kSideNorth:
        return {{center.x, land.max.y}, {0.0f, -1.0f}, {-1.0f, 0.0f}, size.y, size.x};
    case kSideWest:
        return {{land.min.x, center.y}, {1.0f, 0.0f}, {0.0f, -1.0f}, size.x, size.y};
    case kSideEast:
    default:
        return {{land.max.x, center.y}, {-1.0f, 0.0f}, {0.0f, 1.0f}, size.x, size.y};
    }
}

}  // namespace city
