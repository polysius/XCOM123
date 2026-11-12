#pragma once

// Composing a city out of parts: a part is a complete CityLayout built in its own frame, and
// it is added to the city through a rigid transform (a turn and a move). The generator that
// built the part needs no knowledge of where it ends up. Internal to src/world.

#include <glm/glm.hpp>

#include "world/CityGenerator.h"

namespace city {

// Where a part stands in the city: its origin moves to `offset` after the part is turned about
// the origin by `yaw` (the sign convention of BoxInstance::yaw), and it is raised by `lift`
// metres. A lift of zero changes no height.
struct Placement {
    glm::vec2 offset{0.0f};
    float yaw = 0.0f;
    float lift = 0.0f;

    // A direction after the turn; positions also move by `offset`.
    glm::vec2 Turn(glm::vec2 direction) const;
    glm::vec2 Apply(glm::vec2 point) const { return offset + Turn(point); }
    OrientedRect Apply(const OrientedRect& rect) const;
};

// Adds everything in `part` to `city`, placed by `placement`. The street graph of the part is
// appended with its node and edge indices moved past the city's, so the two stay separate
// components until something connects them. `city.bounds` grows to hold the part.
// Both layouts must use the same street widths and heights.
void AppendPart(CityLayout& city, const CityLayout& part, const Placement& placement);

}  // namespace city
