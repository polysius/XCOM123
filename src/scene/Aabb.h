#pragma once

#include <glm/glm.hpp>

// An axis-aligned box in the world, for culling.
struct Aabb {
    glm::vec3 min{0.0f};
    glm::vec3 max{0.0f};
};
