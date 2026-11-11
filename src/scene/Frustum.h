#pragma once

#include <array>

#include <glm/glm.hpp>

// The six planes of a camera's view volume, used to skip objects that are off screen.
class Frustum {
public:
    // Extracts the planes from a view-projection matrix (Gribb and Hartmann's method),
    // using Vulkan's 0..1 clip depth. Works with normal and reversed depth alike.
    explicit Frustum(const glm::mat4& viewProjection) {
        const glm::vec4 row0(viewProjection[0][0], viewProjection[1][0], viewProjection[2][0], viewProjection[3][0]);
        const glm::vec4 row1(viewProjection[0][1], viewProjection[1][1], viewProjection[2][1], viewProjection[3][1]);
        const glm::vec4 row2(viewProjection[0][2], viewProjection[1][2], viewProjection[2][2], viewProjection[3][2]);
        const glm::vec4 row3(viewProjection[0][3], viewProjection[1][3], viewProjection[2][3], viewProjection[3][3]);

        planes = {row3 + row0, row3 - row0, row3 + row1, row3 - row1, row2, row3 - row2};
        for (glm::vec4& plane : planes) {
            plane /= glm::length(glm::vec3(plane));
        }
    }

    bool ContainsSphere(const glm::vec3& center, float radius) const {
        for (const glm::vec4& plane : planes) {
            if (glm::dot(glm::vec3(plane), center) + plane.w < -radius) {
                return false;
            }
        }
        return true;
    }

private:
    std::array<glm::vec4, 6> planes;
};
