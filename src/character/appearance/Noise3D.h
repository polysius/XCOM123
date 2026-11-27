#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <utility>

#include <glm/glm.hpp>

#include "material/MaterialNoise.h"

// Noise over points in space, for textures that are baked from the surface of a body. A pattern made from the position
// on the skin has no seam where the texture atlas is cut, and the same pattern comes out whatever the unwrap is. It builds
// on the hash of the material noise, so it is as deterministic as that is.

namespace character {

inline float Smooth(float t) {
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

inline float SmoothStep(float edge0, float edge1, float x) {
    const float t = glm::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// A number in [0, 1) from three integers and a seed.
inline float Random01(uint32_t a, uint32_t b, uint32_t c) {
    return material::HashToUnit(material::Hash(a, b, c));
}

// Picks an entry of a list of (weight, value) pairs with a number in [0, 1).
template <typename T, size_t N>
T Pick(const std::array<std::pair<float, T>, N>& table, float draw) {
    float total = 0.0f;
    for (const auto& entry : table) {
        total += entry.first;
    }
    float remaining = draw * total;
    for (const auto& entry : table) {
        remaining -= entry.first;
        if (remaining < 0.0f) {
            return entry.second;
        }
    }
    return table.back().second;
}

inline glm::vec3 SrgbToLinear(glm::vec3 c) {
    return glm::pow(c, glm::vec3(2.2f));
}

inline glm::vec3 ToSrgb(glm::vec3 linear) {
    return glm::pow(glm::clamp(linear, glm::vec3(0.0f), glm::vec3(1.0f)), glm::vec3(1.0f / 2.2f));
}

// A value in [0, 1] as an 8 bit channel.
inline uint8_t Byte(float value) {
    return static_cast<uint8_t>(std::lround(glm::clamp(value, 0.0f, 1.0f) * 255.0f));
}

// Smooth value noise, about -1 to 1, with one lattice cell per unit of length.
inline float ValueNoise3(glm::vec3 p, uint32_t seed) {
    const glm::vec3 cell = glm::floor(p);
    const glm::vec3 f = p - cell;
    const glm::ivec3 c(cell);
    float corner[2][2][2];
    for (int dz = 0; dz < 2; ++dz) {
        for (int dy = 0; dy < 2; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
                const uint32_t h = material::Hash(static_cast<uint32_t>(c.x + dx), static_cast<uint32_t>(c.y + dy), static_cast<uint32_t>(c.z + dz) ^ (seed * 0x9E3779B1u));
                corner[dz][dy][dx] = material::HashToUnit(h) * 2.0f - 1.0f;
            }
        }
    }
    const float bx = Smooth(f.x);
    const float by = Smooth(f.y);
    const float bz = Smooth(f.z);
    float layer[2];
    for (int dz = 0; dz < 2; ++dz) {
        const float low = corner[dz][0][0] + (corner[dz][0][1] - corner[dz][0][0]) * bx;
        const float high = corner[dz][1][0] + (corner[dz][1][1] - corner[dz][1][0]) * bx;
        layer[dz] = low + (high - low) * by;
    }
    return layer[0] + (layer[1] - layer[0]) * bz;
}

// Layers of value noise, each with twice the cells of the one before and `gain` times its strength.
inline float Fbm3(glm::vec3 p, int octaves, uint32_t seed, float gain = 0.5f) {
    float sum = 0.0f;
    float strength = 1.0f;
    float total = 0.0f;
    for (int octave = 0; octave < octaves; ++octave) {
        sum += strength * ValueNoise3(p, seed + static_cast<uint32_t>(octave) * 7919u);
        total += strength;
        strength *= gain;
        p *= 2.0f;
    }
    return sum / total;
}

// Cellular noise: the distance to the nearest and the next nearest of random points, one in each unit cell.
struct Cellular3 {
    float nearest = 0.0f;
    float second = 0.0f;
    uint32_t cellHash = 0;
};

inline Cellular3 CellularNoise3(glm::vec3 p, uint32_t seed) {
    const glm::ivec3 base(glm::floor(p));
    Cellular3 result;
    result.nearest = 1e9f;
    result.second = 1e9f;
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const glm::ivec3 cell = base + glm::ivec3(dx, dy, dz);
                const uint32_t hash = material::Hash(static_cast<uint32_t>(cell.x), static_cast<uint32_t>(cell.y), static_cast<uint32_t>(cell.z) ^ (seed * 0x85EBCA6Bu));
                const glm::vec3 point = glm::vec3(cell) + glm::vec3(material::HashToUnit(hash), material::HashToUnit(material::Hash(hash, 1u, seed)),
                                                                     material::HashToUnit(material::Hash(hash, 2u, seed)));
                const float distance = glm::distance(point, p);
                if (distance < result.nearest) {
                    result.second = result.nearest;
                    result.nearest = distance;
                    result.cellHash = hash;
                } else if (distance < result.second) {
                    result.second = distance;
                }
            }
        }
    }
    return result;
}

}  // namespace character
