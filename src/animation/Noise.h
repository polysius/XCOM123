#pragma once

#include <cmath>
#include <cstdint>

// Smooth seeded noise for idle motion. It is hashed by hand, never drawn from the standard distributions, which differ
// between compilers: the same seed gives the same motion on every machine, and the noise is a pure function of the time.

namespace animation {

// A hash of an integer lattice point and a seed to a number in [-1, 1].
inline float LatticeValue(uint32_t seed, int32_t index) {
    uint32_t h = seed * 0x9E3779B1u ^ (static_cast<uint32_t>(index) + 0x7F4A7C15u) * 0x85EBCA6Bu;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    h *= 0x297A2D39u;
    h ^= h >> 15;
    return static_cast<float>(h >> 8) * (2.0f / 16777216.0f) - 1.0f;
}

// Value noise in [-1, 1]: the lattice values joined by a quintic curve, so the first and second derivatives are continuous.
inline float SmoothNoise(uint32_t seed, float x) {
    const float floorX = std::floor(x);
    const float t = x - floorX;
    const float curve = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
    const int32_t index = static_cast<int32_t>(floorX);
    const float a = LatticeValue(seed, index);
    const float b = LatticeValue(seed, index + 1);
    return a + (b - a) * curve;
}

// Two octaves of noise: slow drifts with a little faster detail on top, still in [-1, 1].
inline float DriftNoise(uint32_t seed, float x) {
    return 0.75f * SmoothNoise(seed, x) + 0.25f * SmoothNoise(seed + 101u, x * 2.7f);
}

}  // namespace animation
