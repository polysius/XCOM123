#pragma once

#include <cmath>
#include <cstdint>

#include <glm/glm.hpp>

// Small math and random helpers shared by the crowd code.

namespace crowd {

constexpr float kPi = 3.14159265f;
constexpr float kTwoPi = 6.28318531f;

// Xorshift32: a tiny, fast generator. Each person carries its own state, so people can be
// updated on any thread in any order and still behave the same from run to run.
inline uint32_t NextRandom(uint32_t& state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

// A uniform value in [0, 1).
inline float RandomUnit(uint32_t& state) {
    return static_cast<float>(NextRandom(state) >> 8) * (1.0f / 16777216.0f);
}

inline float RandomRange(uint32_t& state, float low, float high) {
    return low + (high - low) * RandomUnit(state);
}

// A uniform integer in [0, count). count must not be zero.
inline uint32_t RandomIndex(uint32_t& state, uint32_t count) {
    return NextRandom(state) % count;
}

// Mixes the bits of an integer, used to turn a person's index into an independent seed.
inline uint32_t HashInteger(uint32_t value) {
    value ^= value >> 16;
    value *= 0x7feb352du;
    value ^= value >> 15;
    value *= 0x846ca68bu;
    value ^= value >> 16;
    return value;
}

// A seed for a generator: never zero, because xorshift would stay at zero forever.
inline uint32_t SeedFor(uint32_t index, uint32_t seed) {
    return HashInteger(index ^ HashInteger(seed)) | 1u;
}

// The signed difference between two angles, wrapped to [-pi, pi].
inline float AngleDifference(float to, float from) {
    float difference = std::fmod(to - from + kPi, kTwoPi);
    if (difference < 0.0f) {
        difference += kTwoPi;
    }
    return difference - kPi;
}

inline float WrapAngle(float angle) {
    angle = std::fmod(angle, kTwoPi);
    return angle < 0.0f ? angle + kTwoPi : angle;
}

// Rotates a 2D vector a quarter turn counter-clockwise (to its left when looking along it
// with y pointing up the page).
inline glm::vec2 LeftOf(glm::vec2 direction) {
    return {-direction.y, direction.x};
}

// The unit vector along v, or zero when v is too short to have a direction.
inline glm::vec2 SafeNormalize(glm::vec2 v) {
    const float lengthSquared = glm::dot(v, v);
    return lengthSquared > 1e-12f ? v / std::sqrt(lengthSquared) : glm::vec2(0.0f);
}

// Shortens v to at most maxLength.
inline glm::vec2 ClampLength(glm::vec2 v, float maxLength) {
    const float lengthSquared = glm::dot(v, v);
    if (lengthSquared <= maxLength * maxLength) {
        return v;
    }
    return v * (maxLength / std::sqrt(lengthSquared));
}

// Moves value towards target by at most maxStep.
inline float MoveTowards(float value, float target, float maxStep) {
    if (value < target) {
        return std::fmin(value + maxStep, target);
    }
    return std::fmax(value - maxStep, target);
}

inline bool IsFinite(glm::vec2 v) {
    return std::isfinite(v.x) && std::isfinite(v.y);
}

}  // namespace crowd
