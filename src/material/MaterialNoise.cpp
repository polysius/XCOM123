#include "material/MaterialNoise.h"

#include <algorithm>
#include <cmath>

namespace material {
namespace {

constexpr float kTwoPi = 6.28318530718f;

// Wraps a lattice index into [0, period).
int Wrap(int index, int period) {
    const int remainder = index % period;
    return remainder < 0 ? remainder + period : remainder;
}

float Quintic(float t) {
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

float CornerContribution(int cellX, int cellY, float offsetX, float offsetY, int periodX, int periodY, uint32_t seed) {
    const uint32_t hash = Hash(static_cast<uint32_t>(Wrap(cellX, periodX)), static_cast<uint32_t>(Wrap(cellY, periodY)), seed);
    const float angle = kTwoPi * HashToUnit(hash);
    return std::cos(angle) * offsetX + std::sin(angle) * offsetY;
}

}  // namespace

uint32_t Hash(uint32_t a, uint32_t b, uint32_t c) {
    uint32_t h = a * 0x9E3779B1u ^ (b + 0x7F4A7C15u) * 0x85EBCA6Bu ^ (c + 0x165667B1u) * 0xC2B2AE35u;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    h *= 0x297A2D39u;
    h ^= h >> 15;
    return h;
}

float HashToUnit(uint32_t hash) {
    return static_cast<float>(hash >> 8) * (1.0f / 16777216.0f);
}

float GradientNoise(float u, float v, int periodX, int periodY, uint32_t seed) {
    const float x = u * static_cast<float>(periodX);
    const float y = v * static_cast<float>(periodY);
    const int cellX = static_cast<int>(std::floor(x));
    const int cellY = static_cast<int>(std::floor(y));
    const float fractionX = x - static_cast<float>(cellX);
    const float fractionY = y - static_cast<float>(cellY);

    const float bottomLeft = CornerContribution(cellX, cellY, fractionX, fractionY, periodX, periodY, seed);
    const float bottomRight = CornerContribution(cellX + 1, cellY, fractionX - 1.0f, fractionY, periodX, periodY, seed);
    const float topLeft = CornerContribution(cellX, cellY + 1, fractionX, fractionY - 1.0f, periodX, periodY, seed);
    const float topRight = CornerContribution(cellX + 1, cellY + 1, fractionX - 1.0f, fractionY - 1.0f, periodX, periodY, seed);

    const float blendX = Quintic(fractionX);
    const float blendY = Quintic(fractionY);
    const float bottom = bottomLeft + (bottomRight - bottomLeft) * blendX;
    const float top = topLeft + (topRight - topLeft) * blendX;
    // Gradient noise of unit gradients stays within about +-0.7, so scale it up to reach +-1.
    return (bottom + (top - bottom) * blendY) * 1.4f;
}

float Fbm(float u, float v, int periodX, int periodY, int octaves, uint32_t seed, float gain) {
    float sum = 0.0f;
    float strength = 1.0f;
    float total = 0.0f;
    int cellsX = periodX;
    int cellsY = periodY;
    for (int octave = 0; octave < octaves; ++octave) {
        sum += strength * GradientNoise(u, v, cellsX, cellsY, seed + static_cast<uint32_t>(octave) * 7919u);
        total += strength;
        strength *= gain;
        cellsX *= 2;
        cellsY *= 2;
    }
    return sum / total;
}

float RidgedFbm(float u, float v, int period, int octaves, uint32_t seed) {
    float sum = 0.0f;
    float strength = 1.0f;
    float total = 0.0f;
    int cells = period;
    for (int octave = 0; octave < octaves; ++octave) {
        const float ridge = 1.0f - std::abs(GradientNoise(u, v, cells, cells, seed + static_cast<uint32_t>(octave) * 104729u));
        sum += strength * ridge * ridge;  // squared, so the ridges are sharp and the flats between them wide
        total += strength;
        strength *= 0.5f;
        cells *= 2;
    }
    return sum / total;
}

Cellular CellularNoise(float u, float v, int periodX, int periodY, uint32_t seed) {
    const float x = u * static_cast<float>(periodX);
    const float y = v * static_cast<float>(periodY);
    const int cellX = static_cast<int>(std::floor(x));
    const int cellY = static_cast<int>(std::floor(y));

    Cellular result;
    result.nearest = 1e9f;
    result.second = 1e9f;
    for (int offsetY = -1; offsetY <= 1; ++offsetY) {
        for (int offsetX = -1; offsetX <= 1; ++offsetX) {
            const int neighbourX = cellX + offsetX;
            const int neighbourY = cellY + offsetY;
            const uint32_t hash = Hash(static_cast<uint32_t>(Wrap(neighbourX, periodX)), static_cast<uint32_t>(Wrap(neighbourY, periodY)), seed);
            const float pointX = static_cast<float>(neighbourX) + HashToUnit(hash);
            const float pointY = static_cast<float>(neighbourY) + HashToUnit(Hash(hash, 1u, seed));
            const float distance = std::sqrt((pointX - x) * (pointX - x) + (pointY - y) * (pointY - y));
            if (distance < result.nearest) {
                result.second = result.nearest;
                result.nearest = distance;
                result.cellHash = hash;
            } else if (distance < result.second) {
                result.second = distance;
            }
        }
    }
    return result;
}

}  // namespace material
