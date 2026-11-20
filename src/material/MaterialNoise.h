#pragma once

#include <cstdint>

#include <glm/glm.hpp>

// Noise that repeats exactly, so a texture made from it tiles without a seam. Every function takes texture
// coordinates u and v in [0, 1) and a whole number of lattice cells across the tile (the period), and the pattern
// at u = 1 is the pattern at u = 0. The same seed always gives the same values, on any thread.

namespace material {

// Wraps an index into [0, count), also when it is negative.
int WrapIndex(int index, int count);

// A 32 bit hash of three integers, and its value as a number in [0, 1).
uint32_t Hash(uint32_t a, uint32_t b, uint32_t c);
float HashToUnit(uint32_t hash);

// Gradient noise with `periodX` by `periodY` cells across the tile. About -1 to 1, zero mean.
float GradientNoise(float u, float v, int periodX, int periodY, uint32_t seed);

// Fractal noise: `octaves` layers of gradient noise, each with twice the cells of the one before and `gain` times its
// strength. Scaled to about -1 to 1. Equal periods give a round pattern, different periods stretch it, which is
// what grain, brushing and creases need.
float Fbm(float u, float v, int periodX, int periodY, int octaves, uint32_t seed, float gain = 0.5f);

// The same with the valleys folded into sharp ridges (0 to 1, high on the ridges): rock faces, cracks, veins.
float RidgedFbm(float u, float v, int period, int octaves, uint32_t seed);

// Cellular noise: one random point in each of `periodX` by `periodY` cells. `nearest` and `second` are the distances
// to the closest and the next closest point, in cells. `cellHash` identifies the closest point, for a value per cell.
struct Cellular {
    float nearest = 0.0f;
    float second = 0.0f;
    uint32_t cellHash = 0;
};
Cellular CellularNoise(float u, float v, int periodX, int periodY, uint32_t seed);

}  // namespace material
