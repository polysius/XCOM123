#include "material/MaterialGenerator.h"

#include <algorithm>
#include <cmath>

#include "core/JobSystem.h"
#include "material/MaterialNoise.h"

namespace material {
namespace {

constexpr float kPi = 3.14159265f;

// What a recipe says about one point of the tile, before the maps are finished.
struct Sample {
    float height = 0.5f;  // any scale: the heights of the whole tile are fitted to the relief afterwards
    glm::vec3 albedo{0.5f};
    float roughness = 0.5f;
    float metallic = 0.0f;
};

// The numbers a recipe may use besides the position.
struct Context {
    uint32_t seed;
    float wear;
    float detail;
    int microCells;  // cells across the tile of the coarsest layer of fine grain; it grows with the size, so the grain stays at pixel scale
};

// How big the tile really is and how deep its surface is, per kind.
struct Dimensions {
    float tileMeters;
    float reliefMeters;
};

Dimensions DimensionsOf(MaterialKind kind) {
    switch (kind) {
    case MaterialKind::Stone: return {1.0f, 0.040f};
    case MaterialKind::Brick: return {0.8f, 0.020f};
    case MaterialKind::Concrete: return {1.0f, 0.012f};
    case MaterialKind::Plaster: return {1.0f, 0.006f};
    case MaterialKind::Wood: return {0.5f, 0.004f};
    case MaterialKind::Metal: return {0.25f, 0.0008f};
    case MaterialKind::Leather: return {0.25f, 0.0015f};
    case MaterialKind::Fabric: return {0.1f, 0.0012f};
    case MaterialKind::Asphalt: return {0.5f, 0.010f};
    case MaterialKind::Soil: return {1.0f, 0.030f};
    }
    return {1.0f, 0.01f};
}

// Small helpers --------------------------------------------------------------------------------

// A colour given as the sRGB values of a photograph, converted to the linear values that lighting works in.
glm::vec3 Srgb(float r, float g, float b) {
    const auto toLinear = [](float value) {
        return value <= 0.04045f ? value / 12.92f : std::pow((value + 0.055f) / 1.055f, 2.4f);
    };
    return {toLinear(r), toLinear(g), toLinear(b)};
}

// Limits a setting to 0..1. A number that is not a number counts as 0, which glm::clamp would pass on.
float ClampUnit(float value) {
    return std::isnan(value) ? 0.0f : std::clamp(value, 0.0f, 1.0f);
}

float Unit(float noise) {  // noise in -1..1 to 0..1
    return 0.5f + 0.5f * noise;
}

float Smooth(float low, float high, float value) {
    return glm::smoothstep(low, high, value);
}

float Mix(float a, float b, float t) {
    return a + (b - a) * t;
}

float Fraction(float value) {
    return value - std::floor(value);
}

// The finest detail, laid over every kind: grain over the last three octaves, down to two pixels.
float MicroDetail(const Context& c, float u, float v) {
    return c.detail * Fbm(u, v, c.microCells, c.microCells, 3, c.seed + 9001u, 0.6f);
}

// An index below `count` from the seed, to pick between variants of a material.
int PickIndex(const Context& c, uint32_t salt, int count) {
    return std::min(count - 1, static_cast<int>(HashToUnit(Hash(c.seed, salt, 77u)) * static_cast<float>(count)));
}

// A value between 0 and 1 from the seed, to pick between variants of a material.
float SeedChoice(const Context& c, uint32_t salt) {
    return HashToUnit(Hash(c.seed, salt, 77u));
}

// Recipes ----------------------------------------------------------------------------------------

Sample StoneSample(const Context& c, float u, float v) {
    const uint32_t s = c.seed * 977u;  // far apart, so the noise of neighbouring seeds shares no layer
    const float warpedU = u + 0.03f * Fbm(u, v, 4, 4, 3, s + 1);
    const float warpedV = v + 0.03f * Fbm(u, v, 4, 4, 3, s + 2);
    const float bulk = Fbm(warpedU, warpedV, 3, 3, 5, s + 3);
    const float grain = Fbm(warpedU, warpedV, 48, 48, 3, s + 4, 0.6f);
    const float strata = Fbm(warpedU, warpedV, 2, 14, 3, s + 7);

    // Plates of stone with cracks along some of their borders.
    const Cellular plates = CellularNoise(warpedU, warpedV, 5, 5, s + 5);
    const float edgeDistance = plates.second - plates.nearest;
    const float crackAllowed = Smooth(0.15f, 0.5f, Fbm(u, v, 3, 3, 3, s + 6)) * (0.3f + 0.7f * c.wear);
    const float crack = (1.0f - Smooth(0.0f, 0.05f, edgeDistance)) * crackAllowed;
    const float plateTone = HashToUnit(plates.cellHash);

    Sample sample;
    // Broken rock has sharp facets (ridged noise) on top of the rounded bulk, and weathering pits the surface.
    const float facets = RidgedFbm(warpedU, warpedV, 6, 5, s + 10);
    const float pitting = Smooth(0.35f, 0.6f, Fbm(warpedU, warpedV, 24, 24, 3, s + 11));
    sample.height = 0.5f + 0.20f * bulk + 0.10f * strata + 0.08f * grain + 0.22f * facets + (plateTone - 0.5f) * 0.08f - 0.35f * crack -
                    0.06f * pitting * (0.5f + c.wear) + 0.04f * MicroDetail(c, u, v);

    const float warm = SeedChoice(c, 1);
    glm::vec3 colour = glm::mix(Srgb(0.46f, 0.44f, 0.41f), Srgb(0.74f, 0.70f, 0.62f), warm);
    colour *= 0.8f + 0.3f * Unit(bulk);
    colour *= 0.92f + 0.16f * plateTone;
    const float speckle = Fbm(warpedU, warpedV, 160, 160, 2, s + 8);
    colour *= speckle > 0.35f ? 0.6f : (speckle < -0.4f ? 1.25f : 1.0f);
    colour *= 1.0f - 0.5f * crack;
    sample.albedo = colour;
    sample.roughness = 0.62f + 0.18f * Unit(grain) + 0.2f * crack;
    return sample;
}

Sample BrickSample(const Context& c, float u, float v) {
    constexpr int kRows = 8;  // an even number of rows, so the offset of every second row also tiles
    constexpr int kColumns = 4;
    constexpr float kMortarHalfWidth = 0.006f;  // in tile sizes, about 5 mm
    const uint32_t s = c.seed * 977u;  // far apart, so the noise of neighbouring seeds shares no layer

    const float warpedU = u + 0.004f * Fbm(u, v, 24, 24, 2, s + 1);
    const float warpedV = v + 0.004f * Fbm(u, v, 24, 24, 2, s + 2);
    const float rowPosition = warpedV * kRows;
    const int row = WrapIndex(static_cast<int>(std::floor(rowPosition)), kRows);
    const float columnPosition = warpedU * kColumns + ((row & 1) != 0 ? 0.5f : 0.0f);
    const int column = WrapIndex(static_cast<int>(std::floor(columnPosition)), kColumns);
    const float insideX = Fraction(columnPosition);
    const float insideY = Fraction(rowPosition);

    // The distance from the nearest brick border, in tile sizes.
    const float borderDistance = std::min(std::min(insideX, 1.0f - insideX) / kColumns, std::min(insideY, 1.0f - insideY) / kRows);
    const float brickRound = Smooth(kMortarHalfWidth, kMortarHalfWidth + 0.012f, borderDistance);
    const float brickMask = Smooth(kMortarHalfWidth, kMortarHalfWidth + 0.004f, borderDistance);

    const uint32_t brickHash = Hash(static_cast<uint32_t>(row), static_cast<uint32_t>(column), s);
    const float brickTone = HashToUnit(brickHash);
    const float brickLevel = (HashToUnit(Hash(brickHash, 3u, s)) - 0.5f) * 0.05f;

    const float brickHeight = 0.62f + brickLevel + 0.05f * Fbm(u, v, 96, 96, 3, s + 3) + 0.02f * MicroDetail(c, u, v);
    const float mortarHeight = 0.40f + 0.04f * Fbm(u, v, 256, 256, 2, s + 4);

    Sample sample;
    sample.height = Mix(mortarHeight, brickHeight, brickRound);

    const glm::vec3 fired = glm::mix(Srgb(0.40f, 0.18f, 0.14f), Srgb(0.62f, 0.36f, 0.25f), brickTone);
    const glm::vec3 brickColour = fired * (0.85f + 0.3f * Unit(Fbm(u, v, 6, 6, 4, s + 5)));
    const glm::vec3 mortarColour = Srgb(0.62f, 0.60f, 0.55f) * (0.85f + 0.3f * Unit(Fbm(u, v, 200, 200, 2, s + 6)));
    sample.albedo = glm::mix(mortarColour, brickColour, brickMask);
    sample.roughness = Mix(0.95f, 0.80f, brickMask);
    return sample;
}

Sample ConcreteSample(const Context& c, float u, float v) {
    const uint32_t s = c.seed * 977u;  // far apart, so the noise of neighbouring seeds shares no layer
    const float base = Fbm(u, v, 3, 3, 5, s + 1);
    const float pores = Fbm(u, v, 80, 80, 2, s + 2, 0.6f);
    const float pit = Smooth(0.30f, 0.45f, pores);

    // Air bubbles trapped against the formwork leave round bowls, in some of the cells only.
    const Cellular bubbles = CellularNoise(u, v, 36, 36, s + 3);
    const float bubbleRadius = 0.10f + 0.15f * HashToUnit(bubbles.cellHash);
    const bool hasBubble = HashToUnit(Hash(bubbles.cellHash, 5u, s)) > 0.82f;
    const float bowl = hasBubble ? 1.0f - Smooth(0.0f, bubbleRadius, bubbles.nearest) : 0.0f;

    // Seams where the formwork boards met.
    const float seamPosition = Fraction(v * 4.0f);
    const float groove = 1.0f - Smooth(0.0f, 0.008f, std::min(seamPosition, 1.0f - seamPosition));

    Sample sample;
    sample.height = 0.6f + 0.18f * base + 0.05f * Fbm(u, v, 256, 256, 3, s + 4) - 0.12f * pit - 0.35f * bowl - 0.2f * groove +
                    0.03f * MicroDetail(c, u, v);

    glm::vec3 colour = Srgb(0.55f, 0.55f, 0.53f) * (0.85f + 0.25f * Unit(base));
    const float stain = Smooth(0.1f, 0.6f, Fbm(u, v, 12, 3, 4, s + 5)) * c.wear;  // water runs down the wall
    colour *= 1.0f - 0.3f * stain;
    colour *= 1.0f - 0.3f * pit - 0.3f * bowl;
    sample.albedo = colour;
    sample.roughness = 0.85f + 0.1f * Unit(Fbm(u, v, 20, 20, 3, s + 6));
    return sample;
}

Sample PlasterSample(const Context& c, float u, float v) {
    const uint32_t s = c.seed * 977u;  // far apart, so the noise of neighbouring seeds shares no layer
    const float trowel = Fbm(u, v, 3, 10, 3, s + 1);

    // Paint comes off where the noise is high, and more of it the older the wall.
    const float chipThreshold = 0.62f - 0.25f * c.wear;
    const float chip = Smooth(chipThreshold, chipThreshold + 0.08f, Fbm(u, v, 5, 5, 4, s + 2));

    const Cellular cracks = CellularNoise(u, v, 4, 4, s + 3);
    const float crack = (1.0f - Smooth(0.0f, 0.02f, cracks.second - cracks.nearest)) * Smooth(0.1f, 0.4f, Fbm(u, v, 3, 3, 3, s + 4)) * c.wear;

    Sample sample;
    sample.height = 0.7f + 0.05f * trowel + 0.015f * Fbm(u, v, 128, 128, 3, s + 5) - 0.25f * chip - 0.1f * crack +
                    0.03f * MicroDetail(c, u, v);

    const glm::vec3 paints[4] = {Srgb(0.82f, 0.76f, 0.64f), Srgb(0.72f, 0.78f, 0.74f), Srgb(0.80f, 0.62f, 0.50f), Srgb(0.88f, 0.87f, 0.82f)};
    const glm::vec3 paint = paints[PickIndex(c, 2, 4)] * (0.95f + 0.1f * Unit(trowel));
    const glm::vec3 undercoat = glm::mix(Srgb(0.55f, 0.50f, 0.45f), Srgb(0.50f, 0.30f, 0.25f), Unit(Fbm(u, v, 8, 8, 2, s + 6)));
    sample.albedo = glm::mix(paint, undercoat, chip) * (1.0f - 0.5f * crack);
    sample.roughness = Mix(0.75f, 0.95f, chip);
    return sample;
}

Sample WoodSample(const Context& c, float u, float v) {
    constexpr int kPlanks = 4;
    const uint32_t s = c.seed * 977u;  // far apart, so the noise of neighbouring seeds shares no layer
    const int plank = WrapIndex(static_cast<int>(std::floor(u * kPlanks)), kPlanks);
    const float acrossPlank = Fraction(u * kPlanks);
    const uint32_t plankHash = Hash(static_cast<uint32_t>(plank), s, 11u);
    const float tone = HashToUnit(plankHash);
    const float ringOffset = HashToUnit(Hash(plankHash, 1u, s)) * 10.0f;
    const float ringsPerPlank = 4.0f + 6.0f * HashToUnit(Hash(plankHash, 2u, s));

    // Annual rings: lines across the plank, bent along its length by slow noise.
    const float bend = Fbm(u, v, 6, 2, 3, s + 5) + 0.15f * Fbm(u, v, 40, 3, 2, s + 7);
    const float ring = Fraction((acrossPlank + ringOffset + 0.6f * bend) * ringsPerPlank);
    const float lateWood = ring * ring * ring;

    // Pores: short dark dashes along the grain.
    const float pore = Smooth(0.35f, 0.5f, Fbm(u, v, kPlanks * 120, 6, 2, s + 6, 0.6f));

    // Gaps between the planks.
    const float edgeDistance = std::min(acrossPlank, 1.0f - acrossPlank) / kPlanks;
    const float groove = 1.0f - Smooth(0.0015f, 0.004f, edgeDistance);

    Sample sample;
    sample.height = 0.6f + 0.04f * lateWood - 0.08f * pore - 0.35f * groove + 0.01f * Fbm(u, v, 200, 8, 2, s + 8) +
                    0.015f * MicroDetail(c, u, v);

    glm::vec3 colour = glm::mix(Srgb(0.62f, 0.42f, 0.24f), Srgb(0.35f, 0.20f, 0.10f), tone * (0.6f + 0.4f * SeedChoice(c, 3)));
    colour *= 1.0f - 0.35f * lateWood;
    colour *= 1.0f - 0.4f * pore;
    colour = glm::mix(colour, Srgb(0.08f, 0.05f, 0.03f), groove);
    sample.albedo = colour;
    sample.roughness = 0.5f + 0.25f * pore + 0.1f * c.wear;
    return sample;
}

Sample MetalSample(const Context& c, float u, float v) {
    const uint32_t s = c.seed * 977u;  // far apart, so the noise of neighbouring seeds shares no layer
    // Brushing leaves fine lines along one direction, scratches are longer and sparser.
    const float brushed = Fbm(u, v, 8, 300, 2, s + 1);
    const float scratch = Smooth(0.55f, 0.75f, Fbm(u, v, 6, 500, 1, s + 2)) * Smooth(0.0f, 0.4f, Fbm(u, v, 3, 3, 2, s + 3));
    const float dent = Fbm(u, v, 3, 3, 4, s + 4);

    // Rust spreads from a few places over the metal, further with age, and eats pits into it.
    const float rustThreshold = 0.55f - 0.5f * c.wear;
    const float rustField = Fbm(u, v, 4, 4, 5, s + 5) + 0.35f * Fbm(u, v, 24, 24, 3, s + 6);
    const float rust = Smooth(rustThreshold, rustThreshold + 0.12f, rustField);
    const float rustPits = Unit(Fbm(u, v, 64, 64, 4, s + 7));

    Sample sample;
    sample.height = 0.5f + 0.02f * brushed + 0.03f * dent - 0.04f * std::min(scratch, 1.0f) + rust * (0.08f * rustPits - 0.06f) +
                    0.02f * MicroDetail(c, u, v);

    const glm::vec3 steel = glm::vec3(0.60f, 0.61f, 0.62f) * (0.9f + 0.1f * Unit(brushed)) * (1.0f + 0.15f * std::min(scratch, 1.0f));
    const glm::vec3 rustColour = Srgb(0.45f, 0.20f, 0.08f) * (0.7f + 0.6f * rustPits);
    sample.albedo = glm::mix(steel, rustColour, rust);
    sample.metallic = 1.0f - rust;
    sample.roughness = Mix(0.3f + 0.15f * Unit(brushed) + 0.1f * std::min(scratch, 1.0f), 0.85f, rust);
    return sample;
}

Sample LeatherSample(const Context& c, float u, float v) {
    const uint32_t s = c.seed * 977u;  // far apart, so the noise of neighbouring seeds shares no layer
    // Small pebbled cells with creases between them, and a few deeper folds.
    const Cellular grain = CellularNoise(u + 0.01f * Fbm(u, v, 8, 8, 2, s + 8), v, 24, 24, s + 1);
    const float cellSize = 0.12f + 0.2f * HashToUnit(grain.cellHash);  // cells differ in how soft their crease is
    const float pebble = Smooth(0.0f, cellSize + 0.15f, grain.second - grain.nearest);
    const Cellular folds = CellularNoise(u, v, 5, 5, s + 2);
    const float fold = 1.0f - Smooth(0.0f, 0.06f, folds.second - folds.nearest);
    const float pore = Smooth(0.4f, 0.55f, Fbm(u, v, 200, 200, 2, s + 3));

    Sample sample;
    sample.height = 0.55f + 0.25f * pebble - 0.2f * fold * (0.4f + 0.6f * c.wear) - 0.05f * pore + 0.03f * Fbm(u, v, 3, 3, 3, s + 4) +
                    0.02f * MicroDetail(c, u, v);

    const glm::vec3 hides[3] = {Srgb(0.30f, 0.17f, 0.10f), Srgb(0.10f, 0.08f, 0.07f), Srgb(0.55f, 0.38f, 0.22f)};
    glm::vec3 colour = hides[PickIndex(c, 4, 3)];
    colour *= 0.8f + 0.3f * pebble;
    colour *= 1.0f - 0.3f * fold - 0.2f * pore;
    sample.albedo = colour;
    sample.roughness = 0.45f + 0.25f * (1.0f - pebble) + 0.1f * pore;
    return sample;
}

Sample FabricSample(const Context& c, float u, float v) {
    constexpr int kThreads = 64;
    const uint32_t s = c.seed * 977u;  // far apart, so the noise of neighbouring seeds shares no layer
    const float x = u * kThreads;
    const float y = v * kThreads;
    const int column = WrapIndex(static_cast<int>(std::floor(x)), kThreads);
    const int row = WrapIndex(static_cast<int>(std::floor(y)), kThreads);
    const float insideX = Fraction(x);
    const float insideY = Fraction(y);
    const bool warpOnTop = ((column + row) & 1) == 0;  // plain weave: the vertical and the horizontal thread take turns

    const float warpTone = HashToUnit(Hash(static_cast<uint32_t>(column), s, 21u));
    const float weftTone = HashToUnit(Hash(static_cast<uint32_t>(row), s, 22u));
    // Uneven thickness along each thread.
    const float warpSlub = Fbm(u, v, kThreads, 4, 2, s + 1);
    const float weftSlub = Fbm(u, v, 4, kThreads, 2, s + 2);

    const float acrossWarp = std::sin(kPi * insideX);
    const float acrossWeft = std::sin(kPi * insideY);
    Sample sample;
    // The thread on top arches over the crossing, and the one below it dips away.
    if (warpOnTop) {
        sample.height = 0.55f + 0.35f * acrossWarp * (0.65f + 0.35f * acrossWeft) * (0.85f + 0.15f * warpSlub);
    } else {
        sample.height = 0.30f + 0.20f * acrossWeft * (0.65f + 0.35f * acrossWarp) * (0.85f + 0.15f * weftSlub);
    }
    sample.height += 0.02f * Fbm(u, v, 512, 512, 2, s + 3) + 0.01f * MicroDetail(c, u, v);

    struct Palette {
        glm::vec3 warp;
        glm::vec3 weft;
    };
    const Palette palettes[3] = {{Srgb(0.12f, 0.18f, 0.35f), Srgb(0.70f, 0.72f, 0.75f)},   // denim
                                 {Srgb(0.75f, 0.68f, 0.55f), Srgb(0.72f, 0.65f, 0.52f)},   // linen
                                 {Srgb(0.45f, 0.10f, 0.10f), Srgb(0.40f, 0.09f, 0.09f)}};  // red wool
    const Palette& palette = palettes[PickIndex(c, 5, 3)];
    const glm::vec3 warpColour = palette.warp * (0.85f + 0.3f * warpTone);
    const glm::vec3 weftColour = palette.weft * (0.85f + 0.3f * weftTone);
    sample.albedo = warpOnTop ? warpColour : weftColour;
    sample.roughness = 0.88f;
    return sample;
}

// One layer of stones scattered in cells: how far into a stone this point is (0 outside, 1 at the centre).
float StoneLayer(const Context& c, float u, float v, int cells, float density, float minRadius, float maxRadius, uint32_t salt,
                 float& toneOut) {
    const float raggedU = u + 0.15f / static_cast<float>(cells) * Fbm(u, v, cells, cells, 2, salt + 40);
    const float raggedV = v + 0.15f / static_cast<float>(cells) * Fbm(u, v, cells, cells, 2, salt + 41);
    const Cellular stones = CellularNoise(raggedU, raggedV, cells, cells, salt);
    toneOut = HashToUnit(Hash(stones.cellHash, 7u, c.seed));
    if (HashToUnit(Hash(stones.cellHash, 5u, c.seed)) > density) {
        return 0.0f;
    }
    const float radius = minRadius + (maxRadius - minRadius) * HashToUnit(stones.cellHash);
    const float inside = 1.0f - stones.nearest / radius;
    return inside > 0.0f ? std::sqrt(std::min(inside * 1.6f, 1.0f)) : 0.0f;  // a dome, steep at the rim
}

Sample AsphaltSample(const Context& c, float u, float v) {
    const uint32_t s = c.seed * 977u;  // far apart, so the noise of neighbouring seeds shares no layer
    float largeTone;
    float smallTone;
    const float large = StoneLayer(c, u, v, 20, 0.5f, 0.25f, 0.55f, s + 1, largeTone);
    const float small = StoneLayer(c, u, v, 70, 0.55f, 0.2f, 0.5f, s + 2, smallTone);
    const float stone = std::max(large, small);
    const float tone = large >= small ? largeTone : smallTone;

    const float crack = Smooth(0.93f, 0.97f, RidgedFbm(u, v, 3, 4, s + 9)) * c.wear;

    Sample sample;
    sample.height = 0.3f + 0.05f * Fbm(u, v, 200, 200, 2, s + 3) + stone * 0.35f - 0.25f * crack + 0.03f * MicroDetail(c, u, v);

    // Sun and rain fade the binder from black towards grey, and wear lays the stones bare.
    const glm::vec3 binder = Srgb(0.10f, 0.10f, 0.10f) * (1.0f + 0.8f * c.wear) * (0.9f + 0.2f * Unit(Fbm(u, v, 12, 12, 3, s + 4)));
    const float stoneTone = 0.25f + 0.3f * tone;
    const glm::vec3 stoneColour = Srgb(stoneTone * 1.05f, stoneTone, stoneTone * 0.92f);
    const float visible = Smooth(0.0f, 0.3f, stone);  // the edge of a stone is still coated in binder
    sample.albedo = glm::mix(binder, stoneColour, visible * (0.6f + 0.4f * c.wear)) * (1.0f - 0.5f * crack);
    sample.roughness = Mix(0.92f, 0.75f, visible);
    return sample;
}

Sample SoilSample(const Context& c, float u, float v) {
    const uint32_t s = c.seed * 977u;  // far apart, so the noise of neighbouring seeds shares no layer
    const Cellular clods = CellularNoise(u, v, 9, 9, s + 1);
    const float clod = 1.0f - Smooth(0.0f, 0.8f, clods.nearest);
    const float mid = Fbm(u, v, 12, 12, 4, s + 2);

    const Cellular pebbles = CellularNoise(u, v, 14, 14, s + 3);
    const bool hasPebble = HashToUnit(Hash(pebbles.cellHash, 5u, s)) > 0.8f;
    const float pebbleRadius = 0.12f + 0.3f * HashToUnit(Hash(pebbles.cellHash, 6u, s));
    const float pebble = hasPebble ? std::sqrt(std::max(0.0f, 1.0f - pebbles.nearest / pebbleRadius)) : 0.0f;

    // Dry earth cracks into plates.
    const Cellular plates = CellularNoise(u, v, 5, 5, s + 4);
    const float crack = (1.0f - Smooth(0.0f, 0.03f, plates.second - plates.nearest)) * Smooth(-0.2f, 0.3f, Fbm(u, v, 3, 3, 3, s + 5)) *
                        (0.3f + 0.7f * c.wear);

    Sample sample;
    sample.height = 0.35f + 0.2f * clod + 0.15f * mid + 0.1f * Fbm(u, v, 160, 160, 3, s + 6) + 0.35f * pebble - 0.25f * crack +
                    0.03f * MicroDetail(c, u, v);

    const glm::vec3 damp = Srgb(0.30f, 0.20f, 0.12f);
    const glm::vec3 dry = Srgb(0.48f, 0.38f, 0.27f);
    glm::vec3 colour = glm::mix(damp, dry, Unit(Fbm(u, v, 6, 6, 4, s + 7)));
    colour = glm::mix(colour, Srgb(0.45f, 0.42f, 0.38f), pebble);
    sample.albedo = colour * (1.0f - 0.4f * crack);
    sample.roughness = Mix(0.95f, 0.7f, pebble);
    return sample;
}

Sample SampleAt(MaterialKind kind, const Context& c, float u, float v) {
    switch (kind) {
    case MaterialKind::Stone: return StoneSample(c, u, v);
    case MaterialKind::Brick: return BrickSample(c, u, v);
    case MaterialKind::Concrete: return ConcreteSample(c, u, v);
    case MaterialKind::Plaster: return PlasterSample(c, u, v);
    case MaterialKind::Wood: return WoodSample(c, u, v);
    case MaterialKind::Metal: return MetalSample(c, u, v);
    case MaterialKind::Leather: return LeatherSample(c, u, v);
    case MaterialKind::Fabric: return FabricSample(c, u, v);
    case MaterialKind::Asphalt: return AsphaltSample(c, u, v);
    case MaterialKind::Soil: return SoilSample(c, u, v);
    }
    return {};
}

// Finishing -------------------------------------------------------------------------------------

// Runs body(row) for every row, on the job system when there is one.
template <typename Body>
void ForEachRow(JobSystem* jobs, int rows, const Body& body) {
    const auto run = [&](size_t begin, size_t end) {
        for (size_t row = begin; row < end; ++row) {
            body(static_cast<int>(row));
        }
    };
    if (jobs != nullptr) {
        jobs->ParallelFor(static_cast<size_t>(rows), 4, run);
    } else {
        run(0, static_cast<size_t>(rows));
    }
}

// A box blur of the whole tile that wraps round the edges, so it keeps the tile seamless.
std::vector<float> BlurWrapped(const std::vector<float>& image, int size, int radius, JobSystem* jobs) {
    const float weight = 1.0f / static_cast<float>(2 * radius + 1);
    std::vector<float> across(image.size());
    ForEachRow(jobs, size, [&](int y) {
        const float* in = &image[static_cast<size_t>(y) * size];
        float* out = &across[static_cast<size_t>(y) * size];
        float sum = 0.0f;
        for (int offset = -radius; offset <= radius; ++offset) {
            sum += in[WrapIndex(offset, size)];
        }
        for (int x = 0; x < size; ++x) {
            out[x] = sum * weight;
            sum += in[WrapIndex(x + radius + 1, size)] - in[WrapIndex(x - radius, size)];
        }
    });
    std::vector<float> result(image.size());
    // Columns are handed out as "rows" of the transposed picture.
    ForEachRow(jobs, size, [&](int x) {
        float sum = 0.0f;
        for (int offset = -radius; offset <= radius; ++offset) {
            sum += across[static_cast<size_t>(WrapIndex(offset, size)) * size + x];
        }
        for (int y = 0; y < size; ++y) {
            result[static_cast<size_t>(y) * size + x] = sum * weight;
            sum += across[static_cast<size_t>(WrapIndex(y + radius + 1, size)) * size + x] -
                   across[static_cast<size_t>(WrapIndex(y - radius, size)) * size + x];
        }
    });
    return result;
}

}  // namespace

const char* KindName(MaterialKind kind) {
    switch (kind) {
    case MaterialKind::Stone: return "stone";
    case MaterialKind::Brick: return "brick";
    case MaterialKind::Concrete: return "concrete";
    case MaterialKind::Plaster: return "plaster";
    case MaterialKind::Wood: return "wood";
    case MaterialKind::Metal: return "metal";
    case MaterialKind::Leather: return "leather";
    case MaterialKind::Fabric: return "fabric";
    case MaterialKind::Asphalt: return "asphalt";
    case MaterialKind::Soil: return "soil";
    }
    return "stone";
}

bool ParseKind(const std::string& name, MaterialKind& kind) {
    for (int i = 0; i < kMaterialKindCount; ++i) {
        const MaterialKind candidate = static_cast<MaterialKind>(i);
        if (name == KindName(candidate)) {
            kind = candidate;
            return true;
        }
    }
    return false;
}

MaterialMaps GenerateMaterial(const MaterialSettings& settings, JobSystem* jobs) {
    const int size = std::clamp(settings.size, 128, 4096);
    const size_t pixels = static_cast<size_t>(size) * size;
    const Dimensions dimensions = DimensionsOf(settings.kind);
    const Context context{settings.seed, ClampUnit(settings.wear), ClampUnit(settings.scanDetail), size / 8};

    MaterialMaps maps;
    maps.size = size;
    maps.tileMeters = dimensions.tileMeters;
    maps.reliefMeters = dimensions.reliefMeters;
    maps.height.resize(pixels);
    maps.albedo.resize(pixels);
    maps.normal.resize(pixels);
    maps.roughness.resize(pixels);
    maps.metallic.resize(pixels);
    maps.ambientOcclusion.resize(pixels);

    // 1. What the recipe says, point by point.
    ForEachRow(jobs, size, [&](int y) {
        for (int x = 0; x < size; ++x) {
            const Sample sample = SampleAt(settings.kind, context, (static_cast<float>(x) + 0.5f) / size, (static_cast<float>(y) + 0.5f) / size);
            const size_t index = static_cast<size_t>(y) * size + x;
            maps.height[index] = sample.height;
            maps.albedo[index] = sample.albedo;
            maps.roughness[index] = sample.roughness;
            maps.metallic[index] = sample.metallic;
        }
    });

    // 2. Fit the heights to the real relief, and add the scanner's noise: a few percent of a pixel's width.
    const float lowest = *std::min_element(maps.height.begin(), maps.height.end());
    const float highest = *std::max_element(maps.height.begin(), maps.height.end());
    const float pixelMeters = dimensions.tileMeters / static_cast<float>(size);
    const float scanNoise = 0.04f * context.detail * pixelMeters;
    ForEachRow(jobs, size, [&](int y) {
        for (int x = 0; x < size; ++x) {
            const size_t index = static_cast<size_t>(y) * size + x;
            const float noise = HashToUnit(Hash(static_cast<uint32_t>(x), static_cast<uint32_t>(y), settings.seed + 31337u)) - 0.5f;
            const float fitted = (maps.height[index] - lowest) / std::max(highest - lowest, 1e-6f) * dimensions.reliefMeters;
            maps.height[index] = std::clamp(fitted + 2.0f * noise * scanNoise, 0.0f, dimensions.reliefMeters);
        }
    });

    // 3. Normals from the height, by central differences that wrap round the tile.
    ForEachRow(jobs, size, [&](int y) {
        for (int x = 0; x < size; ++x) {
            const auto at = [&](int ix, int iy) { return maps.height[static_cast<size_t>(WrapIndex(iy, size)) * size + WrapIndex(ix, size)]; };
            const float slopeX = (at(x + 1, y) - at(x - 1, y)) / (2.0f * pixelMeters);
            const float slopeUp = (at(x, y - 1) - at(x, y + 1)) / (2.0f * pixelMeters);  // rows run downwards
            maps.normal[static_cast<size_t>(y) * size + x] = glm::normalize(glm::vec3(-slopeX, -slopeUp, 1.0f));
        }
    });

    // 4. Cavity and convexity from the difference between the height and its blur at a few scales.
    const float scale = static_cast<float>(size) / 1024.0f;
    const auto radiusAt = [&](float pixelsAt1024) { return std::max(1, static_cast<int>(std::lround(pixelsAt1024 * scale))); };
    std::vector<float> cavity(pixels, 0.0f);
    const float cavityRadii[3] = {2.0f, 5.0f, 12.0f};
    for (const float radius : cavityRadii) {
        const std::vector<float> blurred = BlurWrapped(maps.height, size, radiusAt(radius), jobs);
        for (size_t i = 0; i < pixels; ++i) {
            cavity[i] += std::max(0.0f, blurred[i] - maps.height[i]) / dimensions.reliefMeters / 3.0f;
        }
    }
    const std::vector<float> edgeBlur = BlurWrapped(maps.height, size, radiusAt(3.0f), jobs);

    // 5. Let the colour follow the shape: dirt in the pits, bright and smooth edges where the surface is worn.
    ForEachRow(jobs, size, [&](int y) {
        for (int x = 0; x < size; ++x) {
            const size_t index = static_cast<size_t>(y) * size + x;
            const float occlusion = glm::clamp(1.0f - 3.0f * cavity[index], 0.1f, 1.0f);
            const float convexity = glm::clamp(3.0f * std::max(0.0f, maps.height[index] - edgeBlur[index]) / dimensions.reliefMeters, 0.0f, 1.0f);
            maps.ambientOcclusion[index] = occlusion;

            const float dirt = (1.0f - occlusion) * (0.4f + 0.4f * context.wear);
            glm::vec3 albedo = maps.albedo[index] * (1.0f - dirt);
            albedo += glm::vec3(0.12f) * convexity * context.wear;
            maps.albedo[index] = glm::clamp(albedo, glm::vec3(0.02f), glm::vec3(0.95f));

            const float roughness = maps.roughness[index] - 0.12f * convexity * context.wear + 0.1f * (1.0f - occlusion) * context.wear;
            maps.roughness[index] = glm::clamp(roughness, 0.05f, 1.0f);
            maps.metallic[index] = glm::clamp(maps.metallic[index], 0.0f, 1.0f);
        }
    });
    return maps;
}

}  // namespace material
