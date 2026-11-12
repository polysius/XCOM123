#pragma once

// Building blocks shared by the parts of the city generator: deterministic random numbers,
// rectangle arithmetic, the box writer and the seed encodings that shaders/lit.frag reads.
// Internal to src/world; the rest of the engine only sees CityGenerator.h.

#include <cstdint>
#include <random>
#include <vector>

#include <glm/glm.hpp>

#include "world/CityGenerator.h"

namespace city {

// Heights in metres. The street surface is at y = 0.
inline constexpr float kPavementHeight = 0.15f;      // walkers stand at this height
inline constexpr float kDistrictAsphaltRaise = 0.01f;  // see CitySettings::districtOnly
inline constexpr float kPlazaHeight = 0.20f;
inline constexpr float kLawnHeight = 0.24f;
inline constexpr float kGroundFloorHeight = 4.5f;    // must match GROUND_FLOOR_HEIGHT in lit.frag
inline constexpr float kUpperFloorHeight = 3.3f;     // must match UPPER_FLOOR_HEIGHT in lit.frag
inline constexpr float kWaterLevel = -4.5f;      // the river surface, well below the quays
inline constexpr float kQuayWallTop = 1.0f;
// Anything that overhangs a pavement (balconies, awnings, cornices) starts at least this high,
// so people walking underneath never clip into it.
inline constexpr float kOverhangClearance = 3.0f;

// Random numbers built only on std::mt19937, whose output the C++ standard fixes exactly.
// (The standard distributions may differ between compilers, which would change the city.)
class Random {
public:
    explicit Random(uint32_t seed) : engine(seed) {}

    float Unit() { return static_cast<float>(engine() >> 8) * (1.0f / 16777216.0f); }  // 0 to just below 1
    float Range(float min, float max) { return min + (max - min) * Unit(); }
    int Int(int min, int max) { return min + static_cast<int>(engine() % static_cast<uint32_t>(max - min + 1)); }  // inclusive
    bool Chance(float probability) { return Unit() < probability; }
    float Sign() { return Chance(0.5f) ? 1.0f : -1.0f; }
    uint32_t Bits() { return static_cast<uint32_t>(engine()); }

    template <typename T, size_t N>
    const T& Pick(const T (&items)[N]) { return items[engine() % N]; }

private:
    std::mt19937 engine;
};

// Combines a seed with a salt into a new, well-mixed seed. Each stage of the generator draws
// from its own stream, so changing one stage does not reshuffle every other one.
uint32_t MixSeed(uint32_t seed, uint32_t salt);

// Sides of an axis-aligned rectangle, as bit flags.
enum Side : uint8_t {
    kSideWest = 1,   // -X
    kSideEast = 2,   // +X
    kSideSouth = 4,  // -Z
    kSideNorth = 8,  // +Z
};
inline constexpr uint8_t kAllSides = kSideWest | kSideEast | kSideSouth | kSideNorth;

// Rectangle helpers. Rectangles with no area count as empty.
Rect Intersection(const Rect& a, const Rect& b);
Rect ExpandSides(const Rect& rect, uint8_t sides, float amount);  // negative amounts shrink
Rect InsetRect(const Rect& rect, float west, float east, float south, float north);
// `area` with every rectangle in `cuts` removed, as a list of non-overlapping rectangles.
std::vector<Rect> Subtract(const Rect& area, const std::vector<Rect>& cuts);
// The rectangle covered by an axis-aligned segment widened by `halfWidth` to each side and
// lengthened by `extend` at both ends.
Rect SegmentRect(glm::vec2 a, glm::vec2 b, float halfWidth, float extend = 0.0f);

// Lines and rectangles handled the same way whichever way they run. "Along" is the direction a
// street runs, "across" the direction perpendicular to it; `alongX` is true for streets at a
// constant Z.
inline glm::vec2 OnLine(bool alongX, float along, float across) {
    return alongX ? glm::vec2(along, across) : glm::vec2(across, along);
}
inline float AlongMin(const Rect& rect, bool alongX) { return alongX ? rect.min.x : rect.min.y; }
inline float AlongMax(const Rect& rect, bool alongX) { return alongX ? rect.max.x : rect.max.y; }
inline float AcrossMin(const Rect& rect, bool alongX) { return alongX ? rect.min.y : rect.min.x; }
inline float AcrossMax(const Rect& rect, bool alongX) { return alongX ? rect.max.y : rect.max.x; }
inline Rect FromRanges(bool alongX, float alongMin, float alongMax, float acrossMin, float acrossMax) {
    return {OnLine(alongX, alongMin, acrossMin), OnLine(alongX, alongMax, acrossMax)};
}

// A ground position lifted to height `y`.
inline glm::vec3 AtHeight(glm::vec2 point, float y) {
    return {point.x, y, point.y};
}

// How a box looks: base colour, shader material, roughness and the per-object seed.
struct Look {
    glm::vec3 color;
    SurfaceMaterial material;
    float roughness;
    float seed = 0.0f;
};

// Appends boxes to the layout and tags each with the part it belongs to.
class BoxWriter {
public:
    explicit BoxWriter(CityLayout& layout) : city(layout) {}

    void Box(glm::vec3 min, glm::vec3 max, const Look& look);
    // A box turned about the vertical axis. `center` is its middle and `halfSize` is measured along
    // its own axes; `yaw` has the sign convention of BoxInstance::yaw.
    void TurnedBox(glm::vec3 center, glm::vec3 halfSize, float yaw, const Look& look);
    // A box standing on a ground footprint, spanning heights `bottom` to `top`.
    void Footprint(const Rect& footprint, float bottom, float top, const Look& look);

    CityPart part = CityPart::Ground;  // tag given to the boxes added from now on

private:
    CityLayout& city;
};

// Sets the part tag of a writer for one scope, then restores the previous one.
class PartScope {
public:
    PartScope(BoxWriter& writer, CityPart part) : boxes(writer), previous(writer.part) { writer.part = part; }
    ~PartScope() { boxes.part = previous; }
    PartScope(const PartScope&) = delete;
    PartScope& operator=(const PartScope&) = delete;

private:
    BoxWriter& boxes;
    CityPart previous;
};

// Seed encodings read by shaders/lit.frag. The integer part selects a pattern variant and the
// fraction is a random value that varies the detail. Keep them in sync with the shader.
enum class GroundPattern : uint32_t {
    Asphalt = 0,
    Slabs = 1,    // pavement flagstones
    Cobbles = 2,  // setts on squares and old streets
    Gravel = 3,   // park paths and boulevard medians
};

enum FacadeFlag : uint32_t {
    kFacadeNoShops = 1,      // plain ground floor windows instead of shop fronts
    kFacadeUpperOnly = 2,    // no ground floor at all: attic storeys, towers, dormers
    kFacadeNarrowBays = 4,   // 2.4 m window bays
    kFacadeWideBays = 8,     // 4.2 m window bays
    kFacadeShutters = 16,    // painted shutters beside the windows
    kFacadeRusticated = 32,  // grooved stone ground floor
    kFacadeArched = 64,      // round-headed windows and shop openings, with a stone ring and keystone
    kFacadeStucco = 128,     // smooth hand-troweled plaster with hairline cracks and a damp foot
};

enum RoofFlag : uint32_t {
    kRoofDormers = 1,  // a row of dormer windows on the vertical faces
};

enum GlassFlag : uint32_t {
    kGlassStained = 1,  // coloured church glass that glows at night
};

enum MetalFlag : uint32_t {
    kMetalRailing = 1,   // vertical balusters
    kMetalVerdigris = 2, // weathered copper gone blue green, with rain streaks
};

inline float PatternSeed(uint32_t variant, float random01) {
    return static_cast<float>(variant) + glm::clamp(random01, 0.0f, 0.999f);
}
inline float GroundSeed(GroundPattern pattern, float random01 = 0.5f) {
    return PatternSeed(static_cast<uint32_t>(pattern), random01);
}

// Common looks.
Look AsphaltLook();
Look PavingLook();
Look CobbleLook(float random01);
Look GravelLook();
Look StoneLook(glm::vec3 color, float random01 = 0.5f);
Look GrassLook(float random01);
Look WaterLook();

}  // namespace city
