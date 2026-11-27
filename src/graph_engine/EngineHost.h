#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include <glm/glm.hpp>

// What a graph can ask of the engine. The engine nodes (EngineNodes.h) call these through NodeContext::Host(), and the engine
// implements them for the live world, so that the graph code never includes the renderer, the physics or the crowd, and a test
// can stand in for the whole engine with a recording host.
//
// Positions are world metres (Y up). Angles are degrees. A host must accept any value the nodes pass, but the nodes pass only
// values that are finite and inside the limits below (see SanitisedCharacter and the Clamp functions).

namespace graph_engine {

enum class PropKind { Crate, Barrel, Ball };

inline const char* PropKindName(PropKind kind) {
    switch (kind) {
    case PropKind::Crate: return "crate";
    case PropKind::Barrel: return "barrel";
    case PropKind::Ball: return "ball";
    }
    return "prop";
}

// A person to make, in the numbers the character generator takes. Outfit names clothes (the generator has no clothes yet; the
// live engine uses the name to colour a stand-in).
struct CharacterSpec {
    float statureMeters = 1.70f;
    float massKg = 70.0f;
    float ageYears = 30.0f;
    float masculinity = 0.5f;  // 0 = a female body, 1 = a male one
    float muscularity = 0.4f;
    uint32_t seed = 1;
    std::string outfit;
};

// ---------------------------------------------------------------------------------------------------------------------
// Limits. Every node that reaches the host keeps its arguments inside these, and replaces a number that is not finite by the
// fallback it is given.
// ---------------------------------------------------------------------------------------------------------------------

inline constexpr float kSunElevationMin = -5.0f;  // as the interactive loop keeps it: a little below the horizon to 85 degrees
inline constexpr float kSunElevationMax = 85.0f;
inline constexpr float kWorldHalfExtent = 5000.0f;  // metres from the origin along X and Z
inline constexpr float kWorldHeightMin = -100.0f;
inline constexpr float kWorldHeightMax = 2000.0f;
inline constexpr float kMaxImpulse = 500.0f;  // newton seconds, about a hard kick of a ball
inline constexpr int kMaxCrowdSize = 20000;
inline constexpr float kRiverLevelRange = 5.0f;  // metres above or below the still level
inline constexpr float kWaveAmplitudeMax = 2.0f;
inline constexpr float kWaveRadiusMin = 0.5f;
inline constexpr float kWaveRadiusMax = 20.0f;
inline constexpr size_t kMaxTextLength = 200;
inline constexpr size_t kMaxOutfitLength = 32;
inline constexpr float kStatureMin = 1.45f;
inline constexpr float kStatureMax = 2.05f;
inline constexpr float kMassMin = 30.0f;
inline constexpr float kMassMax = 200.0f;
inline constexpr float kAgeMin = 18.0f;
inline constexpr float kAgeMax = 80.0f;

inline float FiniteOr(float value, float fallback) {
    return std::isfinite(value) ? value : fallback;
}

inline float ClampOr(float value, float low, float high, float fallback) {
    return std::clamp(FiniteOr(value, fallback), low, high);
}

// An angle of a circle: any finite number turned into [0, 360).
inline float WrapDegrees(float degrees, float fallback = 0.0f) {
    const float wrapped = std::fmod(FiniteOr(degrees, fallback), 360.0f);
    return wrapped < 0.0f ? wrapped + 360.0f : wrapped;
}

inline glm::vec3 ClampPosition(glm::vec3 position, glm::vec3 fallback = glm::vec3(0.0f)) {
    return {ClampOr(position.x, -kWorldHalfExtent, kWorldHalfExtent, fallback.x),
            ClampOr(position.y, kWorldHeightMin, kWorldHeightMax, fallback.y),
            ClampOr(position.z, -kWorldHalfExtent, kWorldHalfExtent, fallback.z)};
}

inline std::string Truncated(const std::string& text, size_t length) {
    return text.size() <= length ? text : text.substr(0, length);
}

inline CharacterSpec SanitisedCharacter(const CharacterSpec& spec) {
    CharacterSpec clean = spec;
    clean.statureMeters = ClampOr(spec.statureMeters, kStatureMin, kStatureMax, 1.70f);
    clean.massKg = ClampOr(spec.massKg, kMassMin, kMassMax, 70.0f);
    clean.ageYears = ClampOr(spec.ageYears, kAgeMin, kAgeMax, 30.0f);
    clean.masculinity = ClampOr(spec.masculinity, 0.0f, 1.0f, 0.5f);
    clean.muscularity = ClampOr(spec.muscularity, 0.0f, 1.0f, 0.4f);
    clean.outfit = Truncated(spec.outfit, kMaxOutfitLength);
    return clean;
}

// The engine, as a graph sees it.
class EngineHost {
public:
    virtual ~EngineHost() = default;

    // ----- World -----
    virtual void SetSunElevation(float degrees) = 0;  // above the horizon
    virtual void SetSunAzimuth(float degrees) = 0;    // the compass direction of the sun, 0 meaning +Z
    virtual void SetTimeOfDay(float hours) = 0;       // 0 to 24; moves the sun along its day arc

    // ----- Camera and player -----
    virtual glm::vec3 CameraPosition() const = 0;
    virtual glm::vec3 CameraDirection() const = 0;  // a unit vector the camera looks along
    virtual void SetCameraPosition(glm::vec3 position) = 0;
    virtual void SetCameraDirection(glm::vec3 direction) = 0;  // not the zero vector
    virtual void TeleportCamera(glm::vec3 position, glm::vec3 direction) = 0;  // both at once, as a jump
    virtual glm::vec3 PlayerPosition() const = 0;

    // ----- Physics props. An id of 0 means the host refused (too many props, or no ground there). -----
    virtual uint32_t SpawnProp(PropKind kind, glm::vec3 position) = 0;
    virtual bool RemoveProp(uint32_t propId) = 0;
    virtual void ApplyImpulse(uint32_t propId, glm::vec3 impulse) = 0;

    // ----- People. An id of 0 means the host refused. -----
    virtual void SetCrowdSize(int people) = 0;
    virtual int CrowdSize() const = 0;
    virtual uint32_t SpawnCharacter(const CharacterSpec& spec, glm::vec3 position) = 0;

    // ----- Water -----
    virtual void SetRiverLevel(float metresAboveRest) = 0;
    virtual void AddRiverWave(glm::vec3 position, float amplitudeMeters, float radiusMeters) = 0;

    // ----- Debug -----
    virtual void PrintToScreen(const std::string& text) = 0;
    virtual float FrameSeconds() const = 0;
    virtual float FramesPerSecond() const = 0;
    // A whole number to seed a generator with. A host that runs the same way twice gives the same numbers.
    virtual uint32_t NextRandomSeed() = 0;
};

}  // namespace graph_engine
