#include "character/CharacterParameters.h"

#include <algorithm>
#include <cmath>

namespace character {
namespace {

constexpr float kMinAge = 18.0f;
constexpr float kMaxAge = 80.0f;
constexpr float kMinStature = 1.45f;
constexpr float kMaxStature = 2.05f;
constexpr float kMinBodyMassIndex = 15.0f;
constexpr float kMaxBodyMassIndex = 45.0f;

float FiniteOr(float value, float fallback) {
    return std::isfinite(value) ? value : fallback;
}

// A small deterministic random number generator (SplitMix64). The standard distributions differ between compilers, so
// the numbers are drawn here by hand, which keeps a seed meaning the same person everywhere.
class Random {
public:
    explicit Random(uint32_t seed) : state(0x9E3779B97F4A7C15ull * (static_cast<uint64_t>(seed) + 1ull)) {}

    float Uniform() {
        state += 0x9E3779B97F4A7C15ull;
        uint64_t z = state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        z ^= z >> 31;
        return static_cast<float>(z >> 40) * (1.0f / 16777216.0f);
    }

    // Normally distributed, by the Box-Muller method. The two uniform numbers are drawn in separate statements:
    // the order in which a compiler evaluates the arguments of a call is not defined.
    float Normal(float mean, float deviation) {
        const float first = std::max(Uniform(), 1e-7f);
        const float second = Uniform();
        return mean + deviation * std::sqrt(-2.0f * std::log(first)) * std::cos(6.28318530718f * second);
    }

private:
    uint64_t state;
};

}  // namespace

CharacterParameters Sanitised(const CharacterParameters& parameters) {
    const CharacterParameters defaults;
    CharacterParameters result = parameters;
    result.masculinity = std::clamp(FiniteOr(parameters.masculinity, defaults.masculinity), 0.0f, 1.0f);
    result.ageYears = std::clamp(FiniteOr(parameters.ageYears, defaults.ageYears), kMinAge, kMaxAge);
    result.statureMeters = std::clamp(FiniteOr(parameters.statureMeters, defaults.statureMeters), kMinStature, kMaxStature);
    result.muscularity = std::clamp(FiniteOr(parameters.muscularity, defaults.muscularity), 0.0f, 1.0f);
    const float stature = result.statureMeters;
    result.massKg = std::clamp(FiniteOr(parameters.massKg, defaults.massKg), kMinBodyMassIndex * stature * stature,
                               kMaxBodyMassIndex * stature * stature);
    return result;
}

float BodyMassIndex(const CharacterParameters& parameters) {
    return parameters.massKg / (parameters.statureMeters * parameters.statureMeters);
}

CharacterParameters RandomParameters(uint32_t seed) {
    Random random(seed);
    CharacterParameters parameters;
    parameters.seed = seed;

    // Sex: nearly always one end or the other of the range, with a little spread in how feminine or masculine the build is.
    const bool male = random.Uniform() < 0.5f;
    const float spread = random.Uniform();
    parameters.masculinity = male ? 0.8f + 0.2f * spread : 0.2f * spread;

    // Stature of adults by sex: men 1.756 m with a spread of 0.069 m, women 1.629 m with 0.064 m (ANSUR II).
    const float statureDraw = random.Normal(0.0f, 1.0f);
    parameters.statureMeters = male ? 1.756f + 0.069f * statureDraw : 1.629f + 0.064f * statureDraw;

    // Body mass index: about 26 with a spread of 4.5 and a longer tail towards the heavy end.
    const float massDraw = random.Normal(0.0f, 1.0f);
    const float bodyMassIndex = 26.0f + 4.5f * massDraw + (massDraw > 0.0f ? 1.0f * massDraw : 0.0f);
    const float clampedIndex = std::clamp(bodyMassIndex, 17.0f, 40.0f);
    parameters.massKg = clampedIndex * parameters.statureMeters * parameters.statureMeters;

    // Age from 18 to 75, weighted towards the middle years of a working population.
    const float ageA = random.Uniform();
    const float ageB = random.Uniform();
    parameters.ageYears = 18.0f + 57.0f * 0.5f * (ageA + ageB);

    // Muscularity: more in men, less with age and with high body mass index counted separately by the generator.
    const float muscleDraw = random.Uniform();
    const float ageFactor = 1.0f - 0.5f * std::clamp((parameters.ageYears - 30.0f) / 50.0f, 0.0f, 1.0f);
    parameters.muscularity = std::clamp((0.25f + 0.4f * parameters.masculinity + 0.3f * muscleDraw) * ageFactor, 0.0f, 1.0f);
    return Sanitised(parameters);
}

}  // namespace character
