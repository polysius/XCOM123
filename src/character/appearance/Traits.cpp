#include "character/appearance/Traits.h"

#include <algorithm>
#include <array>
#include <utility>

#include "character/appearance/Noise3D.h"

namespace character {
namespace {

// Every trait draws its own number from the seed, so that changing one never changes another.
enum Salt : uint32_t {
    kSaltMelanin = 101,
    kSaltMelaninSpread,
    kSaltRuddiness,
    kSaltFreckles,
    kSaltMoles,
    kSaltAgeSpots,
    kSaltWrinkles,
    kSaltVeins,
    kSaltIris,
    kSaltHair,
    kSaltGrey,
    kSaltDensity,
    kSaltRecession,
    kSaltStyle,
    kSaltFacialHair,
};

float Draw(const CharacterParameters& parameters, uint32_t salt) {
    return Random01(parameters.seed, salt, 0x51ED270Bu);
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

// A piecewise linear blend through the colours of light, medium and dark skin.
glm::vec3 SkinColourFor(float melanin) {
    const glm::vec3 light(0.60f, 0.37f, 0.28f);
    const glm::vec3 medium(0.40f, 0.24f, 0.15f);
    const glm::vec3 dark(0.13f, 0.075f, 0.050f);
    return melanin < 0.5f ? glm::mix(light, medium, melanin * 2.0f) : glm::mix(medium, dark, melanin * 2.0f - 1.0f);
}

glm::vec3 HairColourFor(float melanin, float draw) {
    // Dark skin goes with dark hair; pale skin with the whole range.
    const glm::vec3 black(0.012f, 0.010f, 0.009f);
    const glm::vec3 darkBrown(0.040f, 0.022f, 0.013f);
    const glm::vec3 brown(0.090f, 0.050f, 0.025f);
    const glm::vec3 lightBrown(0.200f, 0.125f, 0.060f);
    const glm::vec3 blond(0.420f, 0.300f, 0.140f);
    const glm::vec3 red(0.200f, 0.050f, 0.020f);
    if (melanin > 0.6f) {
        return draw < 0.6f ? black : darkBrown;
    }
    const std::array<std::pair<float, glm::vec3>, 6> table = {{{0.20f, black}, {0.25f, darkBrown}, {0.25f, brown}, {0.12f, lightBrown}, {0.12f, blond}, {0.06f, red}}};
    return Pick(table, draw);
}

glm::vec3 IrisColourFor(float melanin, float draw) {
    const glm::vec3 brown(0.10f, 0.045f, 0.018f);
    const glm::vec3 hazel(0.17f, 0.10f, 0.035f);
    const glm::vec3 green(0.07f, 0.16f, 0.06f);
    const glm::vec3 blue(0.08f, 0.17f, 0.38f);
    const glm::vec3 grey(0.20f, 0.23f, 0.26f);
    if (melanin > 0.55f) {
        return brown;
    }
    const std::array<std::pair<float, glm::vec3>, 5> table = {{{0.40f, brown}, {0.14f, hazel}, {0.12f, green}, {0.28f, blue}, {0.06f, grey}}};
    return Pick(table, draw);
}

}  // namespace

PersonTraits DeriveTraits(const CharacterParameters& raw) {
    const CharacterParameters parameters = Sanitised(raw);
    PersonTraits traits;
    const float age = parameters.ageYears;
    const float male = parameters.masculinity;
    const float lateLife = SmoothStep(35.0f, 75.0f, age);

    // Skin tone is spread over the range, with the middle somewhat more common than the ends.
    const float spread = 2.0f * Draw(parameters, kSaltMelanin) - 1.0f;
    traits.melanin = glm::clamp(0.5f + 0.47f * spread * (0.6f + 0.4f * Draw(parameters, kSaltMelaninSpread)), 0.03f, 0.97f);
    traits.skinColour = SkinColourFor(traits.melanin);
    traits.ruddiness = (0.35f + 0.65f * Draw(parameters, kSaltRuddiness)) * (1.0f - 0.45f * traits.melanin) * (0.9f + 0.2f * male);
    const float freckleDraw = Draw(parameters, kSaltFreckles);
    traits.freckles = glm::clamp((1.0f - traits.melanin) * (1.0f - traits.melanin) * freckleDraw * freckleDraw * (1.0f - 0.4f * lateLife) * 1.6f, 0.0f, 1.0f);
    traits.moles = 0.15f + 0.85f * Draw(parameters, kSaltMoles) * (0.5f + 0.5f * SmoothStep(18.0f, 50.0f, age));
    traits.ageSpots = SmoothStep(40.0f, 75.0f, age) * (0.5f + 0.5f * Draw(parameters, kSaltAgeSpots)) * (1.0f - 0.5f * traits.melanin);
    traits.wrinkles = glm::clamp(SmoothStep(22.0f, 78.0f, age) * (0.75f + 0.5f * Draw(parameters, kSaltWrinkles)) * (1.0f - 0.25f * traits.melanin), 0.0f, 1.0f);
    traits.veins = glm::clamp((0.25f + 0.5f * Draw(parameters, kSaltVeins)) * (1.0f - 0.7f * traits.melanin) * (0.5f + 0.5f * lateLife + 0.4f * parameters.muscularity), 0.0f, 1.0f);

    traits.irisColour = IrisColourFor(traits.melanin, Draw(parameters, kSaltIris));
    traits.hairColour = HairColourFor(traits.melanin, Draw(parameters, kSaltHair));
    traits.greyFraction = glm::clamp(SmoothStep(32.0f, 78.0f, age) * (0.6f + 0.6f * Draw(parameters, kSaltGrey)), 0.0f, 1.0f);
    // Men lose hair with age; women thin much less.
    const float thinning = SmoothStep(28.0f, 75.0f, age) * (0.15f + 0.85f * male) * Draw(parameters, kSaltDensity);
    traits.hairDensity = glm::clamp(1.0f - 0.55f * thinning, 0.35f, 1.0f);
    traits.hairlineRecession = glm::clamp(thinning * 1.4f * (0.6f + 0.8f * Draw(parameters, kSaltRecession)), 0.0f, 1.0f);
    return traits;
}

HairStyle ChooseHairStyle(const CharacterParameters& raw) {
    const CharacterParameters parameters = Sanitised(raw);
    const float draw = Draw(parameters, kSaltStyle);
    if (parameters.masculinity >= 0.5f) {
        const float bald = 0.04f + 0.30f * SmoothStep(35.0f, 75.0f, parameters.ageYears);
        const std::array<std::pair<float, HairStyle>, 5> table = {{{bald, HairStyle::Bald}, {0.46f, HairStyle::Short}, {0.30f, HairStyle::Medium}, {0.07f, HairStyle::Long}, {0.07f, HairStyle::Tied}}};
        return Pick(table, draw);
    }
    const std::array<std::pair<float, HairStyle>, 5> table = {{{0.01f, HairStyle::Bald}, {0.16f, HairStyle::Short}, {0.32f, HairStyle::Medium}, {0.30f, HairStyle::Long}, {0.21f, HairStyle::Tied}}};
    return Pick(table, draw);
}

FacialHair ChooseFacialHair(const CharacterParameters& raw) {
    const CharacterParameters parameters = Sanitised(raw);
    if (parameters.masculinity < 0.65f) {
        return FacialHair::None;
    }
    const std::array<std::pair<float, FacialHair>, 3> table = {{{0.45f, FacialHair::None}, {0.27f, FacialHair::Stubble}, {0.28f, FacialHair::Beard}}};
    return Pick(table, Draw(parameters, kSaltFacialHair));
}

}  // namespace character
