#include "animation/Personality.h"

#include <algorithm>

namespace animation {

Personality PersonalityFrom(const character::CharacterParameters& parameters, const Mood& mood) {
    const character::CharacterParameters clean = character::Sanitised(parameters);
    const float maleness = clean.masculinity - 0.5f;                                  // -0.5 to 0.5
    const float ageness = std::clamp((clean.ageYears - 30.0f) / 50.0f, 0.0f, 1.0f);  // 0 up to 30, 1 at 80
    const float youth = std::clamp((30.0f - clean.ageYears) / 12.0f, 0.0f, 1.0f);    // 0 from 30, 1 at 18
    const float heaviness = std::clamp((character::BodyMassIndex(clean) - 22.0f) / 10.0f, -0.7f, 2.3f);
    const float energy = std::clamp(mood.energy, -1.0f, 1.0f);
    const float confidence = std::clamp(mood.confidence, -1.0f, 1.0f);

    Personality personality;
    personality.strideScale = (1.0f + 0.06f * maleness) * (1.0f - 0.18f * ageness) * (1.0f - 0.05f * heaviness) * (1.0f + 0.10f * energy + 0.04f * confidence);
    personality.tempoScale = (1.0f - 0.06f * ageness) * (1.0f + 0.08f * energy + 0.04f * youth);
    personality.armSwingScale = (1.0f + 0.12f * maleness) * (1.0f - 0.40f * ageness) * (1.0f + 0.30f * energy + 0.10f * confidence);
    personality.pelvisRotationScale = (1.0f - 0.30f * maleness) * (1.0f - 0.20f * ageness);
    personality.pelvisSwayScale = (1.0f - 0.40f * maleness) * (1.0f + 0.30f * heaviness) * (1.0f + 0.15f * ageness);
    personality.stepWidthScale = std::clamp(1.0f + 0.12f * heaviness + 0.08f * ageness - 0.05f * maleness, 0.9f, 1.5f);
    personality.footClearanceScale = (1.0f - 0.45f * ageness) * (1.0f + 0.20f * energy) * (1.0f - 0.10f * std::max(heaviness, 0.0f));
    personality.dutyFactorOffset = 0.035f * ageness + 0.015f * std::max(heaviness, 0.0f) - 0.01f * energy;
    personality.bounceScale = 1.0f + 0.25f * energy;
    personality.stoopDegrees = 9.0f * ageness - 5.0f * confidence - 2.0f * energy;
    personality.headDropDegrees = 6.0f * ageness - 4.0f * confidence;
    personality.standingHeightScale = 1.0f - 0.010f * ageness - 0.012f * std::max(-energy, 0.0f);
    return personality;
}

}  // namespace animation
