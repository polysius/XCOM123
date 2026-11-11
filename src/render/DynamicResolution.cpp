#include "render/DynamicResolution.h"

#include <algorithm>
#include <cmath>

namespace {
// GPU times arrive a frame or two late, so the first samples after a change still describe the
// old scale. They are skipped before the controller judges the new scale.
constexpr uint32_t kSettleSamples = 4;
// Weight of the newest sample in the smoothed time. One slow frame must not drop the scale,
// but a lasting rise should within a few frames.
constexpr double kSmoothingWeight = 0.2;
// Smoothed time above budget * this drops two steps at once instead of one.
constexpr double kFarOverBudget = 1.25;
// The time must stay under budget * this for kSamplesBeforeRaise samples before raising.
constexpr double kWellUnderBudget = 0.8;
constexpr uint32_t kSamplesBeforeRaise = 60;
// A raise must be predicted to land under budget * this, keeping headroom after the change.
constexpr double kRaiseTarget = 0.9;

float SnapToStep(float scale) {
    const float snapped = std::round(scale / DynamicResolution::kScaleStep) * DynamicResolution::kScaleStep;
    return std::clamp(snapped, DynamicResolution::kMinScale, DynamicResolution::kMaxScale);
}
}  // namespace

void DynamicResolution::Configure(const ResolutionSettings& newSettings) {
    settings = newSettings;
    ChangeScale(settings.dynamic ? kMaxScale : std::clamp(settings.fixedScale, kMinScale, kMaxScale));
}

void DynamicResolution::AddMeasurement(double gpuFrameMs) {
    if (!settings.dynamic || gpuFrameMs <= 0.0) {
        return;
    }
    ++samplesSinceChange;
    if (samplesSinceChange <= kSettleSamples) {
        return;
    }
    const bool isFirstSampleAtThisScale = samplesSinceChange == kSettleSamples + 1;
    smoothedMs = isFirstSampleAtThisScale ? gpuFrameMs : smoothedMs + (gpuFrameMs - smoothedMs) * kSmoothingWeight;

    const double budget = settings.gpuBudgetMs;
    if (smoothedMs > budget && scale > kMinScale) {
        const int steps = smoothedMs > budget * kFarOverBudget ? 2 : 1;
        ChangeScale(SnapToStep(scale - kScaleStep * static_cast<float>(steps)));
        return;
    }

    samplesWellUnderBudget = smoothedMs < budget * kWellUnderBudget ? samplesWellUnderBudget + 1 : 0;
    if (samplesWellUnderBudget >= kSamplesBeforeRaise && scale < kMaxScale && HigherScaleFits()) {
        ChangeScale(SnapToStep(scale + kScaleStep));
    }
}

void DynamicResolution::ChangeScale(float newScale) {
    scale = newScale;
    samplesSinceChange = 0;
    samplesWellUnderBudget = 0;
}

bool DynamicResolution::HigherScaleFits() const {
    // Most GPU work grows with the pixel count, which grows with the square of the scale.
    // Fixed costs such as the shadow map make this estimate a little pessimistic, which is the
    // safe side to be wrong on.
    const double ratio = static_cast<double>(SnapToStep(scale + kScaleStep)) / static_cast<double>(scale);
    return smoothedMs * ratio * ratio < settings.gpuBudgetMs * kRaiseTarget;
}
