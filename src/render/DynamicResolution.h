#pragma once

#include <cstdint>

#include "render/IRenderModule.h"

// Picks the render scale (the fraction of the window size the scene is drawn at) that keeps the
// GPU frame time under a budget.
//
// It is built to be calm rather than quick: it moves in small fixed steps, waits after every
// change until measurements at the new scale arrive, lowers the scale as soon as the smoothed
// time is over budget, but raises it only after a long stretch well under budget, and only when
// the higher scale is predicted to fit too. The gap between those two thresholds stops it from
// flipping back and forth between two neighbouring scales.
class DynamicResolution {
public:
    static constexpr float kMinScale = 0.5f;
    static constexpr float kMaxScale = 1.0f;
    static constexpr float kScaleStep = 0.05f;

    void Configure(const ResolutionSettings& settings);

    // Feeds one new GPU frame time, measured at the scale that was current a frame or two ago.
    void AddMeasurement(double gpuFrameMs);

    float Scale() const { return scale; }

private:
    void ChangeScale(float newScale);
    bool HigherScaleFits() const;

    ResolutionSettings settings;
    float scale = 1.0f;
    double smoothedMs = 0.0;
    uint32_t samplesSinceChange = 0;
    uint32_t samplesWellUnderBudget = 0;
};
