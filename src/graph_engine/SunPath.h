#pragma once

#include <algorithm>
#include <cmath>

#include "graph_engine/EngineHost.h"

// Where the sun is: the two angles the renderer is given, and where they are at an hour of the day.

namespace graph_engine {

struct SunSettings {
    float elevationDegrees = 24.0f;  // above the horizon
    float azimuthDegrees = 230.0f;   // compass direction, 0 meaning +Z and 90 meaning +X
};

inline constexpr float kNoonSunElevation = 75.0f;

// The sun at an hour of the day (0 to 24, wrapping): it rises in the east (+X) at 6, stands highest in the south (-Z) at 12 and
// sets in the west at 18, and is below the horizon, as far as the limits of the renderer allow, from 18 to 6.
inline SunSettings SunAtTimeOfDay(float hours) {
    constexpr float kPi = 3.14159265358979f;
    const float hour = FiniteOr(hours, 12.0f);
    SunSettings sun;
    sun.elevationDegrees = std::clamp(kNoonSunElevation * std::sin(kPi * (hour - 6.0f) / 12.0f), kSunElevationMin, kSunElevationMax);
    sun.azimuthDegrees = WrapDegrees(90.0f + 15.0f * (hour - 6.0f));
    return sun;
}

}  // namespace graph_engine
