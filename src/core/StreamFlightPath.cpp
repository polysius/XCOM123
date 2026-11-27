#include "core/StreamFlightPath.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kPi = 3.14159265f;

// The legs of the flight as shares of its frames: where each one ends, its speed and the height above the ground it flies at.
struct Leg {
    const char* name;
    float endsAt;
    float speed;
    float height;
};
constexpr Leg kLegs[] = {
    {"cruise", 0.35f, StreamFlightPath::kCruiseSpeed, 45.0f},
    {"burst", 0.50f, StreamFlightPath::kBurstSpeed, 90.0f},
    {"low cruise", 0.85f, StreamFlightPath::kCruiseSpeed, 22.0f},
    {"burst again", 1.00f, StreamFlightPath::kBurstSpeed, 70.0f},
};
constexpr size_t kLegCount = sizeof(kLegs) / sizeof(kLegs[0]);

// Speed and height change over this share of the flight around the end of a leg, so that the camera never jerks.
constexpr float kBlend = 0.02f;
constexpr float kLookAhead = 120.0f;
constexpr float kLookDownMetres = 14.0f;

float Smooth(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Speed or height at a point of the flight, blending from one leg to the next around the end of the leg.
float Blended(float progress, float Leg::*member) {
    float value = kLegs[0].*member;
    for (size_t leg = 0; leg + 1 < kLegCount; ++leg) {
        value += (kLegs[leg + 1].*member - kLegs[leg].*member) * Smooth(kLegs[leg].endsAt - kBlend, kLegs[leg].endsAt + kBlend, progress);
    }
    return value;
}

// The way the camera faces, in radians (0 is +Z): it weaves to either side of a heading that slowly turns, so that the flight
// crosses chunks in both directions instead of along one row.
float HeadingAt(float progress) {
    return 0.35f + 0.9f * std::sin(2.0f * kPi * 1.5f * progress);
}

}  // namespace

StreamFlightPath::StreamFlightPath(int frames)
    : totalFrames(std::max(frames, 1)), position(0.0f, kLegs[0].height, 0.0f) {}

float StreamFlightPath::Progress() const {
    return std::min(static_cast<float>(frame) / static_cast<float>(totalFrames), 1.0f);
}

StreamFlightPath::Pose StreamFlightPath::PoseHere() const {
    const float progress = Progress();
    const float heading = HeadingAt(progress);
    const glm::vec3 direction(std::sin(heading), 0.0f, std::cos(heading));
    Pose pose;
    pose.position = position;
    pose.target = position + direction * kLookAhead + glm::vec3(0.0f, -kLookDownMetres, 0.0f);
    pose.speed = Blended(progress, &Leg::speed);
    return pose;
}

StreamFlightPath::Pose StreamFlightPath::Start() const {
    return StreamFlightPath(totalFrames).PoseHere();
}

StreamFlightPath::Pose StreamFlightPath::Step(float deltaSeconds) {
    const float progress = Progress();
    const float heading = HeadingAt(progress);
    const float speed = Blended(progress, &Leg::speed);
    position += glm::vec3(std::sin(heading), 0.0f, std::cos(heading)) * (speed * deltaSeconds);
    position.y = Blended(progress, &Leg::height);
    distance += speed * deltaSeconds;
    ++frame;
    return PoseHere();
}

const std::vector<std::string>& StreamFlightPath::LegNames() {
    static const std::vector<std::string> names = [] {
        std::vector<std::string> result;
        for (const Leg& leg : kLegs) {
            result.emplace_back(leg.name);
        }
        return result;
    }();
    return names;
}

size_t StreamFlightPath::LegOfFrame(int frameIndex) const {
    const float progress = static_cast<float>(frameIndex) / static_cast<float>(totalFrames);
    for (size_t leg = 0; leg < kLegCount; ++leg) {
        if (progress < kLegs[leg].endsAt) {
            return leg;
        }
    }
    return kLegCount - 1;
}
