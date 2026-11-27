#include "animation/Secondary.h"

#include <algorithm>
#include <cmath>

namespace animation {
namespace {

constexpr float kHitPelvisFrequency = 8.0f;
constexpr float kHitTrunkFrequency = 7.0f;
constexpr float kHitHeadFrequency = 9.0f;
// How far a body gives at the pelvis, in metres, for each metre a second of change of velocity, and how far it bends, in degrees.
constexpr float kPelvisShiftPerSpeed = 0.035f;
constexpr float kTrunkBendPerSpeed = 7.0f;
constexpr float kHeadWhipPerSpeed = 9.0f;
constexpr float kMaxPelvisShift = 0.10f;
constexpr float kMaxBendDegrees = 28.0f;

glm::quat AboutX(float radians) {
    return glm::angleAxis(radians, glm::vec3(1.0f, 0.0f, 0.0f));
}
glm::quat AboutY(float radians) {
    return glm::angleAxis(radians, glm::vec3(0.0f, 1.0f, 0.0f));
}
glm::quat AboutZ(float radians) {
    return glm::angleAxis(radians, glm::vec3(0.0f, 0.0f, 1.0f));
}

// The rotation for three small angles: pitch about X, then yaw about Y, then roll about Z.
glm::quat FromAngles(const glm::vec3& angles) {
    return glm::normalize(AboutZ(angles.z) * AboutY(angles.y) * AboutX(angles.x));
}

// An acceleration in the world as seen from the body: X to its left, Y up, Z forwards.
glm::vec3 ToBodyFrame(const glm::vec3& world, float heading) {
    return glm::inverse(YawRotation(heading)) * world;
}

glm::vec3 ClampLength(const glm::vec3& v, float maxLength) {
    const float length = glm::length(v);
    return length > maxLength ? v * (maxLength / length) : v;
}

}  // namespace

static bool IsFinite(const glm::vec3& v) {
    return std::isfinite(v.x + v.y + v.z);
}

void StepCriticallyDamped(SpringState& spring, const glm::vec3& force, float omega, float dt) {
    // Nothing that is not a number goes into a spring: it would never come out.
    if (!std::isfinite(dt) || dt <= 0.0f || !std::isfinite(omega) || omega <= 0.0f || !IsFinite(force)) {
        return;
    }
    // About the point where the push balances the spring, the offset y obeys y'' = -2 w y' - w^2 y, which has the solution
    // y(t) = (y0 + (v0 + w y0) t) e^(-w t).
    const glm::vec3 rest = force / (omega * omega);
    const glm::vec3 y0 = spring.value - rest;
    const glm::vec3 v0 = spring.velocity;
    const float decay = std::exp(-omega * dt);
    const glm::vec3 slope = v0 + omega * y0;
    spring.value = rest + (y0 + slope * dt) * decay;
    spring.velocity = (v0 - omega * slope * dt) * decay;
}

SecondaryMotion::SecondaryMotion(const SecondaryMotionSettings& settings_) : settings(settings_) {}

void SecondaryMotion::Reset() {
    chest = SpringState{};
    head = SpringState{};
}

BodyLayer SecondaryMotion::Update(float dt, const glm::vec3& acceleration, float heading) {
    // In the frame of the body: forward acceleration tips the chest back (a negative pitch), acceleration to the left leans it
    // to the right (a positive roll lifts the left side).
    const glm::vec3 local = IsFinite(acceleration) && std::isfinite(heading) ? ToBodyFrame(acceleration, heading) : glm::vec3(0.0f);
    const float gain = settings.chestGain * kDegreesToRadians;
    const glm::vec3 chestForce = glm::vec3(-local.z * gain, 0.0f, local.x * gain);
    // A spring pushed by a constant force rests at force / omega^2, so the force is scaled to rest at the lean the gain asks for.
    StepCriticallyDamped(chest, chestForce * (settings.chestFrequency * settings.chestFrequency), settings.chestFrequency, dt);
    const float limit = settings.maxLeanDegrees * kDegreesToRadians;
    chest.value = ClampLength(chest.value, limit);

    // The head is pushed the same way, and also follows the chest.
    const float headGain = settings.headGain * kDegreesToRadians;
    const glm::vec3 headRest = glm::vec3(-local.z * headGain, 0.0f, local.x * headGain) + chest.value * settings.headFollowsChest;
    StepCriticallyDamped(head, headRest * (settings.headFrequency * settings.headFrequency), settings.headFrequency, dt);
    head.value = ClampLength(head.value, limit);

    BodyLayer layer;
    layer.additive.rotation[character::Index(Joint::Spine2)] = FromAngles(chest.value);
    layer.additive.rotation[character::Index(Joint::Neck)] = FromAngles(head.value * 0.4f);
    layer.additive.rotation[character::Index(Joint::Head)] = FromAngles(head.value * 0.6f);
    return layer;
}

HitReaction::HitReaction() = default;

void HitReaction::Push(const glm::vec3& deltaVelocity, float heading) {
    if (!IsFinite(deltaVelocity) || !std::isfinite(heading)) {
        return;
    }
    const glm::vec3 local = ToBodyFrame(deltaVelocity, heading);
    // A critically damped spring released with a velocity v0 from rest reaches its largest displacement, v0 / (e w), after 1 / w.
    const float e = 2.71828183f;
    pelvis.velocity += ClampLength(glm::vec3(local.x, 0.0f, local.z) * kPelvisShiftPerSpeed * e * kHitPelvisFrequency, kMaxPelvisShift * e * kHitPelvisFrequency);
    // A shove from behind pushes the pelvis forwards and the trunk lags: it tips back, and the knees bend a little as well.
    pelvis.velocity.y -= std::min(glm::length(glm::vec2(local.x, local.z)) * 0.012f * e * kHitPelvisFrequency, 0.05f * e * kHitPelvisFrequency);
    const float bend = kTrunkBendPerSpeed * kDegreesToRadians * e * kHitTrunkFrequency;
    const float maxBend = kMaxBendDegrees * kDegreesToRadians * e * kHitTrunkFrequency;
    trunk.velocity += ClampLength(glm::vec3(-local.z * bend, 0.0f, local.x * bend), maxBend);
    const float whip = kHeadWhipPerSpeed * kDegreesToRadians * e * kHitHeadFrequency;
    head.velocity += ClampLength(glm::vec3(-local.z * whip, 0.0f, local.x * whip), maxBend);
}

BodyLayer HitReaction::Update(float dt) {
    StepCriticallyDamped(pelvis, glm::vec3(0.0f), kHitPelvisFrequency, dt);
    StepCriticallyDamped(trunk, glm::vec3(0.0f), kHitTrunkFrequency, dt);
    StepCriticallyDamped(head, glm::vec3(0.0f), kHitHeadFrequency, dt);

    BodyLayer layer;
    layer.pelvisOffset = pelvis.value;
    layer.additive.rotation[character::Index(Joint::Spine1)] = FromAngles(trunk.value * 0.4f);
    layer.additive.rotation[character::Index(Joint::Spine2)] = FromAngles(trunk.value * 0.6f);
    layer.additive.rotation[character::Index(Joint::Neck)] = FromAngles(head.value * 0.4f);
    layer.additive.rotation[character::Index(Joint::Head)] = FromAngles(head.value * 0.6f);
    return layer;
}

bool HitReaction::Settled() const {
    const float angular = (glm::length(trunk.value) + glm::length(head.value)) * kRadiansToDegrees;
    return glm::length(pelvis.value) < 0.001f && angular < 0.05f && glm::length(pelvis.velocity) < 0.01f &&
           glm::length(trunk.velocity) + glm::length(head.velocity) < 0.02f;
}

}  // namespace animation
