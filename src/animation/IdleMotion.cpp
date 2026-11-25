#include "animation/IdleMotion.h"

#include <cmath>

#include "animation/Noise.h"

namespace animation {
namespace {

constexpr float kTwoPi = 6.28318530718f;

glm::quat AboutX(float degrees) {
    return glm::angleAxis(degrees * kDegreesToRadians, glm::vec3(1.0f, 0.0f, 0.0f));
}
glm::quat AboutY(float degrees) {
    return glm::angleAxis(degrees * kDegreesToRadians, glm::vec3(0.0f, 1.0f, 0.0f));
}
glm::quat AboutZ(float degrees) {
    return glm::angleAxis(degrees * kDegreesToRadians, glm::vec3(0.0f, 0.0f, 1.0f));
}

void SetRotation(BodyLayer& layer, Joint joint, const glm::quat& rotation) {
    layer.additive.rotation[character::Index(joint)] = rotation;
}

}  // namespace

IdleMotion::IdleMotion(uint32_t seed_) : seed(seed_) {
    // A calm adult breathes 12 to 18 times a minute; the rate is a trait of the person.
    breathsPerSecond = 0.20f + 0.10f * (0.5f + 0.5f * LatticeValue(seed_, 7));
}

BodyLayer IdleMotion::Evaluate(float timeSeconds) const {
    BodyLayer layer;

    // Breathing: the chest rises and tips back a little as it fills, and the shoulders lift with it.
    const float breath = std::sin(kTwoPi * breathsPerSecond * timeSeconds);
    SetRotation(layer, Joint::ClavicleL, AboutZ(0.8f * breath));
    SetRotation(layer, Joint::ClavicleR, AboutZ(-0.8f * breath));

    // Weight shift: the pelvis slides over one foot and tilts, the trunk tilts back to keep the shoulders level.
    const float shift = DriftNoise(seed + 1u, timeSeconds * 0.09f);
    const float rock = DriftNoise(seed + 2u, timeSeconds * 0.13f);
    layer.pelvisOffset = glm::vec3(0.022f * shift, 0.0f, 0.0f);
    SetRotation(layer, Joint::Pelvis, AboutZ(2.4f * shift) * AboutX(0.6f * rock));
    SetRotation(layer, Joint::Spine1, AboutX(-0.5f * breath) * AboutZ(-1.2f * shift));
    SetRotation(layer, Joint::Spine2, AboutX(-0.9f * breath) * AboutZ(-1.2f * shift) * AboutX(-0.4f * rock));

    // The head wanders: a slow turn to one side and a small nod, shared between the neck and the head.
    const float turn = DriftNoise(seed + 3u, timeSeconds * 0.17f);
    const float nod = DriftNoise(seed + 4u, timeSeconds * 0.21f);
    const float tilt = DriftNoise(seed + 5u, timeSeconds * 0.11f);
    SetRotation(layer, Joint::Neck, AboutY(1.6f * turn) * AboutX(0.6f * nod));
    SetRotation(layer, Joint::Head, AboutY(2.4f * turn) * AboutX(1.0f * nod) * AboutZ(1.2f * tilt));
    return layer;
}

}  // namespace animation
