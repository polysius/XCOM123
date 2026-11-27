#include "animation/LookAt.h"

#include <algorithm>
#include <cmath>

#include "animation/JointLimits.h"

namespace animation {
namespace {

// Where the eyes are from the skull joint, in the frame of the head: a little in front of it and above.
glm::vec3 EyeOffsetInHead(const BodyModel& body) {
    const float stature = body.skeleton.statureMeters;
    return glm::vec3(0.0f, 0.045f * stature, 0.05f * stature);
}

glm::quat HeadWorldRotation(const Skeleton& skeleton, const Pose& pose) {
    return ForwardKinematics(skeleton, pose).rotation[character::Index(Joint::Head)];
}

// A rotation that turns +Z to a direction with the up axis kept as near to +Y as possible.
glm::quat FacingRotation(const glm::vec3& direction) {
    const glm::vec3 forward = glm::normalize(direction);
    glm::vec3 side = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), forward);
    if (glm::dot(side, side) < 1e-6f) {
        side = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    side = glm::normalize(side);
    const glm::vec3 up = glm::cross(forward, side);
    return glm::normalize(glm::quat_cast(glm::mat3(side, up, forward)));
}

// The yaw and pitch, in degrees, of a direction in the frame of a head: yaw towards +X, pitch up.
EyeAngles AnglesOf(const glm::vec3& directionInHead) {
    EyeAngles angles;
    angles.yawDegrees = std::atan2(directionInHead.x, directionInHead.z) * kRadiansToDegrees;
    angles.pitchDegrees = std::atan2(directionInHead.y, std::hypot(directionInHead.x, directionInHead.z)) * kRadiansToDegrees;
    return angles;
}

EyeAngles ClampedToEyeRange(EyeAngles angles, const LookAtSettings& settings) {
    angles.yawDegrees = std::clamp(angles.yawDegrees, -settings.maxEyeYawDegrees, settings.maxEyeYawDegrees);
    angles.pitchDegrees = std::clamp(angles.pitchDegrees, -settings.maxEyePitchDegrees, settings.maxEyePitchDegrees);
    return angles;
}

}  // namespace

glm::vec3 EyeCentre(const BodyModel& body, const Pose& pose) {
    const WorldPose world = ForwardKinematics(body.skeleton, pose);
    const int head = character::Index(Joint::Head);
    return world.position[head] + world.rotation[head] * EyeOffsetInHead(body);
}

glm::vec3 HeadForward(const BodyModel& body, const Pose& pose) {
    return HeadWorldRotation(body.skeleton, pose) * glm::vec3(0.0f, 0.0f, 1.0f);
}

LookAtResult LookAt(const BodyModel& body, Pose& pose, const glm::vec3& targetWorld, float weight, const LookAtSettings& settings) {
    // A target that is not a number cannot be looked at: leave the pose as it is.
    if (!std::isfinite(targetWorld.x + targetWorld.y + targetWorld.z) || !std::isfinite(weight)) {
        LookAtResult nothing;
        nothing.errorDegrees = 180.0f;
        return nothing;
    }
    const Skeleton& skeleton = body.skeleton;
    const Pose before = pose;
    const WorldPose world = ForwardKinematics(skeleton, pose);
    const glm::quat chest = world.rotation[character::Index(Joint::Spine2)];
    const glm::vec3 eyeCentre = EyeCentre(body, pose);
    const glm::vec3 toTarget = targetWorld - eyeCentre;
    const glm::vec3 direction = glm::dot(toTarget, toTarget) > 1e-8f ? glm::normalize(toTarget) : chest * glm::vec3(0.0f, 0.0f, 1.0f);

    // The eyes take some of the turn, and the neck and head the rest. The share is of the angle from straight ahead of the chest.
    const glm::vec3 inChest = glm::inverse(chest) * direction;
    const EyeAngles wanted = AnglesOf(inChest);
    const EyeAngles eyeTurn = ClampedToEyeRange({wanted.yawDegrees * settings.eyeShare, wanted.pitchDegrees * settings.eyeShare}, settings);
    const float headYaw = (wanted.yawDegrees - eyeTurn.yawDegrees) * kDegreesToRadians;
    const float headPitch = (wanted.pitchDegrees - eyeTurn.pitchDegrees) * kDegreesToRadians;
    const glm::vec3 headDirectionInChest(std::cos(headPitch) * std::sin(headYaw), std::sin(headPitch), std::cos(headPitch) * std::cos(headYaw));
    const glm::quat headWanted = chest * FacingRotation(headDirectionInChest);

    // The neck turns a share of the way from the chest to the head, and the head the rest, each held to its limits.
    const glm::quat neckWorld = glm::slerp(chest, headWanted, settings.neckShare);
    pose.rotation[character::Index(Joint::Neck)] = ClampToLimits(skeleton, Joint::Neck, glm::inverse(chest) * neckWorld);
    pose.rotation[character::Index(Joint::Head)] = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::quat neckActual = chest * pose.rotation[character::Index(Joint::Neck)];
    pose.rotation[character::Index(Joint::Head)] = ClampToLimits(skeleton, Joint::Head, glm::inverse(neckActual) * headWanted);

    // Fade from the pose that was there.
    const float blend = std::clamp(weight, 0.0f, 1.0f);
    for (const Joint joint : {Joint::Neck, Joint::Head}) {
        const int i = character::Index(joint);
        pose.rotation[i] = glm::normalize(glm::slerp(before.rotation[i], pose.rotation[i], blend));
    }

    // What is left for the eyes: the direction of the target in the head as it ended up, for each eye.
    const glm::quat headNow = HeadWorldRotation(skeleton, pose);
    const glm::vec3 eyeOffset = headNow * EyeOffsetInHead(body);
    const glm::vec3 skull = ForwardKinematics(skeleton, pose).position[character::Index(Joint::Head)];
    const glm::vec3 headLeft = headNow * glm::vec3(1.0f, 0.0f, 0.0f);
    LookAtResult result;
    EyeAngles* eyes[2] = {&result.leftEye, &result.rightEye};
    float worstError = 0.0f;
    for (int i = 0; i < 2; ++i) {
        const float side = i == 0 ? 1.0f : -1.0f;
        const glm::vec3 eye = skull + eyeOffset + headLeft * (side * 0.5f * settings.eyeSeparationMeters);
        const glm::vec3 eyeToTarget = glm::normalize(targetWorld - eye);
        const EyeAngles needed = AnglesOf(glm::inverse(headNow) * eyeToTarget);
        *eyes[i] = ClampedToEyeRange(needed, settings);
        const float missYaw = needed.yawDegrees - eyes[i]->yawDegrees;
        const float missPitch = needed.pitchDegrees - eyes[i]->pitchDegrees;
        worstError = std::max(worstError, std::hypot(missYaw, missPitch));
    }
    result.errorDegrees = worstError;
    result.reachedWithHeadAndEyes = worstError < 1.0f;
    return result;
}

}  // namespace animation
