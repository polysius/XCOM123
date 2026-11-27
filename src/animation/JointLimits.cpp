#include "animation/JointLimits.h"

#include <algorithm>
#include <cmath>

namespace animation {
namespace {

// Wraps an angle in degrees into -180 to 180.
float WrapDegrees(float degrees) {
    while (degrees > 180.0f) {
        degrees -= 360.0f;
    }
    while (degrees < -180.0f) {
        degrees += 360.0f;
    }
    return degrees;
}

// The twist of a rotation about an axis, as an angle in degrees from -180 to 180.
float TwistDegrees(const glm::quat& rotation, const glm::vec3& axis) {
    const glm::quat unit = glm::normalize(rotation);
    const float along = glm::dot(glm::vec3(unit.x, unit.y, unit.z), axis);
    return WrapDegrees(2.0f * std::atan2(along, unit.w) * kRadiansToDegrees);
}

float ExcessOutside(float value, float low, float high) {
    return std::max({0.0f, low - value, value - high});
}

}  // namespace

bool IsHinge(Joint joint) {
    return joint == Joint::CalfL || joint == Joint::CalfR || joint == Joint::ForearmL || joint == Joint::ForearmR;
}

glm::vec3 EulerDegrees(const glm::quat& rotation) {
    // For R = Rz(c) * Ry(b) * Rx(a) the bottom row is (-sin b, sin a cos b, cos a cos b) and the first column
    // is (cos b cos c, cos b sin c, -sin b), which gives the three angles below.
    const glm::mat3 m = glm::mat3_cast(glm::normalize(rotation));
    const float sinB = std::clamp(-m[0][2], -1.0f, 1.0f);
    const float b = std::asin(sinB);
    float a;
    float c;
    if (std::abs(sinB) < 0.999999f) {
        a = std::atan2(m[1][2], m[2][2]);
        c = std::atan2(m[0][1], m[0][0]);
    } else {
        // Looking straight along an axis, X and Z turn the same way: put all of it in X.
        c = 0.0f;
        a = std::atan2(sinB * m[1][0], sinB * m[2][0]);
    }
    return glm::vec3(a, b, c) * kRadiansToDegrees;
}

float FlexionDegrees(const Skeleton& skeleton, Joint hinge, const glm::quat& rotation) {
    return TwistDegrees(rotation, skeleton[hinge].hingeAxis);
}

glm::quat WithFlexionDegrees(const Skeleton& skeleton, Joint hinge, const glm::quat& rotation, float degrees) {
    const glm::vec3 axis = skeleton[hinge].hingeAxis;
    const float current = FlexionDegrees(skeleton, hinge, rotation);
    // rotation = swing * twist, with the twist first; take the old twist out and put the new one in.
    const glm::quat oldTwist = glm::angleAxis(current * kDegreesToRadians, axis);
    const glm::quat newTwist = glm::angleAxis(degrees * kDegreesToRadians, axis);
    return glm::normalize(glm::normalize(rotation) * glm::inverse(oldTwist) * newTwist);
}

glm::quat ClampToLimits(const Skeleton& skeleton, Joint joint, const glm::quat& rotation) {
    const character::JointInfo& info = skeleton[joint];
    if (IsHinge(joint)) {
        const float flexion = FlexionDegrees(skeleton, joint, rotation);
        const float clamped = std::clamp(flexion, info.minAngleDegrees.x, info.maxAngleDegrees.x);
        return clamped == flexion ? rotation : WithFlexionDegrees(skeleton, joint, rotation, clamped);
    }
    const glm::vec3 angles = EulerDegrees(rotation);
    const glm::vec3 clamped = glm::clamp(angles, info.minAngleDegrees, info.maxAngleDegrees);
    if (clamped == angles) {
        return rotation;
    }
    return character::FromDegrees(clamped.x, clamped.y, clamped.z);
}

float LimitExcessDegrees(const Skeleton& skeleton, Joint joint, const glm::quat& rotation) {
    const character::JointInfo& info = skeleton[joint];
    if (IsHinge(joint)) {
        return ExcessOutside(FlexionDegrees(skeleton, joint, rotation), info.minAngleDegrees.x, info.maxAngleDegrees.x);
    }
    const glm::vec3 angles = EulerDegrees(rotation);
    return std::max({ExcessOutside(angles.x, info.minAngleDegrees.x, info.maxAngleDegrees.x),
                     ExcessOutside(angles.y, info.minAngleDegrees.y, info.maxAngleDegrees.y),
                     ExcessOutside(angles.z, info.minAngleDegrees.z, info.maxAngleDegrees.z)});
}

void ClampPoseToLimits(const Skeleton& skeleton, Pose& pose) {
    for (int i = 1; i < kJointCount; ++i) {
        const Joint joint = static_cast<Joint>(i);
        pose.rotation[i] = ClampToLimits(skeleton, joint, pose.rotation[i]);
    }
}

}  // namespace animation
