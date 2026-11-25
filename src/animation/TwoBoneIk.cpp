#include "animation/TwoBoneIk.h"

#include <algorithm>
#include <cmath>

#include "animation/JointLimits.h"

namespace animation {
namespace {

struct BoneLengths {
    glm::vec3 upperRest;  // from the upper joint to the lower one, in the rest pose
    glm::vec3 lowerRest;  // from the lower joint to the end
    float upperLength;
    float lowerLength;
};

BoneLengths MeasureLimb(const Skeleton& skeleton, const Limb& limb) {
    BoneLengths bones;
    bones.upperRest = skeleton[limb.lower].restPosition - skeleton[limb.upper].restPosition;
    bones.lowerRest = skeleton[limb.end].restPosition - skeleton[limb.lower].restPosition;
    bones.upperLength = glm::length(bones.upperRest);
    bones.lowerLength = glm::length(bones.lowerRest);
    return bones;
}

// The distance between the ends of two bones that meet at a hinge bent by `flexionDegrees` (0 is straight).
float ReachAtFlexion(const BoneLengths& bones, float flexionDegrees) {
    const float cosine = std::cos(flexionDegrees * kDegreesToRadians);
    const float squared = bones.upperLength * bones.upperLength + bones.lowerLength * bones.lowerLength +
                          2.0f * bones.upperLength * bones.lowerLength * cosine;
    return std::sqrt(std::max(squared, 0.0f));
}

// The bend of the hinge, in degrees, that puts the ends of the bones `distance` apart.
float FlexionForReach(const BoneLengths& bones, float distance) {
    const float cosine = (distance * distance - bones.upperLength * bones.upperLength - bones.lowerLength * bones.lowerLength) /
                         (2.0f * bones.upperLength * bones.lowerLength);
    return std::acos(std::clamp(cosine, -1.0f, 1.0f)) * kRadiansToDegrees;
}

// A direction perpendicular to `along` that is as close as possible to `pole`. If the pole lies along the limb, the rest
// pole stands in for it, and then any perpendicular will do.
glm::vec3 PerpendicularPole(const glm::vec3& along, const glm::vec3& pole, const glm::vec3& restPole) {
    for (const glm::vec3& candidate : {pole, restPole, glm::vec3(1.0f, 0.0f, 0.0f)}) {
        const glm::vec3 perpendicular = candidate - along * glm::dot(candidate, along);
        if (glm::dot(perpendicular, perpendicular) > 1e-8f) {
            return glm::normalize(perpendicular);
        }
    }
    return glm::cross(along, glm::vec3(0.0f, 0.0f, 1.0f));
}

// Where the upper bone points, and the world rotation of the upper bone that puts it there with the hinge bending the
// right way. `hingeAxis` is the axis the hinge turns about in the rest pose.
glm::quat UpperBoneRotation(const BoneLengths& bones, const glm::vec3& hingeAxis, const glm::vec3& upperDirection, const glm::vec3& polePerpendicular) {
    const glm::vec3 restAlong = bones.upperRest / bones.upperLength;
    const glm::vec3 restHinge = glm::normalize(hingeAxis - restAlong * glm::dot(hingeAxis, restAlong));
    // The hinge turns the lower bone from the upper one towards the way it folds, away from the pole: so the hinge axis
    // is the pole crossed with the upper bone, which is the hinge axis of the rest pose when the limb is at rest.
    const glm::vec3 hinge = glm::normalize(glm::cross(polePerpendicular, upperDirection));
    return RotationBetweenFrames(restAlong, restHinge, upperDirection, hinge);
}

}  // namespace

Limb LegLimb(bool left) {
    return left ? Limb{Joint::ThighL, Joint::CalfL, Joint::FootL, glm::vec3(0.0f, 0.0f, 1.0f)}
                : Limb{Joint::ThighR, Joint::CalfR, Joint::FootR, glm::vec3(0.0f, 0.0f, 1.0f)};
}

Limb ArmLimb(bool left) {
    return left ? Limb{Joint::UpperArmL, Joint::ForearmL, Joint::HandL, glm::vec3(0.0f, 0.0f, -1.0f)}
                : Limb{Joint::UpperArmR, Joint::ForearmR, Joint::HandR, glm::vec3(0.0f, 0.0f, -1.0f)};
}

float MaximumReach(const Skeleton& skeleton, const Limb& limb) {
    const BoneLengths bones = MeasureLimb(skeleton, limb);
    return ReachAtFlexion(bones, std::max(skeleton[limb.lower].minAngleDegrees.x, 0.0f));
}

float RestReach(const Skeleton& skeleton, const Limb& limb) {
    return glm::length(skeleton[limb.end].restPosition - skeleton[limb.upper].restPosition);
}

IkResult SolveTwoBone(const Skeleton& skeleton, Pose& pose, const Limb& limb, const glm::vec3& targetWorld, const glm::vec3& poleDirection) {
    const BoneLengths bones = MeasureLimb(skeleton, limb);
    const character::JointInfo& lowerInfo = skeleton[limb.lower];
    const WorldPose before = ForwardKinematics(skeleton, pose);
    const glm::vec3 upperPosition = before.position[character::Index(limb.upper)];
    const glm::quat parentRotation = ParentWorldRotation(skeleton, before, limb.upper);

    // The limb can neither stretch past straight nor fold past the limit of the hinge: bring the target into that range.
    const float minFlexion = std::max(lowerInfo.minAngleDegrees.x, 0.0f);
    const float maxFlexion = lowerInfo.maxAngleDegrees.x;
    const glm::vec3 toTarget = targetWorld - upperPosition;
    const float wantedDistance = std::max(glm::length(toTarget), 1e-5f);
    const glm::vec3 along = toTarget / wantedDistance;
    const float distance = std::clamp(wantedDistance, ReachAtFlexion(bones, maxFlexion), ReachAtFlexion(bones, minFlexion));

    // Triangle upper joint, hinge, end: the angle at the upper joint from the law of cosines.
    const float cosineAtUpper = (bones.upperLength * bones.upperLength + distance * distance - bones.lowerLength * bones.lowerLength) /
                                (2.0f * bones.upperLength * distance);
    const float angleAtUpper = std::acos(std::clamp(cosineAtUpper, -1.0f, 1.0f));
    const glm::vec3 polePerpendicular = PerpendicularPole(along, poleDirection, limb.restPole);
    const glm::vec3 upperDirection = along * std::cos(angleAtUpper) + polePerpendicular * std::sin(angleAtUpper);

    // The upper bone, inside the limits of its joint.
    const glm::quat upperWorld = UpperBoneRotation(bones, lowerInfo.hingeAxis, upperDirection, polePerpendicular);
    const glm::quat upperLocal = ClampToLimits(skeleton, limb.upper, glm::inverse(parentRotation) * upperWorld);
    pose.rotation[character::Index(limb.upper)] = glm::normalize(upperLocal);
    const glm::quat upperActual = parentRotation * pose.rotation[character::Index(limb.upper)];

    // The lower bone points from the hinge to the end of the reachable range; its flexion stays inside the hinge's limits.
    const glm::vec3 hingePosition = upperPosition + upperActual * bones.upperRest;
    const glm::vec3 reachable = upperPosition + along * distance;
    const glm::vec3 lowerDirection = glm::normalize(reachable - hingePosition);
    const glm::quat swing = RotationBetween(upperActual * bones.lowerRest, lowerDirection);
    glm::quat lowerLocal = glm::inverse(upperActual) * swing * upperActual;
    lowerLocal = ClampToLimits(skeleton, limb.lower, lowerLocal);
    pose.rotation[character::Index(limb.lower)] = glm::normalize(lowerLocal);

    IkResult result;
    const WorldPose after = ForwardKinematics(skeleton, pose);
    result.endPosition = after.position[character::Index(limb.end)];
    result.reached = glm::length(result.endPosition - targetWorld) < 1e-3f;
    result.flexionDegrees = FlexionDegrees(skeleton, limb.lower, pose.rotation[character::Index(limb.lower)]);
    return result;
}

}  // namespace animation
