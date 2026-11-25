#pragma once

#include <glm/glm.hpp>

#include "animation/Kinematics.h"

// Analytic inverse kinematics for a limb of two bones: a hip and a knee, or a shoulder and an elbow.
//
// Given where the end of the limb (the ankle or the wrist) should be, the two bone lengths fix the angle at the knee
// or elbow (the law of cosines). What is left open is the plane the limb bends in; a pole direction picks it: the knee or
// elbow bends towards the pole. The solver turns the upper bone, then the lower bone, and respects the limits of the joints.

namespace animation {

struct Limb {
    Joint upper;  // thigh or upper arm
    Joint lower;  // calf or forearm: the hinge
    Joint end;    // foot or hand: the point that is put on the target is the joint itself (ankle or wrist)
    glm::vec3 restPole;  // the direction the hinge points in the rest pose: forwards for a knee, backwards for an elbow
};

Limb LegLimb(bool left);
Limb ArmLimb(bool left);

struct IkResult {
    bool reached = false;       // the end is on the target, within a millimetre
    glm::vec3 endPosition{0.0f};
    float flexionDegrees = 0.0f;  // of the knee or elbow
};

// Turns the two bones of `limb` in `pose` so that its end is on `targetWorld`, with the hinge bending towards
// `poleDirection` (a direction in the world; it need not be normalised). The rest of the pose is not touched, but the parent
// of the upper bone must already be where it is wanted. If the target is out of reach, the limb stretches towards it as far as
// the joint limits allow, or folds as far as they allow, and `reached` is false. The limb never bends backwards.
IkResult SolveTwoBone(const Skeleton& skeleton, Pose& pose, const Limb& limb, const glm::vec3& targetWorld, const glm::vec3& poleDirection);

// The longest the limb can reach from its upper joint, with the knee or elbow not bending backwards.
float MaximumReach(const Skeleton& skeleton, const Limb& limb);

// The distance from the upper joint to the end of the limb in the rest pose.
float RestReach(const Skeleton& skeleton, const Limb& limb);

}  // namespace animation
