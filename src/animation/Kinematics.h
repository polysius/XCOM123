#pragma once

#include <array>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "character/Skeleton.h"

// Forward kinematics on the skeleton of src/character: where every joint is, and how it is turned, in the world.
//
// The animation module reuses character::Pose (a rotation for every joint, relative to its parent in the axes of the
// rest pose, and a translation of the pelvis). All the rotations of a pose are measured from the rest pose, so the
// identity is the A-pose, and an additive layer is simply a pose of small rotations.
//
// Everything here is a pure function of its arguments: no clock, no hidden state. The results agree with
// character::SkinMatrices, which is what a renderer uses, and the tests check that they do.

namespace animation {

using character::Joint;
using character::Pose;
using character::Skeleton;

constexpr int kJointCount = character::kJointCount;

constexpr float kDegreesToRadians = 0.01745329252f;
constexpr float kRadiansToDegrees = 57.2957795131f;

using JointMatrices = std::array<glm::mat4, kJointCount>;

// The position and orientation of every joint in the world.
struct WorldPose {
    std::array<glm::quat, kJointCount> rotation;
    std::array<glm::vec3, kJointCount> position;
};

WorldPose ForwardKinematics(const Skeleton& skeleton, const Pose& pose);

// The world matrix of every joint (a translation and a rotation). Skinning multiplies these by the inverse bind matrices.
JointMatrices WorldMatrices(const Skeleton& skeleton, const Pose& pose);

// The matrices a renderer uploads: the world matrix of each joint times its inverse rest matrix. They are
// character::SkinMatrices, repeated here so that a user of the animation module needs no other header for it.
JointMatrices SkinningMatrices(const Skeleton& skeleton, const Pose& pose);

// The world orientation of the parent of a joint; the identity for the root.
glm::quat ParentWorldRotation(const Skeleton& skeleton, const WorldPose& world, Joint joint);

// Turns a joint so that its world orientation is `worldRotation`, and leaves the other joints as they are. The children
// turn with it, so their world orientation changes.
void SetWorldRotation(const Skeleton& skeleton, Pose& pose, Joint joint, const glm::quat& worldRotation);

// Moves the pelvis, the root of the skeleton, to a position in the world.
void SetPelvisPosition(const Skeleton& skeleton, Pose& pose, const glm::vec3& worldPosition);

// The shortest rotation that takes direction `from` to direction `to`; both need not be normalised.
glm::quat RotationBetween(const glm::vec3& from, const glm::vec3& to);

// The rotation that takes the frame (fromFirst, fromSecond) to the frame (toFirst, toSecond). Each frame is a pair of
// perpendicular unit vectors; the third axis of each follows from the cross product.
glm::quat RotationBetweenFrames(const glm::vec3& fromFirst, const glm::vec3& fromSecond, const glm::vec3& toFirst, const glm::vec3& toSecond);

// A rotation about Y: the heading of a body. Zero faces +Z, and a positive angle turns towards +X.
glm::quat YawRotation(float radians);

// The angle of a rotation in degrees, 0 to 180: how far it turns, whatever the axis.
float RotationAngleDegrees(const glm::quat& rotation);

}  // namespace animation
