#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "animation/Kinematics.h"

// The range of motion of the human joints, taken from the limits stored in each character::JointInfo.
//
// A joint with a free swing (a hip, a shoulder, the spine, the neck) is limited in its three Euler angles: the rotation
// is read as X, then Y, then Z (the order of character::FromDegrees), each angle is held between its minimum and maximum,
// and the rotation is built again. A hinge (knee or elbow) is limited only in its flexion: the part of the rotation that
// turns about the hinge axis. Its other components are small and left alone, so that a leg that is not quite straight at
// rest still reaches what inverse kinematics asked of it.

namespace animation {

// True for the knees and the elbows.
bool IsHinge(Joint joint);

// The angles of a rotation about X, then Y, then Z, in degrees. X and Z run over -180 to 180 and Y over -90 to 90, which
// covers every limit of the skeleton, including the shoulders that go up to 180.
glm::vec3 EulerDegrees(const glm::quat& rotation);

// The flexion of a hinge in degrees: the twist of the rotation about the hinge axis, positive when the joint bends.
float FlexionDegrees(const Skeleton& skeleton, Joint hinge, const glm::quat& rotation);

// The same rotation with the flexion replaced by `degrees`, and the rest of it kept.
glm::quat WithFlexionDegrees(const Skeleton& skeleton, Joint hinge, const glm::quat& rotation, float degrees);

// The nearest rotation inside the limits of the joint.
glm::quat ClampToLimits(const Skeleton& skeleton, Joint joint, const glm::quat& rotation);

// How far a rotation is outside the limits of its joint, in degrees: 0 when inside, otherwise the largest excess of any angle.
float LimitExcessDegrees(const Skeleton& skeleton, Joint joint, const glm::quat& rotation);

// Clamps every joint of a pose. The pelvis is left alone: it is the root, and its rotation is the heading of the body.
void ClampPoseToLimits(const Skeleton& skeleton, Pose& pose);

}  // namespace animation
