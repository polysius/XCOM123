#pragma once

#include <array>
#include <cstdint>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "character/CharacterParameters.h"

// The skeleton of a character: 21 joints in a tree, in the rest pose that the mesh is built around.
//
// Axes. The character stands on the plane y = 0, faces +Z and has their left side on +X (as the people of the crowd
// do). Every joint has the same local axes as the world has at rest, so a rotation means the same thing on every
// joint, and an animation can name them: about X is swinging forwards and backwards (a positive angle swings a hanging
// limb backwards, a negative one forwards), about Y is twisting along the length of the body, and about Z is
// leaning to the side. The rest pose is an A-pose: the arms hang 40 degrees from the body and the feet stand apart.
//
// Segment lengths and the heights of the joints are fractions of stature, after Drillis and Contini (1966), as given
// in Winter, Biomechanics and Motor Control of Human Movement.

namespace character {

enum class Joint : uint8_t {
    Pelvis,  // the root
    Spine1,
    Spine2,
    Neck,
    Head,
    ClavicleL,
    ClavicleR,
    UpperArmL,
    UpperArmR,
    ForearmL,
    ForearmR,
    HandL,
    HandR,
    ThighL,
    ThighR,
    CalfL,
    CalfR,
    FootL,
    FootR,
    ToesL,
    ToesR,
    Count,
};

constexpr int kJointCount = static_cast<int>(Joint::Count);

constexpr int Index(Joint joint) {
    return static_cast<int>(joint);
}

struct JointInfo {
    const char* name = "";
    int parent = -1;                 // index of the parent joint, -1 for the root; always lower than the joint's own index
    glm::vec3 restPosition{0.0f};    // in the rest pose, in the world
    glm::vec3 restOffset{0.0f};      // from the parent joint, in the rest pose
    glm::mat4 inverseBind{1.0f};     // takes a point of the rest pose into the joint's own space
    glm::vec3 minAngleDegrees{0.0f}; // limits of the rotation about X, Y and Z from the rest pose
    glm::vec3 maxAngleDegrees{0.0f};
    // The axis an elbow or a knee bends about, in the rest pose; a positive angle about it flexes the joint. The forearm
    // hangs 40 degrees out from the body, so its hinge is not the world X axis. For these joints the limits in X are limits
    // of the angle about this axis. For every other joint it is the X axis.
    glm::vec3 hingeAxis{1.0f, 0.0f, 0.0f};
};

// Where a foot touches the ground in the rest pose; foot inverse kinematics will need these three.
struct FootContacts {
    glm::vec3 heel{0.0f};
    glm::vec3 ball{0.0f};  // the ball of the foot, where the toes begin
    glm::vec3 toe{0.0f};   // the tip of the toes
};

struct Skeleton {
    std::array<JointInfo, kJointCount> joints;
    FootContacts leftFoot;
    FootContacts rightFoot;
    float statureMeters = 0.0f;

    const JointInfo& operator[](Joint joint) const { return joints[Index(joint)]; }
};

// The skeleton of a person with these (sanitised) parameters. The proportions follow the fractions of stature, with
// small differences by sex and a few percent of variation from the seed.
Skeleton BuildSkeleton(const CharacterParameters& parameters);

// A pose: how far every joint is turned from the rest pose, and where the pelvis is.
struct Pose {
    std::array<glm::quat, kJointCount> rotation;  // of each joint relative to its parent, in the rest pose's axes
    glm::vec3 rootTranslation{0.0f};              // added to the pelvis

    Pose() {
        rotation.fill(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    }
};

// The matrices that carry points of the rest pose to the pose: world matrix of each joint times its inverse bind.
std::array<glm::mat4, kJointCount> SkinMatrices(const Skeleton& skeleton, const Pose& pose);

// The world positions of the joints in a pose.
std::array<glm::vec3, kJointCount> JointPositions(const Skeleton& skeleton, const Pose& pose);

// A rotation about X, then Y, then Z, in degrees: the three angles of a joint.
glm::quat FromDegrees(float aboutX, float aboutY, float aboutZ);

// The rotation that flexes an elbow or a knee by `degrees` about its hinge, a pure bend with no twist or swing.
glm::quat HingeRotation(const Skeleton& skeleton, Joint joint, float degrees);

}  // namespace character
