#include "character/Skeleton.h"

#include <cmath>

#include <glm/gtc/matrix_transform.hpp>

#include "character/Anthropometry.h"

namespace character {
namespace {

constexpr float kDegreesToRadians = 0.01745329252f;

// A small deterministic number in [-1, 1] from the seed and a salt, for the variation of the proportions.
float SeedOffset(uint32_t seed, uint32_t salt) {
    uint32_t h = seed * 0x9E3779B1u ^ (salt + 0x7F4A7C15u) * 0x85EBCA6Bu;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    h *= 0x297A2D39u;
    h ^= h >> 15;
    return static_cast<float>(h >> 8) * (2.0f / 16777216.0f) - 1.0f;
}

struct JointSpec {
    const char* name;
    Joint parent;  // Joint::Count for the root
    glm::vec3 minAngle;
    glm::vec3 maxAngle;
};

// Names, parents and the limits of the rotation of every joint. A positive rotation about X swings a hanging limb
// backwards, so flexion forwards is negative for the shoulder and hip, and the elbow and knee bend the other way.
const JointSpec kSpecs[kJointCount] = {
    {"pelvis", Joint::Count, {-30, -45, -30}, {30, 45, 30}},
    {"spine1", Joint::Pelvis, {-45, -35, -25}, {30, 35, 25}},
    {"spine2", Joint::Spine1, {-35, -35, -25}, {25, 35, 25}},
    {"neck", Joint::Spine2, {-50, -70, -35}, {45, 70, 35}},
    {"head", Joint::Neck, {-40, -50, -30}, {30, 50, 30}},
    {"clavicleL", Joint::Spine2, {-15, -10, -15}, {25, 10, 35}},
    {"clavicleR", Joint::Spine2, {-15, -10, -35}, {25, 10, 15}},
    {"upperArmL", Joint::ClavicleL, {-180, -90, -60}, {60, 90, 180}},
    {"upperArmR", Joint::ClavicleR, {-180, -90, -180}, {60, 90, 60}},
    {"forearmL", Joint::UpperArmL, {-150, -90, 0}, {5, 90, 0}},
    {"forearmR", Joint::UpperArmR, {-150, -90, 0}, {5, 90, 0}},
    {"handL", Joint::ForearmL, {-70, -30, -25}, {80, 30, 30}},
    {"handR", Joint::ForearmR, {-70, -30, -30}, {80, 30, 25}},
    {"thighL", Joint::Pelvis, {-120, -45, -45}, {30, 45, 25}},
    {"thighR", Joint::Pelvis, {-120, -45, -25}, {30, 45, 45}},
    {"calfL", Joint::ThighL, {-5, -10, 0}, {150, 10, 0}},
    {"calfR", Joint::ThighR, {-5, -10, 0}, {150, 10, 0}},
    {"footL", Joint::CalfL, {-45, -25, -25}, {50, 25, 25}},
    {"footR", Joint::CalfR, {-45, -25, -25}, {50, 25, 25}},
    {"toesL", Joint::FootL, {-10, 0, 0}, {60, 0, 0}},
    {"toesR", Joint::FootR, {-10, 0, 0}, {60, 0, 0}},
};

// The positions of the joints in the rest pose.
std::array<glm::vec3, kJointCount> RestPositions(const CharacterParameters& parameters, FootContacts& left, FootContacts& right) {
    const float height = parameters.statureMeters;
    const float female = 1.0f - parameters.masculinity;
    // A few percent of difference in the proportions: longer or shorter legs, broader or narrower shoulders.
    const float legVariation = 1.0f + 0.02f * SeedOffset(parameters.seed, 1);
    const float shoulderVariation = 1.0f + 0.03f * SeedOffset(parameters.seed, 2);

    namespace a = anthropometry;
    // Women have slightly shorter legs against the trunk, and narrower shoulders.
    const float hipHeight = a::kHipHeight * (1.0f - 0.006f * female) * legVariation;
    const float kneeHeight = a::kAnkleHeight + (a::kKneeHeight - a::kAnkleHeight) * (hipHeight - a::kAnkleHeight) / (a::kHipHeight - a::kAnkleHeight);
    const float shoulderHeight = a::kShoulderHeight;
    const float shoulderHalfWidth = a::kShoulderJointHalfWidth * (1.0f - 0.09f * female) * shoulderVariation;
    const float hipHalfWidth = a::kHipJointHalfWidth * (1.0f + 0.10f * female);

    std::array<glm::vec3, kJointCount> position{};
    position[Index(Joint::Pelvis)] = {0.0f, hipHeight * height, 0.0f};
    position[Index(Joint::Spine1)] = {0.0f, (hipHeight + 0.075f * a::kShoulderHeight) * height, 0.0f};
    position[Index(Joint::Spine2)] = {0.0f, (hipHeight + 0.55f * (shoulderHeight - hipHeight)) * height, 0.0f};
    position[Index(Joint::Neck)] = {0.0f, a::kNeckBaseHeight * height, -0.004f * height};
    position[Index(Joint::Head)] = {0.0f, a::kSkullBaseHeight * height, 0.0f};

    const float arm = 40.0f * kDegreesToRadians;  // the A-pose: how far the arms hang from the body
    for (const float side : {1.0f, -1.0f}) {
        const bool leftSide = side > 0.0f;
        const glm::vec3 down(side * std::sin(arm), -std::cos(arm), 0.0f);

        const glm::vec3 clavicle(side * 0.030f * height, a::kNeckBaseHeight * height - 0.012f * height, 0.0f);
        const glm::vec3 shoulder(side * shoulderHalfWidth * height, shoulderHeight * height, -0.004f * height);
        const glm::vec3 elbow = shoulder + down * (a::kUpperArmLength * height);
        const glm::vec3 wrist = elbow + down * (a::kForearmLength * height);
        position[Index(leftSide ? Joint::ClavicleL : Joint::ClavicleR)] = clavicle;
        position[Index(leftSide ? Joint::UpperArmL : Joint::UpperArmR)] = shoulder;
        position[Index(leftSide ? Joint::ForearmL : Joint::ForearmR)] = elbow;
        position[Index(leftSide ? Joint::HandL : Joint::HandR)] = wrist;

        const float hipX = side * hipHalfWidth * height;
        const glm::vec3 hip(hipX, hipHeight * height, 0.0f);
        const glm::vec3 knee(hipX, kneeHeight * height, 0.0f);
        const glm::vec3 ankle(hipX * 1.15f, a::kAnkleHeight * height, 0.0f);
        const glm::vec3 ball(ankle.x, 0.0f, a::kAnkleToBall * height);
        position[Index(leftSide ? Joint::ThighL : Joint::ThighR)] = hip;
        position[Index(leftSide ? Joint::CalfL : Joint::CalfR)] = knee;
        position[Index(leftSide ? Joint::FootL : Joint::FootR)] = ankle;
        position[Index(leftSide ? Joint::ToesL : Joint::ToesR)] = ball;

        FootContacts& foot = leftSide ? left : right;
        foot.heel = {ankle.x, 0.0f, -a::kAnkleToHeel * height};
        foot.ball = ball;
        foot.toe = {ankle.x, 0.0f, a::kAnkleToToe * height};
    }
    return position;
}

}  // namespace

glm::quat FromDegrees(float aboutX, float aboutY, float aboutZ) {
    const glm::quat x = glm::angleAxis(aboutX * kDegreesToRadians, glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::quat y = glm::angleAxis(aboutY * kDegreesToRadians, glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::quat z = glm::angleAxis(aboutZ * kDegreesToRadians, glm::vec3(0.0f, 0.0f, 1.0f));
    return z * y * x;
}

Skeleton BuildSkeleton(const CharacterParameters& parameters) {
    const CharacterParameters clean = Sanitised(parameters);
    Skeleton skeleton;
    skeleton.statureMeters = clean.statureMeters;
    const std::array<glm::vec3, kJointCount> position = RestPositions(clean, skeleton.leftFoot, skeleton.rightFoot);
    for (int i = 0; i < kJointCount; ++i) {
        JointInfo& joint = skeleton.joints[i];
        joint.name = kSpecs[i].name;
        joint.parent = kSpecs[i].parent == Joint::Count ? -1 : Index(kSpecs[i].parent);
        joint.restPosition = position[i];
        joint.restOffset = joint.parent < 0 ? position[i] : position[i] - position[joint.parent];
        joint.inverseBind = glm::translate(glm::mat4(1.0f), -position[i]);
        joint.minAngleDegrees = kSpecs[i].minAngle;
        joint.maxAngleDegrees = kSpecs[i].maxAngle;
    }
    return skeleton;
}

std::array<glm::mat4, kJointCount> SkinMatrices(const Skeleton& skeleton, const Pose& pose) {
    std::array<glm::mat4, kJointCount> world{};
    std::array<glm::mat4, kJointCount> skin{};
    for (int i = 0; i < kJointCount; ++i) {
        const JointInfo& joint = skeleton.joints[i];
        glm::mat4 local = glm::translate(glm::mat4(1.0f), joint.restOffset) * glm::mat4_cast(pose.rotation[i]);
        if (joint.parent < 0) {
            local = glm::translate(glm::mat4(1.0f), pose.rootTranslation) * local;
            world[i] = local;
        } else {
            world[i] = world[joint.parent] * local;
        }
        skin[i] = world[i] * joint.inverseBind;
    }
    return skin;
}

std::array<glm::vec3, kJointCount> JointPositions(const Skeleton& skeleton, const Pose& pose) {
    const std::array<glm::mat4, kJointCount> skin = SkinMatrices(skeleton, pose);
    std::array<glm::vec3, kJointCount> positions{};
    for (int i = 0; i < kJointCount; ++i) {
        positions[i] = glm::vec3(skin[i] * glm::vec4(skeleton.joints[i].restPosition, 1.0f));
    }
    return positions;
}

}  // namespace character
