#include "animation/SitStand.h"

#include <algorithm>
#include <cmath>

#include "animation/JointLimits.h"
#include "animation/TwoBoneIk.h"

namespace animation {
namespace {

constexpr float kMaxLeanDegrees = 32.0f;

float GroundAt(const GroundFunction& ground, float x, float z) {
    return ground ? ground(x, z) : 0.0f;
}

float Smoothstep01(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

glm::vec3 Forward(float heading) {
    return glm::vec3(std::sin(heading), 0.0f, std::cos(heading));
}

glm::vec3 LeftOf(float heading) {
    return glm::vec3(std::cos(heading), 0.0f, -std::sin(heading));
}

glm::quat AboutX(float degrees) {
    return glm::angleAxis(degrees * kDegreesToRadians, glm::vec3(1.0f, 0.0f, 0.0f));
}

// The pitch of the ground along a foot that has its heel at (x, z) and points along `heading`.
float GroundPitch(const BodyModel& body, const GroundFunction& ground, float x, float z, float heading) {
    const glm::vec3 ahead = Forward(heading) * body.ballFromHeel;
    return std::atan2(GroundAt(ground, x + ahead.x, z + ahead.z) - GroundAt(ground, x, z), body.ballFromHeel);
}

// Where the ankles are when seated: below the knees, which are a thigh's length in front of the hips.
glm::vec2 SeatedAnkleXZ(const BodyModel& body, const Seat& seat, bool left) {
    const glm::vec3 spot = seat.center + Forward(seat.facing) * (0.9f * body.thighLength) + LeftOf(seat.facing) * ((left ? 1.0f : -1.0f) * body.footHalfWidth);
    return glm::vec2(spot.x, spot.z);
}

}  // namespace

bool PlantFootOnGround(const BodyModel& body, Pose& pose, bool left, float heelX, float heelZ, float yaw, const GroundFunction& ground) {
    const float slope = GroundPitch(body, ground, heelX, heelZ, yaw);
    const glm::quat footRotation = FootOrientation(yaw, slope);
    const glm::vec3 heel(heelX, GroundAt(ground, heelX, heelZ), heelZ);
    const glm::vec3 ankle = heel - footRotation * body.heelOffset;

    const bool reached = SolveTwoBone(body.skeleton, pose, LegLimb(left), ankle, Forward(yaw)).reached;
    const Joint foot = left ? Joint::FootL : Joint::FootR;
    SetWorldRotation(body.skeleton, pose, foot, footRotation);
    pose.rotation[character::Index(foot)] = ClampToLimits(body.skeleton, foot, pose.rotation[character::Index(foot)]);
    return reached;
}

glm::vec3 SitHeel(const BodyModel& body, const Seat& seat, bool left, const GroundFunction& ground) {
    const glm::vec2 ankle = SeatedAnkleXZ(body, seat, left);
    const glm::vec3 heel = glm::vec3(ankle.x, 0.0f, ankle.y) + Forward(seat.facing) * body.heelOffset.z;
    return glm::vec3(heel.x, GroundAt(ground, heel.x, heel.z), heel.z);
}

Pose SitPose(const BodyModel& body, const Seat& seat, float progress, const GroundFunction& ground) {
    const Skeleton& skeleton = body.skeleton;
    const float eased = Smoothstep01(progress);
    // The weight comes over the feet, and then back: the lean is a bump that is nothing at both ends.
    const float lean = std::sin(3.14159265f * eased) * kMaxLeanDegrees;

    Pose pose;
    const glm::vec2 middleOfAnkles = 0.5f * (SeatedAnkleXZ(body, seat, true) + SeatedAnkleXZ(body, seat, false));
    const float groundHeight = GroundAt(ground, middleOfAnkles.x, middleOfAnkles.y);

    // The pelvis goes from over the feet, standing, to over the seat, with the hip joint a hand's breadth above the surface.
    const float standingHeight = groundHeight + body.ankleHeight + 0.997f * (body.thighLength + body.shankLength);
    const float seatedHeight = seat.center.y + 0.058f * skeleton.statureMeters;
    const glm::vec3 standing(middleOfAnkles.x, standingHeight, middleOfAnkles.y);
    const glm::vec3 seated(seat.center.x, seatedHeight, seat.center.z);
    const glm::vec3 pelvis = glm::mix(standing, seated, eased);
    SetPelvisPosition(skeleton, pose, pelvis);

    // The trunk bends forward over the hips, and the head keeps looking ahead.
    pose.rotation[character::Index(Joint::Pelvis)] = glm::normalize(YawRotation(seat.facing) * AboutX(0.25f * lean));
    pose.rotation[character::Index(Joint::Spine1)] = AboutX(0.30f * lean);
    pose.rotation[character::Index(Joint::Spine2)] = AboutX(0.30f * lean);
    pose.rotation[character::Index(Joint::Neck)] = AboutX(0.15f * lean);
    pose.rotation[character::Index(Joint::Head)] = AboutX(-0.60f * lean);

    // The arms reach forward for balance while the trunk leans, and settle with the hands on the lap.
    for (const bool left : {true, false}) {
        const float side = left ? 1.0f : -1.0f;
        const float swing = -(30.0f * std::sin(3.14159265f * eased) + 14.0f * eased);
        const float bringIn = (40.0f - 8.0f) * -side;
        const Joint upperArm = left ? Joint::UpperArmL : Joint::UpperArmR;
        const Joint forearm = left ? Joint::ForearmL : Joint::ForearmR;
        pose.rotation[character::Index(upperArm)] = AboutX(swing) * glm::angleAxis(bringIn * kDegreesToRadians, glm::vec3(0.0f, 0.0f, 1.0f));
        pose.rotation[character::Index(forearm)] = character::HingeRotation(skeleton, forearm, 12.0f + 60.0f * eased);
    }

    // The feet stay where they were put, flat on the ground, and the legs bend to the pelvis.
    for (const bool left : {true, false}) {
        const glm::vec3 heel = SitHeel(body, seat, left, ground);
        PlantFootOnGround(body, pose, left, heel.x, heel.z, seat.facing, ground);
    }
    return pose;
}

}  // namespace animation
