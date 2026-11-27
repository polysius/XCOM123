#pragma once

#include <glm/glm.hpp>

#include "animation/BodyModel.h"
#include "animation/Kinematics.h"

// Sitting down and standing up, and planting a foot on the ground.
//
// The motion is made as a person does it: the feet stay where they are, the trunk leans forward to bring the weight over
// the feet, the hips go back and down onto the seat, and the trunk comes upright. Inverse kinematics keeps the feet on the
// ground and bends the knees.

namespace animation {

// Plants a foot flat on the ground with the heel at (heelX, heelZ), pointing along `yaw`, and solves the leg to it. The pelvis
// must already be where it is wanted. Returns whether the leg could reach.
bool PlantFootOnGround(const BodyModel& body, Pose& pose, bool left, float heelX, float heelZ, float yaw, const GroundFunction& ground);

struct Seat {
    glm::vec3 center{0.0f};  // the middle of the seat surface, in the world
    float facing = 0.0f;     // the direction a person on the seat faces: 0 is +Z, positive turns towards +X
};

// The pose of a person at `progress` from 0 (standing in front of the seat, with their back to it, facing the way that the
// seated person will face) to 1 (seated). Played from 1 to 0 it is standing up. The feet are planted on the ground, and do
// not move at any progress.
Pose SitPose(const BodyModel& body, const Seat& seat, float progress, const GroundFunction& ground = {});

// Where the heels of a person who sits on the seat are, in the world, left and right.
glm::vec3 SitHeel(const BodyModel& body, const Seat& seat, bool left, const GroundFunction& ground = {});

}  // namespace animation
