#pragma once

#include <glm/glm.hpp>

#include "character/CharacterParameters.h"
#include "character/Skeleton.h"

// The measurements of a body that the gait needs, taken from the skeleton: lengths of the leg and where the foot touches
// the ground. Nothing is assumed about the proportions; a tall person and a short one both read their own skeleton.

namespace animation {

struct BodyModel {
    character::CharacterParameters parameters;  // sanitised
    character::Skeleton skeleton;
    float legLength = 0.0f;       // from the floor to the hip joint, standing: the leg that sets the stride
    float thighLength = 0.0f;
    float shankLength = 0.0f;
    float ankleHeight = 0.0f;     // of the ankle joint above the floor, at rest
    float footHalfWidth = 0.0f;   // how far the ankles stand from the middle, at rest
    float ballFromHeel = 0.0f;    // along the sole
    // From the ankle to the points of the sole that touch the ground, in the frame of the foot at rest (X across, Y up,
    // Z forwards). They are the same for both feet.
    glm::vec3 heelOffset{0.0f};
    glm::vec3 ballOffset{0.0f};
    glm::vec3 toeOffset{0.0f};
    glm::vec3 hipOffsetLeft{0.0f};  // from the pelvis joint to the hip joint
    glm::vec3 hipOffsetRight{0.0f};
};

BodyModel MakeBodyModel(const character::CharacterParameters& parameters);

}  // namespace animation
