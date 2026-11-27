#pragma once

#include <glm/glm.hpp>

#include "animation/BodyModel.h"
#include "animation/Kinematics.h"

// Looking at a point: the neck and the head turn towards it, as far as a person can turn them, and the eyes take the rest.
//
// The skeleton of src/character has no eye joints, so the eyes are not posed: the result carries the angles the two eyes
// would have in the head, for a renderer or a face rig to use.

namespace animation {

struct LookAtSettings {
    float neckShare = 0.4f;          // of the turn of the head and neck that the neck takes; the head takes the rest
    float eyeShare = 0.25f;          // of the turn that the eyes try to take, before the head and neck turn the remainder
    float maxEyeYawDegrees = 35.0f;  // how far an eye can turn from straight ahead in the head
    float maxEyePitchDegrees = 25.0f;
    float eyeSeparationMeters = 0.063f;
};

// The direction of an eye in the head, from straight ahead: yaw towards +X (the character's left) and pitch up, in degrees.
struct EyeAngles {
    float yawDegrees = 0.0f;
    float pitchDegrees = 0.0f;
};

struct LookAtResult {
    bool reachedWithHeadAndEyes = false;  // the gaze is on the target within a degree
    float errorDegrees = 0.0f;            // the angle between the gaze and the direction of the target
    EyeAngles leftEye;
    EyeAngles rightEye;
};

// Turns the neck and the head of `pose` to look at `targetWorld`. `weight` from 0 to 1 blends from the pose as it is to the
// full look. The pose below the neck must already be as it is wanted (the chest decides where the neck starts), and the neck
// and head are replaced. The joints stay inside their limits; a target behind the character is looked at as far as possible.
LookAtResult LookAt(const BodyModel& body, Pose& pose, const glm::vec3& targetWorld, float weight = 1.0f, const LookAtSettings& settings = {});

// The point between the eyes in a pose, in the world.
glm::vec3 EyeCentre(const BodyModel& body, const Pose& pose);

// The direction the head faces in a pose, in the world.
glm::vec3 HeadForward(const BodyModel& body, const Pose& pose);

}  // namespace animation
