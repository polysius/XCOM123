#pragma once

#include "character/CharacterParameters.h"

// How a particular person moves. The numbers follow from the body (sex, age, mass) and from the mood of the moment, and
// they only scale or shift what the gait does; a person of any kind walks in the same way, a little differently.

namespace animation {

// The mood of the moment, each from -1 to 1.
struct Mood {
    float energy = 0.0f;      // -1 worn out, 1 brisk: stride, tempo, arm swing and bounce
    float confidence = 0.0f;  // -1 downcast, 1 proud: the posture, the head, the stride
};

struct Personality {
    float strideScale = 1.0f;          // on the length of a step at a given speed
    float tempoScale = 1.0f;           // on the number of steps a second at a given speed
    float armSwingScale = 1.0f;
    float pelvisRotationScale = 1.0f;  // how far the hips turn about the vertical axis
    float pelvisSwayScale = 1.0f;      // how far the hips shift and drop from side to side
    float stepWidthScale = 1.0f;
    float footClearanceScale = 1.0f;
    float dutyFactorOffset = 0.0f;     // added to the fraction of the cycle a foot is on the ground
    float bounceScale = 1.0f;
    float stoopDegrees = 0.0f;         // how far the trunk leans forwards on top of what the speed asks for
    float headDropDegrees = 0.0f;      // how far the head hangs forwards of the trunk
    float standingHeightScale = 1.0f;  // how straight the legs stand: below 1 the knees stay a little bent
};

// A person's way of moving from their parameters (see character/CharacterParameters.h) and their mood. A man's stride is
// a little longer and a woman's hips turn and sway a little more; with age the stride shortens, the arms swing less, the
// trunk bends forward and the feet stay lower; a heavy body stands wider and sways more and steps shorter.
Personality PersonalityFrom(const character::CharacterParameters& parameters, const Mood& mood = {});

}  // namespace animation
