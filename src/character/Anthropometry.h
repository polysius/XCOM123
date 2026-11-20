#pragma once

// Proportions of the adult body as fractions of stature, and the density of the body. The segment lengths and the
// heights of the joints are those of Drillis and Contini (1966), tabulated in Winter, Biomechanics and Motor Control
// of Human Movement; they are what the tests check the skeleton against. The remaining numbers are used to place joints
// that the table does not give, and are measured in the same way.

namespace character::anthropometry {

// Heights of landmarks above the floor, as fractions of stature (Drillis and Contini).
constexpr float kAnkleHeight = 0.039f;
constexpr float kKneeHeight = 0.285f;
constexpr float kHipHeight = 0.530f;
constexpr float kShoulderHeight = 0.818f;
constexpr float kChinHeight = 0.870f;

// Lengths of segments, as fractions of stature (Drillis and Contini).
constexpr float kUpperArmLength = 0.186f;
constexpr float kForearmLength = 0.146f;
constexpr float kHandLength = 0.108f;
constexpr float kThighLength = 0.245f;
constexpr float kShankLength = 0.246f;
constexpr float kFootLength = 0.152f;
constexpr float kShoulderWidth = 0.259f;  // across the deltoids
constexpr float kHipWidth = 0.191f;       // across the hips at the trochanters

// Joint centres and landmarks the table does not give, as fractions of stature.
constexpr float kNeckBaseHeight = 0.820f;    // the seventh neck vertebra
constexpr float kSkullBaseHeight = 0.880f;   // where the skull sits on the spine
constexpr float kShoulderJointHalfWidth = 0.105f;
constexpr float kHipJointHalfWidth = 0.052f;
constexpr float kAnkleToBall = 0.082f;       // forwards from the ankle to the ball of the foot
constexpr float kAnkleToToe = 0.112f;
constexpr float kAnkleToHeel = 0.040f;       // backwards from the ankle to the back of the heel

// The mean density of a living body, kg per cubic metre. It decides the mass of a mesh of a given volume.
constexpr float kBodyDensity = 1010.0f;

}  // namespace character::anthropometry
