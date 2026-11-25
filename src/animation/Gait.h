#pragma once

#include <functional>

#include <glm/glm.hpp>

#include "animation/BodyLayer.h"
#include "animation/BodyModel.h"
#include "animation/IdleMotion.h"
#include "animation/Kinematics.h"
#include "animation/Personality.h"

// Procedural walking and running.
//
// The gait is built the way a person walks: the feet are the fixed points. When a foot lands, it is planted on the ground
// at a place in the world and stays there, rolling from the heel over the sole to the ball, until the body has passed over
// it; then it swings forward to the next place. The pelvis is driven by speed, and the legs follow by inverse kinematics.
// Because a planted foot is held at one point in the world, it cannot slide, whatever the speed, the turning or the slope.
//
// How long the stride is and how long the foot stays down come from the speed and the length of the leg. In one cycle
// (two steps) the body travels `strideLength = speed * period`, and the foot is on the ground for a fraction of it, the duty
// factor: above one half is walking, below it there is a moment with both feet in the air, running.

namespace animation {

struct LocomotionCommand {
    float speed = 0.0f;     // metres a second, forwards
    float turnRate = 0.0f;  // radians a second about the vertical; positive turns towards +X, the character's left
};

// The shape of the gait at a speed, for a body and a personality.
struct GaitShape {
    float speed = 0.0f;
    float runBlend = 0.0f;       // 0 walking, 1 running, smooth between them
    float cadence = 0.0f;        // steps a second
    float period = 1.0f;         // seconds for a cycle of two steps
    float strideLength = 0.0f;   // metres travelled in a period: speed times period
    float dutyFactor = 0.6f;     // the fraction of a period a foot spends on the ground
    float stepHalfWidth = 0.0f;  // the sideways distance from the middle line to each foot
    float footClearance = 0.0f;  // how high the ankle lifts above the line between the two footfalls
    float footLead = 0.0f;       // how far in front of the middle of the pelvis the ankle lands, along the direction of travel
    float rollAmount = 0.0f;     // 0 for tiny steps, 1 from half a metre of step: scales the heel strike and toe-off
};

GaitShape ComputeGaitShape(const BodyModel& body, const Personality& personality, float speed);

// The fastest speed the gait supports for this body: a sprint scaled by the length of the leg.
float MaximumSpeed(const BodyModel& body);

struct WalkerSettings {
    float acceleration = 2.5f;   // metres a second, each second
    float deceleration = 3.5f;
    float turnResponse = 6.0f;   // how fast the turning rate follows the command, 1 over seconds
    uint32_t seed = 1;           // of the idle motion
};

// A character that walks, runs and stands. Each Update moves it on by a time step and makes its pose.
class Walker {
public:
    Walker(const BodyModel& body, const Personality& personality, GroundFunction ground = {}, const WalkerSettings& settings = {});

    // Puts the character, standing, at a place on the ground facing a direction (0 is +Z; positive turns towards +X).
    void Reset(const glm::vec3& position, float headingRadians);

    // Advances by `dt` seconds towards the command. The speed and the turning follow the command with limited
    // acceleration, so a sudden change of command is a smooth change of motion.
    void Update(float dt, const LocomotionCommand& command);

    // A layer added on top of the gait from now on (for example a reaction to a push); the idle motion is separate.
    void SetLayer(const BodyLayer& layer);

    const Pose& CurrentPose() const { return pose; }
    const GaitShape& Shape() const { return shape; }
    const BodyModel& Body() const { return body; }
    glm::vec3 Position() const { return position; }  // the point on the ground below the middle of the body's path
    float Heading() const { return heading; }
    float Speed() const { return speed; }
    float TurnRate() const { return turnRate; }
    float Phase() const { return phase; }  // 0 to 1: how far through the cycle of the left foot
    double Time() const { return time; }

    // True while a foot is planted: the part of the cycle from heel strike to toe-off.
    bool IsPlanted(bool left) const { return feet[left ? 0 : 1].planted; }
    // False if, at the last update, the leg could not reach where the gait put the ankle (it is then stretched towards it).
    bool LegReachesFoot(bool left) const { return feet[left ? 0 : 1].legReachesFoot; }
    // How many times a foot has landed since Reset.
    int FootfallCount(bool left) const { return feet[left ? 0 : 1].footfalls; }

private:
    // Where an ankle is and how the foot is turned.
    struct FootPose {
        glm::vec3 ankle{0.0f};
        float yaw = 0.0f;
        float pitch = 0.0f;  // of the sole against the horizontal, in radians, toes up positive
        float slope = 0.0f;  // the part of the pitch that is the slope of the ground under the foot
        float toeFlex = 0.0f;  // of the toes joint about X, in radians
    };
    // Where a foot lands or is planted: the heel on the ground and how the foot lies.
    struct Footfall {
        glm::vec3 heel{0.0f};
        float yaw = 0.0f;
        float slope = 0.0f;  // the pitch of the ground under the foot along its length
    };
    struct FootTrack {
        bool planted = true;
        float stanceProgress = 0.0f;  // 0 at heel strike, 1 at toe-off, and 0 while the foot swings
        float swingLift = 0.0f;       // 0 on the ground; 1 through most of a swing; fades out as the foot lands
        float liftCorrection = 0.0f;  // extra lift, in metres, that the pose needed (see PoseLeg)
        bool legReachesFoot = true;   // whether the leg could put the ankle where the gait wanted it at the last update
        int footfalls = 0;
        Footfall footfall;   // where the foot is planted, or is going to land
        FootPose swingStart; // the pose at toe-off
        FootPose state;      // the pose now
    };

    float GroundHeight(float x, float z) const;
    float GroundPitch(const glm::vec3& heel, float yaw) const;
    Footfall PredictFootfall(bool left, float secondsAhead) const;
    FootPose FootAtFootfall(const Footfall& footfall, float rollPitch) const;
    FootPose PlantedFoot(const FootTrack& foot, float stanceProgress) const;
    FootPose PlantedFootAtPitch(const FootTrack& foot, float stanceProgress, float pitch) const;
    float TerrainDeficit(const glm::vec3& ankle, const glm::quat& footRotation) const;
    void PoseLeg(Pose& built, bool left, float dt);
    FootPose SwingingFoot(const FootTrack& foot, float swingProgress, const Footfall& landing) const;
    void UpdateFoot(bool left, float cyclePosition);
    void Step(float dt, float wantedSpeed, float wantedTurnRate);
    float PelvisHeight(float dt, const glm::vec3& pelvisPosition, const glm::quat& pelvisWorld, float moveWeight, float pelvisPitchDegrees, float extraHeight);
    void BuildPose(float dt);

    BodyModel body;
    Personality personality;
    GroundFunction ground;
    WalkerSettings settings;
    IdleMotion idle;
    BodyLayer layer;

    glm::vec3 position{0.0f};
    float heading = 0.0f;
    float speed = 0.0f;
    float acceleration = 0.0f;
    float turnRate = 0.0f;
    float phase = 0.0f;
    double time = 0.0;  // a double: a float stops counting after about a million seconds
    GaitShape shape;
    float pelvisHeightState = 0.0f;  // the smoothed height of the pelvis
    bool pelvisHeightKnown = false;
    FootTrack feet[2];  // left, right
    Pose pose;
};

}  // namespace animation
