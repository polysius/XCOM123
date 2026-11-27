#include "animation/Gait.h"

#include <algorithm>
#include <cmath>

#include "animation/JointLimits.h"
#include "animation/TwoBoneIk.h"

namespace animation {
namespace {

constexpr float kPi = 3.14159265359f;
constexpr float kTwoPi = 6.28318530718f;
constexpr float kGravity = 9.81f;
// Updates are made in steps of at most a sixtieth of a second, and an update never covers more than half a second.
constexpr float kMaxStep = 1.0f / 60.0f;
constexpr float kLongestUpdate = 0.5f;
// The quickest the steps of a turn on the spot can be, a cycle of two steps in seconds.
constexpr float kShortestPeriod = 0.3f;
// A foot that swings keeps at least this part of its walking lift, so that a short step is never dragged along the floor.
constexpr float kMinimumLiftShare = 0.6f;
// The turn of the pelvis, in radians, that a planted foot is asked to stand through, from landing to toe-off.
constexpr float kTurnOverStance = 0.6f;
// How fast the lift that a swinging foot needs over and above its planned path may change, in metres a second.
constexpr float kCorrectionSpeed = 2.2f;
constexpr float kReferenceLegLength = 0.92f;  // of the person the cadence and speed numbers below were fitted on

// The pitch of the foot, toes up, at heel strike and at toe-off, at full roll (the toes can only bend up 10 degrees, so toe-off is modest).
constexpr float kHeelStrikePitch = 18.0f * kDegreesToRadians;
constexpr float kToesBendUp = 9.5f * kDegreesToRadians;  // the toes joint bends up 10 degrees
constexpr float kToeOffPitch = 20.0f * kDegreesToRadians;
// The fractions of the stance at which the heel has come down, and at which the heel starts to rise again.
constexpr float kHeelDownBy = 0.15f;
constexpr float kHeelRisesFrom = 0.40f;
constexpr float kToeOutRadians = 0.10f;
// How near to straight the knee may get before the pelvis has to come down: a knee is never quite locked, and this keeps
// the solver clear of a fully stretched leg.
constexpr float kLongestLegReach = 0.997f;
// The time, in seconds, the height of the pelvis takes to follow the pendulum of the planted leg.
// How far above the ground a swinging foot keeps its sole, in metres.
constexpr float kSwingGroundMargin = 0.2f;  // of the foot clearance, so a foot that barely lifts is not lifted by it
constexpr float kPelvisHeightSmoothing = 0.07f;

float Smoothstep(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float SmootherStep(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

float Lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

// How a leg of two bones of length `upper` and `lower` is turned back at the hip when the ankle is `backward` behind the hip,
// `lateral` to the side of it and `height` below it, with the knee bent forwards. The angle is that of the thigh from the
// vertical, in the plane of walking: how far the hip is extended.
float HipExtension(float upper, float lower, float backward, float lateral, float height) {
    const float distance = std::sqrt(backward * backward + lateral * lateral + height * height);
    const float cosineAtHip = (upper * upper + distance * distance - lower * lower) / (2.0f * upper * distance);
    const float bendAtHip = std::acos(std::clamp(cosineAtHip, -1.0f, 1.0f));
    return std::atan2(backward, height) - bendAtHip;
}

// The highest the hip can be above the ankle without overstretching the leg and without extending the hip past `maxExtension`.
// Lowering the pelvis bends the knee, which brings the thigh forward again, so the extension is found by bisection.
float HighestHipAboveAnkle(float upper, float lower, float backward, float lateral, float maxExtension) {
    const float longest = kLongestLegReach * (upper + lower);
    const float across2 = backward * backward + lateral * lateral;
    const float stretched = std::sqrt(std::max(longest * longest - across2, 0.0f));
    if (HipExtension(upper, lower, backward, lateral, stretched) <= maxExtension) {
        return stretched;
    }
    // Walk down from the stretched leg to the first height that is allowed (the extension falls as the knee bends, but
    // rises again if the leg is folded right up), then close in on the edge.
    constexpr int kScanSteps = 24;
    float high = stretched;
    float low = stretched;
    for (int step = 1; step <= kScanSteps; ++step) {
        low = stretched * (1.0f - static_cast<float>(step) / kScanSteps);
        if (HipExtension(upper, lower, backward, lateral, low) <= maxExtension) {
            break;
        }
        high = low;
    }
    for (int i = 0; i < 16; ++i) {
        const float middle = 0.5f * (low + high);
        (HipExtension(upper, lower, backward, lateral, middle) <= maxExtension ? low : high) = middle;
    }
    return low;
}

// A smooth minimum: like std::min, but without a corner where the two are equal (the log of a sum of exponentials). It is
// never above the smaller of the two, and below it by at most `softness` times 0.69, so a limit that is smoothed this way is still kept.
float SmoothMin(float a, float b, float softness) {
    return std::min(a, b) - softness * std::log1p(std::exp(-std::abs(a - b) / softness));
}

float LerpAngle(float from, float to, float t) {
    float difference = std::fmod(to - from, kTwoPi);
    if (difference > kPi) {
        difference -= kTwoPi;
    } else if (difference < -kPi) {
        difference += kTwoPi;
    }
    return from + difference * t;
}

glm::vec3 Forward(float heading) {
    return glm::vec3(std::sin(heading), 0.0f, std::cos(heading));
}

// The character's left, which is +X when it faces +Z.
glm::vec3 LeftOf(float heading) {
    return glm::vec3(std::cos(heading), 0.0f, -std::sin(heading));
}

glm::quat AboutX(float degrees) {
    return glm::angleAxis(degrees * kDegreesToRadians, glm::vec3(1.0f, 0.0f, 0.0f));
}

glm::quat FootRotation(float yaw, float pitch) {
    return FootOrientation(yaw, pitch);
}

// The angle the walking and running shapes use for a speed: the speed of a person with the reference leg that moves like this one.
float SimilarSpeed(const BodyModel& body, float speed) {
    return speed / std::sqrt(body.legLength / kReferenceLegLength);
}

// ---------------------------------------------------------------------------
// The upper body, driven by the phase of the gait
// ---------------------------------------------------------------------------

struct TorsoAngles {
    float pelvisPitch = 0.0f, pelvisYaw = 0.0f, pelvisRoll = 0.0f;  // degrees, in the frame of the body
    float spine1Pitch = 0.0f, spine1Yaw = 0.0f, spine1Roll = 0.0f;
    float spine2Pitch = 0.0f, spine2Yaw = 0.0f, spine2Roll = 0.0f;
    float neckPitch = 0.0f, neckYaw = 0.0f;
    float headPitch = 0.0f, headYaw = 0.0f, headRoll = 0.0f;
    float sway = 0.0f;  // metres: how far the pelvis has shifted to the left
};

struct TorsoInputs {
    float cycle;        // the phase of the left foot, in radians
    float swayCycle;    // the same, shifted so that it is 0 when the left foot is in mid-stance
    float speed;        // similar-body speed
    float runBlend;
    float moveWeight;   // 0 standing, 1 moving: scales every rhythmic motion
    float acceleration; // metres a second, each second
    float turnRate;
    float slopeDegrees; // of the ground along the heading, positive uphill
};

TorsoAngles TorsoMotion(const Personality& personality, const TorsoInputs& in) {
    const float walk = std::min(in.speed, 2.2f);
    const float cosine = std::cos(in.cycle);
    const float swayCosine = std::cos(in.swayCycle);
    const float move = in.moveWeight;
    TorsoAngles t;

    // The hips turn about the vertical axis, the left hip forward when the left foot is, and the shoulders turn the other way.
    const float pelvisYawAmplitude = Lerp(3.5f + 1.2f * walk, 6.0f, in.runBlend) * personality.pelvisRotationScale * move;
    const float shoulderYawAmplitude = Lerp(5.0f + 1.5f * walk, 11.0f, in.runBlend) * personality.pelvisRotationScale * move;
    t.pelvisYaw = -pelvisYawAmplitude * cosine;
    const float spineTwist = (shoulderYawAmplitude + pelvisYawAmplitude) * cosine;
    t.spine1Yaw = 0.4f * spineTwist;
    t.spine2Yaw = 0.6f * spineTwist;
    // The head looks where the body goes: it takes back most of the turn of the shoulders.
    t.neckYaw = -0.3f * shoulderYawAmplitude * cosine;
    t.headYaw = -0.3f * shoulderYawAmplitude * cosine;

    // The pelvis shifts over the foot that carries the weight and drops on the side of the foot that swings; the trunk leans back over it.
    t.sway = 0.022f * (1.0f - 0.5f * in.runBlend) * personality.pelvisSwayScale * move * swayCosine;
    t.pelvisRoll = 3.5f * (1.0f - 0.5f * in.runBlend) * personality.pelvisSwayScale * move * swayCosine;
    t.spine1Roll = -0.4f * t.pelvisRoll;
    t.spine2Roll = -0.5f * t.pelvisRoll;
    t.headRoll = 0.3f * t.pelvisRoll;

    // The trunk leans forwards with speed, with acceleration, uphill and with a stoop; the head keeps looking ahead.
    const float accelerationLean = std::clamp(1.2f * in.acceleration, -4.0f, 4.0f);
    const float slopeLean = std::clamp(0.6f * in.slopeDegrees, -8.0f, 12.0f);
    const float lean = personality.stoopDegrees + (1.5f + 6.5f * in.runBlend) * move + accelerationLean * move + slopeLean;
    t.pelvisPitch = 0.25f * lean;
    t.spine1Pitch = 0.30f * lean;
    t.spine2Pitch = 0.30f * lean;
    t.neckPitch = 0.15f * lean;
    t.headPitch = -0.70f * lean + personality.headDropDegrees;

    // Into a turn: the whole body leans towards the inside of the curve, as a cyclist does.
    const float turnLean = -std::clamp(std::atan(in.speed * in.turnRate / kGravity) * kRadiansToDegrees * 0.55f, -12.0f, 12.0f);
    t.pelvisRoll += 0.3f * turnLean;
    t.spine1Roll += 0.3f * turnLean;
    t.spine2Roll += 0.3f * turnLean;
    t.headRoll -= 0.5f * turnLean;
    return t;
}

// An upper arm swung about the X axis (positive backwards) after being brought in from the A-pose to hang `hangDegrees` from the body.
glm::quat UpperArmRotation(bool left, float swingDegrees, float hangDegrees) {
    const float bringIn = (40.0f - hangDegrees) * (left ? -1.0f : 1.0f);
    return AboutX(swingDegrees) * glm::angleAxis(bringIn * kDegreesToRadians, glm::vec3(0.0f, 0.0f, 1.0f));
}

}  // namespace

float MaximumSpeed(const BodyModel& body) {
    // Faster than this the stance legs would have to reach farther behind than the hip can extend and in front than it can bend.
    return 3.2f * std::sqrt(body.legLength / kReferenceLegLength);
}

float MaximumTurnRate(const BodyModel& body, float speed) {
    const float similar = SimilarSpeed(body, std::max(speed, 0.0f));
    const float walking = Lerp(2.5f, 0.75f, Smoothstep(0.0f, 1.2f, similar));
    return walking * (1.0f - 0.4f * Smoothstep(1.9f, 2.6f, similar));
}

GaitShape ComputeGaitShape(const BodyModel& body, const Personality& personality, float speed, float turnRate) {
    GaitShape shape;
    speed = std::isfinite(speed) ? speed : 0.0f;
    turnRate = std::isfinite(turnRate) ? turnRate : 0.0f;
    shape.speed = std::clamp(speed, 0.0f, MaximumSpeed(body));
    const float similar = SimilarSpeed(body, shape.speed);
    const float legScale = std::sqrt(body.legLength / kReferenceLegLength);
    shape.runBlend = Smoothstep(1.9f, 2.6f, similar);

    // Steps a second: a walker takes more steps the faster they go, a runner a few more again. Longer legs swing slower.
    const float walkCadence = 1.2f + 0.5f * std::min(similar, 2.6f);
    const float runCadence = 2.45f + 0.15f * similar;
    shape.cadence = Lerp(walkCadence, runCadence, shape.runBlend) / legScale * personality.tempoScale / personality.strideScale;
    shape.period = 2.0f / shape.cadence;

    // The share of the cycle on the ground: about 0.6 for a slow walk, shrinking through 0.5 to 0.3 in a run.
    const float walkDuty = std::clamp(0.64f - 0.07f * similar, 0.50f, 0.64f);
    const float runDuty = std::clamp(0.42f - 0.025f * (similar - 2.5f), 0.26f, 0.42f);
    shape.dutyFactor = std::clamp(Lerp(walkDuty, runDuty, shape.runBlend) + personality.dutyFactorOffset, 0.24f, 0.70f);

    // A foot is planted for `duty` of the period, and the pelvis turns by turnRate over that time: quicker steps for quicker turns.
    if (std::abs(turnRate) > 1e-3f) {
        const float quickestAllowed = kTurnOverStance / (std::abs(turnRate) * shape.dutyFactor);
        shape.period = std::min(shape.period, std::max(quickestAllowed, kShortestPeriod));
        shape.cadence = 2.0f / shape.period;
    }
    shape.strideLength = shape.speed * shape.period;

    shape.stepHalfWidth = body.footHalfWidth * personality.stepWidthScale * (1.0f - 0.35f * shape.runBlend);
    shape.rollAmount = Smoothstep(0.05f, 0.5f, 0.5f * shape.strideLength);
    const float liftScale = Smoothstep(0.0f, 0.35f, 0.5f * shape.strideLength);
    shape.footClearance = body.legLength * (0.045f + 0.10f * shape.runBlend) * personality.footClearanceScale * Lerp(kMinimumLiftShare, 1.0f, liftScale);
    // The foot lands ahead of the hip by a quarter of the stride (half a step) when the other foot is still down; in a run,
    // with nothing else on the ground, it lands half of the contact ahead, so that the contact is symmetric about the hip.
    shape.footLead = shape.strideLength * 0.5f * shape.dutyFactor;
    return shape;
}

Walker::Walker(const BodyModel& body_, const Personality& personality_, GroundFunction ground_, const WalkerSettings& settings_)
    : body(body_), personality(personality_), ground(std::move(ground_)), settings(settings_), idle(settings_.seed) {
    Reset(glm::vec3(0.0f), 0.0f);
}

void Walker::SetLayer(const BodyLayer& newLayer) {
    // A layer with a number that is not finite would poison every pose after it: it is replaced by no layer at all.
    bool finite = std::isfinite(newLayer.pelvisOffset.x + newLayer.pelvisOffset.y + newLayer.pelvisOffset.z);
    for (const glm::quat& q : newLayer.additive.rotation) {
        finite = finite && std::isfinite(q.x + q.y + q.z + q.w) && glm::dot(q, q) > 1e-6f;
    }
    layer = finite ? newLayer : BodyLayer();
}

float Walker::GroundHeight(float x, float z) const {
    return ground ? ground(x, z) : 0.0f;
}

// The pitch of the ground along a foot that lies with its heel at `heel` and points along `yaw`.
float Walker::GroundPitch(const glm::vec3& heel, float yaw) const {
    const glm::vec3 toBall = Forward(yaw) * body.ballFromHeel;
    const float rise = GroundHeight(heel.x + toBall.x, heel.z + toBall.z) - GroundHeight(heel.x, heel.z);
    return std::clamp(std::atan2(rise, body.ballFromHeel), -0.6f, 0.6f);
}

void Walker::Reset(const glm::vec3& startPosition, float headingRadians) {
    position = glm::vec3(startPosition.x, GroundHeight(startPosition.x, startPosition.z), startPosition.z);
    heading = headingRadians;
    pelvisHeightKnown = false;
    speed = 0.0f;
    acceleration = 0.0f;
    turnRate = 0.0f;
    phase = 0.0f;
    time = 0.0;
    shape = ComputeGaitShape(body, personality, 0.0f);
    for (const bool left : {true, false}) {
        FootTrack& foot = feet[left ? 0 : 1];
        foot.planted = true;
        foot.footfalls = 0;
        foot.footfall = PredictFootfall(left, 0.0f);
        foot.state = PlantedFoot(foot, 0.5f);
        foot.swingStart = foot.state;
    }
    BuildPose(0.0f);
}

void Walker::Update(float dt, const LocomotionCommand& command) {
    // A time step that is not a number, or not positive, is ignored; a long one is taken in small steps, so that the feet
    // are planted and lifted at the right moments however long the frame was.
    if (!std::isfinite(dt) || dt <= 0.0f) {
        return;
    }
    const float wantedSpeed = std::isfinite(command.speed) ? std::clamp(command.speed, 0.0f, MaximumSpeed(body)) : 0.0f;
    const float wantedTurnRate = std::isfinite(command.turnRate) ? command.turnRate : 0.0f;
    const float total = std::min(dt, kLongestUpdate);
    const int steps = std::max(1, static_cast<int>(std::ceil(total / kMaxStep)));
    for (int i = 0; i < steps; ++i) {
        Step(total / static_cast<float>(steps), wantedSpeed, wantedTurnRate);
    }
}

// Whether the walker is to stand: no speed and no turn, now or asked for.
bool Walker::StandingStill(float wantedSpeed, float wantedTurnRate) const {
    constexpr float kStill = 0.02f;
    return speed < kStill && wantedSpeed < kStill && std::abs(turnRate) < kStill && std::abs(wantedTurnRate) < kStill;
}

// Whether both feet are on, or coming down on, the places where a standing person puts them, beside the middle of the body.
bool Walker::FeetAreTogether() const {
    for (const bool left : {true, false}) {
        const FootTrack& foot = feet[left ? 0 : 1];
        const Footfall resting = PredictFootfall(left, 0.0f);
        if (glm::length(glm::vec2(resting.heel.x - foot.footfall.heel.x, resting.heel.z - foot.footfall.heel.z)) > 0.04f) {
            return false;
        }
    }
    return true;
}

// How far the body will have gone along its path in `seconds`, if it goes on speeding up or slowing down at the rate it does
// now until it reaches the speed it is heading for.
float Walker::DistanceAhead(float seconds) const {
    if (std::abs(acceleration) < 1e-4f) {
        return speed * seconds;
    }
    const float timeToTarget = std::max((targetSpeed - speed) / acceleration, 0.0f);
    const float accelerating = std::min(seconds, timeToTarget);
    return speed * accelerating + 0.5f * acceleration * accelerating * accelerating + targetSpeed * (seconds - accelerating);
}

void Walker::Step(float dt, float wantedSpeed, float wantedTurnRate) {
    targetSpeed = wantedSpeed;
    // Speed and turning follow the command at a limited rate.
    const float previousSpeed = speed;
    speed = wantedSpeed > speed ? std::min(wantedSpeed, speed + settings.acceleration * dt) : std::max(wantedSpeed, speed - settings.deceleration * dt);
    acceleration = (speed - previousSpeed) / dt;
    // The turning is limited to what the legs can follow at this speed, and follows the command smoothly.
    const float turnLimit = MaximumTurnRate(body, speed);
    turnRate += (std::clamp(wantedTurnRate, -turnLimit, turnLimit) - turnRate) * (1.0f - std::exp(-settings.turnResponse * dt));

    shape = ComputeGaitShape(body, personality, speed, turnRate);
    if (StandingStill(wantedSpeed, std::clamp(wantedTurnRate, -turnLimit, turnLimit))) {
        // At rest: finish the step in hand, and stop when a foot has landed at the end of one (a half cycle) with both feet
        // side by side. Without that the body would step on the spot for ever.
        const float halves = phase * 2.0f;
        const float next = std::ceil(halves - 1e-4f);
        const float advanced = halves + 2.0f * dt / shape.period;
        phase = (advanced >= next && FeetAreTogether() ? next : advanced) * 0.5f;
    } else {
        phase += dt / shape.period;
    }
    phase -= std::floor(phase);
    time += dt;

    // Move along the middle heading of the step, which is exact for a steady turn.
    const float middleHeading = heading + 0.5f * turnRate * dt;
    heading += turnRate * dt;
    position += Forward(middleHeading) * (speed * dt);
    position.y = GroundHeight(position.x, position.z);

    UpdateFoot(true, phase);
    UpdateFoot(false, phase + 0.5f - std::floor(phase + 0.5f));
    BuildPose(dt);
}

// Where the foot that comes down `secondsAhead` from now should land: ahead of the middle of the body, on its own side.
Walker::Footfall Walker::PredictFootfall(bool left, float secondsAhead) const {
    const float side = left ? 1.0f : -1.0f;
    const float headingThen = heading + turnRate * secondsAhead;
    const float middleHeading = heading + 0.5f * turnRate * secondsAhead;
    const glm::vec3 bodyThen = position + Forward(middleHeading) * DistanceAhead(secondsAhead);
    const glm::vec3 heelXZ = bodyThen + LeftOf(headingThen) * (side * shape.stepHalfWidth) + Forward(headingThen) * (shape.footLead + body.heelOffset.z);

    Footfall footfall;
    footfall.heel = glm::vec3(heelXZ.x, GroundHeight(heelXZ.x, heelXZ.z), heelXZ.z);
    footfall.yaw = headingThen + side * kToeOutRadians * (1.0f - 0.7f * shape.runBlend);
    footfall.slope = GroundPitch(footfall.heel, footfall.yaw);
    return footfall;
}

// The foot with its heel on a footfall and its sole pitched `rollPitch` above the ground.
Walker::FootPose Walker::FootAtFootfall(const Footfall& footfall, float rollPitch) const {
    FootPose foot;
    foot.yaw = footfall.yaw;
    foot.slope = footfall.slope;
    foot.pitch = footfall.slope + rollPitch;
    foot.ankle = footfall.heel - FootRotation(foot.yaw, foot.pitch) * body.heelOffset;
    return foot;
}

// The foot while it is planted: the heel stays at the footfall as the sole comes down, then the ball stays there as the heel rises.
Walker::FootPose Walker::PlantedFoot(const FootTrack& foot, float stanceProgress) const {
    const Footfall& footfall = foot.footfall;
    float roll = 0.0f;  // toes up positive
    if (stanceProgress < kHeelDownBy) {
        roll = kHeelStrikePitch * shape.rollAmount * (1.0f - Smoothstep(0.0f, kHeelDownBy, stanceProgress));
    } else if (stanceProgress > kHeelRisesFrom) {
        roll = -kToeOffPitch * shape.rollAmount * Smoothstep(kHeelRisesFrom, 1.0f, stanceProgress);
    }

    return PlantedFootAtPitch(foot, stanceProgress, footfall.slope + roll);
}

// The planted foot turned to a pitch about its pivot: the heel until the heel rises, then the ball.
Walker::FootPose Walker::PlantedFootAtPitch(const FootTrack& foot, float stanceProgress, float pitch) const {
    const Footfall& footfall = foot.footfall;
    const glm::quat flat = FootRotation(footfall.yaw, footfall.slope);
    const glm::vec3 ball = footfall.heel + flat * (body.ballOffset - body.heelOffset);
    const bool onBall = stanceProgress >= kHeelRisesFrom;

    FootPose planted;
    planted.yaw = footfall.yaw;
    planted.slope = footfall.slope;
    planted.pitch = pitch;
    // With the heel up, the toes bend back to stay flat on the ground, as far as the joint allows.
    const float roll = pitch - footfall.slope;
    planted.toeFlex = std::clamp(roll, -kToesBendUp, 0.0f);

    glm::vec3 pivot = onBall ? ball : footfall.heel;
    glm::vec3 pivotInFoot = onBall ? body.ballOffset : body.heelOffset;
    if (onBall && roll < -kToesBendUp) {
        // Past that, the heel rises further than the toes can follow, and the foot rolls on from the ball to the tips of the
        // toes: the tip stays where it was on the floor, so the toes do not dig in or slide.
        const glm::vec3 beyondBall = body.toeOffset - body.ballOffset;
        pivot = ball + flat * beyondBall;
        pivotInFoot = body.ballOffset + glm::angleAxis(planted.toeFlex, glm::vec3(1.0f, 0.0f, 0.0f)) * beyondBall;
    }
    planted.ankle = pivot - FootRotation(planted.yaw, planted.pitch) * pivotInFoot;
    return planted;
}

// The foot while it swings from its toe-off to the next footfall: it lifts, passes the other leg and comes down on the heel.
Walker::FootPose Walker::SwingingFoot(const FootTrack& foot, float swingProgress, const Footfall& landing) const {
    const FootPose& start = foot.swingStart;
    const FootPose end = FootAtFootfall(landing, kHeelStrikePitch * shape.rollAmount);
    const float travel = SmootherStep(swingProgress);
    const float lift = std::sin(kPi * swingProgress);

    FootPose swinging;
    swinging.ankle = glm::mix(start.ankle, end.ankle, travel);
    // A walker keeps the foot close to the ground at both ends of the swing; a runner kicks it up at once and brings it down late.
    const float walkLift = swingProgress < 0.5f ? lift : lift * lift;  // rises at once, and comes down gently
    const float runLift = Smoothstep(0.0f, 0.15f, swingProgress) * Smoothstep(0.0f, 0.3f, 1.0f - swingProgress);
    swinging.ankle.y += shape.footClearance * Lerp(walkLift, runLift, shape.runBlend);
    swinging.yaw = LerpAngle(start.yaw, end.yaw, travel);
    // The sole follows the ground under it, and rolls from its toe-off angle to its heel-strike angle on top of that. The
    // toes come up in mid-swing so that they clear the ground.
    swinging.slope = Lerp(start.slope, end.slope, travel);
    const float startRoll = start.pitch - start.slope;
    const float endRoll = end.pitch - end.slope;
    swinging.pitch = swinging.slope + Lerp(startRoll, endRoll, Smoothstep(0.0f, 1.0f, swingProgress)) + 0.10f * shape.rollAmount * lift;
    swinging.toeFlex = start.toeFlex * (1.0f - Smoothstep(0.0f, 0.5f, swingProgress));
    // Over uneven ground the foot lifts further, as much as it takes to keep the sole above what is under it.
    swinging.ankle.y += lift * TerrainDeficit(swinging.ankle, FootRotation(swinging.yaw, swinging.pitch), swinging.toeFlex);
    return swinging;
}

// How far a swinging foot would have to rise to keep its heel, ball and toe a little above the ground.
float Walker::TerrainDeficit(const glm::vec3& ankle, const glm::quat& footRotation, float toeFlex) const {
    float deficit = 0.0f;
    // The tip of the toes moves with the toes joint, which is bent back by `toeFlex` as far as it allows.
    const float flex = std::clamp(toeFlex, -kToesBendUp, 0.0f);
    const glm::vec3 toeTip = body.ballOffset + glm::angleAxis(flex, glm::vec3(1.0f, 0.0f, 0.0f)) * (body.toeOffset - body.ballOffset);
    for (const glm::vec3& offset : {body.heelOffset, body.ballOffset, toeTip}) {
        const glm::vec3 point = ankle + footRotation * offset;
        deficit = std::max(deficit, GroundHeight(point.x, point.z) + kSwingGroundMargin * shape.footClearance - point.y);
    }
    return deficit;
}

// Solves one leg to its foot, and turns the foot. The ankle can only bend so far, so a foot that the gait wants turned
// further than that ends up turned less; for a swinging foot the toes might then catch the ground, so the foot is lifted by
// what the pose actually needs and the leg is solved again.
void Walker::PoseLeg(Pose& built, bool left, float dt) {
    const character::Skeleton& skeleton = body.skeleton;
    FootTrack& track = feet[left ? 0 : 1];
    const FootPose& foot = track.state;
    const Limb leg = LegLimb(left);
    const Joint footJoint = left ? Joint::FootL : Joint::FootR;
    const Joint toes = left ? Joint::ToesL : Joint::ToesR;

    // A planted foot has to stay on its pivot, so if the ankle cannot be turned as far as the gait wants, the foot is turned
    // to the limit and the pivot is kept: the ankle target follows from the pitch the ankle can really reach.
    FootPose wanted = foot;
    if (track.planted) {
        track.liftCorrection = 0.0f;
    } else {
        wanted.ankle.y += track.liftCorrection * track.swingLift;
    }
    for (int pass = 0; pass < 3; ++pass) {
        track.legReachesFoot = SolveTwoBone(skeleton, built, leg, wanted.ankle, Forward(wanted.yaw)).reached;
        SetWorldRotation(skeleton, built, footJoint, FootRotation(wanted.yaw, wanted.pitch));
        built.rotation[character::Index(footJoint)] = ClampToLimits(skeleton, footJoint, built.rotation[character::Index(footJoint)]);
        const glm::quat actualRotation = ForwardKinematics(skeleton, built).rotation[character::Index(footJoint)];
        if (track.planted) {
            const float actualPitch = std::asin(std::clamp((actualRotation * glm::vec3(0.0f, 0.0f, 1.0f)).y, -1.0f, 1.0f));
            if (std::abs(actualPitch - wanted.pitch) < 0.002f) {
                break;
            }
            wanted = PlantedFootAtPitch(track, track.stanceProgress, actualPitch);
            track.state = wanted;
        } else {
            // The lift that the pose needs goes into a correction that may change only at a limited speed, so that the ankle
            // rises smoothly (a step of a frame shrinks with the frame) and not all at once; it fades out as the foot lands.
            const float deficit = TerrainDeficit(wanted.ankle, actualRotation, wanted.toeFlex);
            const float needed = std::max(track.liftCorrection + deficit, 0.0f);
            const float change = std::clamp(needed - track.liftCorrection, -kCorrectionSpeed * dt, kCorrectionSpeed * dt);
            if (pass > 0 || std::abs(change) < 1e-6f) {
                break;
            }
            track.liftCorrection += change;
            wanted.ankle.y = foot.ankle.y + track.liftCorrection * track.swingLift;
        }
    }
    built.rotation[character::Index(toes)] = ClampToLimits(skeleton, toes, glm::angleAxis(wanted.toeFlex, glm::vec3(1.0f, 0.0f, 0.0f)));
}

void Walker::UpdateFoot(bool left, float cyclePosition) {
    FootTrack& foot = feet[left ? 0 : 1];
    const bool shouldBePlanted = cyclePosition < shape.dutyFactor;
    if (foot.planted && !shouldBePlanted) {
        foot.swingStart = foot.state;  // toe-off
    }
    if (!foot.planted && shouldBePlanted) {
        // Heel strike: the foot stays where the swing was taking it until toe-off, so there is no jump at the landing.
        ++foot.footfalls;
    }
    foot.planted = shouldBePlanted;

    foot.stanceProgress = foot.planted ? cyclePosition / shape.dutyFactor : 0.0f;
    foot.swingLift = 0.0f;
    if (foot.planted) {
        foot.state = PlantedFoot(foot, foot.stanceProgress);
    } else {
        const float swingProgress = (cyclePosition - shape.dutyFactor) / (1.0f - shape.dutyFactor);
        // The place to land is predicted again at every step, as the speed and the turn change; over the last sixth of the
        // swing it is held, so that the heel comes down on a place that is not moving.
        const Footfall predicted = PredictFootfall(left, (1.0f - cyclePosition) * shape.period);
        const float following = Smoothstep(0.0f, 0.17f, 1.0f - swingProgress);
        foot.footfall.heel = glm::mix(foot.footfall.heel, predicted.heel, following);
        foot.footfall.yaw = LerpAngle(foot.footfall.yaw, predicted.yaw, following);
        foot.footfall.slope = Lerp(foot.footfall.slope, predicted.slope, following);
        foot.state = SwingingFoot(foot, swingProgress, foot.footfall);
        foot.swingLift = Smoothstep(0.0f, 0.12f, 1.0f - swingProgress);  // the correction goes out over the last tenth of the swing
    }
}

// How high the pelvis is. Over a planted foot the body is an inverted pendulum: the hip moves on an arc about the ankle, with
// the radius of a slightly bent leg, so it is highest as it passes over the foot and lowest as the foot lands and leaves.
// With two feet down the two arcs are blended by how much weight each foot carries. In the air (a run) the pelvis keeps the
// height of a gently bent leg. Whatever the arcs say, the pelvis stays below what the legs can reach.
float Walker::PelvisHeight(float dt, const glm::vec3& pelvisPosition, const glm::quat& pelvisWorld, float moveWeight, float pelvisPitchDegrees, float extraHeight) {
    const float legReach = body.thighLength + body.shankLength;
    const glm::vec3 pelvisForward = pelvisWorld * glm::vec3(0.0f, 0.0f, 1.0f);
    const glm::vec3 pelvisLeft = pelvisWorld * glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 hipOffsets[2] = {pelvisWorld * body.hipOffsetLeft, pelvisWorld * body.hipOffsetRight};

    // How bent the leg is, as the fraction of its full length that the hip stands from the ankle. Standing: nearly straight.
    // Walking: slightly bent. Running: bent most when the body is over the foot, and almost straight as it lands and leaves.
    const float movingEnds = Lerp(0.985f, 0.995f, shape.runBlend);
    const float movingMiddle = Lerp(0.985f, 0.91f, shape.runBlend);
    const float restStraightness = personality.standingHeightScale;

    float arcsHeight = 0.0f;
    float arcsWeight = 0.0f;
    for (int i = 0; i < 2; ++i) {
        const float carried = std::sin(kPi * feet[i].stanceProgress);  // the weight on the foot: 0 at landing and at toe-off
        const float moving = Lerp(movingEnds, movingMiddle, carried) * personality.standingHeightScale;
        const float straightness = Lerp(0.997f * restStraightness, moving, moveWeight);
        const glm::vec3 hip = pelvisPosition + hipOffsets[i];
        const glm::vec3 ankle = feet[i].state.ankle;
        const float across = glm::length(glm::vec2(hip.x - ankle.x, hip.z - ankle.z));
        const float radius = straightness * legReach;
        const float armHeight = ankle.y + std::sqrt(std::max(radius * radius - across * across, 0.0f)) - hipOffsets[i].y;
        arcsHeight += carried * armHeight;
        arcsWeight += carried;
    }
    const float airborneStraightness = Lerp(0.997f * restStraightness, Lerp(0.985f, 0.96f, shape.runBlend) * personality.standingHeightScale, moveWeight);
    const float airborneHeight = position.y + body.ankleHeight + airborneStraightness * legReach;
    const float onArcs = Smoothstep(0.0f, 0.3f, arcsWeight);
    const float wantedHeight = Lerp(airborneHeight, arcsWeight > 1e-4f ? arcsHeight / arcsWeight : airborneHeight, onArcs) + extraHeight;
    // A body is not a rigid pendulum: its mass takes a moment to follow, which rounds the corners where the weight changes feet.
    if (!pelvisHeightKnown || dt <= 0.0f) {
        pelvisHeightState = wantedHeight;
        pelvisHeightKnown = true;
    } else {
        pelvisHeightState += (wantedHeight - pelvisHeightState) * (1.0f - std::exp(-dt / kPelvisHeightSmoothing));
    }
    float height = pelvisHeightState;

    // The legs cannot stretch farther than they are long, and the hip extends only so far (the pelvis leaning forward gives it a little more).
    const float maxExtension = (0.9f * body.skeleton[Joint::ThighL].maxAngleDegrees.x + pelvisPitchDegrees) * kDegreesToRadians;
    for (int i = 0; i < 2; ++i) {
        const glm::vec3 hip = pelvisPosition + hipOffsets[i];
        const glm::vec3 ankle = feet[i].state.ankle;
        const glm::vec3 toAnkle = ankle - hip;
        const float backward = -glm::dot(glm::vec2(toAnkle.x, toAnkle.z), glm::vec2(pelvisForward.x, pelvisForward.z));
        const float lateral = glm::dot(glm::vec2(toAnkle.x, toAnkle.z), glm::vec2(pelvisLeft.x, pelvisLeft.z));
        const float hipHeightLimit = ankle.y + HighestHipAboveAnkle(body.thighLength, body.shankLength, backward, lateral, maxExtension);
        height = SmoothMin(height, hipHeightLimit - hipOffsets[i].y, 0.02f);
    }
    return height;
}

void Walker::BuildPose(float dt) {
    const character::Skeleton& skeleton = body.skeleton;
    Pose built;
    auto rotation = [&](Joint joint) -> glm::quat& { return built.rotation[character::Index(joint)]; };
    auto layered = [&](Joint joint, const glm::quat& base) { return glm::normalize(base * layer.additive.rotation[character::Index(joint)]); };

    // The weight of the idle motion is what the gait leaves over.
    const float similar = SimilarSpeed(body, speed);
    const float moveWeight = Smoothstep(0.05f, 0.6f, speed);
    const BodyLayer idleLayer = ScaledLayer(idle.Evaluate(time), 1.0f - moveWeight);
    const BodyLayer extra = CombinedLayers(idleLayer, layer);

    // The torso, from the phase of the gait.
    const float cycle = kTwoPi * phase;
    const glm::vec3 slopeProbe = Forward(heading) * 0.4f;
    const float slope = std::atan2(GroundHeight(position.x + slopeProbe.x, position.z + slopeProbe.z) - GroundHeight(position.x - slopeProbe.x, position.z - slopeProbe.z), 0.8f);
    TorsoInputs inputs;
    inputs.cycle = cycle;
    inputs.swayCycle = kTwoPi * (phase - 0.5f * shape.dutyFactor);
    inputs.speed = similar;
    inputs.runBlend = shape.runBlend;
    inputs.moveWeight = moveWeight;
    inputs.acceleration = acceleration;
    inputs.turnRate = turnRate;
    inputs.slopeDegrees = slope * kRadiansToDegrees;
    const TorsoAngles torso = TorsoMotion(personality, inputs);

    const glm::quat pelvisInBody = glm::normalize(character::FromDegrees(torso.pelvisPitch, torso.pelvisYaw, torso.pelvisRoll) * extra.additive.rotation[character::Index(Joint::Pelvis)]);
    const glm::quat pelvisWorld = YawRotation(heading) * pelvisInBody;
    rotation(Joint::Pelvis) = pelvisWorld;
    rotation(Joint::Spine1) = layered(Joint::Spine1, character::FromDegrees(torso.spine1Pitch, torso.spine1Yaw, torso.spine1Roll));
    rotation(Joint::Spine2) = layered(Joint::Spine2, character::FromDegrees(torso.spine2Pitch, torso.spine2Yaw, torso.spine2Roll));
    rotation(Joint::Neck) = layered(Joint::Neck, character::FromDegrees(torso.neckPitch, torso.neckYaw, 0.0f));
    rotation(Joint::Head) = layered(Joint::Head, character::FromDegrees(torso.headPitch, torso.headYaw, torso.headRoll));
    rotation(Joint::ClavicleL) = glm::normalize(extra.additive.rotation[character::Index(Joint::ClavicleL)]);
    rotation(Joint::ClavicleR) = glm::normalize(extra.additive.rotation[character::Index(Joint::ClavicleR)]);

    // The arms swing against the legs: the left arm goes back as the left foot goes forward.
    const float armSwing = std::min(Lerp(6.0f + 9.0f * std::min(similar, 2.2f), 30.0f + 2.0f * std::max(similar - 2.5f, 0.0f), shape.runBlend) * personality.armSwingScale * moveWeight, 55.0f);
    const float hang = 6.0f + 12.0f * (personality.stepWidthScale - 1.0f) + 8.0f * shape.runBlend;
    const float elbowBase = Lerp(12.0f + 4.0f * std::min(similar, 2.2f), 85.0f, shape.runBlend) * moveWeight + 8.0f * (1.0f - moveWeight);
    for (const bool left : {true, false}) {
        const float swing = (left ? 1.0f : -1.0f) * armSwing * std::cos(cycle);
        const Joint upperArm = left ? Joint::UpperArmL : Joint::UpperArmR;
        const Joint forearm = left ? Joint::ForearmL : Joint::ForearmR;
        rotation(upperArm) = layered(upperArm, UpperArmRotation(left, swing, hang));
        const float elbow = std::max(elbowBase + 0.5f * std::max(-swing, 0.0f), 3.0f);
        rotation(forearm) = layered(forearm, character::HingeRotation(skeleton, forearm, elbow));
    }

    // The pelvis: over the middle of the path, shifted by the sway, as high as the legs allow.
    const glm::vec3 layerOffset = YawRotation(heading) * extra.pelvisOffset;
    glm::vec3 pelvisPosition = position + LeftOf(heading) * torso.sway + glm::vec3(layerOffset.x, 0.0f, layerOffset.z);

    pelvisPosition.y = PelvisHeight(dt, pelvisPosition, pelvisWorld, moveWeight, torso.pelvisPitch, layerOffset.y);
    SetPelvisPosition(skeleton, built, pelvisPosition);

    // The legs, from the planted and swinging feet.
    PoseLeg(built, true, dt);
    PoseLeg(built, false, dt);

    // Everything above the pelvis is held to the range of the human body.
    for (const Joint joint : {Joint::Spine1, Joint::Spine2, Joint::Neck, Joint::Head, Joint::ClavicleL, Joint::ClavicleR, Joint::UpperArmL, Joint::UpperArmR,
                              Joint::ForearmL, Joint::ForearmR, Joint::HandL, Joint::HandR}) {
        rotation(joint) = ClampToLimits(skeleton, joint, rotation(joint));
    }
    pose = built;
}

}  // namespace animation
