// Tests of the procedural animation (no window, no GPU). Like the other test programs there is no test framework: each test
// is a function that calls Check, and main reports every failure it finds. Run with a name as the argument to run one test.
//
// What the animation is held to: forward kinematics that agrees with the skinning of the character module, joint limits
// that hold, inverse kinematics that reaches what can be reached the right way round, a gait whose planted feet do not
// slide, whose feet stand on the ground and whose knees never bend backwards, a pelvis that moves smoothly, blends and
// layers without jumps, the same result every time, and bodies of every size.
//
// The feet and the knees are measured on the final pose, through character::SkinMatrices and character::JointPositions
// rather than through the code under test, so that a fault in the planner cannot hide itself.
#include <algorithm>
#include <cmath>
#include <functional>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "animation/BodyModel.h"
#include "animation/Gait.h"
#include "animation/IdleMotion.h"
#include "animation/JointLimits.h"
#include "animation/Kinematics.h"
#include "animation/Noise.h"
#include "animation/Personality.h"
#include "animation/TwoBoneIk.h"
#include "character/CharacterParameters.h"
#include "character/Skeleton.h"

namespace {

using namespace animation;
using character::CharacterParameters;

int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

// A small deterministic random number generator (SplitMix64); the standard distributions differ between compilers.
class Random {
public:
    explicit Random(uint32_t seed) : state(0x9E3779B97F4A7C15ull * (static_cast<uint64_t>(seed) + 1ull)) {}

    float Uniform() {
        state += 0x9E3779B97F4A7C15ull;
        uint64_t z = state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        z ^= z >> 31;
        return static_cast<float>(z >> 40) * (1.0f / 16777216.0f);
    }

    float Between(float low, float high) { return low + (high - low) * Uniform(); }

private:
    uint64_t state;
};

CharacterParameters Person(float masculinity, float age, float stature, float mass, float muscularity, uint32_t seed = 1) {
    CharacterParameters parameters;
    parameters.masculinity = masculinity;
    parameters.ageYears = age;
    parameters.statureMeters = stature;
    parameters.massKg = mass;
    parameters.muscularity = muscularity;
    parameters.seed = seed;
    return parameters;
}

// A spread of people, from the smallest and lightest to the tallest and heaviest the generator allows.
std::vector<CharacterParameters> SomePeople() {
    return {Person(1.0f, 30.0f, 1.90f, 100.0f, 0.8f, 1), Person(0.0f, 25.0f, 1.55f, 48.0f, 0.2f, 2), Person(1.0f, 72.0f, 1.70f, 82.0f, 0.2f, 3),
            Person(0.0f, 28.0f, 1.72f, 66.0f, 0.9f, 4),  Person(0.5f, 45.0f, 1.66f, 70.0f, 0.4f, 5)};
}

std::vector<CharacterParameters> ExtremePeople() {
    return {Person(0.0f, 20.0f, 1.45f, 15.0f * 1.45f * 1.45f, 0.1f, 7), Person(1.0f, 20.0f, 2.05f, 15.0f * 2.05f * 2.05f, 0.1f, 8),
            Person(0.0f, 60.0f, 1.45f, 45.0f * 1.45f * 1.45f, 0.1f, 9), Person(1.0f, 60.0f, 2.05f, 45.0f * 2.05f * 2.05f, 0.1f, 10)};
}

glm::quat RandomRotation(Random& random, float maxDegrees) {
    const glm::vec3 axis = glm::normalize(glm::vec3(random.Between(-1, 1), random.Between(-1, 1), random.Between(-1, 1)) + glm::vec3(1e-3f));
    return glm::angleAxis(random.Between(0.0f, maxDegrees) * kDegreesToRadians, axis);
}

// A rotation inside the limits of a joint, drawn from the whole range.
glm::quat RandomRotationInLimits(Random& random, const Skeleton& skeleton, Joint joint) {
    const character::JointInfo& info = skeleton[joint];
    if (IsHinge(joint)) {
        return character::HingeRotation(skeleton, joint, random.Between(info.minAngleDegrees.x, info.maxAngleDegrees.x));
    }
    return character::FromDegrees(random.Between(info.minAngleDegrees.x, info.maxAngleDegrees.x),
                                  random.Between(info.minAngleDegrees.y, info.maxAngleDegrees.y),
                                  random.Between(info.minAngleDegrees.z, info.maxAngleDegrees.z));
}

float DegreesBetween(const glm::quat& a, const glm::quat& b) {
    return RotationAngleDegrees(glm::inverse(a) * b);
}

// ---------------------------------------------------------------------------
// Kinematics
// ---------------------------------------------------------------------------

void ForwardKinematicsOfTheRestPoseIsTheRestPose() {
    for (const CharacterParameters& person : SomePeople()) {
        const Skeleton skeleton = character::BuildSkeleton(person);
        const Pose rest;
        const WorldPose world = ForwardKinematics(skeleton, rest);
        const JointMatrices skin = SkinningMatrices(skeleton, rest);
        for (int i = 0; i < kJointCount; ++i) {
            Check(glm::length(world.position[i] - skeleton.joints[i].restPosition) < 1e-5f, "a joint of the rest pose is where the skeleton puts it");
            Check(DegreesBetween(world.rotation[i], glm::quat(1, 0, 0, 0)) < 1e-3f, "every joint of the rest pose is unturned");
            Check(glm::length(skin[i][3] - glm::vec4(0, 0, 0, 1)) < 1e-5f, "the skinning matrix of the rest pose has no translation");
        }
    }
}

void ForwardKinematicsAgreesWithTheSkinningOfTheCharacterModule() {
    Random random(11);
    for (const CharacterParameters& person : SomePeople()) {
        const Skeleton skeleton = character::BuildSkeleton(person);
        for (int trial = 0; trial < 20; ++trial) {
            Pose pose;
            for (int i = 0; i < kJointCount; ++i) {
                pose.rotation[i] = RandomRotation(random, 90.0f);
            }
            pose.rootTranslation = glm::vec3(random.Between(-2, 2), random.Between(-1, 1), random.Between(-2, 2));
            const WorldPose world = ForwardKinematics(skeleton, pose);
            const std::array<glm::vec3, kJointCount> reference = character::JointPositions(skeleton, pose);
            const JointMatrices matrices = WorldMatrices(skeleton, pose);
            const JointMatrices skin = SkinningMatrices(skeleton, pose);
            for (int i = 0; i < kJointCount; ++i) {
                Check(glm::length(world.position[i] - reference[i]) < 1e-4f, "forward kinematics puts a joint where character::JointPositions does");
                const glm::mat4 expectedSkin = matrices[i] * skeleton.joints[i].inverseBind;
                Check(glm::length(skin[i][0] - expectedSkin[0]) + glm::length(skin[i][3] - expectedSkin[3]) < 1e-4f, "world matrix times inverse bind is the skinning matrix");
            }
        }
    }
}

void PelvisAndRotationHelpersWork() {
    const Skeleton skeleton = character::BuildSkeleton(Person(0.5f, 30, 1.7f, 70, 0.4f));
    Pose pose;
    SetPelvisPosition(skeleton, pose, glm::vec3(3, 1, -2));
    Check(glm::length(ForwardKinematics(skeleton, pose).position[0] - glm::vec3(3, 1, -2)) < 1e-5f, "SetPelvisPosition puts the pelvis where it was asked");

    Random random(5);
    for (int trial = 0; trial < 20; ++trial) {
        Pose messy;
        for (int i = 0; i < kJointCount; ++i) {
            messy.rotation[i] = RandomRotation(random, 60.0f);
        }
        const glm::quat wanted = RandomRotation(random, 170.0f);
        SetWorldRotation(skeleton, messy, Joint::FootL, wanted);
        Check(DegreesBetween(ForwardKinematics(skeleton, messy).rotation[character::Index(Joint::FootL)], wanted) < 1e-2f, "SetWorldRotation gives the joint the world rotation");
    }
    const glm::vec3 a = glm::normalize(glm::vec3(1, 2, 3));
    const glm::vec3 b = glm::normalize(glm::vec3(-3, 1, 0.5f));
    Check(glm::length(RotationBetween(a, b) * a - b) < 1e-5f, "RotationBetween turns one direction into the other");
    Check(glm::length(RotationBetween(a, -a) * a + a) < 1e-5f, "RotationBetween handles opposite directions");
}

// ---------------------------------------------------------------------------
// Joint limits
// ---------------------------------------------------------------------------

void EulerAnglesRoundTripAcrossEveryLimitRange() {
    Random random(21);
    const Skeleton skeleton = character::BuildSkeleton(Person(0.5f, 30, 1.7f, 70, 0.4f));
    for (int i = 1; i < kJointCount; ++i) {
        const Joint joint = static_cast<Joint>(i);
        if (IsHinge(joint)) {
            continue;
        }
        const character::JointInfo& info = skeleton[joint];
        for (int trial = 0; trial < 200; ++trial) {
            const glm::vec3 angles(random.Between(info.minAngleDegrees.x, info.maxAngleDegrees.x), random.Between(info.minAngleDegrees.y, info.maxAngleDegrees.y),
                                   random.Between(info.minAngleDegrees.z, info.maxAngleDegrees.z));
            const glm::quat rotation = character::FromDegrees(angles.x, angles.y, angles.z);
            const glm::vec3 back = EulerDegrees(rotation);
            Check(glm::length(back - angles) < 0.05f, std::string("Euler angles round trip for ") + info.name);
            Check(DegreesBetween(character::FromDegrees(back.x, back.y, back.z), rotation) < 0.01f, std::string("Euler angles rebuild the rotation for ") + info.name);
        }
    }
    // The gimbal: looking straight up the Y axis the angles are not unique, but they must still rebuild the rotation.
    const glm::quat gimbal = character::FromDegrees(30.0f, 90.0f, 20.0f);
    const glm::vec3 angles = EulerDegrees(gimbal);
    Check(DegreesBetween(character::FromDegrees(angles.x, angles.y, angles.z), gimbal) < 0.05f, "Euler angles at the gimbal rebuild the rotation");
}

void ClampingKeepsEveryJointInsideItsLimits() {
    Random random(22);
    const Skeleton skeleton = character::BuildSkeleton(Person(0.5f, 30, 1.7f, 70, 0.4f));
    for (int trial = 0; trial < 300; ++trial) {
        Pose pose;
        for (int i = 1; i < kJointCount; ++i) {
            pose.rotation[i] = RandomRotation(random, 180.0f);
        }
        ClampPoseToLimits(skeleton, pose);
        for (int i = 1; i < kJointCount; ++i) {
            const float excess = LimitExcessDegrees(skeleton, static_cast<Joint>(i), pose.rotation[i]);
            Check(excess < 0.05f, std::string("a clamped joint is inside its limits: ") + skeleton.joints[i].name);
        }
    }
}

void ClampingLeavesARotationInsideTheLimitsAlone() {
    Random random(23);
    const Skeleton skeleton = character::BuildSkeleton(Person(0.5f, 30, 1.7f, 70, 0.4f));
    for (int trial = 0; trial < 200; ++trial) {
        for (int i = 1; i < kJointCount; ++i) {
            const Joint joint = static_cast<Joint>(i);
            const glm::quat inside = RandomRotationInLimits(random, skeleton, joint);
            Check(DegreesBetween(ClampToLimits(skeleton, joint, inside), inside) < 0.02f, std::string("a rotation inside the limits is unchanged: ") + skeleton.joints[i].name);
        }
    }
}

void HingesAreLimitedInFlexionAndKneesDoNotBendBackwards() {
    const Skeleton skeleton = character::BuildSkeleton(Person(0.5f, 30, 1.7f, 70, 0.4f));
    for (const Joint knee : {Joint::CalfL, Joint::CalfR, Joint::ForearmL, Joint::ForearmR}) {
        const float flexion = FlexionDegrees(skeleton, knee, character::HingeRotation(skeleton, knee, 70.0f));
        Check(std::abs(flexion - 70.0f) < 0.01f, "flexion reads back the hinge angle");
        const glm::quat tooFar = ClampToLimits(skeleton, knee, character::HingeRotation(skeleton, knee, 170.0f));
        Check(FlexionDegrees(skeleton, knee, tooFar) <= skeleton[knee].maxAngleDegrees.x + 0.01f, "a hinge cannot flex past its limit");
        const glm::quat backwards = ClampToLimits(skeleton, knee, character::HingeRotation(skeleton, knee, -40.0f));
        Check(FlexionDegrees(skeleton, knee, backwards) >= skeleton[knee].minAngleDegrees.x - 0.01f, "a hinge cannot bend backwards past its limit");
    }
}

// ---------------------------------------------------------------------------
// Two-bone inverse kinematics
// ---------------------------------------------------------------------------

// The bend of the hinge towards a direction: how far the mid joint lies off the line from the root to the end, along the pole.
float BendTowards(const WorldPose& world, const Limb& limb, const glm::vec3& pole) {
    const glm::vec3 root = world.position[character::Index(limb.upper)];
    const glm::vec3 end = world.position[character::Index(limb.end)];
    const glm::vec3 mid = world.position[character::Index(limb.lower)];
    const glm::vec3 along = glm::normalize(end - root);
    const glm::vec3 off = (mid - root) - along * glm::dot(mid - root, along);
    return glm::dot(off, pole);
}

void LegIkReachesTargetsWithTheKneeTowardsThePole() {
    Random random(31);
    for (const CharacterParameters& person : SomePeople()) {
        const Skeleton skeleton = character::BuildSkeleton(person);
        for (const bool left : {true, false}) {
            const Limb leg = LegLimb(left);
            for (int trial = 0; trial < 60; ++trial) {
                Pose pose;
                const glm::vec3 hip = skeleton[leg.upper].restPosition;
                const float reach = MaximumReach(skeleton, leg);
                // A target in front of and below the hip, inside what the leg can reach and the hip can turn to.
                const glm::vec3 direction = glm::normalize(glm::vec3(random.Between(-0.4f, 0.4f), -1.0f, random.Between(-0.6f, 0.6f)));
                const glm::vec3 target = hip + direction * random.Between(0.45f, 0.97f) * reach;
                const IkResult result = SolveTwoBone(skeleton, pose, leg, target, glm::vec3(0, 0, 1));
                Check(result.reached, "the leg reaches a reachable target");
                const WorldPose world = ForwardKinematics(skeleton, pose);
                Check(glm::length(world.position[character::Index(leg.end)] - target) < 2e-3f, "the ankle is on the target");
                Check(BendTowards(world, leg, glm::vec3(0, 0, 1)) > 0.0f, "the knee points forwards, towards the pole");
                Check(result.flexionDegrees > -0.5f, "the knee does not bend backwards");
                Check(LimitExcessDegrees(skeleton, leg.upper, pose.rotation[character::Index(leg.upper)]) < 0.1f, "the hip stays inside its limits");
            }
        }
    }
}

void ArmIkReachesTargetsWithTheElbowTowardsThePole() {
    Random random(32);
    for (const CharacterParameters& person : SomePeople()) {
        const Skeleton skeleton = character::BuildSkeleton(person);
        for (const bool left : {true, false}) {
            const Limb arm = ArmLimb(left);
            const glm::vec3 shoulder = skeleton[arm.upper].restPosition;
            const float side = left ? 1.0f : -1.0f;
            for (int trial = 0; trial < 60; ++trial) {
                Pose pose;
                const float reach = MaximumReach(skeleton, arm);
                const glm::vec3 direction = glm::normalize(glm::vec3(side * random.Between(0.1f, 0.9f), random.Between(-1.0f, 0.4f), random.Between(0.1f, 1.0f)));
                const glm::vec3 target = shoulder + direction * random.Between(0.4f, 0.95f) * reach;
                const glm::vec3 pole = glm::normalize(glm::vec3(side * 0.5f, -0.3f, -1.0f));
                const IkResult result = SolveTwoBone(skeleton, pose, arm, target, pole);
                const WorldPose world = ForwardKinematics(skeleton, pose);
                Check(glm::length(world.position[character::Index(arm.end)] - target) < 2e-3f || !result.reached, "the wrist is on the target when the solver says it reached");
                Check(result.reached, "the arm reaches a reachable target");
                Check(BendTowards(world, arm, pole) > 0.0f, "the elbow points towards the pole");
                Check(result.flexionDegrees > -0.5f, "the elbow does not bend backwards");
            }
        }
    }
}

void IkAtTheRestPositionKeepsTheLimbNearRest() {
    for (const CharacterParameters& person : SomePeople()) {
        const Skeleton skeleton = character::BuildSkeleton(person);
        for (const Limb& limb : {LegLimb(true), LegLimb(false), ArmLimb(true), ArmLimb(false)}) {
            Pose pose;
            const IkResult result = SolveTwoBone(skeleton, pose, limb, skeleton[limb.end].restPosition, glm::vec3(0, 0, limb.restPole.z));
            Check(result.reached, "IK to the rest position reaches it");
            const WorldPose world = ForwardKinematics(skeleton, pose);
            Check(glm::length(world.position[character::Index(limb.lower)] - skeleton[limb.lower].restPosition) < 0.02f,
                  "IK to the rest position leaves the knee or elbow where it was");
        }
    }
}

void IkOutOfReachStretchesTowardsTheTargetWithoutHyperextending() {
    const Skeleton skeleton = character::BuildSkeleton(Person(0.5f, 30, 1.7f, 70, 0.4f));
    const Limb leg = LegLimb(true);
    Pose pose;
    const glm::vec3 hip = skeleton[leg.upper].restPosition;
    const glm::vec3 far = hip + glm::vec3(0.0f, -3.0f, 0.5f);
    const IkResult result = SolveTwoBone(skeleton, pose, leg, far, glm::vec3(0, 0, 1));
    Check(!result.reached, "a target out of reach is not reached");
    const float stretch = glm::length(result.endPosition - hip);
    Check(stretch > MaximumReach(skeleton, leg) - 0.02f && stretch <= MaximumReach(skeleton, leg) + 0.01f, "the leg stretches to its full length");
    Check(result.flexionDegrees >= -0.5f, "a stretched leg is straight, not bent backwards");
    Check(glm::dot(glm::normalize(result.endPosition - hip), glm::normalize(far - hip)) > 0.995f, "the stretched leg points at the target");

    // And a target too close folds the limb as far as the limit lets it, and no further.
    Pose folded;
    const IkResult close = SolveTwoBone(skeleton, folded, leg, hip + glm::vec3(0.0f, -0.02f, 0.0f), glm::vec3(0, 0, 1));
    Check(close.flexionDegrees <= skeleton[leg.lower].maxAngleDegrees.x + 0.5f, "a folded leg stays inside the limit of the knee");
}

void IkRespectsALimitedHip() {
    const Skeleton skeleton = character::BuildSkeleton(Person(0.5f, 30, 1.7f, 70, 0.4f));
    const Limb leg = LegLimb(true);
    Pose pose;
    const glm::vec3 hip = skeleton[leg.upper].restPosition;
    // Straight up behind the body is far outside the range of the hip.
    const IkResult result = SolveTwoBone(skeleton, pose, leg, hip + glm::vec3(0.0f, 0.3f, -0.4f), glm::vec3(0, 0, 1));
    Check(!result.reached, "a hip that cannot turn that far does not reach");
    Check(LimitExcessDegrees(skeleton, leg.upper, pose.rotation[character::Index(leg.upper)]) < 0.1f, "the hip is held to its limits");
    Check(LimitExcessDegrees(skeleton, leg.lower, pose.rotation[character::Index(leg.lower)]) < 0.1f, "the knee is held to its limits");
}

// ---------------------------------------------------------------------------
// Walking and running
// ---------------------------------------------------------------------------

GroundFunction FlatGround() {
    return [](float, float) { return 0.0f; };
}

GroundFunction SlopedGround(float degrees) {
    const float slope = std::tan(degrees * kDegreesToRadians);
    return [slope](float, float z) { return slope * z; };
}

// Hills in both directions, up to about 8 degrees.
GroundFunction RollingGround() {
    return [](float x, float z) { return 0.10f * std::sin(1.1f * z) + 0.05f * std::sin(0.7f * x + 0.3f); };
}

struct FootSample {
    glm::vec3 heel{0.0f};
    glm::vec3 ball{0.0f};
    glm::vec3 toe{0.0f};
    bool planted = false;
    bool legReaches = true;
};

// What the tests record of every frame of a walk, measured on the final pose through the skinning of the character module.
struct Frame {
    Pose pose;
    glm::vec3 position{0.0f};  // of the walker on the ground
    glm::vec3 pelvis{0.0f};
    glm::vec3 head{0.0f};
    float heading = 0.0f;
    float speed = 0.0f;
    float phase = 0.0f;
    FootSample foot[2];  // left, right
    float knee[2] = {0.0f, 0.0f};
    float elbow[2] = {0.0f, 0.0f};
    float period = 1.0f;
};

struct Segment {
    float seconds;
    float speed;
    float turnRate;
};

FootSample MeasureFoot(const Skeleton& skeleton, const Pose& pose, bool left) {
    const std::array<glm::mat4, kJointCount> skin = character::SkinMatrices(skeleton, pose);
    const character::FootContacts& contacts = left ? skeleton.leftFoot : skeleton.rightFoot;
    FootSample sample;
    sample.heel = glm::vec3(skin[character::Index(left ? Joint::FootL : Joint::FootR)] * glm::vec4(contacts.heel, 1.0f));
    sample.ball = glm::vec3(skin[character::Index(left ? Joint::FootL : Joint::FootR)] * glm::vec4(contacts.ball, 1.0f));
    // The toe tip moves with the toes joint.
    sample.toe = glm::vec3(skin[character::Index(left ? Joint::ToesL : Joint::ToesR)] * glm::vec4(contacts.toe, 1.0f));
    return sample;
}

Frame RecordFrame(const Walker& walker) {
    const Skeleton& skeleton = walker.Body().skeleton;
    Frame frame;
    frame.pose = walker.CurrentPose();
    frame.position = walker.Position();
    frame.heading = walker.Heading();
    frame.speed = walker.Speed();
    frame.phase = walker.Phase();
    frame.period = walker.Shape().period;
    const std::array<glm::vec3, kJointCount> joints = character::JointPositions(skeleton, frame.pose);
    frame.pelvis = joints[0];
    frame.head = joints[character::Index(Joint::Head)];
    for (const bool left : {true, false}) {
        const int i = left ? 0 : 1;
        frame.foot[i] = MeasureFoot(skeleton, frame.pose, left);
        frame.foot[i].planted = walker.IsPlanted(left);
        frame.foot[i].legReaches = walker.LegReachesFoot(left);
        frame.knee[i] = FlexionDegrees(skeleton, left ? Joint::CalfL : Joint::CalfR, frame.pose.rotation[character::Index(left ? Joint::CalfL : Joint::CalfR)]);
        frame.elbow[i] = FlexionDegrees(skeleton, left ? Joint::ForearmL : Joint::ForearmR, frame.pose.rotation[character::Index(left ? Joint::ForearmL : Joint::ForearmR)]);
    }
    return frame;
}

std::vector<Frame> Simulate(const CharacterParameters& person, const GroundFunction& ground, const std::vector<Segment>& script, float dt, const Mood& mood = {}) {
    const BodyModel body = MakeBodyModel(person);
    Walker walker(body, PersonalityFrom(body.parameters, mood), ground);
    walker.Reset(glm::vec3(0.0f), 0.0f);
    std::vector<Frame> frames;
    frames.push_back(RecordFrame(walker));
    for (const Segment& segment : script) {
        const int steps = static_cast<int>(std::round(segment.seconds / dt));
        for (int i = 0; i < steps; ++i) {
            walker.Update(dt, LocomotionCommand{segment.speed, segment.turnRate});
            frames.push_back(RecordFrame(walker));
        }
    }
    return frames;
}

struct FootReport {
    float worstSlip = 0.0f;          // metres a second, horizontal, of the lowest contact point of a planted foot
    float lowestBelowGround = 0.0f;  // of the heel and the ball (negative: under the ground)
    float toeBelowGround = 0.0f;     // of the toe tip
    float worstPlantedHeight = 0.0f; // the lowest contact point of a planted foot above the ground, the largest value
    float worstPlantedSink = 0.0f;   // the same, the most negative value
    float lowestKnee = 1000.0f;
    float lowestElbow = 1000.0f;
    int unreachedFrames = 0;
};

FootReport MeasureFeet(const std::vector<Frame>& frames, const GroundFunction& ground, float dt, size_t firstFrame = 0) {
    FootReport report;
    for (size_t k = std::max<size_t>(firstFrame, 1); k < frames.size(); ++k) {
        for (int i = 0; i < 2; ++i) {
            const FootSample& now = frames[k].foot[i];
            const FootSample& before = frames[k - 1].foot[i];
            const float heelHeight = now.heel.y - ground(now.heel.x, now.heel.z);
            const float ballHeight = now.ball.y - ground(now.ball.x, now.ball.z);
            report.lowestBelowGround = std::min({report.lowestBelowGround, heelHeight, ballHeight});
            report.toeBelowGround = std::min(report.toeBelowGround, now.toe.y - ground(now.toe.x, now.toe.z));
            if (now.planted && before.planted) {
                // A planted foot rests on its heel, then flat on its heel and ball, then on its ball. A point that touches the
                // ground must not move; the one that does not touch is swinging up or down about the other, and may.
                const float heelSpeed = glm::length(glm::vec2(now.heel.x - before.heel.x, now.heel.z - before.heel.z)) / dt;
                const float ballSpeed = glm::length(glm::vec2(now.ball.x - before.ball.x, now.ball.z - before.ball.z)) / dt;
                constexpr float kTouching = 0.003f;
                float slip = 0.0f;
                if (heelHeight < kTouching && ballHeight < kTouching) {
                    slip = std::max(heelSpeed, ballSpeed);
                } else if (heelHeight < kTouching) {
                    slip = heelSpeed;
                } else if (ballHeight < kTouching) {
                    slip = ballSpeed;
                }
                report.worstSlip = std::max(report.worstSlip, slip);
                const float lowest = std::min(heelHeight, ballHeight);
                report.worstPlantedHeight = std::max(report.worstPlantedHeight, lowest);
                report.worstPlantedSink = std::min(report.worstPlantedSink, lowest);
            }
            report.lowestKnee = std::min(report.lowestKnee, frames[k].knee[i]);
            report.lowestElbow = std::min(report.lowestElbow, frames[k].elbow[i]);
            if (!now.legReaches) {
                ++report.unreachedFrames;
            }
        }
    }
    return report;
}

// The checks of a walk: the feet do not slide, they stand on the ground and never go through it, the knees and elbows do not bend backwards.
void CheckWalk(const std::vector<Frame>& frames, const GroundFunction& ground, float dt, const std::string& label, size_t firstFrame = 0, float slipLimit = 0.15f) {
    const FootReport report = MeasureFeet(frames, ground, dt, firstFrame);
    // The slip limit is in metres a second: a planted foot is still, to a few percent of running speed. Sliding feet slip at
    // the speed of the body, 1 to 3 m/s.
    Check(report.worstSlip < slipLimit, label + ": a planted foot slides at " + std::to_string(report.worstSlip) + " m/s");
    Check(report.lowestBelowGround > -0.008f, label + ": a heel or ball goes through the ground by " + std::to_string(-report.lowestBelowGround) + " m");
    Check(report.toeBelowGround > -0.013f, label + ": a toe goes through the ground by " + std::to_string(-report.toeBelowGround) + " m");
    Check(report.worstPlantedHeight < 0.006f, label + ": a planted foot floats " + std::to_string(report.worstPlantedHeight) + " m above the ground");
    Check(report.worstPlantedSink > -0.008f, label + ": a planted foot sinks " + std::to_string(-report.worstPlantedSink) + " m");
    Check(report.lowestKnee > -0.5f, label + ": a knee bends backwards: " + std::to_string(report.lowestKnee) + " degrees");
    Check(report.lowestElbow > -0.5f, label + ": an elbow bends backwards: " + std::to_string(report.lowestElbow) + " degrees");
    Check(report.unreachedFrames == 0, label + ": a leg could not reach its foot in " + std::to_string(report.unreachedFrames) + " frames");
}

void GaitShapeFollowsSpeedAndLegLength() {
    const BodyModel average = MakeBodyModel(Person(0.5f, 30, 1.75f, 70, 0.4f));
    const BodyModel tall = MakeBodyModel(Person(1.0f, 30, 2.00f, 90, 0.4f));
    const BodyModel small = MakeBodyModel(Person(0.0f, 30, 1.50f, 50, 0.4f));
    const Personality neutral;
    float previousDuty = 1.0f;
    for (float speed = 0.2f; speed <= MaximumSpeed(average); speed += 0.2f) {
        const GaitShape shape = ComputeGaitShape(average, neutral, speed);
        Check(std::abs(shape.strideLength - speed * shape.period) < 1e-5f, "the stride is the speed times the period");
        Check(std::abs(shape.cadence * shape.period - 2.0f) < 1e-4f, "two steps make a period");
        Check(shape.dutyFactor <= previousDuty + 1e-4f, "the foot spends less of the cycle on the ground as the speed rises");
        previousDuty = shape.dutyFactor;
    }
    const GaitShape walk = ComputeGaitShape(average, neutral, 1.4f);
    Check(walk.cadence * 60.0f > 100.0f && walk.cadence * 60.0f < 125.0f, "an ordinary walk is 100 to 125 steps a minute");
    Check(walk.dutyFactor > 0.5f && walk.dutyFactor < 0.7f, "in a walk a foot is down for more than half of the cycle");
    Check(walk.runBlend == 0.0f, "1.4 metres a second is a walk");
    const GaitShape run = ComputeGaitShape(average, neutral, 4.0f);
    Check(run.dutyFactor < 0.5f, "in a run both feet leave the ground for a moment");
    Check(run.runBlend == 1.0f, "4 metres a second is a run");
    Check(run.cadence > walk.cadence, "a runner takes more steps a minute than a walker");
    // The stride at the same speed grows with the legs: a tall person takes longer, slower steps than a short one.
    const GaitShape tallWalk = ComputeGaitShape(tall, neutral, 1.4f);
    const GaitShape smallWalk = ComputeGaitShape(small, neutral, 1.4f);
    Check(tallWalk.strideLength > walk.strideLength && walk.strideLength > smallWalk.strideLength, "a longer leg takes a longer stride at the same speed");
    Check(tallWalk.cadence < smallWalk.cadence, "a longer leg takes fewer steps a minute");
    Check(MaximumSpeed(tall) > MaximumSpeed(average) && MaximumSpeed(average) > MaximumSpeed(small), "a longer leg runs faster");
}

void FeetStayOnTheGroundAndDoNotSlide() {
    const float dt = 1.0f / 120.0f;
    for (const CharacterParameters& person : SomePeople()) {
        const BodyModel body = MakeBodyModel(person);
        for (const float speed : {0.6f, 1.0f, 1.4f, 2.2f, 3.0f, MaximumSpeed(body)}) {
            const std::vector<Frame> frames = Simulate(person, FlatGround(), {{5.0f, speed, 0.0f}}, dt);
            CheckWalk(frames, FlatGround(), dt, "speed " + std::to_string(speed) + " stature " + std::to_string(person.statureMeters));
        }
    }
}

void StrideMatchesSpeed() {
    const float dt = 1.0f / 120.0f;
    for (const float speed : {0.9f, 1.4f, 3.0f}) {
        const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.4f);
        const std::vector<Frame> frames = Simulate(person, FlatGround(), {{10.0f, speed, 0.0f}}, dt);
        const size_t settled = static_cast<size_t>(5.0f / dt);
        for (int i = 0; i < 2; ++i) {
            std::vector<glm::vec3> heelStrikes;
            for (size_t k = settled; k < frames.size(); ++k) {
                if (frames[k].foot[i].planted && !frames[k - 1].foot[i].planted) {
                    heelStrikes.push_back(frames[k].foot[i].heel);
                }
            }
            Check(heelStrikes.size() >= 2, "a walk of two seconds has at least two heel strikes of each foot");
            for (size_t s = 1; s < heelStrikes.size(); ++s) {
                const float stride = glm::length(heelStrikes[s] - heelStrikes[s - 1]);
                const float expected = speed * frames[settled].period;
                Check(std::abs(stride - expected) < 0.02f + 0.02f * expected,
                      "the stride is " + std::to_string(stride) + " m but speed times period is " + std::to_string(expected));
            }
        }
    }
}

void TheGaitRepeatsEveryPeriod() {
    for (const float speed : {1.2f, 3.0f}) {
        for (const CharacterParameters& person : SomePeople()) {
            const BodyModel body = MakeBodyModel(person);
            const Personality personality = PersonalityFrom(body.parameters);
            const float period = ComputeGaitShape(body, personality, speed).period;
            const int stepsPerPeriod = 40;
            const float dt = period / stepsPerPeriod;
            Walker walker(body, personality);
            walker.Reset(glm::vec3(0.0f), 0.0f);
            // Walk up to speed and for a few cycles more, so that the start is forgotten.
            for (int i = 0; i < stepsPerPeriod * 8; ++i) {
                walker.Update(dt, LocomotionCommand{speed, 0.0f});
            }
            std::vector<std::array<glm::vec3, kJointCount>> first;
            std::vector<Pose> firstPoses;
            for (int i = 0; i < stepsPerPeriod; ++i) {
                walker.Update(dt, LocomotionCommand{speed, 0.0f});
                std::array<glm::vec3, kJointCount> joints = character::JointPositions(body.skeleton, walker.CurrentPose());
                for (glm::vec3& joint : joints) {
                    joint -= walker.Position();
                    joint.y -= 0.0f;
                }
                first.push_back(joints);
                firstPoses.push_back(walker.CurrentPose());
            }
            for (int i = 0; i < stepsPerPeriod; ++i) {
                walker.Update(dt, LocomotionCommand{speed, 0.0f});
                std::array<glm::vec3, kJointCount> joints = character::JointPositions(body.skeleton, walker.CurrentPose());
                for (glm::vec3& joint : joints) {
                    joint -= walker.Position();
                }
                for (int j = 0; j < kJointCount; ++j) {
                    Check(glm::length(joints[j] - first[i][j]) < 2e-3f, "a joint is where it was one period ago, relative to the path");
                    Check(DegreesBetween(walker.CurrentPose().rotation[j], firstPoses[i].rotation[j]) < 0.2f, "a joint is turned as it was one period ago");
                }
            }
        }
    }
}

void ThePelvisMovesSmoothly() {
    const float dt = 1.0f / 120.0f;
    for (const float speed : {1.0f, 1.4f, 2.2f, 3.0f, 4.0f}) {
        const CharacterParameters person = Person(0.5f, 35, 1.75f, 70, 0.4f);
        const std::vector<Frame> frames = Simulate(person, FlatGround(), {{6.0f, speed, 0.0f}}, dt);
        float largestAcceleration = 0.0f;
        float lowest = 10.0f;
        float highest = -10.0f;
        for (size_t k = static_cast<size_t>(3.0f / dt) + 1; k + 1 < frames.size(); ++k) {
            const glm::vec3 acceleration = (frames[k + 1].pelvis - 2.0f * frames[k].pelvis + frames[k - 1].pelvis) / (dt * dt);
            // The forward speed is constant; what is left is the bob and the sway.
            largestAcceleration = std::max(largestAcceleration, std::max(std::abs(acceleration.y), std::abs(acceleration.x)));
            lowest = std::min(lowest, frames[k].pelvis.y);
            highest = std::max(highest, frames[k].pelvis.y);
        }
        Check(largestAcceleration < 80.0f, "the pelvis does not jerk at " + std::to_string(speed) + " m/s: acceleration " + std::to_string(largestAcceleration));
        Check(highest - lowest < 0.14f, "the pelvis does not bob more than 14 cm at " + std::to_string(speed) + " m/s: " + std::to_string(highest - lowest));
        if (speed >= 1.0f && speed <= 1.5f) {
            Check(highest - lowest > 0.02f, "a walker's pelvis rises and falls by a few centimetres: " + std::to_string(highest - lowest));
        }
    }
}

void FeetFollowSlopesAndHills() {
    const float dt = 1.0f / 120.0f;
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.4f);
    struct Terrain {
        const char* name;
        GroundFunction ground;
    };
    const Terrain terrains[] = {{"uphill 12 degrees", SlopedGround(12.0f)}, {"downhill 12 degrees", SlopedGround(-12.0f)}, {"rolling hills", RollingGround()}};
    for (const Terrain& terrain : terrains) {
        for (const float speed : {1.0f, 1.4f, 2.0f}) {
            const std::vector<Frame> frames = Simulate(person, terrain.ground, {{6.0f, speed, 0.0f}}, dt);
            CheckWalk(frames, terrain.ground, dt, std::string(terrain.name) + " at " + std::to_string(speed), 0, 0.25f);
            // The pelvis keeps its height over the ground it walks on.
            const float leg = MakeBodyModel(person).legLength;
            for (size_t k = frames.size() / 2; k < frames.size(); ++k) {
                const float above = frames[k].pelvis.y - terrain.ground(frames[k].position.x, frames[k].position.z);
                Check(above > 0.7f * leg && above < 1.0f * leg, std::string(terrain.name) + ": the pelvis is " + std::to_string(above) + " m above the ground");
            }
        }
    }
    // Uphill the trunk leans in to the hill, downhill it stays more upright: the slope is felt in the pose.
    const std::vector<Frame> up = Simulate(person, SlopedGround(12.0f), {{4.0f, 1.2f, 0.0f}}, dt);
    const std::vector<Frame> down = Simulate(person, SlopedGround(-12.0f), {{4.0f, 1.2f, 0.0f}}, dt);
    const float upLean = up.back().head.z - up.back().pelvis.z;
    const float downLean = down.back().head.z - down.back().pelvis.z;
    Check(upLean > downLean, "the trunk leans further forward going uphill than downhill");
}

void ABodyTurnsAndLeansIntoTheTurn() {
    const float dt = 1.0f / 120.0f;
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.4f);
    for (const float turnRate : {0.6f, -0.6f}) {
        const std::vector<Frame> frames = Simulate(person, FlatGround(), {{2.0f, 2.0f, 0.0f}, {5.0f, 2.0f, turnRate}}, dt);
        CheckWalk(frames, FlatGround(), dt, "turning at " + std::to_string(turnRate), static_cast<size_t>(2.0f / dt));
        // Heading changes at the rate asked for, once the turn has settled.
        const float turned = (frames.back().heading - frames[static_cast<size_t>(5.0f / dt)].heading) / 2.0f;
        Check(std::abs(turned - turnRate) < 0.05f, "the heading turns at the rate asked for: " + std::to_string(turned));
        // The head is on the inside of the pelvis, over a cycle: to the character's left (positive X turned by the heading) for a left turn.
        float lean = 0.0f;
        int count = 0;
        for (size_t k = static_cast<size_t>(5.0f / dt); k < frames.size(); ++k) {
            const glm::vec3 left(std::cos(frames[k].heading), 0.0f, -std::sin(frames[k].heading));
            lean += glm::dot(frames[k].head - frames[k].pelvis, left);
            ++count;
        }
        lean /= static_cast<float>(count);
        Check(lean * turnRate > 0.0f && std::abs(lean) > 0.01f, "the body leans into the turn: " + std::to_string(lean));
    }
}

void AWalkerStartsAndStopsGently() {
    const float dt = 1.0f / 60.0f;
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.4f);
    const BodyModel body = MakeBodyModel(person);
    Walker walker(body, PersonalityFrom(body.parameters));
    walker.Reset(glm::vec3(0.0f), 0.0f);
    const WalkerSettings settings;
    float previousSpeed = 0.0f;
    float peak = 0.0f;
    std::vector<Frame> frames;
    for (int i = 0; i < 60 * 12; ++i) {
        const float command = (i > 60 && i < 60 * 7) ? 1.5f : 0.0f;
        walker.Update(dt, LocomotionCommand{command, 0.0f});
        Check(std::abs(walker.Speed() - previousSpeed) <= settings.deceleration * dt * 1.001f, "the speed changes no faster than the limit");
        previousSpeed = walker.Speed();
        peak = std::max(peak, walker.Speed());
        frames.push_back(RecordFrame(walker));
    }
    Check(std::abs(peak - 1.5f) < 1e-3f, "the walker reaches the speed asked for");
    Check(walker.Speed() == 0.0f, "the walker stops");
    CheckWalk(frames, FlatGround(), dt, "start and stop");

    // Stopped for a while, the feet stand flat and still, whatever step the walker was in.
    const Frame& end = frames.back();
    const Frame& earlier = frames[frames.size() - 60];
    for (int i = 0; i < 2; ++i) {
        Check(end.foot[i].heel.y < 0.004f && end.foot[i].ball.y < 0.004f && end.foot[i].heel.y > -0.004f && end.foot[i].ball.y > -0.004f,
              "a stopped foot stands flat on the ground");
        Check(glm::length(end.foot[i].heel - earlier.foot[i].heel) < 0.002f, "a stopped foot does not move");
    }
}

// The largest change of a joint's rotation from one frame to the next, in degrees a second, over a script of motion.
float LargestAngularSpeed(const std::vector<Frame>& frames, float dt, size_t from, Joint* worstJoint = nullptr) {
    float largest = 0.0f;
    for (size_t k = std::max<size_t>(from, 1); k < frames.size(); ++k) {
        for (int j = 1; j < kJointCount; ++j) {
            const float speed = DegreesBetween(frames[k - 1].pose.rotation[j], frames[k].pose.rotation[j]) / dt;
            if (speed > largest) {
                largest = speed;
                if (worstJoint != nullptr) {
                    *worstJoint = static_cast<Joint>(j);
                }
            }
        }
    }
    return largest;
}

void EveryChangeOfGaitIsContinuous() {
    // Idle, walk, run, turn, stop, idle: no pops at any step of the way, at two frame rates.
    const std::vector<Segment> script = {{1.5f, 0.0f, 0.0f}, {4.0f, 1.4f, 0.0f}, {4.0f, 3.8f, 0.0f}, {3.0f, 3.8f, 0.9f}, {3.0f, 1.4f, -0.5f}, {3.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}};
    for (const float dt : {1.0f / 120.0f, 1.0f / 60.0f, 1.0f / 30.0f}) {
        for (const CharacterParameters& person : SomePeople()) {
            const std::vector<Frame> frames = Simulate(person, FlatGround(), script, dt);
            Joint worst = Joint::Pelvis;
            const float speed = LargestAngularSpeed(frames, dt, 0, &worst);
            // A knee swings through 100 degrees in a tenth of a second when running; a pop would be several times faster.
            Check(speed < 2300.0f, "no joint turns faster than 2300 degrees a second (" + std::string(MakeBodyModel(person).skeleton[worst].name) + " at " +
                                       std::to_string(speed) + ") at dt " + std::to_string(dt));
            // Slower frames and a changing speed spread the planted foot's error over bigger steps.
            CheckWalk(frames, FlatGround(), dt, "script at dt " + std::to_string(dt) + " stature " + std::to_string(person.statureMeters), 0, 0.4f);
            // The pelvis never jumps: from one frame to the next it moves by no more than the speed allows forwards and 2 m/s up or down.
            for (size_t k = 1; k < frames.size(); ++k) {
                Check(glm::length(frames[k].pelvis - frames[k - 1].pelvis) < (frames[k].speed + 2.0f) * dt + 0.02f, "the pelvis does not jump");
            }
        }
    }
}

void TheSameScriptGivesTheSameMotionEveryTime() {
    const std::vector<Segment> script = {{1.0f, 0.0f, 0.0f}, {3.0f, 1.4f, 0.0f}, {3.0f, 3.5f, 0.4f}, {2.0f, 0.0f, 0.0f}};
    for (const CharacterParameters& person : SomePeople()) {
        const std::vector<Frame> a = Simulate(person, RollingGround(), script, 1.0f / 60.0f);
        const std::vector<Frame> b = Simulate(person, RollingGround(), script, 1.0f / 60.0f);
        Check(a.size() == b.size(), "the same number of frames");
        bool same = a.size() == b.size();
        for (size_t k = 0; same && k < a.size(); ++k) {
            same = std::memcmp(&a[k].pose, &b[k].pose, sizeof(Pose)) == 0;
        }
        Check(same, "two runs of the same script give the same poses, bit for bit");
    }
    // Another seed fidgets differently when standing.
    const BodyModel body = MakeBodyModel(Person(0.5f, 30, 1.75f, 70, 0.4f));
    WalkerSettings first, second;
    first.seed = 1;
    second.seed = 2;
    Walker one(body, PersonalityFrom(body.parameters), {}, first);
    Walker two(body, PersonalityFrom(body.parameters), {}, second);
    for (int i = 0; i < 300; ++i) {
        one.Update(1.0f / 60.0f, {});
        two.Update(1.0f / 60.0f, {});
    }
    Check(std::memcmp(&one.CurrentPose(), &two.CurrentPose(), sizeof(Pose)) != 0, "different seeds idle differently");
}

void TheGaitWorksForEveryBodyFromTheSmallestToTheLargest() {
    const float dt = 1.0f / 120.0f;
    for (const CharacterParameters& person : ExtremePeople()) {
        const BodyModel body = MakeBodyModel(person);
        for (const float speed : {1.0f, 1.6f, 2.5f, MaximumSpeed(body)}) {
            const std::string label = "stature " + std::to_string(person.statureMeters) + " mass " + std::to_string(person.massKg) + " speed " + std::to_string(speed);
            CheckWalk(Simulate(person, FlatGround(), {{5.0f, speed, 0.0f}}, dt), FlatGround(), dt, label);
            CheckWalk(Simulate(person, RollingGround(), {{5.0f, speed, 0.0f}}, dt), RollingGround(), dt, label + " on hills", 0, 0.3f);
        }
    }
}

// The angle of the left upper arm in the walking plane, from hanging straight down, forwards positive.
float ArmAngleDegrees(const Skeleton& skeleton, const Pose& pose, float heading) {
    const std::array<glm::vec3, kJointCount> joints = character::JointPositions(skeleton, pose);
    const glm::vec3 arm = joints[character::Index(Joint::ForearmL)] - joints[character::Index(Joint::UpperArmL)];
    const glm::vec3 forward(std::sin(heading), 0.0f, std::cos(heading));
    return std::atan2(glm::dot(arm, forward), -arm.y) * kRadiansToDegrees;
}

float ArmSwingRange(const CharacterParameters& person, const Mood& mood, float speed) {
    const float dt = 1.0f / 60.0f;
    const std::vector<Frame> frames = Simulate(person, FlatGround(), {{8.0f, speed, 0.0f}}, dt, mood);
    const Skeleton skeleton = character::BuildSkeleton(person);
    float low = 1000.0f;
    float high = -1000.0f;
    for (size_t k = frames.size() / 2; k < frames.size(); ++k) {
        const float angle = ArmAngleDegrees(skeleton, frames[k].pose, frames[k].heading);
        low = std::min(low, angle);
        high = std::max(high, angle);
    }
    return high - low;
}

void PersonalityFollowsFromTheBodyAndTheMood() {
    const CharacterParameters young = Person(0.5f, 22, 1.75f, 70, 0.4f);
    const CharacterParameters old = Person(0.5f, 78, 1.75f, 70, 0.4f);
    const CharacterParameters heavy = Person(0.5f, 30, 1.75f, 110, 0.4f);
    const CharacterParameters slim = Person(0.5f, 30, 1.75f, 55, 0.4f);
    const CharacterParameters man = Person(1.0f, 30, 1.75f, 70, 0.4f);
    const CharacterParameters woman = Person(0.0f, 30, 1.75f, 70, 0.4f);

    const BodyModel youngBody = MakeBodyModel(young);
    const GaitShape youngWalk = ComputeGaitShape(youngBody, PersonalityFrom(youngBody.parameters), 1.2f);
    const GaitShape oldWalk = ComputeGaitShape(youngBody, PersonalityFrom(MakeBodyModel(old).parameters), 1.2f);
    Check(oldWalk.strideLength < 0.95f * youngWalk.strideLength, "an old person takes a shorter stride at the same speed");
    Check(oldWalk.dutyFactor > youngWalk.dutyFactor, "an old person spends longer with both feet down");
    Check(PersonalityFrom(old).armSwingScale < PersonalityFrom(young).armSwingScale, "an old person swings the arms less");
    Check(PersonalityFrom(old).stoopDegrees > PersonalityFrom(young).stoopDegrees, "an old person stoops");
    Check(PersonalityFrom(heavy).stepWidthScale > PersonalityFrom(slim).stepWidthScale, "a heavy person stands wider");
    Check(PersonalityFrom(heavy).pelvisSwayScale > PersonalityFrom(slim).pelvisSwayScale, "a heavy person sways more");
    Check(PersonalityFrom(man).strideScale > PersonalityFrom(woman).strideScale, "a man steps a little longer");
    Check(PersonalityFrom(woman).pelvisRotationScale > PersonalityFrom(man).pelvisRotationScale, "a woman's hips turn more");

    Mood brisk;
    brisk.energy = 1.0f;
    brisk.confidence = 1.0f;
    Mood weary;
    weary.energy = -1.0f;
    weary.confidence = -1.0f;
    const Personality briskPersonality = PersonalityFrom(young, brisk);
    const Personality wearyPersonality = PersonalityFrom(young, weary);
    Check(briskPersonality.strideScale > wearyPersonality.strideScale, "a brisk mood strides longer");
    Check(briskPersonality.tempoScale > wearyPersonality.tempoScale, "a brisk mood steps quicker");
    Check(briskPersonality.stoopDegrees < wearyPersonality.stoopDegrees, "a confident mood stands straighter");
    Check(briskPersonality.armSwingScale > wearyPersonality.armSwingScale, "a brisk mood swings the arms more");

    // The effect shows in the final pose: the arms swing through a wider angle.
    const float youngSwing = ArmSwingRange(young, {}, 1.4f);
    const float oldSwing = ArmSwingRange(old, {}, 1.4f);
    Check(youngSwing > 15.0f && youngSwing < 70.0f, "the arms of a walker swing by a plausible angle: " + std::to_string(youngSwing));
    Check(oldSwing < 0.85f * youngSwing, "an old person's arms swing through a smaller angle: " + std::to_string(oldSwing) + " against " + std::to_string(youngSwing));
    Check(ArmSwingRange(young, brisk, 1.4f) > ArmSwingRange(young, weary, 1.4f), "a brisk mood swings the arms through a wider angle");
}

void TheArmsSwingAgainstTheLegsAndTheShouldersAgainstTheHips() {
    const float dt = 1.0f / 120.0f;
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.4f);
    const std::vector<Frame> frames = Simulate(person, FlatGround(), {{8.0f, 1.4f, 0.0f}}, dt);
    const Skeleton skeleton = character::BuildSkeleton(person);
    // Over a cycle the left arm is forward when the left foot is behind, and the left shoulder is forward when the left hip is back.
    float armTimesFoot = 0.0f;
    float shoulderTimesHip = 0.0f;
    for (size_t k = frames.size() / 2; k < frames.size(); ++k) {
        const std::array<glm::vec3, kJointCount> joints = character::JointPositions(skeleton, frames[k].pose);
        const float arm = ArmAngleDegrees(skeleton, frames[k].pose, frames[k].heading);
        const float foot = frames[k].foot[0].heel.z - frames[k].foot[1].heel.z;  // the left foot is ahead of the right
        armTimesFoot += arm * foot;
        const float shoulders = joints[character::Index(Joint::UpperArmL)].z - joints[character::Index(Joint::UpperArmR)].z;
        const float hips = joints[character::Index(Joint::ThighL)].z - joints[character::Index(Joint::ThighR)].z;
        shoulderTimesHip += shoulders * hips;
    }
    Check(armTimesFoot < 0.0f, "the left arm swings back when the left foot is forward");
    Check(shoulderTimesHip < 0.0f, "the shoulders turn against the hips");
}

void TheFeetPointTheWayTheyGoAndTheHeadStaysLevel() {
    const float dt = 1.0f / 120.0f;
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.4f);
    const std::vector<Frame> frames = Simulate(person, FlatGround(), {{6.0f, 1.4f, 0.0f}}, dt);
    const Skeleton skeleton = character::BuildSkeleton(person);
    float highestPitch = -1000.0f;
    float lowestPitch = 1000.0f;
    for (size_t k = frames.size() / 2; k < frames.size(); ++k) {
        const WorldPose world = ForwardKinematics(skeleton, frames[k].pose);
        const glm::vec3 headForward = world.rotation[character::Index(Joint::Head)] * glm::vec3(0.0f, 0.0f, 1.0f);
        const float pitch = std::asin(std::clamp(headForward.y, -1.0f, 1.0f)) * kRadiansToDegrees;
        highestPitch = std::max(highestPitch, pitch);
        lowestPitch = std::min(lowestPitch, pitch);
        Check(headForward.z > 0.95f, "the head looks ahead");
    }
    Check(highestPitch - lowestPitch < 6.0f, "the head does not nod more than a few degrees while walking: " + std::to_string(highestPitch - lowestPitch));
}

void NoiseIsDeterministicSmoothAndBounded() {
    float largestStep = 0.0f;
    for (uint32_t seed = 0; seed < 5; ++seed) {
        for (float x = -20.0f; x < 20.0f; x += 0.01f) {
            const float value = SmoothNoise(seed, x);
            Check(value >= -1.0f && value <= 1.0f, "noise stays within -1 to 1");
            Check(DriftNoise(seed, x) >= -1.0f && DriftNoise(seed, x) <= 1.0f, "drift noise stays within -1 to 1");
            Check(value == SmoothNoise(seed, x), "noise is a pure function of its arguments");
            largestStep = std::max(largestStep, std::abs(SmoothNoise(seed, x + 0.01f) - value));
        }
    }
    Check(largestStep < 0.04f, "noise changes smoothly: the largest step in 0.01 is " + std::to_string(largestStep));
    Check(SmoothNoise(1, 3.3f) != SmoothNoise(2, 3.3f), "different seeds give different noise");
}

void StandingStillBreathesAndShiftsWeightWithoutMovingTheFeet() {
    const float dt = 1.0f / 60.0f;
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.4f);
    const std::vector<Frame> frames = Simulate(person, FlatGround(), {{20.0f, 0.0f, 0.0f}}, dt);
    float chestLow = 1000.0f;
    float chestHigh = -1000.0f;
    float pelvisLow = 1000.0f;
    float pelvisHigh = -1000.0f;
    const Skeleton skeleton = character::BuildSkeleton(person);
    for (const Frame& frame : frames) {
        const std::array<glm::vec3, kJointCount> joints = character::JointPositions(skeleton, frame.pose);
        const float chest = joints[character::Index(Joint::Neck)].z;
        chestLow = std::min(chestLow, chest);
        chestHigh = std::max(chestHigh, chest);
        pelvisLow = std::min(pelvisLow, frame.pelvis.x);
        pelvisHigh = std::max(pelvisHigh, frame.pelvis.x);
        for (int i = 0; i < 2; ++i) {
            Check(glm::length(frame.foot[i].heel - frames[0].foot[i].heel) < 1e-3f, "the feet of a standing person do not move");
        }
    }
    Check(chestHigh - chestLow > 0.002f, "the chest moves with the breath: " + std::to_string(chestHigh - chestLow));
    Check(pelvisHigh - pelvisLow > 0.004f && pelvisHigh - pelvisLow < 0.06f, "the weight shifts a little: " + std::to_string(pelvisHigh - pelvisLow));
    Check(LargestAngularSpeed(frames, dt, 0) < 40.0f, "a standing person moves slowly");
}

}  // namespace

int main(int argc, char** argv) {
    const std::string filter = argc > 1 ? argv[1] : "";
    struct Test {
        const char* name;
        std::function<void()> run;
    };
    const Test tests[] = {
        {"ForwardKinematicsOfTheRestPoseIsTheRestPose", ForwardKinematicsOfTheRestPoseIsTheRestPose},
        {"ForwardKinematicsAgreesWithTheSkinningOfTheCharacterModule", ForwardKinematicsAgreesWithTheSkinningOfTheCharacterModule},
        {"PelvisAndRotationHelpersWork", PelvisAndRotationHelpersWork},
        {"EulerAnglesRoundTripAcrossEveryLimitRange", EulerAnglesRoundTripAcrossEveryLimitRange},
        {"ClampingKeepsEveryJointInsideItsLimits", ClampingKeepsEveryJointInsideItsLimits},
        {"ClampingLeavesARotationInsideTheLimitsAlone", ClampingLeavesARotationInsideTheLimitsAlone},
        {"HingesAreLimitedInFlexionAndKneesDoNotBendBackwards", HingesAreLimitedInFlexionAndKneesDoNotBendBackwards},
        {"LegIkReachesTargetsWithTheKneeTowardsThePole", LegIkReachesTargetsWithTheKneeTowardsThePole},
        {"ArmIkReachesTargetsWithTheElbowTowardsThePole", ArmIkReachesTargetsWithTheElbowTowardsThePole},
        {"IkAtTheRestPositionKeepsTheLimbNearRest", IkAtTheRestPositionKeepsTheLimbNearRest},
        {"IkOutOfReachStretchesTowardsTheTargetWithoutHyperextending", IkOutOfReachStretchesTowardsTheTargetWithoutHyperextending},
        {"IkRespectsALimitedHip", IkRespectsALimitedHip},
        {"GaitShapeFollowsSpeedAndLegLength", GaitShapeFollowsSpeedAndLegLength},
        {"FeetStayOnTheGroundAndDoNotSlide", FeetStayOnTheGroundAndDoNotSlide},
        {"StrideMatchesSpeed", StrideMatchesSpeed},
        {"TheGaitRepeatsEveryPeriod", TheGaitRepeatsEveryPeriod},
        {"ThePelvisMovesSmoothly", ThePelvisMovesSmoothly},
        {"FeetFollowSlopesAndHills", FeetFollowSlopesAndHills},
        {"ABodyTurnsAndLeansIntoTheTurn", ABodyTurnsAndLeansIntoTheTurn},
        {"AWalkerStartsAndStopsGently", AWalkerStartsAndStopsGently},
        {"EveryChangeOfGaitIsContinuous", EveryChangeOfGaitIsContinuous},
        {"TheSameScriptGivesTheSameMotionEveryTime", TheSameScriptGivesTheSameMotionEveryTime},
        {"TheGaitWorksForEveryBodyFromTheSmallestToTheLargest", TheGaitWorksForEveryBodyFromTheSmallestToTheLargest},
        {"PersonalityFollowsFromTheBodyAndTheMood", PersonalityFollowsFromTheBodyAndTheMood},
        {"TheArmsSwingAgainstTheLegsAndTheShouldersAgainstTheHips", TheArmsSwingAgainstTheLegsAndTheShouldersAgainstTheHips},
        {"TheFeetPointTheWayTheyGoAndTheHeadStaysLevel", TheFeetPointTheWayTheyGoAndTheHeadStaysLevel},
        {"NoiseIsDeterministicSmoothAndBounded", NoiseIsDeterministicSmoothAndBounded},
        {"StandingStillBreathesAndShiftsWeightWithoutMovingTheFeet", StandingStillBreathesAndShiftsWeightWithoutMovingTheFeet},
    };

    int ran = 0;
    for (const Test& test : tests) {
        if (!filter.empty() && filter != test.name) {
            continue;
        }
        const int before = failureCount;
        test.run();
        ++ran;
        std::cout << (failureCount == before ? "ok      " : "FAILED  ") << test.name << "\n";
    }
    if (ran == 0) {
        std::cerr << "no test named " << filter << "\n";
        return 2;
    }
    std::cout << ran << " tests, " << failureCount << " failed checks\n";
    return failureCount == 0 ? 0 : 1;
}
