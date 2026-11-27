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
#include <map>
#include <limits>
#include <string>
#include <vector>

#include "animation/BodyModel.h"
#include "animation/Blending.h"
#include "animation/Gait.h"
#include "animation/LookAt.h"
#include "animation/Secondary.h"
#include "animation/SitStand.h"
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

bool PoseIsFinite(const Pose& pose);

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
    float lowestKnee = 1000.0f;
    float lowestElbow = 1000.0f;
    int unreachedFrames = 0;
    // Metres along the floor that each heel, ball and toe tip (left, right) moved while it touched the floor.
    float dragged[2][3] = {};
    float MostDragged() const {
        float most = 0.0f;
        for (const auto& foot : dragged) {
            for (const float metres : foot) {
                most = std::max(most, metres);
            }
        }
        return most;
    }
};

FootReport MeasureFeet(const std::vector<Frame>& frames, const GroundFunction& ground, float dt, size_t firstFrame = 0) {
    FootReport report;
    for (size_t k = std::max<size_t>(firstFrame, 1); k < frames.size(); ++k) {
        for (int i = 0; i < 2; ++i) {
            const FootSample& now = frames[k].foot[i];
            const FootSample& before = frames[k - 1].foot[i];
            // The heel, the ball and the tip of the toes, measured on the pose and not on the planner's flags. Any of them that
            // is within 3 mm of the ground, in two frames running, is on the ground, and must not move along it.
            const glm::vec3 points[3] = {now.heel, now.ball, now.toe};
            const glm::vec3 pointsBefore[3] = {before.heel, before.ball, before.toe};
            constexpr float kTouching = 0.003f;
            float lowestHeight = 1000.0f;
            for (int p = 0; p < 3; ++p) {
                const float height = points[p].y - ground(points[p].x, points[p].z);
                const float heightBefore = pointsBefore[p].y - ground(pointsBefore[p].x, pointsBefore[p].z);
                lowestHeight = std::min(lowestHeight, height);
                if (p < 2) {
                    report.lowestBelowGround = std::min(report.lowestBelowGround, height);
                } else {
                    report.toeBelowGround = std::min(report.toeBelowGround, height);
                }
                // On the ground for three frames running: a toe that grazes the floor for one frame is not dragged along it.
                const FootSample& twoBefore = frames[std::max<size_t>(k, 2) - 2].foot[i];
                const glm::vec3 pointTwoBefore[3] = {twoBefore.heel, twoBefore.ball, twoBefore.toe};
                const float heightTwoBefore = pointTwoBefore[p].y - ground(pointTwoBefore[p].x, pointTwoBefore[p].z);
                if (height < kTouching && heightBefore < kTouching && heightTwoBefore < kTouching) {
                    const float slip = glm::length(glm::vec2(points[p].x - pointsBefore[p].x, points[p].z - pointsBefore[p].z)) / dt;
                    report.worstSlip = std::max(report.worstSlip, slip);
                    report.dragged[i][p] += slip * dt;                }
            }
            if (now.planted) {
                report.worstPlantedHeight = std::max(report.worstPlantedHeight, lowestHeight);
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

// The worst numbers the checks of a walk met, by the kind of walk, printed at the end of a run so that the limits can be set from
// what the gait does, with a margin, and not from a guess.
struct WorstNumbers {
    float slip = 0.0f;
    float belowGround = 0.0f;
    float toeBelowGround = 0.0f;
    float plantedHeight = 0.0f;
    float lowestKnee = 1000.0f;
};
std::map<std::string, WorstNumbers>& WorstByKind() {
    static std::map<std::string, WorstNumbers> worst;
    return worst;
}
std::string currentKind = "other";

// The checks of a walk: the feet do not slide, they stand on the ground and never go through it, the knees and elbows do not bend backwards.
void CheckWalk(const std::vector<Frame>& frames, const GroundFunction& ground, float dt, const std::string& label, size_t firstFrame = 0, float slipLimit = 0.1f) {
    const FootReport report = MeasureFeet(frames, ground, dt, firstFrame);
    WorstNumbers& worst = WorstByKind()[currentKind];
    worst.slip = std::max(worst.slip, report.worstSlip);
    worst.belowGround = std::min(worst.belowGround, report.lowestBelowGround);
    worst.toeBelowGround = std::min(worst.toeBelowGround, report.toeBelowGround);
    worst.plantedHeight = std::max(worst.plantedHeight, report.worstPlantedHeight);
    worst.lowestKnee = std::min(worst.lowestKnee, report.lowestKnee);
    // The slip limit is in metres a second: a planted foot is still, to a few percent of running speed. Sliding feet slip at
    // the speed of the body, 1 to 3 m/s.
    Check(report.worstSlip < slipLimit, label + ": a planted foot slides at " + std::to_string(report.worstSlip) + " m/s");
    Check(report.lowestBelowGround > -0.003f, label + ": a heel or ball goes through the ground by " + std::to_string(-report.lowestBelowGround) + " m");
    Check(report.toeBelowGround > -0.003f, label + ": a toe goes through the ground by " + std::to_string(-report.toeBelowGround) + " m");
    Check(report.worstPlantedHeight < 0.003f, label + ": a planted foot floats " + std::to_string(report.worstPlantedHeight) + " m above the ground");
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
    currentKind = "steady walk and run";
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
        const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.1f);
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
        const CharacterParameters person = Person(0.5f, 35, 1.75f, 70, 0.1f);
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
        Check(largestAcceleration < (speed < 1.5f ? 40.0f : 90.0f), "the pelvis does not jerk at " + std::to_string(speed) + " m/s: acceleration " + std::to_string(largestAcceleration));
        Check(highest - lowest < 0.14f, "the pelvis does not bob more than 14 cm at " + std::to_string(speed) + " m/s: " + std::to_string(highest - lowest));
        if (speed >= 1.0f && speed <= 1.5f) {
            Check(highest - lowest > 0.02f, "a walker's pelvis rises and falls by a few centimetres: " + std::to_string(highest - lowest));
        }
    }
}

void FeetFollowSlopesAndHills() {
    currentKind = "slopes and hills";
    const float dt = 1.0f / 120.0f;
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.1f);
    struct Terrain {
        const char* name;
        GroundFunction ground;
    };
    const Terrain terrains[] = {{"uphill 12 degrees", SlopedGround(12.0f)}, {"downhill 12 degrees", SlopedGround(-12.0f)}, {"rolling hills", RollingGround()}};
    for (const Terrain& terrain : terrains) {
        for (const float speed : {1.0f, 1.4f, 2.0f}) {
            const std::vector<Frame> frames = Simulate(person, terrain.ground, {{6.0f, speed, 0.0f}}, dt);
            CheckWalk(frames, terrain.ground, dt, std::string(terrain.name) + " at " + std::to_string(speed), 0, 0.1f);
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

void SteepGroundIsFollowedAsWellAsTheLegsAllow() {
    // The gait follows slopes of up to 12 degrees with its feet on the ground. On steeper ground the legs run out of reach
    // (the planted foot of a walker going up 20 degrees sinks about 6 cm, 15 degrees about 3 cm): the pose stays finite and
    // inside the limits, the knees do not bend backwards, and the feet do not sink by more than that. A person on such ground
    // wants stairs or climbing, which this module does not make.
    const float dt = 1.0f / 120.0f;
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.4f);
    for (const float degrees : {15.0f, 20.0f, -15.0f, -20.0f}) {
        const GroundFunction ground = SlopedGround(degrees);
        const std::vector<Frame> frames = Simulate(person, ground, {{6.0f, 1.2f, 0.0f}}, dt);
        const FootReport report = MeasureFeet(frames, ground, dt, static_cast<size_t>(2.0f / dt));
        const std::string label = "on a slope of " + std::to_string(degrees) + " degrees";
        Check(report.lowestBelowGround > -0.07f && report.toeBelowGround > -0.07f, label + ": the feet sink by " + std::to_string(-report.lowestBelowGround) + " m");
        Check(report.lowestKnee > -0.5f, label + ": a knee bends backwards");
        Check(report.worstSlip < 0.7f, label + ": a foot slides at " + std::to_string(report.worstSlip) + " m/s");
        for (const Frame& frame : frames) {
            Check(PoseIsFinite(frame.pose), label + ": a pose is not finite");
        }
    }
}

void ABodyTurnsAndLeansIntoTheTurn() {
    currentKind = "turning while moving";
    const float dt = 1.0f / 120.0f;
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.1f);
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

void EveryTurnRateIsFollowedOrLimitedWithoutBreakingTheLegs() {
    currentKind = "turn rate sweep";
    // Turn rates from nothing to 4 radians a second, to the left and to the right, standing, walking and running: the legs
    // reach, the knees stay inside their range and the feet do not slide. The walker limits what it follows.
    const float dt = 1.0f / 120.0f;
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.1f);
    const BodyModel body = MakeBodyModel(person);
    for (const float speed : {0.0f, 1.4f, 3.0f}) {
        for (const float rate : {0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 3.0f, 4.0f, -1.0f, -2.5f, -4.0f}) {
            const std::vector<Frame> frames = Simulate(person, FlatGround(), {{4.0f, speed, 0.0f}, {6.0f, speed, rate}}, dt);
            const std::string label = "speed " + std::to_string(speed) + " turning at " + std::to_string(rate);
            CheckWalk(frames, FlatGround(), dt, label, static_cast<size_t>(4.0f / dt), 0.1f);
            // The heading turns by at most what the gait allows.
            const float turned = std::abs(frames.back().heading - frames[static_cast<size_t>(4.0f / dt) + static_cast<size_t>(3.0f / dt)].heading) / 3.0f;
            Check(turned <= MaximumTurnRate(body, speed) + 0.05f, label + ": the body turns faster than the gait follows: " + std::to_string(turned));
            Check(std::abs(rate) > MaximumTurnRate(body, speed) || std::abs(turned - std::abs(rate)) < 0.1f, label + ": a turn that the gait can follow is followed: " + std::to_string(turned));
        }
    }
}

// Any heel, ball or toe tip within 3 mm of the floor, in two frames running, whatever the walker says about its feet.
float WorstSlideOnTheFloor(const std::vector<Frame>& frames, float dt, size_t from) {
    return MeasureFeet(frames, FlatGround(), dt, from).worstSlip;
}

void StoppingAndTurningOnTheSpotDoNotDragTheFeet() {
    currentKind = "stopping and turning on the spot";
    const float dt = 1.0f / 120.0f;
    constexpr float kSlideLimit = 0.1f;  // metres a second
    for (const CharacterParameters& person : {Person(0.5f, 30, 1.75f, 70, 0.4f), Person(1.0f, 30, 1.90f, 100, 0.8f, 1), Person(0.0f, 25, 1.55f, 48, 0.2f, 2)}) {
        const std::string who = "stature " + std::to_string(person.statureMeters);
        // Stopping from a walk and from a run: feet that swing to their last places are lifted, not dragged.
        for (const float speed : {1.4f, 3.0f}) {
            const std::vector<Frame> stopping = Simulate(person, FlatGround(), {{5.0f, speed, 0.0f}, {5.0f, 0.0f, 0.0f}}, dt);
            const std::string label = who + " stopping from " + std::to_string(speed);
            std::cerr << "INFO " << label << " dragged " << MeasureFeet(stopping, FlatGround(), dt, static_cast<size_t>(2.0f / dt)).MostDragged() << "\n";
            Check(WorstSlideOnTheFloor(stopping, dt, static_cast<size_t>(2.0f / dt)) < kSlideLimit, label + ": a foot slides on the floor at " + std::to_string(WorstSlideOnTheFloor(stopping, dt, static_cast<size_t>(2.0f / dt))) + " m/s");
            CheckWalk(stopping, FlatGround(), dt, label, static_cast<size_t>(2.0f / dt), kSlideLimit);
            // And the feet end side by side, flat, and still.
            const Frame& end = stopping.back();
            const Frame& before = stopping[stopping.size() - 120];
            for (int i = 0; i < 2; ++i) {
                Check(glm::length(end.foot[i].heel - before.foot[i].heel) < 1e-3f, label + ": the feet are still after the stop");
                Check(std::abs(end.foot[i].heel.y) < 0.004f && std::abs(end.foot[i].ball.y) < 0.004f, label + ": the feet are flat on the floor after the stop");
            }
            Check(std::abs(end.foot[0].heel.z - end.foot[1].heel.z) < 0.06f, label + ": the feet end side by side: " + std::to_string(end.foot[0].heel.z - end.foot[1].heel.z));
        }
        // Turning on the spot, to either side, with small steps.
        for (const float rate : {0.6f, -0.6f, 2.0f, -2.0f}) {
            const std::vector<Frame> turning = Simulate(person, FlatGround(), {{2.0f, 0.0f, 0.0f}, {6.0f, 0.0f, rate}, {3.0f, 0.0f, 0.0f}}, dt);
            const std::string label = who + " turning on the spot at " + std::to_string(rate);
            std::cerr << "INFO " << label << " dragged " << MeasureFeet(turning, FlatGround(), dt, 0).MostDragged() << "\n";
            Check(WorstSlideOnTheFloor(turning, dt, 0) < kSlideLimit, label + ": a foot is dragged along the floor at " + std::to_string(WorstSlideOnTheFloor(turning, dt, 0)) + " m/s");
            CheckWalk(turning, FlatGround(), dt, label, 0, kSlideLimit);
            // The body does turn, and the feet go with it.
            const float turned = std::abs(turning[static_cast<size_t>(8.0f / dt)].heading - turning[static_cast<size_t>(2.0f / dt)].heading);
            Check(turned > 0.8f * std::min(std::abs(rate), 2.0f) * 6.0f * 0.8f, label + ": the body turns: " + std::to_string(turned) + " radians");
        }
    }
}

void AWalkerStartsAndStopsGently() {
    currentKind = "start and stop";
    const float dt = 1.0f / 60.0f;
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.1f);
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
    currentKind = "scripted changes";
    // Idle, walk, run, turn, stop, idle: no pops at any step of the way, at two frame rates.
    const std::vector<Segment> script = {{1.5f, 0.0f, 0.0f}, {4.0f, 1.4f, 0.0f}, {4.0f, 3.8f, 0.0f}, {3.0f, 3.8f, 0.9f}, {3.0f, 1.4f, -0.5f}, {3.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}};
    for (const float dt : {1.0f / 120.0f, 1.0f / 60.0f, 1.0f / 30.0f}) {
        for (const CharacterParameters& person : SomePeople()) {
            const std::vector<Frame> frames = Simulate(person, FlatGround(), script, dt);
            Joint worst = Joint::Pelvis;
            const float speed = LargestAngularSpeed(frames, dt, 0, &worst);
            // A knee swings through 100 degrees in a tenth of a second when running; a pop would be several times faster.
            Check(speed < 1150.0f, "no joint turns faster than 1150 degrees a second (" + std::string(MakeBodyModel(person).skeleton[worst].name) + " at " +
                                       std::to_string(speed) + ") at dt " + std::to_string(dt));
            // Slower frames and a changing speed spread the planted foot's error over bigger steps.
            CheckWalk(frames, FlatGround(), dt, "script at dt " + std::to_string(dt) + " stature " + std::to_string(person.statureMeters), 0, 0.1f);
            // The pelvis never jumps: from one frame to the next it moves by no more than the speed allows forwards and 2 m/s up or down.
            for (size_t k = 1; k < frames.size(); ++k) {
                Check(glm::length(frames[k].pelvis - frames[k - 1].pelvis) < (frames[k].speed + 2.0f) * dt + 0.02f, "the pelvis does not jump");
            }
        }
    }
}

// The largest turn of any joint, and of the knees, between two frames of a steady gait, in degrees per frame.
struct StepSizes {
    float anyJoint = 0.0f;
    float knee = 0.0f;
};

StepSizes LargestFrameSteps(const CharacterParameters& person, float speed, float dt) {
    const std::vector<Frame> frames = Simulate(person, FlatGround(), {{6.0f, speed, 0.0f}}, dt);
    StepSizes sizes;
    for (size_t k = static_cast<size_t>(3.5f / dt); k < frames.size(); ++k) {
        for (int j = 1; j < kJointCount; ++j) {
            const float step = DegreesBetween(frames[k - 1].pose.rotation[j], frames[k].pose.rotation[j]);
            sizes.anyJoint = std::max(sizes.anyJoint, step);
            if (j == character::Index(Joint::CalfL) || j == character::Index(Joint::CalfR)) {
                sizes.knee = std::max(sizes.knee, step);
            }
        }
    }
    return sizes;
}

void AStepOfAFrameShrinksWithTheFrame() {
    // A pop is a jump that does not get smaller when the frames do. In a steady gait the change of every joint between two
    // frames is proportional to the length of the frame, and the speed of a knee is what a knee can do.
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.1f);
    struct Case {
        float speed;
        float kneeLimit;  // degrees a second
        float jointLimit;
    };
    for (const Case& gait : {Case{1.4f, 650.0f, 650.0f}, Case{3.0f, 900.0f, 900.0f}}) {
        const float dt = 1.0f / 120.0f;
        const StepSizes coarse = LargestFrameSteps(person, gait.speed, dt);
        const StepSizes fine = LargestFrameSteps(person, gait.speed, dt / 4.0f);
        Check(fine.anyJoint < 0.45f * coarse.anyJoint, "at " + std::to_string(gait.speed) + " m/s a joint step shrinks with the frame: " + std::to_string(coarse.anyJoint) + " to " + std::to_string(fine.anyJoint));
        Check(fine.knee < 0.45f * coarse.knee, "at " + std::to_string(gait.speed) + " m/s a knee step shrinks with the frame: " + std::to_string(coarse.knee) + " to " + std::to_string(fine.knee));
        Check(coarse.knee / dt < gait.kneeLimit, "a knee turns no faster than " + std::to_string(gait.kneeLimit) + " degrees a second at " + std::to_string(gait.speed) + ": " + std::to_string(coarse.knee / dt));
        Check(coarse.anyJoint / dt < gait.jointLimit, "a joint turns no faster than " + std::to_string(gait.jointLimit) + " degrees a second at " + std::to_string(gait.speed) + ": " + std::to_string(coarse.anyJoint / dt));
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
    currentKind = "extreme bodies";
    const float dt = 1.0f / 120.0f;
    for (const CharacterParameters& person : ExtremePeople()) {
        const BodyModel body = MakeBodyModel(person);
        for (const float speed : {1.0f, 1.6f, 2.5f, MaximumSpeed(body)}) {
            const std::string label = "stature " + std::to_string(person.statureMeters) + " mass " + std::to_string(person.massKg) + " speed " + std::to_string(speed);
            CheckWalk(Simulate(person, FlatGround(), {{5.0f, speed, 0.0f}}, dt), FlatGround(), dt, label);
            CheckWalk(Simulate(person, RollingGround(), {{5.0f, speed, 0.0f}}, dt), RollingGround(), dt, label + " on hills", 0, 0.12f);
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
    const CharacterParameters young = Person(0.5f, 22, 1.75f, 70, 0.1f);
    const CharacterParameters old = Person(0.5f, 78, 1.75f, 70, 0.1f);
    const CharacterParameters heavy = Person(0.5f, 30, 1.75f, 110, 0.1f);
    const CharacterParameters slim = Person(0.5f, 30, 1.75f, 55, 0.4f);
    const CharacterParameters man = Person(1.0f, 30, 1.75f, 70, 0.1f);
    const CharacterParameters woman = Person(0.0f, 30, 1.75f, 70, 0.1f);

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
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.1f);
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
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.1f);
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
    const CharacterParameters person = Person(0.5f, 30, 1.75f, 70, 0.1f);
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
    Check(LargestAngularSpeed(frames, dt, 0) < 10.0f, "a standing person moves slowly");
}

// ---------------------------------------------------------------------------
// Looking
// ---------------------------------------------------------------------------

Pose StandingPose(const BodyModel& body) {
    Walker walker(body, PersonalityFrom(body.parameters));
    walker.Reset(glm::vec3(0.0f), 0.0f);
    return walker.CurrentPose();
}

float AngleBetweenDegrees(const glm::vec3& a, const glm::vec3& b) {
    return std::acos(std::clamp(glm::dot(glm::normalize(a), glm::normalize(b)), -1.0f, 1.0f)) * kRadiansToDegrees;
}

void TheHeadAndEyesLookAtWhatIsInRange() {
    Random random(41);
    for (const CharacterParameters& person : SomePeople()) {
        const BodyModel body = MakeBodyModel(person);
        for (int trial = 0; trial < 60; ++trial) {
            Pose pose = StandingPose(body);
            const float yaw = random.Between(-70.0f, 70.0f) * kDegreesToRadians;
            const float pitch = random.Between(-30.0f, 35.0f) * kDegreesToRadians;
            const float distance = random.Between(0.8f, 6.0f);
            const glm::vec3 eyes = EyeCentre(body, pose);
            const glm::vec3 target = eyes + distance * glm::vec3(std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw));
            const LookAtResult result = LookAt(body, pose, target);
            Check(result.reachedWithHeadAndEyes, "a target in range is looked at: error " + std::to_string(result.errorDegrees) + " degrees");
            // The gaze, from the eye centre, is on the target: the head faces it up to what the eyes can add.
            const glm::vec3 toTarget = target - EyeCentre(body, pose);
            Check(AngleBetweenDegrees(HeadForward(body, pose), toTarget) < 35.0f, "the head faces roughly at the target");
            Check(std::abs(result.leftEye.yawDegrees) <= 35.01f && std::abs(result.rightEye.yawDegrees) <= 35.01f, "the eyes stay within their range");
            Check(LimitExcessDegrees(body.skeleton, Joint::Neck, pose.rotation[character::Index(Joint::Neck)]) < 0.1f, "the neck is inside its limits");
            Check(LimitExcessDegrees(body.skeleton, Joint::Head, pose.rotation[character::Index(Joint::Head)]) < 0.1f, "the head is inside its limits");
        }
    }
}

void ATargetBehindIsLookedAtAsFarAsTheNeckAllows() {
    const BodyModel body = MakeBodyModel(Person(0.5f, 30, 1.75f, 70, 0.4f));
    Pose pose = StandingPose(body);
    const glm::vec3 target = EyeCentre(body, pose) + glm::vec3(0.0f, 0.0f, -3.0f);
    const LookAtResult result = LookAt(body, pose, target);
    Check(!result.reachedWithHeadAndEyes, "a target behind cannot be looked at");
    Check(result.errorDegrees > 20.0f, "the gaze misses a target behind by a good deal");
    for (const Joint joint : {Joint::Neck, Joint::Head}) {
        Check(LimitExcessDegrees(body.skeleton, joint, pose.rotation[character::Index(joint)]) < 0.1f, "the neck and head are held to their limits");
    }
    // The head still turns round as far as it can.
    Check(HeadForward(body, pose).z < 0.5f || std::abs(HeadForward(body, pose).x) > 0.8f, "the head turns a long way towards the target");
}

void TheLookFadesInWithItsWeight() {
    const BodyModel body = MakeBodyModel(Person(0.5f, 30, 1.75f, 70, 0.4f));
    const Pose standing = StandingPose(body);
    const glm::vec3 target = EyeCentre(body, standing) + glm::vec3(2.0f, 0.3f, 2.0f);
    Pose none = standing;
    LookAt(body, none, target, 0.0f);
    Check(DegreesBetween(none.rotation[character::Index(Joint::Head)], standing.rotation[character::Index(Joint::Head)]) < 1e-3f, "a look of weight 0 changes nothing");
    Pose half = standing;
    Pose full = standing;
    LookAt(body, half, target, 0.5f);
    LookAt(body, full, target, 1.0f);
    const float halfTurn = DegreesBetween(half.rotation[character::Index(Joint::Head)], standing.rotation[character::Index(Joint::Head)]);
    const float fullTurn = DegreesBetween(full.rotation[character::Index(Joint::Head)], standing.rotation[character::Index(Joint::Head)]);
    Check(halfTurn > 0.3f * fullTurn && halfTurn < 0.7f * fullTurn, "a look of weight a half turns the head about half as far");
}

void TheEyesConvergeOnANearTarget() {
    const BodyModel body = MakeBodyModel(Person(0.5f, 30, 1.75f, 70, 0.4f));
    Pose pose = StandingPose(body);
    const glm::vec3 near = EyeCentre(body, pose) + glm::vec3(0.0f, 0.0f, 0.4f);
    const LookAtResult closeLook = LookAt(body, pose, near);
    Check(closeLook.leftEye.yawDegrees < -1.0f && closeLook.rightEye.yawDegrees > 1.0f, "the eyes turn in towards a target close to the nose");
    Pose farPose = StandingPose(body);
    const glm::vec3 far = EyeCentre(body, farPose) + glm::vec3(0.0f, 0.0f, 20.0f);
    const LookAtResult farLook = LookAt(body, farPose, far);
    Check(std::abs(farLook.leftEye.yawDegrees) < 0.5f && std::abs(farLook.rightEye.yawDegrees) < 0.5f, "the eyes look nearly straight at a distant target");
}

// ---------------------------------------------------------------------------
// Springs, secondary motion and hits
// ---------------------------------------------------------------------------

void ACriticallyDampedSpringFollowsTheExactSolutionAtAnyStepSize() {
    const float omega = 9.0f;
    SpringState fine{glm::vec3(1.0f, -0.5f, 0.2f), glm::vec3(2.0f, 1.0f, -3.0f)};
    SpringState coarse = fine;
    for (int i = 0; i < 100; ++i) {
        StepCriticallyDamped(fine, glm::vec3(0.3f, 0.0f, -0.2f), omega, 0.001f);
    }
    StepCriticallyDamped(coarse, glm::vec3(0.3f, 0.0f, -0.2f), omega, 0.1f);
    Check(glm::length(fine.value - coarse.value) < 1e-4f && glm::length(fine.velocity - coarse.velocity) < 1e-3f, "one big step gives what many small steps give");
    // And the closed form: from 1 with no velocity and no force, y(t) = (1 + w t) e^(-w t).
    SpringState spring{glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f)};
    StepCriticallyDamped(spring, glm::vec3(0.0f), omega, 0.2f);
    Check(std::abs(spring.value.x - (1.0f + omega * 0.2f) * std::exp(-omega * 0.2f)) < 1e-5f, "the spring matches the closed form");
    // A step of zero time changes nothing.
    SpringState same = spring;
    StepCriticallyDamped(same, glm::vec3(1.0f), omega, 0.0f);
    Check(same.value == spring.value && same.velocity == spring.velocity, "a step of no time changes nothing");
}

void ACriticallyDampedSpringNeverOvershoots() {
    const float omega = 10.0f;
    // Released from a displacement: it falls to zero and stays above it.
    SpringState released{glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f)};
    float previous = 1.0f;
    for (int i = 0; i < 400; ++i) {
        StepCriticallyDamped(released, glm::vec3(0.0f), omega, 0.005f);
        Check(released.value.x >= -1e-6f && released.value.x <= previous + 1e-6f, "a released spring falls without crossing zero");
        previous = released.value.x;
    }
    Check(released.value.x < 0.001f, "and settles");
    // Kicked from rest: it goes out and returns, and never goes the other way.
    SpringState kicked{glm::vec3(0.0f), glm::vec3(3.0f, 0.0f, 0.0f)};
    float peak = 0.0f;
    for (int i = 0; i < 400; ++i) {
        StepCriticallyDamped(kicked, glm::vec3(0.0f), omega, 0.005f);
        Check(kicked.value.x >= -1e-6f, "a kicked spring does not swing past rest");
        peak = std::max(peak, kicked.value.x);
    }
    Check(std::abs(peak - 3.0f / (2.71828183f * omega)) < 0.005f, "the peak of a kick is v0 over e omega");
    // Pushed by a steady force: it climbs to force over omega squared and does not pass it.
    SpringState pushed{};
    for (int i = 0; i < 800; ++i) {
        StepCriticallyDamped(pushed, glm::vec3(5.0f, 0.0f, 0.0f), omega, 0.005f);
        Check(pushed.value.x <= 5.0f / (omega * omega) + 1e-6f, "a pushed spring does not overshoot its balance point");
    }
    Check(std::abs(pushed.value.x - 0.05f) < 1e-4f, "and reaches it");
}

void TheChestAndHeadLagBehindAcceleration() {
    SecondaryMotion motion;
    const float dt = 1.0f / 120.0f;
    BodyLayer layer;
    for (int i = 0; i < 240; ++i) {
        layer = motion.Update(dt, glm::vec3(0.0f, 0.0f, 0.0f), 0.0f);
    }
    Check(RotationAngleDegrees(layer.additive.rotation[character::Index(Joint::Spine2)]) < 1e-3f, "a body that is not accelerating has no secondary motion");
    // Speeding up forwards tips the chest back (negative about X) by an amount that grows with the acceleration.
    float chestPitch = 0.0f;
    for (int i = 0; i < 240; ++i) {
        layer = motion.Update(dt, glm::vec3(0.0f, 0.0f, 3.0f), 0.0f);
        chestPitch = EulerDegrees(layer.additive.rotation[character::Index(Joint::Spine2)]).x;
        Check(chestPitch <= 1e-4f, "the chest tips back as the body speeds up");
        Check(RotationAngleDegrees(layer.additive.rotation[character::Index(Joint::Spine2)]) <= 8.01f, "the lean stays within the limit");
    }
    Check(chestPitch < -1.5f, "the chest settles tipped back: " + std::to_string(chestPitch));
    // It comes back when the acceleration stops, without swinging forwards.
    for (int i = 0; i < 600; ++i) {
        layer = motion.Update(dt, glm::vec3(0.0f), 0.0f);
        Check(EulerDegrees(layer.additive.rotation[character::Index(Joint::Spine2)]).x <= 1e-3f, "the chest does not swing forwards past upright");
    }
    Check(RotationAngleDegrees(layer.additive.rotation[character::Index(Joint::Spine2)]) < 0.05f, "the chest returns to upright");
    // The legs are not touched.
    for (const Joint joint : {Joint::ThighL, Joint::CalfR, Joint::FootL, Joint::Pelvis}) {
        Check(RotationAngleDegrees(layer.additive.rotation[character::Index(joint)]) == 0.0f, "secondary motion leaves the legs alone");
    }
    // Frame rate does not matter.
    SecondaryMotion slow;
    SecondaryMotion fast;
    BodyLayer a;
    BodyLayer b;
    for (int i = 0; i < 30; ++i) {
        a = slow.Update(1.0f / 30.0f, glm::vec3(0.0f, 0.0f, 3.0f), 0.0f);
        for (int j = 0; j < 4; ++j) {
            b = fast.Update(1.0f / 120.0f, glm::vec3(0.0f, 0.0f, 3.0f), 0.0f);
        }
    }
    Check(DegreesBetween(a.additive.rotation[character::Index(Joint::Head)], b.additive.rotation[character::Index(Joint::Head)]) < 0.05f, "secondary motion is the same at 30 and 120 frames a second");
}

void APushMakesTheBodyGiveAndComeBack() {
    const float dt = 1.0f / 120.0f;
    HitReaction hit;
    Check(hit.Settled(), "a body that has not been hit is settled");
    hit.Push(glm::vec3(0.0f, 0.0f, 2.0f), 0.0f);  // a shove from behind: the body is sent forwards
    float peakShift = 0.0f;
    float lowestPitch = 0.0f;
    float previousShift = 0.0f;
    bool falling = false;
    BodyLayer layer;
    for (int i = 0; i < 600; ++i) {
        layer = hit.Update(dt);
        const float shift = layer.pelvisOffset.z;
        Check(shift >= -1e-5f, "the pelvis is shoved the way of the push and does not swing back past rest");
        if (shift < previousShift) {
            falling = true;
        }
        Check(!falling || shift <= previousShift + 1e-6f, "after its peak the pelvis only comes back");
        previousShift = shift;
        peakShift = std::max(peakShift, shift);
        lowestPitch = std::min(lowestPitch, EulerDegrees(layer.additive.rotation[character::Index(Joint::Spine2)]).x);
    }
    Check(peakShift > 0.03f && peakShift <= 0.1001f, "the pelvis gives by a few centimetres: " + std::to_string(peakShift));
    Check(lowestPitch < -3.0f, "the trunk tips back: " + std::to_string(lowestPitch));
    Check(hit.Settled(), "after five seconds the body has settled");

    // A push twice as big is a body that moves twice as far (until the limit).
    HitReaction small;
    HitReaction big;
    small.Push(glm::vec3(1.0f, 0.0f, 0.0f), 0.0f);
    big.Push(glm::vec3(2.0f, 0.0f, 0.0f), 0.0f);
    float smallPeak = 0.0f;
    float bigPeak = 0.0f;
    for (int i = 0; i < 240; ++i) {
        smallPeak = std::max(smallPeak, std::abs(small.Update(dt).pelvisOffset.x));
        bigPeak = std::max(bigPeak, std::abs(big.Update(dt).pelvisOffset.x));
    }
    Check(std::abs(bigPeak - 2.0f * smallPeak) < 0.002f, "a bigger push moves the body further in proportion");
    // The push is in the world, the body is turned: a push towards +X on a body that faces +X is a push forwards.
    HitReaction turned;
    turned.Push(glm::vec3(2.0f, 0.0f, 0.0f), 3.14159265f * 0.5f);
    Check(std::abs(turned.Update(0.1f).pelvisOffset.z) > 0.01f, "a push is taken in the frame of the body");
}

void ABodyWalksOnWhileItIsPushed() {
    currentKind = "pushed";
    // A push while walking: the feet stay planted, the pose has no jumps (the onset has its own, looser bound).
    const float dt = 1.0f / 60.0f;
    const BodyModel body = MakeBodyModel(Person(0.5f, 30, 1.75f, 70, 0.4f));
    Walker walker(body, PersonalityFrom(body.parameters));
    walker.Reset(glm::vec3(0.0f), 0.0f);
    HitReaction hit;
    SecondaryMotion secondary;
    glm::vec3 previousVelocity(0.0f);
    glm::vec3 previousPosition = walker.Position();
    std::vector<Frame> frames;
    for (int i = 0; i < 60 * 8; ++i) {
        if (i == 60 * 5) {
            hit.Push(glm::vec3(1.5f, 0.0f, -1.0f), walker.Heading());
        }
        const glm::vec3 velocity = (walker.Position() - previousPosition) / dt;
        const glm::vec3 acceleration = (velocity - previousVelocity) / dt;
        previousPosition = walker.Position();
        previousVelocity = velocity;
        walker.SetLayer(CombinedLayers(hit.Update(dt), secondary.Update(dt, acceleration, walker.Heading())));
        walker.Update(dt, LocomotionCommand{1.4f, 0.0f});
        frames.push_back(RecordFrame(walker));
    }
    CheckWalk(frames, FlatGround(), dt, "walking and pushed", 0, 0.1f);
    const size_t hitFrame = 60 * 5;
    Check(LargestAngularSpeed(frames, dt, 60 * 3) < 1150.0f, "the pose has no pops while pushed");
    // The onset of the hit is the quickest thing the body does: the trunk reaches its peak within a fraction of a second.
    float onset = 0.0f;
    for (size_t k = hitFrame; k < hitFrame + 30; ++k) {
        onset = std::max(onset, DegreesBetween(frames[k - 1].pose.rotation[character::Index(Joint::Spine2)], frames[k].pose.rotation[character::Index(Joint::Spine2)]) / dt);
    }
    Check(onset < 200.0f, "the trunk does not jerk at the onset of a hit: " + std::to_string(onset) + " degrees a second");
}

// ---------------------------------------------------------------------------
// Sitting down and standing up
// ---------------------------------------------------------------------------

std::vector<float> SitProgressSteps(int count) {
    std::vector<float> steps;
    for (int i = 0; i <= count; ++i) {
        steps.push_back(static_cast<float>(i) / count);
    }
    return steps;
}

void CheckSitting(const BodyModel& body, const Seat& seat, const GroundFunction& ground, const std::string& label) {
    const Skeleton& skeleton = body.skeleton;
    const std::array<glm::vec3, 2> heels = {SitHeel(body, seat, true, ground), SitHeel(body, seat, false, ground)};
    Pose previous = SitPose(body, seat, 0.0f, ground);
    float previousPelvisHeight = 1000.0f;
    for (const float progress : SitProgressSteps(60)) {
        const Pose pose = SitPose(body, seat, progress, ground);
        const std::array<glm::vec3, kJointCount> joints = character::JointPositions(skeleton, pose);
        for (int i = 0; i < 2; ++i) {
            const FootSample foot = MeasureFoot(skeleton, pose, i == 0);
            const std::string at = label + " at " + std::to_string(progress);
            Check(glm::length(glm::vec2(foot.heel.x - heels[i].x, foot.heel.z - heels[i].z)) < 0.004f, at + ": a foot slides while sitting");
            Check(std::abs(foot.heel.y - ground(foot.heel.x, foot.heel.z)) < 0.006f, at + ": a heel is off the ground");
            Check(std::abs(foot.ball.y - ground(foot.ball.x, foot.ball.z)) < 0.012f, at + ": a ball is off the ground");
            const Joint knee = i == 0 ? Joint::CalfL : Joint::CalfR;
            Check(FlexionDegrees(skeleton, knee, pose.rotation[character::Index(knee)]) > -0.5f, at + ": a knee bends backwards");
            Check(FlexionDegrees(skeleton, knee, pose.rotation[character::Index(knee)]) < skeleton[knee].maxAngleDegrees.x, at + ": a knee bends too far");
        }
        for (int j = 1; j < kJointCount; ++j) {
            Check(LimitExcessDegrees(skeleton, static_cast<Joint>(j), pose.rotation[j]) < 0.3f, label + ": a joint is outside its limits: " + skeleton.joints[j].name);
        }
        Check(joints[0].y <= previousPelvisHeight + 1e-4f, label + ": the pelvis goes down all the way");
        previousPelvisHeight = joints[0].y;
        float largest = 0.0f;
        for (int j = 1; j < kJointCount; ++j) {
            largest = std::max(largest, DegreesBetween(previous.rotation[j], pose.rotation[j]));
        }
        Check(largest < 6.0f, label + ": the pose does not jump between steps of a sixtieth of the motion: " + std::to_string(largest) + " degrees");
        previous = pose;
    }

    const std::array<glm::vec3, kJointCount> standing = character::JointPositions(skeleton, SitPose(body, seat, 0.0f, ground));
    const Pose seatedPose = SitPose(body, seat, 1.0f, ground);
    const std::array<glm::vec3, kJointCount> seated = character::JointPositions(skeleton, seatedPose);
    const float legReach = body.thighLength + body.shankLength;
    Check(standing[0].y - heels[0].y > 0.9f * (legReach + body.ankleHeight), label + ": a person at the start of a sit stands");
    Check(std::abs(seated[0].y - (seat.center.y + 0.058f * skeleton.statureMeters)) < 0.01f, label + ": the pelvis rests at the height of the seat");
    Check(glm::length(glm::vec2(seated[0].x - seat.center.x, seated[0].z - seat.center.z)) < 0.01f, label + ": the pelvis is over the seat");
    // Seated, the thighs are close to level, the knees bent to a right angle, and the trunk upright.
    for (const bool left : {true, false}) {
        const glm::vec3 thigh = seated[character::Index(left ? Joint::CalfL : Joint::CalfR)] - seated[character::Index(left ? Joint::ThighL : Joint::ThighR)];
        Check(std::abs(thigh.y) < 0.45f * glm::length(thigh), label + ": a seated thigh is about level");
        const Joint knee = left ? Joint::CalfL : Joint::CalfR;
        const float flexion = FlexionDegrees(skeleton, knee, seatedPose.rotation[character::Index(knee)]);
        Check(flexion > 55.0f && flexion < 120.0f, label + ": a seated knee is bent to about a right angle: " + std::to_string(flexion));
    }
    Check(std::abs(seated[character::Index(Joint::Head)].x - seated[0].x) < 0.06f, label + ": the trunk is upright when seated");
}

void ASittingPersonKeepsTheirFeetDownAndEndsOnTheSeat() {
    for (const CharacterParameters& person : SomePeople()) {
        const BodyModel body = MakeBodyModel(person);
        Seat seat;
        seat.center = glm::vec3(2.0f, 0.45f * body.skeleton.statureMeters / 1.75f, -1.0f);
        seat.facing = 0.7f;
        CheckSitting(body, seat, FlatGround(), "stature " + std::to_string(person.statureMeters));
    }
    for (const CharacterParameters& person : ExtremePeople()) {
        const BodyModel body = MakeBodyModel(person);
        Seat seat;
        seat.center = glm::vec3(0.0f, 0.45f * body.skeleton.statureMeters / 1.75f, 0.0f);
        seat.facing = 0.0f;
        CheckSitting(body, seat, FlatGround(), "extreme stature " + std::to_string(person.statureMeters) + " mass " + std::to_string(person.massKg));
    }
    // On a slope the seat is higher above the sea and the feet follow the ground.
    const BodyModel body = MakeBodyModel(Person(0.5f, 30, 1.75f, 70, 0.4f));
    Seat seat;
    seat.facing = 0.0f;
    const GroundFunction slope = SlopedGround(8.0f);
    seat.center = glm::vec3(0.0f, slope(0.0f, 0.0f) + 0.45f, 0.0f);
    CheckSitting(body, seat, slope, "on a slope");
}

void StandingUpIsSittingDownBackwards() {
    const BodyModel body = MakeBodyModel(Person(0.5f, 30, 1.75f, 70, 0.4f));
    Seat seat;
    seat.center = glm::vec3(0.0f, 0.45f, 0.0f);
    float previous = -1000.0f;
    for (const float progress : SitProgressSteps(40)) {
        const Pose standingUp = SitPose(body, seat, 1.0f - progress);
        const float height = character::JointPositions(body.skeleton, standingUp)[0].y;
        Check(height >= previous - 1e-4f, "the pelvis rises all the way when standing up");
        previous = height;
    }
}

// ---------------------------------------------------------------------------
// Blending
// ---------------------------------------------------------------------------

Pose PoseOfSmallTurns(Random& random, float maxDegrees) {
    Pose pose;
    for (int i = 0; i < kJointCount; ++i) {
        pose.rotation[i] = RandomRotation(random, maxDegrees);
    }
    pose.rootTranslation = glm::vec3(random.Between(-1, 1), random.Between(-1, 1), random.Between(-1, 1));
    return pose;
}

void BlendingTwoPosesGoesFromOneToTheOther() {
    Random random(51);
    const Pose a = PoseOfSmallTurns(random, 80.0f);
    const Pose b = PoseOfSmallTurns(random, 80.0f);
    const Pose atZero = BlendPoses(a, b, 0.0f);
    const Pose atOne = BlendPoses(a, b, 1.0f);
    const Pose middle = BlendPoses(a, b, 0.5f);
    for (int i = 0; i < kJointCount; ++i) {
        Check(DegreesBetween(atZero.rotation[i], a.rotation[i]) < 1e-3f, "a blend of weight 0 is the first pose");
        Check(DegreesBetween(atOne.rotation[i], b.rotation[i]) < 1e-3f, "a blend of weight 1 is the second pose");
        const float whole = DegreesBetween(a.rotation[i], b.rotation[i]);
        Check(std::abs(DegreesBetween(a.rotation[i], middle.rotation[i]) - 0.5f * whole) < 0.05f, "the middle of a blend is half way round");
    }
    Check(glm::length(middle.rootTranslation - 0.5f * (a.rootTranslation + b.rootTranslation)) < 1e-5f, "the root moves in a straight line");
    // The shortest way round, even when the quaternions have opposite signs.
    Pose flipped = b;
    for (glm::quat& q : flipped.rotation) {
        q = -q;
    }
    const Pose viaFlipped = BlendPoses(a, flipped, 0.5f);
    for (int i = 0; i < kJointCount; ++i) {
        Check(DegreesBetween(viaFlipped.rotation[i], middle.rotation[i]) < 0.05f, "a pose and its negated quaternions blend the same way");
    }
}

void WeightedBlendsFollowTheirWeights() {
    Random random(52);
    const Pose a = PoseOfSmallTurns(random, 30.0f);
    const Pose b = PoseOfSmallTurns(random, 30.0f);
    const Pose c = PoseOfSmallTurns(random, 30.0f);
    const WeightedPose onlyA[] = {{&a, 1.0f}, {&b, 0.0f}, {&c, 0.0f}};
    const Pose same = BlendWeighted(onlyA);
    for (int i = 0; i < kJointCount; ++i) {
        Check(DegreesBetween(same.rotation[i], a.rotation[i]) < 1e-3f, "a weight of 1 on one pose gives that pose");
    }
    // Only the ratio matters: 1 to 3 is the same as 0.25 to 0.75, and near a quarter of the way for small turns.
    const WeightedPose quarter[] = {{&a, 1.0f}, {&b, 3.0f}};
    const WeightedPose scaled[] = {{&a, 0.25f}, {&b, 0.75f}};
    const Pose x = BlendWeighted(quarter);
    const Pose y = BlendWeighted(scaled);
    const Pose slerped = BlendPoses(a, b, 0.75f);
    for (int i = 0; i < kJointCount; ++i) {
        Check(DegreesBetween(x.rotation[i], y.rotation[i]) < 1e-3f, "the weights are scaled to add up to 1");
        Check(DegreesBetween(x.rotation[i], slerped.rotation[i]) < 1.0f, "a weighted blend of two poses is close to the spherical blend");
    }
    // A negative weight counts as nothing; all-zero weights give the first pose.
    const WeightedPose negative[] = {{&a, 1.0f}, {&b, -5.0f}};
    Check(DegreesBetween(BlendWeighted(negative).rotation[3], a.rotation[3]) < 1e-3f, "a negative weight counts as nothing");
    const WeightedPose zero[] = {{&b, 0.0f}, {&a, 0.0f}};
    Check(DegreesBetween(BlendWeighted(zero).rotation[3], b.rotation[3]) < 1e-3f, "weights of 0 give the first pose");
}

void AdditiveLayersApplyOnTopOfAPose() {
    Random random(53);
    const Pose reference = PoseOfSmallTurns(random, 40.0f);
    const Pose animated = PoseOfSmallTurns(random, 40.0f);
    const Pose base = PoseOfSmallTurns(random, 40.0f);
    const Pose additive = MakeAdditive(animated, reference);
    // The additive pose put back on its reference is the animated pose.
    const Pose back = ApplyAdditive(reference, additive, 1.0f);
    for (int i = 0; i < kJointCount; ++i) {
        Check(DegreesBetween(back.rotation[i], animated.rotation[i]) < 1e-2f, "an additive pose applied to its reference gives the animated pose");
    }
    Check(glm::length(back.rootTranslation - animated.rootTranslation) < 1e-5f, "and so does the root");
    const Pose none = ApplyAdditive(base, additive, 0.0f);
    const Pose half = ApplyAdditive(base, additive, 0.5f);
    const Pose full = ApplyAdditive(base, additive, 1.0f);
    for (int i = 0; i < kJointCount; ++i) {
        Check(DegreesBetween(none.rotation[i], base.rotation[i]) < 1e-3f, "an additive layer of weight 0 changes nothing");
        const float whole = DegreesBetween(base.rotation[i], full.rotation[i]);
        Check(std::abs(DegreesBetween(base.rotation[i], half.rotation[i]) - 0.5f * whole) < 0.05f, "an additive layer of weight a half turns half as far");
    }

    BlendTree tree;
    tree.AddPose(base, 1.0f);
    tree.AddPose(animated, 3.0f);
    tree.AddAdditive(additive, 0.5f);
    const WeightedPose both[] = {{&base, 1.0f}, {&animated, 3.0f}};
    const Pose expected = ApplyAdditive(BlendWeighted(both), additive, 0.5f);
    const Pose built = tree.Evaluate();
    for (int i = 0; i < kJointCount; ++i) {
        Check(DegreesBetween(built.rotation[i], expected.rotation[i]) < 1e-3f, "a blend tree is its poses blended and its layers applied");
    }
    tree.Clear();
    Check(DegreesBetween(tree.Evaluate().rotation[5], glm::quat(1, 0, 0, 0)) < 1e-6f, "an empty blend tree is the rest pose");
}

void ACrossFadeIsSmoothAndHasNoPops() {
    CrossFade fade;
    fade.Begin(0.5f);
    Check(fade.Weight() == 0.0f && fade.Active(), "a fade starts at 0");
    float previous = 0.0f;
    float previousStep = 0.0f;
    for (int i = 0; i < 30; ++i) {
        fade.Advance(1.0f / 60.0f);
        Check(fade.Weight() >= previous - 1e-6f, "the weight of a fade only rises");
        previousStep = fade.Weight() - previous;
        previous = fade.Weight();
    }
    (void)previousStep;
    Check(fade.Weight() == 1.0f && !fade.Active(), "a fade ends at 1 after its time");
    CrossFade instant;
    instant.Begin(0.0f);
    Check(instant.Weight() == 1.0f, "a fade of no time is over at once");

    // A fade from a walk to a run: both states keep running, and the blend has no pops.
    const float dt = 1.0f / 60.0f;
    const BodyModel body = MakeBodyModel(Person(0.5f, 30, 1.75f, 70, 0.4f));
    Walker walking(body, PersonalityFrom(body.parameters));
    Walker running(body, PersonalityFrom(body.parameters));
    walking.Reset(glm::vec3(0.0f), 0.0f);
    running.Reset(glm::vec3(0.0f), 0.0f);
    for (int i = 0; i < 60 * 4; ++i) {
        walking.Update(dt, LocomotionCommand{1.4f, 0.0f});
        running.Update(dt, LocomotionCommand{3.0f, 0.0f});
    }
    CrossFade state;
    std::vector<Pose> shown;
    for (int i = 0; i < 60 * 3; ++i) {
        if (i == 60) {
            state.Begin(0.5f);
        }
        walking.Update(dt, LocomotionCommand{1.4f, 0.0f});
        running.Update(dt, LocomotionCommand{3.0f, 0.0f});
        state.Advance(dt);
        shown.push_back(i < 60 ? walking.CurrentPose() : state.Blend(walking.CurrentPose(), running.CurrentPose()));
    }
    float worst = 0.0f;
    for (size_t k = 1; k < shown.size(); ++k) {
        for (int j = 1; j < kJointCount; ++j) {
            worst = std::max(worst, DegreesBetween(shown[k - 1].rotation[j], shown[k].rotation[j]) / dt);
        }
    }
    Check(worst < 1000.0f, "a cross fade between a walk and a run has no pops: " + std::to_string(worst) + " degrees a second");
}

void AMixOfAllTheLayersStaysInsideTheLimitsAndContinuous() {
    // Walking, pushed, looking about, with secondary motion: the whole stack.
    const float dt = 1.0f / 60.0f;
    const BodyModel body = MakeBodyModel(Person(0.0f, 40, 1.65f, 60, 0.3f));
    Walker walker(body, PersonalityFrom(body.parameters));
    walker.Reset(glm::vec3(0.0f), 0.0f);
    HitReaction hit;
    SecondaryMotion secondary;
    glm::vec3 previousPosition = walker.Position();
    glm::vec3 previousVelocity(0.0f);
    Pose previous = walker.CurrentPose();
    float worst = 0.0f;
    for (int i = 0; i < 60 * 10; ++i) {
        if (i == 60 * 6) {
            hit.Push(glm::vec3(-2.0f, 0.0f, 0.5f), walker.Heading());
        }
        const glm::vec3 velocity = (walker.Position() - previousPosition) / dt;
        const glm::vec3 acceleration = (velocity - previousVelocity) / dt;
        previousPosition = walker.Position();
        previousVelocity = velocity;
        walker.SetLayer(CombinedLayers(hit.Update(dt), secondary.Update(dt, acceleration, walker.Heading())));
        const float speed = i < 60 * 2 ? 0.0f : (i < 60 * 5 ? 1.5f : 2.6f);
        walker.Update(dt, LocomotionCommand{speed, i > 60 * 7 ? 0.5f : 0.0f});
        Pose pose = walker.CurrentPose();
        const glm::vec3 target = walker.Position() + glm::vec3(std::sin(i * 0.02f) * 3.0f, 1.6f, 4.0f);
        LookAt(body, pose, target, 0.8f);
        for (int j = 1; j < kJointCount; ++j) {
            const Joint joint = static_cast<Joint>(j);
            Check(LimitExcessDegrees(body.skeleton, joint, pose.rotation[j]) < 0.5f, std::string("a joint leaves its range in the full stack: ") + body.skeleton.joints[j].name);
            worst = std::max(worst, DegreesBetween(previous.rotation[j], pose.rotation[j]) / dt);
        }
        previous = pose;
    }
    Check(worst < 1000.0f, "the whole stack has no pops: " + std::to_string(worst));
}

// ---------------------------------------------------------------------------
// Bad input, long frames and long runs
// ---------------------------------------------------------------------------

bool PoseIsFinite(const Pose& pose) {
    bool finite = std::isfinite(pose.rootTranslation.x + pose.rootTranslation.y + pose.rootTranslation.z);
    for (const glm::quat& q : pose.rotation) {
        finite = finite && std::isfinite(q.x + q.y + q.z + q.w) && std::abs(glm::length(q) - 1.0f) < 1e-3f;
    }
    return finite;
}

void NumbersThatAreNotNumbersLeaveLaterFramesFinite() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const BodyModel body = MakeBodyModel(Person(0.5f, 30, 1.75f, 70, 0.4f));

    // The walker: a time step, a speed and a turn that are not numbers.
    Walker walker(body, PersonalityFrom(body.parameters));
    walker.Reset(glm::vec3(0.0f), 0.0f);
    for (int i = 0; i < 120; ++i) {
        walker.Update(1.0f / 60.0f, LocomotionCommand{1.4f, 0.0f});
    }
    const float badValues[] = {nan, inf, -inf};
    for (const float bad : badValues) {
        walker.Update(bad, LocomotionCommand{1.4f, 0.0f});
        walker.Update(1.0f / 60.0f, LocomotionCommand{bad, 0.0f});
        walker.Update(1.0f / 60.0f, LocomotionCommand{1.4f, bad});
        walker.Update(1.0f / 60.0f, LocomotionCommand{bad, bad});
        Check(PoseIsFinite(walker.CurrentPose()), "the walker's pose is finite after a bad input");
        BodyLayer poisoned;
        poisoned.pelvisOffset = glm::vec3(bad);
        poisoned.additive.rotation[3] = glm::quat(bad, bad, bad, bad);
        walker.SetLayer(poisoned);
    }
    for (int i = 0; i < 120; ++i) {
        walker.Update(1.0f / 60.0f, LocomotionCommand{1.4f, 0.0f});
        Check(PoseIsFinite(walker.CurrentPose()), "later frames are finite");
        Check(std::isfinite(walker.Position().x + walker.Position().z + walker.Heading() + walker.Speed()), "the walker's place is finite");
    }
    const std::vector<Frame> after = {RecordFrame(walker)};
    Check(after[0].foot[0].heel.y > -0.01f && after[0].foot[1].heel.y > -0.01f, "the feet are not under the ground after the bad frames");

    // Inverse kinematics: a target that is not a number, and a target on the hip itself.
    Pose pose;
    const Limb leg = LegLimb(true);
    SolveTwoBone(body.skeleton, pose, leg, glm::vec3(nan, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    SolveTwoBone(body.skeleton, pose, leg, glm::vec3(0.0f, 0.5f, 0.0f), glm::vec3(inf, 0.0f, 1.0f));
    Check(PoseIsFinite(pose), "inverse kinematics leaves the pose alone for a target that is not a number");
    SolveTwoBone(body.skeleton, pose, leg, body.skeleton[leg.upper].restPosition, glm::vec3(0.0f, 0.0f, 1.0f));
    Check(PoseIsFinite(pose), "a target on the hip itself gives a finite pose");
    const WorldPose folded = ForwardKinematics(body.skeleton, pose);
    Check(std::isfinite(folded.position[character::Index(leg.end)].x), "and a finite ankle");
    SolveTwoBone(body.skeleton, pose, leg, body.skeleton[leg.upper].restPosition + glm::vec3(0, -0.5f, 0), glm::vec3(0.0f));
    Check(PoseIsFinite(pose), "a pole of nothing gives a finite pose");

    // Looking, springs, hits and blends.
    Pose standing = StandingPose(body);
    LookAt(body, standing, glm::vec3(nan, 1.0f, 1.0f));
    LookAt(body, standing, glm::vec3(1.0f, 1.0f, 1.0f), nan);
    Check(PoseIsFinite(standing), "a look at nothing leaves a finite pose");
    SecondaryMotion secondary;
    HitReaction hit;
    for (const float bad : badValues) {
        secondary.Update(bad, glm::vec3(1.0f), 0.0f);
        secondary.Update(0.01f, glm::vec3(bad), bad);
        hit.Push(glm::vec3(bad, 0.0f, 1.0f), 0.0f);
        hit.Push(glm::vec3(1.0f, 0.0f, 0.0f), bad);
    }
    const BodyLayer afterBad = CombinedLayers(secondary.Update(0.01f, glm::vec3(1.0f, 0.0f, 0.0f), 0.0f), hit.Update(0.01f));
    Check(std::isfinite(afterBad.pelvisOffset.x + afterBad.pelvisOffset.y + afterBad.pelvisOffset.z), "a spring is not poisoned by bad numbers");
    for (const glm::quat& q : afterBad.additive.rotation) {
        Check(std::isfinite(q.x + q.y + q.z + q.w), "a layer is finite after bad numbers");
    }
    const Pose a = StandingPose(body);
    const Pose b = standing;
    for (const float bad : badValues) {
        Check(PoseIsFinite(BlendPoses(a, b, bad)), "a blend with a weight that is not a number is finite");
        const WeightedPose entries[] = {{&a, bad}, {&b, 1.0f}};
        Check(PoseIsFinite(BlendWeighted(entries)), "a weighted blend with a bad weight is finite");
        Check(PoseIsFinite(ApplyAdditive(a, b, bad)), "an additive layer with a bad weight is finite");
    }
    CrossFade fade;
    fade.Begin(nan);
    fade.Advance(nan);
    Check(std::isfinite(fade.Weight()), "a cross fade with bad times has a finite weight");
}

void ALongFrameIsTakenInSmallSteps() {
    // A frame of a quarter of a second, or of ten seconds, plants and lifts the feet as shorter frames would.
    const BodyModel body = MakeBodyModel(Person(0.5f, 30, 1.75f, 70, 0.4f));
    for (const float frame : {0.1f, 0.25f, 0.5f, 10.0f}) {
        Walker walker(body, PersonalityFrom(body.parameters));
        walker.Reset(glm::vec3(0.0f), 0.0f);
        std::vector<Frame> frames;
        for (int i = 0; i < 40; ++i) {
            walker.Update(frame, LocomotionCommand{1.4f, 0.0f});
            frames.push_back(RecordFrame(walker));
        }
        for (const Frame& f : frames) {
            Check(PoseIsFinite(f.pose), "a pose after a long frame is finite");
            for (const FootSample& foot : f.foot) {
                Check(foot.heel.y > -0.008f && foot.ball.y > -0.008f, "a foot is not under the ground after a frame of " + std::to_string(frame) + " seconds");
                Check(foot.legReaches, "the leg reaches the foot after a frame of " + std::to_string(frame) + " seconds");
            }
        }
    }
}

void LongRunsKeepCounting() {
    // A float of seconds stops counting when its step is smaller than its last digit, at about a million seconds; the time of
    // the walker and the fidgets of an idle person go on.
    const BodyModel body = MakeBodyModel(Person(0.5f, 30, 1.75f, 70, 0.4f));
    Walker walker(body, PersonalityFrom(body.parameters));
    walker.Reset(glm::vec3(0.0f), 0.0f);
    for (int i = 0; i < 3; ++i) {
        walker.Update(0.5f, LocomotionCommand{});
    }
    Check(std::abs(walker.Time() - 1.5) < 1e-4, "the time of a walker adds up");
    IdleMotion idle(3);
    for (const double start : {1.0e6, 1.0e7, 1.0e9}) {
        const BodyLayer first = idle.Evaluate(start);
        const BodyLayer later = idle.Evaluate(start + 0.3);
        float change = 0.0f;
        for (int j = 0; j < kJointCount; ++j) {
            Check(std::isfinite(first.additive.rotation[j].w), "idle motion is finite at a very late time");
            change += DegreesBetween(first.additive.rotation[j], later.additive.rotation[j]);
        }
        Check(change > 0.01f, "idle motion still moves at a very late time");
        const BodyLayer close = idle.Evaluate(start + 0.001);
        float jump = 0.0f;
        for (int j = 0; j < kJointCount; ++j) {
            jump = std::max(jump, DegreesBetween(first.additive.rotation[j], close.additive.rotation[j]));
        }
        Check(jump < 0.1f, "idle motion is smooth at a very late time: " + std::to_string(jump));
    }
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
        {"SteepGroundIsFollowedAsWellAsTheLegsAllow", SteepGroundIsFollowedAsWellAsTheLegsAllow},
        {"ABodyTurnsAndLeansIntoTheTurn", ABodyTurnsAndLeansIntoTheTurn},
        {"EveryTurnRateIsFollowedOrLimitedWithoutBreakingTheLegs", EveryTurnRateIsFollowedOrLimitedWithoutBreakingTheLegs},
        {"StoppingAndTurningOnTheSpotDoNotDragTheFeet", StoppingAndTurningOnTheSpotDoNotDragTheFeet},
        {"AWalkerStartsAndStopsGently", AWalkerStartsAndStopsGently},
        {"EveryChangeOfGaitIsContinuous", EveryChangeOfGaitIsContinuous},
        {"AStepOfAFrameShrinksWithTheFrame", AStepOfAFrameShrinksWithTheFrame},
        {"TheSameScriptGivesTheSameMotionEveryTime", TheSameScriptGivesTheSameMotionEveryTime},
        {"TheGaitWorksForEveryBodyFromTheSmallestToTheLargest", TheGaitWorksForEveryBodyFromTheSmallestToTheLargest},
        {"PersonalityFollowsFromTheBodyAndTheMood", PersonalityFollowsFromTheBodyAndTheMood},
        {"TheArmsSwingAgainstTheLegsAndTheShouldersAgainstTheHips", TheArmsSwingAgainstTheLegsAndTheShouldersAgainstTheHips},
        {"TheFeetPointTheWayTheyGoAndTheHeadStaysLevel", TheFeetPointTheWayTheyGoAndTheHeadStaysLevel},
        {"NoiseIsDeterministicSmoothAndBounded", NoiseIsDeterministicSmoothAndBounded},
        {"StandingStillBreathesAndShiftsWeightWithoutMovingTheFeet", StandingStillBreathesAndShiftsWeightWithoutMovingTheFeet},
        {"TheHeadAndEyesLookAtWhatIsInRange", TheHeadAndEyesLookAtWhatIsInRange},
        {"ATargetBehindIsLookedAtAsFarAsTheNeckAllows", ATargetBehindIsLookedAtAsFarAsTheNeckAllows},
        {"TheLookFadesInWithItsWeight", TheLookFadesInWithItsWeight},
        {"TheEyesConvergeOnANearTarget", TheEyesConvergeOnANearTarget},
        {"ACriticallyDampedSpringFollowsTheExactSolutionAtAnyStepSize", ACriticallyDampedSpringFollowsTheExactSolutionAtAnyStepSize},
        {"ACriticallyDampedSpringNeverOvershoots", ACriticallyDampedSpringNeverOvershoots},
        {"TheChestAndHeadLagBehindAcceleration", TheChestAndHeadLagBehindAcceleration},
        {"APushMakesTheBodyGiveAndComeBack", APushMakesTheBodyGiveAndComeBack},
        {"ABodyWalksOnWhileItIsPushed", ABodyWalksOnWhileItIsPushed},
        {"ASittingPersonKeepsTheirFeetDownAndEndsOnTheSeat", ASittingPersonKeepsTheirFeetDownAndEndsOnTheSeat},
        {"StandingUpIsSittingDownBackwards", StandingUpIsSittingDownBackwards},
        {"BlendingTwoPosesGoesFromOneToTheOther", BlendingTwoPosesGoesFromOneToTheOther},
        {"WeightedBlendsFollowTheirWeights", WeightedBlendsFollowTheirWeights},
        {"AdditiveLayersApplyOnTopOfAPose", AdditiveLayersApplyOnTopOfAPose},
        {"ACrossFadeIsSmoothAndHasNoPops", ACrossFadeIsSmoothAndHasNoPops},
        {"AMixOfAllTheLayersStaysInsideTheLimitsAndContinuous", AMixOfAllTheLayersStaysInsideTheLimitsAndContinuous},
        {"NumbersThatAreNotNumbersLeaveLaterFramesFinite", NumbersThatAreNotNumbersLeaveLaterFramesFinite},
        {"ALongFrameIsTakenInSmallSteps", ALongFrameIsTakenInSmallSteps},
        {"LongRunsKeepCounting", LongRunsKeepCounting},
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
    for (const auto& [kind, worst] : WorstByKind()) {
        std::cout << "worst " << kind << ": slide " << worst.slip << " m/s, heel or ball below the floor " << -worst.belowGround * 1000.0f << " mm, toe tip "
                  << -worst.toeBelowGround * 1000.0f << " mm, planted foot above the floor " << worst.plantedHeight * 1000.0f << " mm, lowest knee " << worst.lowestKnee << " deg\n";
    }
    std::cout << ran << " tests, " << failureCount << " failed checks\n";
    return failureCount == 0 ? 0 : 1;
}
