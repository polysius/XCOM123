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
#include <iostream>
#include <string>
#include <vector>

#include "animation/JointLimits.h"
#include "animation/Kinematics.h"
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
