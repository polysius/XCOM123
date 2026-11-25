// Tests of the character generator (no window, no GPU). Like the other test programs there is no test framework: each
// test is a function that calls Check, and main reports every failure it finds.
//
// What a generated body must be, and what these tests hold the generator to: the same every time, one closed surface
// without holes or loops and with its triangles facing outwards, as tall and as heavy as it was asked to be, with the
// proportions of a human skeleton, the same on the left as on the right, skinned so that the rest pose is the mesh and
// bending a joint keeps the body's volume, and made from parameters that cannot break it.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <string>
#include <vector>

#include "character/Anthropometry.h"
#include "character/BodyShape.h"
#include "character/DetailShapes.h"
#include "character/CharacterGenerator.h"
#include "character/CharacterParameters.h"
#include "character/Skeleton.h"
#include "core/JobSystem.h"

namespace {

using namespace character;

int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

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

// A spread of people: a tall heavy man, a short light woman, an old man, a young muscular woman, and two in between.
std::vector<CharacterParameters> SomePeople() {
    return {Person(1.0f, 30.0f, 1.90f, 100.0f, 0.8f, 1), Person(0.0f, 25.0f, 1.55f, 48.0f, 0.2f, 2), Person(1.0f, 72.0f, 1.70f, 82.0f, 0.2f, 3),
            Person(0.0f, 28.0f, 1.72f, 66.0f, 0.9f, 4),    Person(0.5f, 45.0f, 1.66f, 70.0f, 0.4f, 5), Person(0.9f, 50.0f, 1.78f, 105.0f, 0.3f, 6)};
}

const CharacterMesh& CachedMesh(size_t personIndex) {
    static std::vector<CharacterMesh> cache;
    if (cache.empty()) {
        for (const CharacterParameters& person : SomePeople()) {
            cache.push_back(GenerateCharacter(person));
        }
    }
    return cache[personIndex];
}

constexpr size_t kPeopleCount = 6;

std::string Describe(size_t personIndex) {
    return "person " + std::to_string(personIndex);
}

// Mesh checks -----------------------------------------------------------------------------------

// Whether a point is inside a closed mesh, by counting how many triangles a ray along +X crosses. With a piece given,
// only that piece counts; without, the whole character does, where the overlap of two pieces would confuse the count.
bool PointInside(const CharacterMesh& mesh, glm::vec3 origin, int piece = -1) {
    int crossings = 0;
    const size_t first = piece < 0 ? 0 : mesh.pieces[piece].firstIndex;
    const size_t last = piece < 0 ? mesh.indices.size() : first + mesh.pieces[piece].indexCount;
    for (size_t t = first; t + 2 < last; t += 3) {
        const glm::vec3 a = mesh.positions[mesh.indices[t]];
        const glm::vec3 b = mesh.positions[mesh.indices[t + 1]];
        const glm::vec3 c = mesh.positions[mesh.indices[t + 2]];
        const glm::vec3 edge1 = b - a;
        const glm::vec3 edge2 = c - a;
        const glm::vec3 direction(1.0f, 0.0f, 0.0f);
        const glm::vec3 h = glm::cross(direction, edge2);
        const float det = glm::dot(edge1, h);
        if (std::abs(det) < 1e-12f) {
            continue;
        }
        const float f = 1.0f / det;
        const glm::vec3 s = origin - a;
        const float u = f * glm::dot(s, h);
        if (u < 0.0f || u > 1.0f) {
            continue;
        }
        const glm::vec3 q = glm::cross(s, edge1);
        const float v = f * glm::dot(direction, q);
        if (v < 0.0f || u + v > 1.0f) {
            continue;
        }
        if (f * glm::dot(edge2, q) > 0.0f) {
            ++crossings;
        }
    }
    return crossings % 2 == 1;
}

// Tests ------------------------------------------------------------------------------------------

void TheSameParametersGiveTheSameBodyOnAnyNumberOfThreads() {
    const CharacterParameters person = Person(0.7f, 38.0f, 1.74f, 76.0f, 0.5f, 9);
    const CharacterMesh first = GenerateCharacter(person);
    const CharacterMesh second = GenerateCharacter(person);
    JobSystem many(5);
    const CharacterMesh threaded = GenerateCharacter(person, {}, &many);
    Check(first.positions == second.positions && first.indices == second.indices, "the same parameters give the same mesh");
    Check(first.positions == threaded.positions && first.indices == threaded.indices && first.boneWeights == threaded.boneWeights,
          "on one thread and on six, to the last bit");
    const CharacterMesh other = GenerateCharacter(Person(0.7f, 38.0f, 1.74f, 76.0f, 0.5f, 10));
    Check(other.positions != first.positions || other.skeleton[Joint::Pelvis].restPosition != first.skeleton[Joint::Pelvis].restPosition,
          "and another seed gives another body");
}

void EveryPieceIsOneClosedManifoldWithoutHoles() {
    for (size_t i = 0; i < kPeopleCount; ++i) {
        const CharacterMesh& mesh = CachedMesh(i);
        Check(mesh.pieces.size() == 4, Describe(i) + ": a body, a head and two hands: " + std::to_string(mesh.pieces.size()) + " pieces");
        for (size_t piece = 0; piece < mesh.pieces.size(); ++piece) {
            const MeshTopology topology = MeasureTopology(mesh, piece);
            const std::string what = Describe(i) + ", piece " + std::to_string(piece);
            Check(topology.badEdges == 0, what + ": every edge is shared by exactly two triangles, " + std::to_string(topology.badEdges) + " are not");
            Check(topology.eulerCharacteristic == 2, what + ": the surface is a sphere without handles, Euler characteristic " + std::to_string(topology.eulerCharacteristic));
            Check(topology.pieces == 1, what + ": it is one connected surface, not " + std::to_string(topology.pieces));
        }
        Check(mesh.volumeCubicMeters > 0.0f, Describe(i) + ": the triangles face outwards");
    }
}

void NormalsPointOutAndAreUnitLength() {
    for (size_t i = 0; i < kPeopleCount; ++i) {
        const CharacterMesh& mesh = CachedMesh(i);
        bool unit = true;
        bool finite = true;
        for (size_t v = 0; v < mesh.positions.size(); ++v) {
            unit = unit && std::abs(glm::length(mesh.normals[v]) - 1.0f) < 1e-3f;
            finite = finite && std::isfinite(mesh.positions[v].x + mesh.positions[v].y + mesh.positions[v].z);
        }
        Check(unit && finite, Describe(i) + ": normals have length 1 and positions are numbers");
        // The triangles and the vertex normals must agree on which way is out.
        double agreement = 0.0;
        size_t triangles = 0;
        for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            const glm::vec3 face = glm::cross(mesh.positions[mesh.indices[t + 1]] - mesh.positions[mesh.indices[t]],
                                              mesh.positions[mesh.indices[t + 2]] - mesh.positions[mesh.indices[t]]);
            if (glm::length(face) > 1e-12f) {
                const glm::vec3 average = mesh.normals[mesh.indices[t]] + mesh.normals[mesh.indices[t + 1]] + mesh.normals[mesh.indices[t + 2]];
                agreement += glm::dot(glm::normalize(face), glm::normalize(average)) > 0.0f ? 1.0 : 0.0;
                ++triangles;
            }
        }
        Check(agreement / static_cast<double>(triangles) > 0.995, Describe(i) + ": triangles and normals face the same way");
    }
}

void TheBodyIsAsTallAndAsHeavyAsAskedAndStandsOnTheFloor() {
    for (size_t i = 0; i < kPeopleCount; ++i) {
        const CharacterMesh& mesh = CachedMesh(i);
        float lowest = 1e9f;
        float highest = -1e9f;
        for (const glm::vec3& p : mesh.positions) {
            lowest = std::min(lowest, p.y);
            highest = std::max(highest, p.y);
        }
        const CharacterParameters& wanted = mesh.parameters;
        Check(std::abs(highest - wanted.statureMeters) < 0.015f, Describe(i) + ": stature " + std::to_string(highest) + " m against " + std::to_string(wanted.statureMeters));
        Check(std::abs(lowest) < 0.006f, Describe(i) + ": the soles are on the floor, lowest point " + std::to_string(lowest));
        Check(std::abs(mesh.massKg / wanted.massKg - 1.0f) < 0.03f, Describe(i) + ": mass " + std::to_string(mesh.massKg) + " kg against " + std::to_string(wanted.massKg));
        Check(std::abs(MeshVolume(mesh.positions, mesh.indices) - mesh.volumeCubicMeters) < 1e-5f, Describe(i) + ": the stated volume is the volume of the mesh");
    }
}

void TheSkeletonHasTheProportionsOfTheLiterature() {
    namespace a = anthropometry;
    for (size_t i = 0; i < kPeopleCount; ++i) {
        const Skeleton& skeleton = CachedMesh(i).skeleton;
        const float height = skeleton.statureMeters;
        const auto length = [&](Joint from, Joint to) { return glm::distance(skeleton[from].restPosition, skeleton[to].restPosition) / height; };
        const auto near = [&](float value, float expected) { return std::abs(value / expected - 1.0f) < 0.05f; };
        Check(near(length(Joint::UpperArmL, Joint::ForearmL), a::kUpperArmLength), Describe(i) + ": the upper arm is 0.186 of stature: " + std::to_string(length(Joint::UpperArmL, Joint::ForearmL)));
        Check(near(length(Joint::ForearmL, Joint::HandL), a::kForearmLength), Describe(i) + ": the forearm is 0.146 of stature");
        Check(near(length(Joint::ThighL, Joint::CalfL), a::kThighLength), Describe(i) + ": the thigh is 0.245 of stature: " + std::to_string(length(Joint::ThighL, Joint::CalfL)));
        Check(near(length(Joint::CalfL, Joint::FootL), a::kShankLength), Describe(i) + ": the shank is 0.246 of stature: " + std::to_string(length(Joint::CalfL, Joint::FootL)));
        Check(near(skeleton[Joint::ThighL].restPosition.y / height, a::kHipHeight), Describe(i) + ": the hip joint is at 0.530 of stature");
        Check(near(skeleton[Joint::CalfL].restPosition.y / height, a::kKneeHeight), Describe(i) + ": the knee is at 0.285 of stature");
        Check(near(skeleton[Joint::UpperArmL].restPosition.y / height, a::kShoulderHeight), Describe(i) + ": the shoulder is at 0.818 of stature");
        Check(near(skeleton[Joint::FootL].restPosition.y / height, a::kAnkleHeight), Describe(i) + ": the ankle is at 0.039 of stature");
    }
}

void TheSkeletonIsMirroredAndWellFormed() {
    for (size_t i = 0; i < kPeopleCount; ++i) {
        const Skeleton& skeleton = CachedMesh(i).skeleton;
        const std::pair<Joint, Joint> pairs[] = {{Joint::ClavicleL, Joint::ClavicleR}, {Joint::UpperArmL, Joint::UpperArmR}, {Joint::ForearmL, Joint::ForearmR},
                                                 {Joint::HandL, Joint::HandR},         {Joint::ThighL, Joint::ThighR},       {Joint::CalfL, Joint::CalfR},
                                                 {Joint::FootL, Joint::FootR},         {Joint::ToesL, Joint::ToesR}};
        for (const auto& [left, right] : pairs) {
            const glm::vec3 l = skeleton[left].restPosition;
            const glm::vec3 r = skeleton[right].restPosition;
            Check(std::abs(l.x + r.x) < 1e-5f && std::abs(l.y - r.y) < 1e-5f && std::abs(l.z - r.z) < 1e-5f, Describe(i) + ": " + skeleton[left].name + " mirrors " + skeleton[right].name);
        }
        for (int j = 0; j < kJointCount; ++j) {
            const JointInfo& joint = skeleton.joints[j];
            Check(joint.parent < j, Describe(i) + ": the parent of " + joint.name + " comes before it");
            Check(joint.minAngleDegrees.x <= 0.0f && joint.maxAngleDegrees.x >= 0.0f && joint.minAngleDegrees.y <= 0.0f && joint.maxAngleDegrees.y >= 0.0f &&
                      joint.minAngleDegrees.z <= 0.0f && joint.maxAngleDegrees.z >= 0.0f,
                  Describe(i) + ": the rest pose of " + joint.name + " is inside its limits");
            const glm::vec3 expected = joint.restPosition;
            const glm::vec3 mapped = glm::vec3(joint.inverseBind * glm::vec4(expected, 1.0f));
            Check(glm::length(mapped) < 1e-5f, Describe(i) + ": the inverse bind of " + joint.name + " takes the joint to the origin");
            if (joint.parent >= 0) {
                Check(glm::distance(joint.restOffset, joint.restPosition - skeleton.joints[joint.parent].restPosition) < 1e-5f, Describe(i) + ": the offset of " + joint.name + " is from its parent");
            }
        }
        // The feet stand on the floor with the heel behind the ball and the ball behind the toe.
        for (const FootContacts* foot : {&skeleton.leftFoot, &skeleton.rightFoot}) {
            Check(foot->heel.y == 0.0f && foot->ball.y == 0.0f && foot->toe.y == 0.0f, Describe(i) + ": the contact points are on the floor");
            Check(foot->heel.z < foot->ball.z && foot->ball.z < foot->toe.z, Describe(i) + ": the heel is behind the ball and the ball behind the toe");
        }
        Check(skeleton.leftFoot.heel.x > 0.0f && skeleton.rightFoot.heel.x < 0.0f, Describe(i) + ": the left foot is on the +X side");
    }
}

void TheBodyIsTheSameOnTheLeftAsOnTheRight() {
    for (size_t i = 0; i < kPeopleCount; i += 2) {
        const CharacterMesh& mesh = CachedMesh(i);
        // Every vertex has a vertex of the mirrored mesh close by. The grid is not exactly centred, so the match is
        // within about a cell.
        float worst = 0.0f;
        for (size_t v = 0; v < mesh.positions.size(); v += 7) {
            const glm::vec3 mirrored(-mesh.positions[v].x, mesh.positions[v].y, mesh.positions[v].z);
            float nearest = 1e9f;
            for (const glm::vec3& other : mesh.positions) {
                nearest = std::min(nearest, glm::distance(mirrored, other));
            }
            worst = std::max(worst, nearest);
        }
        Check(worst < 0.03f, Describe(i) + ": the mesh mirrors itself to within " + std::to_string(worst) + " m");
    }
}

void SkinWeightsFollowTheRules() {
    for (size_t i = 0; i < kPeopleCount; ++i) {
        const CharacterMesh& mesh = CachedMesh(i);
        bool sumsToOne = true;
        bool indicesValid = true;
        bool inHierarchy = true;
        bool noNegative = true;
        for (size_t v = 0; v < mesh.positions.size(); ++v) {
            float total = 0.0f;
            for (int slot = 0; slot < 4; ++slot) {
                total += mesh.boneWeights[v][slot];
                noNegative = noNegative && mesh.boneWeights[v][slot] >= 0.0f;
                indicesValid = indicesValid && mesh.boneIndices[v][slot] < kJointCount;
            }
            sumsToOne = sumsToOne && std::abs(total - 1.0f) < 1e-4f;
            // Some bone must be the one the others are neighbours of: itself, its parent or its children.
            bool found = false;
            for (int centre = 0; centre < kJointCount && !found; ++centre) {
                bool all = true;
                for (int slot = 0; slot < 4; ++slot) {
                    if (mesh.boneWeights[v][slot] <= 0.0f) {
                        continue;
                    }
                    const int bone = mesh.boneIndices[v][slot];
                    const bool neighbour = bone == centre || bone == mesh.skeleton.joints[centre].parent || mesh.skeleton.joints[bone].parent == centre;
                    all = all && neighbour;
                }
                found = all;
            }
            inHierarchy = inHierarchy && found;
        }
        Check(sumsToOne && noNegative, Describe(i) + ": weights are not negative and add up to 1");
        Check(indicesValid, Describe(i) + ": every bone index names a bone");
        Check(inHierarchy, Describe(i) + ": the bones of a vertex are a bone with its parent and its children");
    }
    // The upper arm vertices are not pulled by the torso: a vertex at the middle of the upper arm follows only arm bones.
    const CharacterMesh& mesh = CachedMesh(4);
    const glm::vec3 middle = 0.5f * (mesh.skeleton[Joint::UpperArmL].restPosition + mesh.skeleton[Joint::ForearmL].restPosition);
    size_t nearest = 0;
    for (size_t v = 0; v < mesh.positions.size(); ++v) {
        if (glm::distance(mesh.positions[v], middle) < glm::distance(mesh.positions[nearest], middle)) {
            nearest = v;
        }
    }
    for (int slot = 0; slot < 4; ++slot) {
        const int bone = mesh.boneIndices[nearest][slot];
        Check(mesh.boneWeights[nearest][slot] <= 0.0f || bone == Index(Joint::UpperArmL) || bone == Index(Joint::ForearmL) || bone == Index(Joint::ClavicleL),
              "the middle of the upper arm follows only the arm and the shoulder");
    }
}

void SkinningInTheRestPoseGivesBackTheMesh() {
    for (size_t i = 0; i < kPeopleCount; ++i) {
        const CharacterMesh& mesh = CachedMesh(i);
        const std::vector<glm::vec3> skinned = SkinPositions(mesh, Pose());
        float worst = 0.0f;
        for (size_t v = 0; v < skinned.size(); ++v) {
            worst = std::max(worst, glm::distance(skinned[v], mesh.positions[v]));
        }
        Check(worst < 1e-4f, Describe(i) + ": the rest pose moves a vertex by " + std::to_string(worst));
    }
}

// Both elbows bent by `elbowDegrees` and twisted by `twistDegrees`, and both knees bent by `kneeDegrees`.
Pose BentPose(const Skeleton& skeleton, float elbowDegrees, float kneeDegrees, float twistDegrees) {
    Pose pose;
    pose.rotation[Index(Joint::ForearmL)] = HingeRotation(skeleton, Joint::ForearmL, elbowDegrees) * FromDegrees(0.0f, twistDegrees, 0.0f);
    pose.rotation[Index(Joint::ForearmR)] = HingeRotation(skeleton, Joint::ForearmR, elbowDegrees) * FromDegrees(0.0f, -twistDegrees, 0.0f);
    pose.rotation[Index(Joint::CalfL)] = HingeRotation(skeleton, Joint::CalfL, kneeDegrees);
    pose.rotation[Index(Joint::CalfR)] = HingeRotation(skeleton, Joint::CalfR, kneeDegrees);
    return pose;
}

void BendingJointsKeepsTheVolumeOfTheBody() {
    // Linear blend skinning loses volume at a bent joint (the candy wrapper), and this is where it must stay small.
    for (size_t i = 0; i < kPeopleCount; ++i) {
        const CharacterMesh& mesh = CachedMesh(i);
        const float rest = mesh.volumeCubicMeters;
        const float bent = MeshVolume(SkinPositions(mesh, BentPose(mesh.skeleton, 90.0f, 90.0f, 0.0f)), mesh.indices);
        Check(std::abs(bent / rest - 1.0f) < 0.06f, Describe(i) + ": elbows and knees at 90 degrees change the volume by " + std::to_string((bent / rest - 1.0f) * 100.0f) + "%");
        const float twisted = MeshVolume(SkinPositions(mesh, BentPose(mesh.skeleton, 0.0f, 0.0f, 80.0f)), mesh.indices);
        Check(std::abs(twisted / rest - 1.0f) < 0.03f, Describe(i) + ": twisting the forearms by 80 degrees changes the volume by " + std::to_string((twisted / rest - 1.0f) * 100.0f) + "%");
    }
}

void TheLegsStayApartAndTheArmsStayOffTheBody() {
    // A smooth union of the whole body would fuse the thighs and the arms into the ribs. The point between the knees
    // and the point between an arm and the side of the body must be outside.
    for (size_t i = 0; i < kPeopleCount; i += 2) {
        const CharacterMesh& mesh = CachedMesh(i);
        const float height = mesh.skeleton.statureMeters;
        const float kneeHeight = mesh.skeleton[Joint::CalfL].restPosition.y;
        const float hipX = mesh.skeleton[Joint::ThighL].restPosition.x;
        const float midThigh = 0.5f * (kneeHeight + mesh.skeleton[Joint::ThighL].restPosition.y);
        const float midCalf = 0.5f * (kneeHeight + mesh.skeleton[Joint::FootL].restPosition.y);
        // Thighs may touch on the inside high up, as they do on many people, so the air is looked for lower down.
        Check(PointInside(mesh, {hipX, midThigh, 0.0f}), Describe(i) + ": a thigh is where the hip joint says");
        Check(!PointInside(mesh, {0.0f, kneeHeight, 0.0f}), Describe(i) + ": there is air between the knees");
        Check(!PointInside(mesh, {0.0f, midCalf, 0.0f}), Describe(i) + ": and between the calves");
        Check(PointInside(mesh, {hipX, midCalf, 0.0f}), Describe(i) + ": and a calf is where the leg line says");
        const glm::vec3 wrist = mesh.skeleton[Joint::HandL].restPosition;
        Check(!PointInside(mesh, {0.5f * (wrist.x + 0.2f * height), wrist.y, 0.0f}), Describe(i) + ": there is air between the hand and the hip");
        Check(PointInside(mesh, {0.0f, 0.72f * height, 0.0f}), Describe(i) + ": and the chest is solid");
    }
}

void ParametersThatAreNotNumbersOrOutOfRangeAreCleanedUp() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    CharacterParameters broken;
    broken.masculinity = nan;
    broken.ageYears = inf;
    broken.statureMeters = -3.0f;
    broken.massKg = nan;
    broken.muscularity = 9.0f;
    const CharacterParameters clean = Sanitised(broken);
    Check(clean.masculinity >= 0.0f && clean.masculinity <= 1.0f && std::isfinite(clean.masculinity), "masculinity that is not a number is replaced");
    Check(clean.ageYears >= 18.0f && clean.ageYears <= 80.0f, "an age of infinity is brought into range");
    Check(clean.statureMeters >= 1.45f && clean.statureMeters <= 2.05f, "a negative stature is brought into range");
    Check(std::isfinite(clean.massKg) && BodyMassIndex(clean) >= 14.99f && BodyMassIndex(clean) <= 45.01f, "a mass that is not a number gives a body mass index in range");
    Check(clean.muscularity == 1.0f, "muscularity above 1 is limited to 1");
    const CharacterMesh mesh = GenerateCharacter(broken);
    bool finite = !mesh.positions.empty();
    for (const glm::vec3& p : mesh.positions) {
        finite = finite && std::isfinite(p.x + p.y + p.z);
    }
    Check(finite && mesh.volumeCubicMeters > 0.0f, "a character made from broken parameters is a normal mesh");

    // A mass far from what the stature allows is held to the body mass index limits.
    CharacterParameters heavy = Person(0.5f, 40.0f, 1.60f, 400.0f, 0.5f);
    CharacterParameters light = Person(0.5f, 40.0f, 1.60f, 10.0f, 0.5f);
    Check(BodyMassIndex(Sanitised(heavy)) <= 45.01f && BodyMassIndex(Sanitised(light)) >= 14.99f, "masses are held to a body mass index of 15 to 45");
}

void RandomPeopleFollowThePopulation() {
    double maleStatureSum = 0.0;
    double maleStatureSquares = 0.0;
    double femaleStatureSum = 0.0;
    double bodyMassIndexSum = 0.0;
    int males = 0;
    int females = 0;
    bool inRange = true;
    for (uint32_t seed = 0; seed < 4000; ++seed) {
        const CharacterParameters person = RandomParameters(seed);
        inRange = inRange && person.ageYears >= 18.0f && person.ageYears <= 80.0f && person.statureMeters >= 1.45f && person.statureMeters <= 2.05f &&
                  BodyMassIndex(person) >= 14.99f && BodyMassIndex(person) <= 45.01f && person.muscularity >= 0.0f && person.muscularity <= 1.0f;
        bodyMassIndexSum += BodyMassIndex(person);
        if (person.masculinity > 0.5f) {
            ++males;
            maleStatureSum += person.statureMeters;
            maleStatureSquares += static_cast<double>(person.statureMeters) * person.statureMeters;
        } else {
            ++females;
            femaleStatureSum += person.statureMeters;
        }
    }
    Check(inRange, "every random person has parameters inside the ranges");
    Check(males > 1700 && females > 1700, "about half are men and half women: " + std::to_string(males) + " and " + std::to_string(females));
    const double maleMean = maleStatureSum / males;
    const double maleDeviation = std::sqrt(maleStatureSquares / males - maleMean * maleMean);
    Check(std::abs(maleMean - 1.756) < 0.01, "men are 1.756 m tall on average: " + std::to_string(maleMean));
    Check(std::abs(femaleStatureSum / females - 1.629) < 0.01, "and women 1.629 m: " + std::to_string(femaleStatureSum / females));
    Check(std::abs(maleDeviation - 0.069) < 0.008, "with the spread of the survey, 0.069 m: " + std::to_string(maleDeviation));
    Check(bodyMassIndexSum / 4000.0 > 24.5 && bodyMassIndexSum / 4000.0 < 28.5, "the body mass index is about 26: " + std::to_string(bodyMassIndexSum / 4000.0));
    Check(RandomParameters(77).statureMeters == RandomParameters(77).statureMeters && RandomParameters(77).massKg == RandomParameters(77).massKg, "a seed always gives the same person");
    Check(RandomParameters(77).statureMeters != RandomParameters(78).statureMeters, "and another seed another person");
}

void ThePopulationLooksRight() {
    // Men are broader at the shoulder and women at the hip, and a heavier person is rounder, at the same stature.
    const CharacterMesh man = GenerateCharacter(Person(1.0f, 30.0f, 1.75f, 75.0f, 0.5f, 3));
    const CharacterMesh woman = GenerateCharacter(Person(0.0f, 30.0f, 1.75f, 75.0f, 0.5f, 3));
    // The width of the body in a thin slice at a fraction of stature, within `reach` of the middle.
    const auto widthAt = [](const CharacterMesh& mesh, float heightFraction, float reach) {
        float low = 1e9f;
        float high = -1e9f;
        const float y = heightFraction * mesh.skeleton.statureMeters;
        for (const glm::vec3& p : mesh.positions) {
            if (std::abs(p.y - y) < 0.012f && std::abs(p.x) < reach) {
                low = std::min(low, p.x);
                high = std::max(high, p.x);
            }
        }
        return high - low;
    };
    const float manShoulders = widthAt(man, 0.80f, 0.50f);
    const float womanShoulders = widthAt(woman, 0.80f, 0.50f);
    Check(manShoulders > womanShoulders * 1.03f, "at the same mass a man is broader across the shoulders: " + std::to_string(manShoulders) + " against " + std::to_string(womanShoulders));
    // The arms hang well clear of the hips, so a slice there is the hips alone.
    const float manHips = widthAt(man, 0.50f, 0.30f);
    const float womanHips = widthAt(woman, 0.50f, 0.30f);
    Check(womanHips > manHips * 0.98f, "and a woman at least as broad across the hips: " + std::to_string(womanHips) + " against " + std::to_string(manHips));
    const CharacterMesh light = GenerateCharacter(Person(0.5f, 30.0f, 1.70f, 55.0f, 0.3f, 4));
    const CharacterMesh heavy = GenerateCharacter(Person(0.5f, 30.0f, 1.70f, 100.0f, 0.3f, 4));
    Check(widthAt(heavy, 0.60f, 0.30f) > widthAt(light, 0.60f, 0.30f) + 0.04f, "a heavier person is broader at the waist");
}

void EveryRandomPersonIsAClosedBodyOfTheRightSize() {
    // A pair of limbs that touch, or a muscle that closes an armpit, would make a handle in the surface. People from
    // the whole spread of the population must all come out as one closed sphere, as heavy and as tall as drawn.
    JobSystem jobs(3);
    std::vector<uint32_t> seeds;
    for (uint32_t seed = 0; seed < 60; ++seed) {
        seeds.push_back(seed);
    }
    // These two once came out with a tunnel between the thighs, where the gap was thinner than a cell of the grid.
    seeds.insert(seeds.end(), {184u, 243u});
    for (const uint32_t seed : seeds) {
        const CharacterMesh mesh = GenerateCharacter(RandomParameters(seed), {}, &jobs);
        const std::string who = "random person " + std::to_string(seed);
        for (size_t piece = 0; piece < mesh.pieces.size(); ++piece) {
            const MeshTopology topology = MeasureTopology(mesh, piece);
            Check(topology.IsClosedSphere(), who + ", piece " + std::to_string(piece) + " is one closed sphere: " + std::to_string(topology.badEdges) +
                                                 " bad edges, Euler characteristic " + std::to_string(topology.eulerCharacteristic) + ", " + std::to_string(topology.pieces) + " pieces");
        }
        Check(std::abs(mesh.massKg / mesh.parameters.massKg - 1.0f) < 0.04f, who + " has the mass asked for: " + std::to_string(mesh.massKg) + " against " + std::to_string(mesh.parameters.massKg));
    }
}

void AnElbowBendsAboutItsHingeAndAHipOpensTheRightWay() {
    for (size_t i = 0; i < kPeopleCount; i += 2) {
        const Skeleton& skeleton = CachedMesh(i).skeleton;
        // Bent by 90 degrees about its hinge, the forearm makes a right angle with the upper arm.
        for (const Joint elbow : {Joint::ForearmL, Joint::ForearmR}) {
            Pose pose;
            pose.rotation[Index(elbow)] = HingeRotation(skeleton, elbow, 90.0f);
            const std::array<glm::vec3, kJointCount> joints = JointPositions(skeleton, pose);
            const Joint shoulder = elbow == Joint::ForearmL ? Joint::UpperArmL : Joint::UpperArmR;
            const Joint wrist = elbow == Joint::ForearmL ? Joint::HandL : Joint::HandR;
            const glm::vec3 upperArm = glm::normalize(joints[Index(shoulder)] - joints[Index(elbow)]);
            const glm::vec3 forearm = glm::normalize(joints[Index(wrist)] - joints[Index(elbow)]);
            const float angle = glm::degrees(std::acos(glm::clamp(glm::dot(upperArm, forearm), -1.0f, 1.0f)));
            Check(std::abs(angle - 90.0f) < 1.0f, Describe(i) + ": an elbow bent 90 degrees makes an angle of " + std::to_string(angle));
            Check(joints[Index(wrist)].z > joints[Index(elbow)].z, Describe(i) + ": and brings the hand forwards");
        }
        // A knee bends the shank backwards.
        Pose knee;
        knee.rotation[Index(Joint::CalfL)] = HingeRotation(skeleton, Joint::CalfL, 90.0f);
        const std::array<glm::vec3, kJointCount> legJoints = JointPositions(skeleton, knee);
        Check(legJoints[Index(Joint::FootL)].z < legJoints[Index(Joint::CalfL)].z - 0.1f, Describe(i) + ": a bent knee swings the foot backwards");
        // A hip opens further outwards than it closes across the body, on both sides.
        for (const bool left : {true, false}) {
            const Joint hip = left ? Joint::ThighL : Joint::ThighR;
            const float outward = left ? 1.0f : -1.0f;  // which way along X is out for this leg
            const JointInfo& info = skeleton[hip];
            const float outLimit = left ? info.maxAngleDegrees.z : -info.minAngleDegrees.z;
            const float acrossLimit = left ? -info.minAngleDegrees.z : info.maxAngleDegrees.z;
            Check(outLimit > acrossLimit, Describe(i) + ": the hip opens more than it closes");
            Pose pose;
            pose.rotation[Index(hip)] = FromDegrees(0.0f, 0.0f, outward * 20.0f);
            const std::array<glm::vec3, kJointCount> joints = JointPositions(skeleton, pose);
            const Joint knee2 = left ? Joint::CalfL : Joint::CalfR;
            Check((joints[Index(knee2)].x - skeleton[knee2].restPosition.x) * outward > 0.05f, Describe(i) + ": a turn of +20 degrees opens the leg outwards");
        }
    }
}

void FingertipsAndToesAreNeverLost() {
    namespace a = anthropometry;
    JobSystem jobs(3);
    std::vector<CharacterParameters> people;
    for (uint32_t seed = 0; seed < 50; ++seed) {
        people.push_back(RandomParameters(seed));
    }
    // The corners of the parameter space: the smallest and lightest, and the largest and heaviest, of each sex.
    for (const float masculinity : {0.0f, 1.0f}) {
        people.push_back(Person(masculinity, 80.0f, 1.45f, 31.5f, 1.0f, 3));
        people.push_back(Person(masculinity, 18.0f, 2.05f, 189.0f, 0.0f, 4));
        people.push_back(Person(masculinity, 40.0f, 1.45f, 94.0f, 0.5f, 5));
    }
    for (size_t index = 0; index < people.size(); ++index) {
        const CharacterMesh mesh = GenerateCharacter(people[index], {}, &jobs);
        const auto nearest = [&](glm::vec3 landmark) {
            float best = 1e9f;
            for (const glm::vec3& p : mesh.positions) {
                best = std::min(best, glm::distance(p, landmark));
            }
            return best;
        };
        const float height = mesh.skeleton.statureMeters;
        for (const bool left : {true, false}) {
            const glm::vec3 wrist = mesh.skeleton[left ? Joint::HandL : Joint::HandR].restPosition;
            const glm::vec3 elbow = mesh.skeleton[left ? Joint::ForearmL : Joint::ForearmR].restPosition;
            const glm::vec3 along = glm::normalize(wrist - elbow);
            const float sign = left ? 1.0f : -1.0f;
            const glm::vec3 inward = glm::normalize(sign * glm::cross(along, glm::vec3(0.0f, 0.0f, 1.0f)));
            glm::vec3 acrossHand = glm::normalize(glm::cross(inward, along));
            if (acrossHand.z < 0.0f) {
                acrossHand = -acrossHand;
            }
            // The tip of the middle finger: it runs 0.149 along the hand and has curled 0.037 towards the palm, and lies 0.011 across.
            const float handScale = height / 1.75f;
            const glm::vec3 fingertip = wrist + (along * 0.149f + inward * 0.037f + acrossHand * 0.011f) * handScale;
            (void)a::kHandLength;
            Check(nearest(fingertip) < 0.016f * handScale, "person " + std::to_string(index) + ": the " + (left ? "left" : "right") + " hand reaches its fingertips");
            const glm::vec3 ankle = mesh.skeleton[left ? Joint::FootL : Joint::FootR].restPosition;
            const glm::vec3 toe(ankle.x, 0.02f, 0.9f * a::kAnkleToToe * height);
            Check(nearest(toe) < 0.03f, "person " + std::to_string(index) + ": the " + (left ? "left" : "right") + " foot reaches its toes");
        }
    }
}

// The head and the hands --------------------------------------------------------------------------

constexpr int kHeadPiece = 1;
constexpr int kLeftHandPiece = 2;
constexpr int kRightHandPiece = 3;

// The front of the face at a height and width: the largest Z among the vertices of the head within a small box.
float FaceFront(const CharacterMesh& mesh, float x, float y, float halfWidth, float halfHeight) {
    const MeshPiece& piece = mesh.pieces[kHeadPiece];
    float front = -1e9f;
    for (uint32_t v = piece.firstVertex; v < piece.firstVertex + piece.vertexCount; ++v) {
        const glm::vec3 p = mesh.positions[v];
        if (std::abs(p.x - x) < halfWidth && std::abs(p.y - y) < halfHeight) {
            front = std::max(front, p.z);
        }
    }
    return front;
}

void TheHeadHasAFace() {
    for (size_t i = 0; i < kPeopleCount; ++i) {
        const CharacterMesh& mesh = CachedMesh(i);
        const float s = mesh.skeleton.statureMeters / 1.75f;
        const std::string who = Describe(i);
        // The nose stands out of the face: its tip is well in front of the cheeks beside it and of the forehead above.
        const float noseTip = FaceFront(mesh, 0.0f, 1.598f * s, 0.006f * s, 0.012f * s);
        const float cheek = FaceFront(mesh, 0.040f * s, 1.598f * s, 0.008f * s, 0.012f * s);
        const float forehead = FaceFront(mesh, 0.0f, 1.690f * s, 0.010f * s, 0.012f * s);
        Check(noseTip - cheek > 0.020f * s, who + ": the nose stands out of the face by " + std::to_string(noseTip - cheek) + " m");
        Check(noseTip > forehead + 0.008f * s, who + ": and in front of the forehead");
        // The eyes lie in hollows below the brow ridge and above the cheekbones.
        // A hollow is a dip: the surface at the eye lies behind both the brow above it and the cheekbone below it, where a
        // face without hollows would curve smoothly between the two and lie in front of the lower of them.
        const float brow = FaceFront(mesh, 0.030f * s, 1.668f * s, 0.012f * s, 0.006f * s);
        const float cheekbone = FaceFront(mesh, 0.031f * s, 1.608f * s, 0.008f * s, 0.006f * s);
        const float eye = FaceFront(mesh, 0.031f * s, 1.636f * s, 0.008f * s, 0.006f * s);
        Check(std::min(brow, cheekbone) - eye > 0.003f * s, who + ": the eye is hollowed behind the brow (" + std::to_string(brow) + ") and the cheekbone (" +
                                                                 std::to_string(cheekbone) + "): " + std::to_string(eye));
        // The lips are in front of the chin line and the groove above them is behind them.
        const float lips = FaceFront(mesh, 0.0f, 1.566f * s, 0.008f * s, 0.008f * s);
        const float belowChin = FaceFront(mesh, 0.0f, 1.528f * s, 0.008f * s, 0.004f * s);
        Check(lips > belowChin, who + ": the lips are in front of the underside of the chin");
        // Both ears stand beside the head.
        for (const float side : {1.0f, -1.0f}) {
            Check(PointInside(mesh, {side * 0.078f * s, 1.628f * s, -0.004f * s}, kHeadPiece), who + ": there is an ear on each side of the head");
            Check(!PointInside(mesh, {side * 0.115f * s, 1.628f * s, -0.004f * s}, kHeadPiece), who + ": and nothing beyond it");
        }
        // The head is the same on the left as on the right.
        const MeshPiece& piece = mesh.pieces[kHeadPiece];
        float worst = 0.0f;
        for (uint32_t v = piece.firstVertex; v < piece.firstVertex + piece.vertexCount; v += 5) {
            const glm::vec3 mirrored(-mesh.positions[v].x, mesh.positions[v].y, mesh.positions[v].z);
            float nearest = 1e9f;
            for (uint32_t other = piece.firstVertex; other < piece.firstVertex + piece.vertexCount; ++other) {
                nearest = std::min(nearest, glm::distance(mirrored, mesh.positions[other]));
            }
            worst = std::max(worst, nearest);
        }
        Check(worst < 0.012f, who + ": the head mirrors itself to within " + std::to_string(worst) + " m");
    }
}

// How far forward the surface of a shape lies at a width and a height: looking in from the front, the first place that is
// inside. Measured on the shape itself, to a fifth of a millimetre, so that the cells of a mesh do not blur it.
float FrontOf(const BodyShape& shape, float x, float y) {
    for (float z = 0.20f; z > -0.20f; z -= 0.0002f) {
        if (shape.Distance({x, y, z}) < 0.0f) {
            return z;
        }
    }
    return -1.0f;
}

void SexAndAgeChangeTheFace() {
    // The same person, a man and a woman, a young one and an old one, at the same stature and with the same seed.
    const auto head = [](float masculinity, float age, uint32_t seed) { return BuildHeadShape(Person(masculinity, age, 1.75f, 72.0f, 0.5f, seed)); };
    const BodyShape man = head(1.0f, 30.0f, 7);
    const BodyShape woman = head(0.0f, 30.0f, 7);
    const BodyShape young = head(0.5f, 20.0f, 7);
    const BodyShape old = head(0.5f, 75.0f, 7);
    const float browMan = FrontOf(man, 0.030f, 1.668f) - FrontOf(man, 0.031f, 1.636f);
    const float browWoman = FrontOf(woman, 0.030f, 1.668f) - FrontOf(woman, 0.031f, 1.636f);
    Check(browMan > browWoman, "a man's brow stands further over the eye: " + std::to_string(browMan) + " against " + std::to_string(browWoman));
    Check(FrontOf(woman, 0.0f, 1.5595f) > FrontOf(man, 0.0f, 1.5595f) + 0.002f, "a woman's lower lip stands further out: " + std::to_string(FrontOf(woman, 0.0f, 1.5595f)) + " against " + std::to_string(FrontOf(man, 0.0f, 1.5595f)));
    Check(FrontOf(young, 0.0f, 1.5725f) > FrontOf(old, 0.0f, 1.5725f) + 0.001f, "and a young person's upper lip is fuller than an old person's: " + std::to_string(FrontOf(young, 0.0f, 1.5725f)) + " against " + std::to_string(FrontOf(old, 0.0f, 1.5725f)));
    Check(FrontOf(old, 0.0f, 1.585f) < FrontOf(old, 0.0f, 1.58f) + 0.05f, "the front of the face is a number that exists");
    const BodyShape other = head(1.0f, 30.0f, 8);
    Check(FrontOf(man, 0.0f, 1.598f) != FrontOf(other, 0.0f, 1.598f), "another seed gives another nose");
}

// How many separate fingers a line across the hand passes through: the number of runs of points inside the hand.
int FingersAcross(const CharacterMesh& mesh, int piece, bool left, float alongFraction) {
    const float s = mesh.skeleton.statureMeters / 1.75f;
    const Joint wristJoint = left ? Joint::HandL : Joint::HandR;
    const Joint elbowJoint = left ? Joint::ForearmL : Joint::ForearmR;
    const glm::vec3 wrist = mesh.skeleton[wristJoint].restPosition;
    const glm::vec3 along = glm::normalize(wrist - mesh.skeleton[elbowJoint].restPosition);
    const float sign = left ? 1.0f : -1.0f;
    const glm::vec3 inward = glm::normalize(sign * glm::cross(along, glm::vec3(0.0f, 0.0f, 1.0f)));
    glm::vec3 across = glm::normalize(glm::cross(inward, along));
    if (across.z < 0.0f) {
        across = -across;
    }
    // Curled fingers lie further from the plane of the back of the hand the further along they go, so every width of the
    // hand is looked at through the whole depth of it.
    int runs = 0;
    bool inside = false;
    for (float b = -0.06f; b <= 0.07f; b += 0.001f) {
        bool hit = false;
        for (float c = -0.004f; c <= 0.040f && !hit; c += 0.003f) {
            hit = PointInside(mesh, wrist + (along * alongFraction + across * b + inward * c) * s, piece);
        }
        runs += hit && !inside ? 1 : 0;
        inside = hit;
    }
    return runs;
}

void AHandHasFiveSeparateFingersAndAPalm() {
    for (size_t i = 0; i < kPeopleCount; i += 2) {
        const CharacterMesh& mesh = CachedMesh(i);
        const std::string who = Describe(i);
        for (const bool left : {true, false}) {
            const int piece = left ? kLeftHandPiece : kRightHandPiece;
            const int fingers = FingersAcross(mesh, piece, left, 0.120f);
            Check(fingers >= 4 && fingers <= 6, who + ": a line across the " + (left ? "left" : "right") + " hand at the fingers passes through " + std::to_string(fingers) + " of them");
            const int palm = FingersAcross(mesh, piece, left, 0.050f);
            Check(palm >= 1 && palm <= 2, who + ": and across the palm through " + std::to_string(palm) + " solid piece");
            // The thumb reaches out past the edge of the palm, which is 4.25 cm from the middle.
            const float s = mesh.skeleton.statureMeters / 1.75f;
            const glm::vec3 wrist = mesh.skeleton[left ? Joint::HandL : Joint::HandR].restPosition;
            const glm::vec3 along = glm::normalize(wrist - mesh.skeleton[left ? Joint::ForearmL : Joint::ForearmR].restPosition);
            const glm::vec3 inward = glm::normalize((left ? 1.0f : -1.0f) * glm::cross(along, glm::vec3(0.0f, 0.0f, 1.0f)));
            glm::vec3 across = glm::normalize(glm::cross(inward, along));
            if (across.z < 0.0f) {
                across = -across;
            }
            float reach = 0.0f;
            const MeshPiece& range = mesh.pieces[piece];
            for (uint32_t v = range.firstVertex; v < range.firstVertex + range.vertexCount; ++v) {
                reach = std::max(reach, glm::dot(mesh.positions[v] - wrist, across));
            }
            Check(reach > 0.062f * s, who + ": the thumb reaches " + std::to_string(reach / s) + " from the middle of the hand");
        }
    }
}

void TheHeadAndHandsOverlapTheBodyAtTheNeckAndTheWrists() {
    for (size_t i = 0; i < kPeopleCount; i += 2) {
        const CharacterMesh& mesh = CachedMesh(i);
        const float s = mesh.skeleton.statureMeters / 1.75f;
        const std::string who = Describe(i);
        // The neck runs up into the head, so that the top of it cannot be seen.
        const glm::vec3 neckTop = mesh.skeleton[Joint::Head].restPosition + glm::vec3(0.0f, 0.008f * s, 0.006f * s);
        Check(PointInside(mesh, neckTop, kHeadPiece), who + ": the top of the neck is inside the head");
        Check(PointInside(mesh, neckTop, 0), who + ": and inside the body");
        // The forearm ends inside the cuff of the hand.
        for (const bool left : {true, false}) {
            const Joint wristJoint = left ? Joint::HandL : Joint::HandR;
            const Joint elbowJoint = left ? Joint::ForearmL : Joint::ForearmR;
            const glm::vec3 wrist = mesh.skeleton[wristJoint].restPosition;
            const glm::vec3 along = glm::normalize(wrist - mesh.skeleton[elbowJoint].restPosition);
            const glm::vec3 inForearm = wrist - along * (0.006f * s);
            Check(PointInside(mesh, inForearm, left ? kLeftHandPiece : kRightHandPiece), who + ": the wrist is inside the cuff of the hand");
            Check(PointInside(mesh, inForearm, 0), who + ": and inside the forearm");
        }
    }
}

void HeadAndHandsFollowTheirOwnBones() {
    const CharacterMesh& mesh = CachedMesh(4);
    const std::pair<int, Joint> expected[3] = {{kHeadPiece, Joint::Head}, {kLeftHandPiece, Joint::HandL}, {kRightHandPiece, Joint::HandR}};
    for (const auto& [piece, bone] : expected) {
        const MeshPiece& range = mesh.pieces[piece];
        bool onlyThatBone = true;
        for (uint32_t v = range.firstVertex; v < range.firstVertex + range.vertexCount; ++v) {
            onlyThatBone = onlyThatBone && mesh.boneIndices[v][0] == Index(bone) && mesh.boneWeights[v][0] == 1.0f;
        }
        Check(onlyThatBone, "piece " + std::to_string(piece) + " follows only " + mesh.skeleton[bone].name);
    }
    // Turning the head moves the head piece and leaves the body piece where it was.
    Pose pose;
    pose.rotation[Index(Joint::Head)] = FromDegrees(0.0f, 40.0f, 0.0f);
    const std::vector<glm::vec3> turned = SkinPositions(mesh, pose);
    const MeshPiece& head = mesh.pieces[kHeadPiece];
    uint32_t noseVertex = head.firstVertex;  // the vertex furthest forward on the head is the tip of the nose
    for (uint32_t v = head.firstVertex; v < head.firstVertex + head.vertexCount; ++v) {
        if (mesh.positions[v].z > mesh.positions[noseVertex].z) {
            noseVertex = v;
        }
    }
    Check(glm::distance(turned[noseVertex], mesh.positions[noseVertex]) > 0.01f, "turning the head 40 degrees moves the tip of the nose by more than a centimetre");
    float bodyMoved = 0.0f;
    for (uint32_t v = 0; v < mesh.pieces[0].vertexCount; ++v) {
        bodyMoved = std::max(bodyMoved, glm::distance(turned[v], mesh.positions[v]));
    }
    Check(bodyMoved > 0.0f, "turning the head turns the neck of the body a little");
}

// Builds at the ends of the range, where a fixed-size feature meets a body that has grown or shrunk around it.
std::vector<CharacterParameters> ExtremePeople() {
    return {Person(0.0f, 80.0f, 1.45f, 45.0f * 1.45f * 1.45f, 0.2f, 31), Person(1.0f, 60.0f, 1.45f, 45.0f * 1.45f * 1.45f, 0.5f, 32),
            Person(1.0f, 25.0f, 2.05f, 15.0f * 2.05f * 2.05f, 1.0f, 33), Person(0.0f, 30.0f, 1.45f, 15.0f * 1.45f * 1.45f, 0.0f, 34),
            Person(1.0f, 40.0f, 2.05f, 45.0f * 2.05f * 2.05f, 0.5f, 35)};
}

// The width of a shape across the arm at a distance along it from the wrist, in metres, found on the shape itself: where a
// line across the arm is inside it. `alongFraction` is in units of stature / 1.75, and is negative towards the elbow.
float ShapeWidthAcross(const BodyShape& shape, const Skeleton& skeleton, bool left, float alongFraction) {
    const float s = skeleton.statureMeters / 1.75f;
    const glm::vec3 wrist = skeleton[left ? Joint::HandL : Joint::HandR].restPosition;
    const glm::vec3 along = glm::normalize(wrist - skeleton[left ? Joint::ForearmL : Joint::ForearmR].restPosition);
    const glm::vec3 inward = glm::normalize((left ? 1.0f : -1.0f) * glm::cross(along, glm::vec3(0.0f, 0.0f, 1.0f)));
    glm::vec3 across = glm::normalize(glm::cross(inward, along));
    if (across.z < 0.0f) {
        across = -across;
    }
    float low = 1e9f;
    float high = -1e9f;
    for (float b = -0.10f; b <= 0.10f; b += 0.0002f) {
        if (shape.Distance(wrist + (along * alongFraction + across * b) * s) < 0.0f) {
            low = std::min(low, b);
            high = std::max(high, b);
        }
    }
    return high > low ? (high - low) * s : 0.0f;
}

void TheCuffMeetsTheForearmWithoutAStep() {
    std::vector<CharacterParameters> people = ExtremePeople();
    for (const CharacterParameters& person : SomePeople()) {
        people.push_back(person);
    }
    for (size_t index = 0; index < people.size(); ++index) {
        const CharacterMesh mesh = GenerateCharacter(people[index]);
        const float s = mesh.skeleton.statureMeters / 1.75f;
        const float minimumRadius = 0.55f * MeshSettings{}.voxelMeters;
        const BodyShape body = BuildBodyShape(mesh.skeleton, mesh.parameters, {.girth = mesh.girth, .minimumRadius = minimumRadius, .includeHead = false, .includeHands = false});
        for (const bool left : {true, false}) {
            const BodyShape hand = BuildHandShape(mesh.skeleton, mesh.parameters, left, mesh.girth);
            for (const float position : {-0.018f, -0.012f}) {
                const float forearm = ShapeWidthAcross(body, mesh.skeleton, left, position);
                const float cuff = ShapeWidthAcross(hand, mesh.skeleton, left, position);
                const std::string who = "person " + std::to_string(index) + ", " + (left ? "left" : "right") + " wrist at " + std::to_string(position);
                Check(forearm > 0.0f && cuff > 0.0f, who + ": both the forearm and the cuff are there");
                // The cuff covers the forearm, a little wider and never narrower, and never by more than 8 mm.
                Check(cuff - forearm > -0.001f * s && cuff - forearm < 0.008f * s,
                      who + ": the cuff is " + std::to_string((cuff - forearm) * 1000.0f) + " mm wider than the forearm of " + std::to_string(forearm * 1000.0f) + " mm");
            }
        }
    }
}

void TheNeckStaysUnderTheChinAtAnyBuild() {
    std::vector<CharacterParameters> people = ExtremePeople();
    for (const CharacterParameters& person : SomePeople()) {
        people.push_back(person);
    }
    for (size_t index = 0; index < people.size(); ++index) {
        const CharacterMesh mesh = GenerateCharacter(people[index]);
        const float s = mesh.skeleton.statureMeters / 1.75f;
        // Every vertex of the body in front of the face, at the height of the mouth and above, must be inside the head: the
        // neck comes out from under the chin, and the body must not come up over the mouth.
        int outside = 0;
        glm::vec3 firstOutside(0.0f);
        const MeshPiece& body = mesh.pieces[0];
        for (uint32_t v = body.firstVertex; v < body.firstVertex + body.vertexCount; ++v) {
            const glm::vec3 p = mesh.positions[v];
            if (std::abs(p.x) < 0.05f * s && p.z > 0.03f * s && p.y > 1.56f * s && !PointInside(mesh, p, kHeadPiece)) {
                firstOutside = outside == 0 ? p / s : firstOutside;
                ++outside;
            }
        }
        Check(outside == 0, "person " + std::to_string(index) + ": " + std::to_string(outside) + " vertices of the body come up in front of the face, the first at (" + std::to_string(firstOutside.x) + ", " +
                                std::to_string(firstOutside.y) + ", " + std::to_string(firstOutside.z) + ")");
    }
}

void HandsAreClosedSurfacesAtFineGridSizes() {
    // A finer grid has more room for the gaps between the fingers to become tunnels, so every size is checked.
    JobSystem jobs(3);
    for (const float voxel : {0.006f, 0.007f, 0.008f, 0.010f}) {
        for (uint32_t seed = 40; seed < 46; ++seed) {
            MeshSettings settings;
            settings.handVoxelMeters = voxel;
            const CharacterMesh mesh = GenerateCharacter(RandomParameters(seed), settings, &jobs);
            for (const size_t piece : {size_t{kLeftHandPiece}, size_t{kRightHandPiece}}) {
                const MeshTopology topology = MeasureTopology(mesh, piece);
                Check(topology.IsClosedSphere(), "seed " + std::to_string(seed) + " at " + std::to_string(voxel * 1000.0f) + " mm: hand piece " + std::to_string(piece) + " is a closed sphere, Euler characteristic " +
                                                     std::to_string(topology.eulerCharacteristic) + ", " + std::to_string(topology.badEdges) + " bad edges");
            }
        }
    }
}

void ACharacterWithoutFaceAndHandsIsOneWholeBody() {
    MeshSettings settings;
    settings.faceAndHands = false;
    const CharacterMesh mesh = GenerateCharacter(Person(0.5f, 30.0f, 1.70f, 70.0f, 0.4f, 21), settings);
    Check(mesh.pieces.size() == 1, "one piece when the face and the hands are left out");
    Check(MeasureTopology(mesh, 0).IsClosedSphere(), "and it is a closed sphere");
    Check(std::abs(mesh.massKg / mesh.parameters.massKg - 1.0f) < 0.03f, "of the right mass");
}

void AVoxelSizeThatIsNotANumberIsReplaced() {
    MeshSettings settings;
    settings.voxelMeters = std::numeric_limits<float>::quiet_NaN();
    const CharacterMesh mesh = GenerateCharacter(Person(0.5f, 30.0f, 1.70f, 70.0f, 0.4f), settings);
    Check(!mesh.positions.empty() && mesh.volumeCubicMeters > 0.0f, "a voxel size that is not a number gives the default grid");
    settings.voxelMeters = 1e-9f;
    const CharacterMesh fine = GenerateCharacter(Person(0.5f, 30.0f, 1.70f, 70.0f, 0.4f), {.voxelMeters = 0.04f});
    Check(fine.volumeCubicMeters > 0.0f, "and the coarsest grid still makes a body");
}

void GenerationFitsTheBudget() {
    const auto start = std::chrono::steady_clock::now();
    const CharacterMesh mesh = GenerateCharacter(Person(0.5f, 35.0f, 1.72f, 72.0f, 0.4f, 11));
    const double milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    const size_t triangles = mesh.indices.size() / 3;
    Check(triangles >= 20000 && triangles <= 100000, "a character has 20000 to 100000 triangles: " + std::to_string(triangles));
    Check(milliseconds < 4000.0, "and takes under 4 s on one thread: " + std::to_string(milliseconds) + " ms");
}

}  // namespace

// With a word on the command line, only the tests whose names contain it run, which is quick when one test is being worked on.
int main(int argc, char** argv) {
    const std::string filter = argc > 1 ? argv[1] : "";
    struct Test {
        const char* name;
        std::function<void()> run;
    };
    const Test tests[] = {
        {"TheSameParametersGiveTheSameBodyOnAnyNumberOfThreads", TheSameParametersGiveTheSameBodyOnAnyNumberOfThreads},
        {"EveryPieceIsOneClosedManifoldWithoutHoles", EveryPieceIsOneClosedManifoldWithoutHoles},
        {"NormalsPointOutAndAreUnitLength", NormalsPointOutAndAreUnitLength},
        {"TheBodyIsAsTallAndAsHeavyAsAskedAndStandsOnTheFloor", TheBodyIsAsTallAndAsHeavyAsAskedAndStandsOnTheFloor},
        {"TheSkeletonHasTheProportionsOfTheLiterature", TheSkeletonHasTheProportionsOfTheLiterature},
        {"TheSkeletonIsMirroredAndWellFormed", TheSkeletonIsMirroredAndWellFormed},
        {"TheBodyIsTheSameOnTheLeftAsOnTheRight", TheBodyIsTheSameOnTheLeftAsOnTheRight},
        {"SkinWeightsFollowTheRules", SkinWeightsFollowTheRules},
        {"SkinningInTheRestPoseGivesBackTheMesh", SkinningInTheRestPoseGivesBackTheMesh},
        {"BendingJointsKeepsTheVolumeOfTheBody", BendingJointsKeepsTheVolumeOfTheBody},
        {"TheLegsStayApartAndTheArmsStayOffTheBody", TheLegsStayApartAndTheArmsStayOffTheBody},
        {"ParametersThatAreNotNumbersOrOutOfRangeAreCleanedUp", ParametersThatAreNotNumbersOrOutOfRangeAreCleanedUp},
        {"RandomPeopleFollowThePopulation", RandomPeopleFollowThePopulation},
        {"ThePopulationLooksRight", ThePopulationLooksRight},
        {"EveryRandomPersonIsAClosedBodyOfTheRightSize", EveryRandomPersonIsAClosedBodyOfTheRightSize},
        {"AnElbowBendsAboutItsHingeAndAHipOpensTheRightWay", AnElbowBendsAboutItsHingeAndAHipOpensTheRightWay},
        {"FingertipsAndToesAreNeverLost", FingertipsAndToesAreNeverLost},
        {"TheHeadHasAFace", TheHeadHasAFace},
        {"SexAndAgeChangeTheFace", SexAndAgeChangeTheFace},
        {"AHandHasFiveSeparateFingersAndAPalm", AHandHasFiveSeparateFingersAndAPalm},
        {"TheHeadAndHandsOverlapTheBodyAtTheNeckAndTheWrists", TheHeadAndHandsOverlapTheBodyAtTheNeckAndTheWrists},
        {"HeadAndHandsFollowTheirOwnBones", HeadAndHandsFollowTheirOwnBones},
        {"TheCuffMeetsTheForearmWithoutAStep", TheCuffMeetsTheForearmWithoutAStep},
        {"TheNeckStaysUnderTheChinAtAnyBuild", TheNeckStaysUnderTheChinAtAnyBuild},
        {"HandsAreClosedSurfacesAtFineGridSizes", HandsAreClosedSurfacesAtFineGridSizes},
        {"ACharacterWithoutFaceAndHandsIsOneWholeBody", ACharacterWithoutFaceAndHandsIsOneWholeBody},
        {"AVoxelSizeThatIsNotANumberIsReplaced", AVoxelSizeThatIsNotANumberIsReplaced},
        {"GenerationFitsTheBudget", GenerationFitsTheBudget},
    };
    for (const Test& test : tests) {
        if (!filter.empty() && std::string(test.name).find(filter) == std::string::npos) {
            continue;
        }
        const int failuresBefore = failureCount;
        try {
            test.run();
        } catch (const std::exception& error) {
            ++failureCount;
            std::cerr << "  threw: " << error.what() << "\n";
        }
        std::cout << (failureCount == failuresBefore ? "PASS  " : "FAIL  ") << test.name << "\n";
    }
    std::cout << (failureCount == 0 ? "all character tests passed\n" : "character tests failed\n");
    return failureCount == 0 ? 0 : 1;
}
