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

struct EdgeCount {
    std::map<std::pair<uint32_t, uint32_t>, int> uses;
};

EdgeCount CountEdges(const std::vector<uint32_t>& indices) {
    EdgeCount result;
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        for (int corner = 0; corner < 3; ++corner) {
            uint32_t a = indices[t + corner];
            uint32_t b = indices[t + (corner + 1) % 3];
            if (a > b) {
                std::swap(a, b);
            }
            ++result.uses[{a, b}];
        }
    }
    return result;
}

int ComponentCount(size_t vertexCount, const std::vector<uint32_t>& indices) {
    std::vector<int> parent(vertexCount);
    std::iota(parent.begin(), parent.end(), 0);
    const std::function<int(int)> find = [&](int v) {
        while (parent[v] != v) {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        return v;
    };
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        parent[find(static_cast<int>(indices[t]))] = find(static_cast<int>(indices[t + 1]));
        parent[find(static_cast<int>(indices[t + 1]))] = find(static_cast<int>(indices[t + 2]));
    }
    int components = 0;
    std::vector<bool> used(vertexCount, false);
    for (const uint32_t index : indices) {
        used[index] = true;
    }
    for (size_t v = 0; v < vertexCount; ++v) {
        if (used[v] && find(static_cast<int>(v)) == static_cast<int>(v)) {
            ++components;
        }
    }
    return components;
}

// Whether a point is inside a closed mesh, by counting how many triangles a ray along +X crosses.
bool PointInside(const CharacterMesh& mesh, glm::vec3 origin) {
    int crossings = 0;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
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
    Person(0.7f, 38.0f, 1.74f, 76.0f, 0.5f, 10);
    const CharacterMesh other = GenerateCharacter(Person(0.7f, 38.0f, 1.74f, 76.0f, 0.5f, 10));
    Check(other.positions != first.positions || other.skeleton[Joint::Pelvis].restPosition != first.skeleton[Joint::Pelvis].restPosition,
          "and another seed gives another body");
}

void TheSurfaceIsOneClosedManifoldWithoutHoles() {
    for (size_t i = 0; i < kPeopleCount; ++i) {
        const CharacterMesh& mesh = CachedMesh(i);
        const EdgeCount edges = CountEdges(mesh.indices);
        int badEdges = 0;
        for (const auto& [edge, uses] : edges.uses) {
            badEdges += uses != 2 ? 1 : 0;
        }
        Check(badEdges == 0, Describe(i) + ": every edge is shared by exactly two triangles, " + std::to_string(badEdges) + " are not");
        const long long euler = static_cast<long long>(mesh.positions.size()) - static_cast<long long>(edges.uses.size()) +
                                static_cast<long long>(mesh.indices.size() / 3);
        Check(euler == 2, Describe(i) + ": the surface is a sphere without handles, Euler characteristic " + std::to_string(euler));
        Check(ComponentCount(mesh.positions.size(), mesh.indices) == 1, Describe(i) + ": it is one piece");
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
        Check(std::abs(lowest) < 0.012f, Describe(i) + ": the soles are on the floor, lowest point " + std::to_string(lowest));
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

Pose BentPose(float elbowDegrees, float kneeDegrees, float twistDegrees) {
    Pose pose;
    pose.rotation[Index(Joint::ForearmL)] = FromDegrees(-elbowDegrees, 0.0f, 0.0f);
    pose.rotation[Index(Joint::ForearmR)] = FromDegrees(-elbowDegrees, 0.0f, 0.0f);
    pose.rotation[Index(Joint::CalfL)] = FromDegrees(kneeDegrees, 0.0f, 0.0f);
    pose.rotation[Index(Joint::CalfR)] = FromDegrees(kneeDegrees, 0.0f, 0.0f);
    pose.rotation[Index(Joint::ForearmL)] = FromDegrees(-elbowDegrees, twistDegrees, 0.0f);
    pose.rotation[Index(Joint::ForearmR)] = FromDegrees(-elbowDegrees, -twistDegrees, 0.0f);
    return pose;
}

void BendingJointsKeepsTheVolumeOfTheBody() {
    // Linear blend skinning loses volume at a bent joint (the candy wrapper), and this is where it must stay small.
    for (size_t i = 0; i < kPeopleCount; ++i) {
        const CharacterMesh& mesh = CachedMesh(i);
        const float rest = mesh.volumeCubicMeters;
        const float bent = MeshVolume(SkinPositions(mesh, BentPose(90.0f, 90.0f, 0.0f)), mesh.indices);
        Check(std::abs(bent / rest - 1.0f) < 0.06f, Describe(i) + ": elbows and knees at 90 degrees change the volume by " + std::to_string((bent / rest - 1.0f) * 100.0f) + "%");
        const float twisted = MeshVolume(SkinPositions(mesh, BentPose(0.0f, 0.0f, 80.0f)), mesh.indices);
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
    for (uint32_t seed = 100; seed < 130; ++seed) {
        const CharacterMesh mesh = GenerateCharacter(RandomParameters(seed), {}, &jobs);
        const EdgeCount edges = CountEdges(mesh.indices);
        int badEdges = 0;
        for (const auto& [edge, uses] : edges.uses) {
            badEdges += uses != 2 ? 1 : 0;
        }
        const long long euler = static_cast<long long>(mesh.positions.size()) - static_cast<long long>(edges.uses.size()) +
                                static_cast<long long>(mesh.indices.size() / 3);
        const std::string who = "random person " + std::to_string(seed);
        Check(badEdges == 0 && euler == 2 && ComponentCount(mesh.positions.size(), mesh.indices) == 1,
              who + " is one closed sphere: " + std::to_string(badEdges) + " bad edges, Euler characteristic " + std::to_string(euler));
        Check(std::abs(mesh.massKg / mesh.parameters.massKg - 1.0f) < 0.04f, who + " has the mass asked for: " + std::to_string(mesh.massKg) + " against " + std::to_string(mesh.parameters.massKg));
    }
}

void GenerationFitsTheBudget() {
    const auto start = std::chrono::steady_clock::now();
    const CharacterMesh mesh = GenerateCharacter(Person(0.5f, 35.0f, 1.72f, 72.0f, 0.4f, 11));
    const double milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    const size_t triangles = mesh.indices.size() / 3;
    Check(triangles >= 8000 && triangles <= 40000, "a character has 8000 to 40000 triangles: " + std::to_string(triangles));
    Check(milliseconds < 1500.0, "and takes under 1.5 s on one thread: " + std::to_string(milliseconds) + " ms");
}

}  // namespace

int main() {
    struct Test {
        const char* name;
        std::function<void()> run;
    };
    const Test tests[] = {
        {"TheSameParametersGiveTheSameBodyOnAnyNumberOfThreads", TheSameParametersGiveTheSameBodyOnAnyNumberOfThreads},
        {"TheSurfaceIsOneClosedManifoldWithoutHoles", TheSurfaceIsOneClosedManifoldWithoutHoles},
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
        {"GenerationFitsTheBudget", GenerationFitsTheBudget},
    };
    for (const Test& test : tests) {
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
