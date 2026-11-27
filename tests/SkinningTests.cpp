// Tests of the CPU side of skinned characters (no window, no GPU): the blend that shaders/skinned.vert does, the layout of
// palettes and instances, the choice of which characters to draw, and the bounds the choice uses. Like the other test programs
// there is no test framework: each test is a function that calls Check, and main reports every failure it finds.
//
// The shader cannot run here, so these tests hold the CPU copy of its arithmetic (skinning::SkinPosition and SkinNormal) to the
// skinning of the character module, which is the reference, and hold the layout of the data the shader reads to what the
// shader expects.
#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

#include "character/CharacterGenerator.h"
#include "character/CharacterParameters.h"
#include "character/Skeleton.h"
#include "render/SkinningMath.h"

namespace {

using namespace character;

int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

bool Near(const glm::vec3& a, const glm::vec3& b, float tolerance) {
    return glm::length(a - b) <= tolerance;
}

// A coarse body, so that the tests that need a real mesh stay quick. It has the same skeleton and weights as a fine one.
const CharacterMesh& CoarseBody() {
    static const CharacterMesh body = [] {
        MeshSettings settings;
        settings.voxelMeters = 0.04f;
        settings.faceAndHands = false;
        return GenerateCharacter(RandomParameters(7), settings);
    }();
    return body;
}

std::vector<glm::mat4> PaletteOf(const Skeleton& skeleton, const Pose& pose) {
    const std::array<glm::mat4, kJointCount> skin = SkinMatrices(skeleton, pose);
    return {skin.begin(), skin.end()};
}

// A pose with every joint turned a little in a way that depends on the seed, so no two seeds are alike.
Pose RandomPose(uint32_t seed) {
    std::mt19937 random(seed);
    std::uniform_real_distribution<float> angle(-35.0f, 35.0f);
    Pose pose;
    for (glm::quat& rotation : pose.rotation) {
        rotation = FromDegrees(angle(random), angle(random), angle(random));
    }
    pose.rootTranslation = glm::vec3(0.1f, -0.05f, 0.2f);
    return pose;
}

skinning::Vertex VertexOn(glm::vec3 position, std::array<uint8_t, 4> joints, std::array<float, 4> weights) {
    skinning::Vertex vertex{};
    vertex.position = position;
    vertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
    vertex.joints = joints;
    vertex.weights = weights;
    return vertex;
}

glm::mat4 Translation(glm::vec3 offset) {
    return glm::translate(glm::mat4(1.0f), offset);
}

// ---------------------------------------------------------------------------
// The blend
// ---------------------------------------------------------------------------

void AnIdentityPaletteGivesBackTheRestMesh() {
    // With identity matrices a vertex comes back unchanged. A vertex on one joint with a weight of 1 comes back bit for bit; the
    // others only to within the rounding of weights that add up to 1.
    const std::vector<glm::mat4> identity(kJointCount, glm::mat4(1.0f));
    const skinning::MeshData data = skinning::BuildMeshData(CoarseBody());
    size_t moved = 0;
    for (size_t v = 0; v < data.vertices.size(); ++v) {
        const glm::vec3 skinned = skinning::SkinPosition(data.vertices[v], identity);
        // The weights add up to 1 only to within rounding, so the position may differ in the last digits.
        if (!Near(skinned, CoarseBody().positions[v], 1e-5f)) {
            ++moved;
        }
    }
    Check(moved == 0, "an identity palette moved " + std::to_string(moved) + " vertices");

    const skinning::Vertex single = VertexOn({0.3f, 1.2f, -0.4f}, {5, 0, 0, 0}, {1.0f, 0.0f, 0.0f, 0.0f});
    const glm::vec3 same = skinning::SkinPosition(single, identity);
    Check(same == single.position, "a single weight of 1 on an identity matrix returns the position exactly");
}

void TheRestPoseOfTheSkeletonGivesBackTheMesh() {
    const CharacterMesh& body = CoarseBody();
    const std::vector<glm::mat4> palette = PaletteOf(body.skeleton, Pose{});
    const skinning::MeshData data = skinning::BuildMeshData(body);
    float worst = 0.0f;
    for (size_t v = 0; v < data.vertices.size(); ++v) {
        worst = std::max(worst, glm::length(skinning::SkinPosition(data.vertices[v], palette) - body.positions[v]));
    }
    // world matrix times inverse bind is an identity only to within float rounding.
    Check(worst < 1e-4f, "the rest pose moves a vertex by " + std::to_string(worst));
}

void TheCpuBlendMatchesTheSkinningOfTheCharacterModule() {
    const CharacterMesh& body = CoarseBody();
    const skinning::MeshData data = skinning::BuildMeshData(body);
    for (uint32_t seed = 1; seed <= 5; ++seed) {
        const Pose pose = RandomPose(seed);
        const std::vector<glm::vec3> expected = SkinPositions(body, pose);
        const std::vector<glm::mat4> palette = PaletteOf(body.skeleton, pose);
        float worst = 0.0f;
        for (size_t v = 0; v < data.vertices.size(); ++v) {
            worst = std::max(worst, glm::length(skinning::SkinPosition(data.vertices[v], palette) - expected[v]));
        }
        Check(worst < 1e-4f, "pose " + std::to_string(seed) + ": the blend differs from SkinPositions by " + std::to_string(worst));
    }
}

void ARotatedElbowMovesTheForearmAndLeavesTheRest() {
    const CharacterMesh& body = CoarseBody();
    const Skeleton& skeleton = body.skeleton;
    const glm::vec3 elbow = skeleton[Joint::ForearmL].restPosition;
    const glm::vec3 hand = skeleton[Joint::HandL].restPosition;
    const glm::vec3 shoulder = skeleton[Joint::UpperArmL].restPosition;
    const float bend = 70.0f;

    Pose pose;
    pose.rotation[Index(Joint::ForearmL)] = HingeRotation(skeleton, Joint::ForearmL, bend);
    const std::vector<glm::mat4> palette = PaletteOf(skeleton, pose);

    // The expected result, worked out from the geometry alone: a point on the forearm turns about the elbow, on the hinge axis.
    const glm::quat turn = glm::angleAxis(glm::radians(bend), skeleton[Joint::ForearmL].hingeAxis);
    const glm::vec3 expectedHand = elbow + turn * (hand - elbow);

    const auto joint = [](Joint which) { return static_cast<uint8_t>(Index(which)); };
    const skinning::Vertex onForearm = VertexOn(hand, {joint(Joint::ForearmL), 0, 0, 0}, {1.0f, 0.0f, 0.0f, 0.0f});
    const skinning::Vertex onElbow = VertexOn(elbow, {joint(Joint::ForearmL), 0, 0, 0}, {1.0f, 0.0f, 0.0f, 0.0f});
    const skinning::Vertex onUpperArm = VertexOn(shoulder, {joint(Joint::UpperArmL), 0, 0, 0}, {1.0f, 0.0f, 0.0f, 0.0f});
    const skinning::Vertex shared = VertexOn(hand, {joint(Joint::ForearmL), joint(Joint::UpperArmL), 0, 0}, {0.5f, 0.5f, 0.0f, 0.0f});

    Check(Near(skinning::SkinPosition(onForearm, palette), expectedHand, 1e-4f), "a vertex of the forearm turns about the elbow");
    Check(glm::length(skinning::SkinPosition(onForearm, palette) - hand) > 0.1f, "the bent forearm has moved the hand");
    Check(Near(skinning::SkinPosition(onElbow, palette), elbow, 1e-4f), "the elbow itself stays");
    Check(Near(skinning::SkinPosition(onUpperArm, palette), shoulder, 1e-4f), "the upper arm does not follow the forearm");
    Check(Near(skinning::SkinPosition(shared, palette), 0.5f * (expectedHand + hand), 1e-4f),
          "a vertex shared half and half is halfway between where each joint would put it");

    // The normal turns with the forearm, and stays a unit vector, even where two joints share a vertex.
    const glm::vec3 normal = skinning::SkinNormal(onForearm, palette);
    Check(Near(normal, turn * onForearm.normal, 1e-4f), "the normal of the forearm turns with it");
    Check(std::abs(glm::length(skinning::SkinNormal(shared, palette)) - 1.0f) < 1e-5f, "a shared normal is a unit vector");
}

void MeshDataHasWeightsThatAddUpToOne() {
    CharacterMesh body;
    body.skeleton = BuildSkeleton(CharacterParameters{});
    body.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    body.normals.assign(4, glm::vec3(0, 0, 1));
    body.boneIndices = {{1, 2, 0, 0}, {3, 0, 0, 0}, {4, 5, 6, 7}, {1, 2, 3, 4}};
    body.boneWeights = {std::array<float, 4>{2.0f, 2.0f, 0.0f, 0.0f},  // too heavy: halved
                        std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f},  // none at all: all on the first joint
                        std::array<float, 4>{0.1f, 0.2f, 0.3f, 0.2f},  // too light
                        std::array<float, 4>{0.5f, 0.5f, -0.5f, 0.0f}};  // a negative weight is dropped
    body.indices = {0, 1, 2, 0, 2, 3};

    const skinning::MeshData data = skinning::BuildMeshData(body);
    for (size_t v = 0; v < data.vertices.size(); ++v) {
        float sum = 0.0f;
        for (float weight : data.vertices[v].weights) {
            Check(weight >= 0.0f, "vertex " + std::to_string(v) + " has a negative weight");
            sum += weight;
        }
        Check(std::abs(sum - 1.0f) < 1e-6f, "vertex " + std::to_string(v) + " has weights that add up to " + std::to_string(sum));
    }
    Check(data.vertices[0].weights[0] == 0.5f && data.vertices[0].weights[1] == 0.5f, "weights keep their proportions");
    Check(data.vertices[1].weights[0] == 1.0f, "a vertex with no weight follows its first joint");
    Check(data.vertices[3].weights[2] == 0.0f && data.vertices[3].weights[0] == 0.5f, "a negative weight is dropped");
    Check(data.info.jointCount == static_cast<uint32_t>(kJointCount), "the palette has a matrix for every joint");
    Check(data.indices == body.indices, "triangles are copied as they are");
}

void TheRealMeshHasNormalisedWeightsAndKnownJoints() {
    const skinning::MeshData data = skinning::BuildMeshData(CoarseBody());
    size_t badSum = 0;
    size_t badJoint = 0;
    for (const skinning::Vertex& vertex : data.vertices) {
        float sum = 0.0f;
        for (int slot = 0; slot < 4; ++slot) {
            sum += vertex.weights[slot];
            badJoint += vertex.weights[slot] > 0.0f && vertex.joints[slot] >= data.info.jointCount ? 1 : 0;
        }
        badSum += std::abs(sum - 1.0f) > 1e-5f ? 1 : 0;
    }
    Check(badSum == 0, std::to_string(badSum) + " vertices have weights that do not add up to 1");
    Check(badJoint == 0, std::to_string(badJoint) + " vertices follow a joint that is not in the palette");
}

// ---------------------------------------------------------------------------
// Layout of what the shader reads
// ---------------------------------------------------------------------------

void TheLayoutsMatchTheShader() {
    // skinned.vert reads these at locations 0 to 3 (vertex) and 4 to 10 (instance); PipelineBuilder.cpp uses the same offsets.
    static_assert(offsetof(skinning::Vertex, position) == 0);
    static_assert(offsetof(skinning::Vertex, normal) == 12);
    static_assert(offsetof(skinning::Vertex, joints) == 24);
    static_assert(offsetof(skinning::Vertex, weights) == 28);
    static_assert(offsetof(skinning::GpuInstance, world) == 0);
    static_assert(offsetof(skinning::GpuInstance, color) == 64);
    static_assert(offsetof(skinning::GpuInstance, roughness) == 76);
    static_assert(offsetof(skinning::GpuInstance, paletteOffset) == 80);
    static_assert(offsetof(skinning::GpuInstance, material) == 84);
    static_assert(offsetof(skinning::GpuInstance, seed) == 88);
    static_assert(sizeof(glm::mat4) == 64, "a palette entry is one std430 mat4");
    Check(true, "layouts are checked at compile time");
}

// ---------------------------------------------------------------------------
// Palettes, offsets and the choice of what to draw
// ---------------------------------------------------------------------------

// Two small meshes with different numbers of joints, so that offsets cannot be right by luck.
std::vector<skinning::MeshInfo> TwoMeshes() {
    skinning::MeshInfo small;
    small.jointCount = 3;
    small.bounds.restPositions = {{0, 0, 0}, {0, 1, 0}, {0, 2, 0}};
    small.bounds.radii = {0.5f, 0.5f, 0.5f};
    skinning::MeshInfo large;
    large.jointCount = 5;
    large.bounds.restPositions = {{0, 0, 0}, {0, 1, 0}, {0, 2, 0}, {1, 1, 0}, {-1, 1, 0}};
    large.bounds.radii = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
    return {small, large};
}

// A palette whose matrices all differ, and carry `tag` in their translation, so that a mixed up offset is seen.
std::vector<glm::mat4> TaggedPalette(size_t jointCount, float tag) {
    std::vector<glm::mat4> palette;
    for (size_t joint = 0; joint < jointCount; ++joint) {
        palette.push_back(Translation({tag, static_cast<float>(joint), 0.0f}));
    }
    return palette;
}

SkinnedInstance InstanceOf(uint32_t mesh, glm::vec3 position, const std::vector<glm::mat4>& palette) {
    SkinnedInstance instance;
    instance.mesh.index = mesh;
    instance.world = Translation(position);
    instance.palette = palette;
    return instance;
}

// A camera that looks along -Z from the origin, and a sun that sees everything within 100 m of the origin.
Frustum CameraLookingDownMinusZ() {
    return Frustum(glm::perspective(glm::radians(60.0f), 1.0f, 0.1f, 100.0f));
}
Frustum SeesEverythingNearby() {
    return Frustum(glm::ortho(-100.0f, 100.0f, -100.0f, 100.0f, -100.0f, 100.0f));
}

void EveryInstanceFindsItsOwnPalette() {
    const std::vector<skinning::MeshInfo> meshes = TwoMeshes();
    const std::vector<glm::mat4> paletteA = TaggedPalette(3, 1.0f);  // mesh 0 has 3 joints
    const std::vector<glm::mat4> paletteB = TaggedPalette(5, 2.0f);  // mesh 1 has 5
    const std::vector<glm::mat4> paletteC = TaggedPalette(5, 3.0f);
    // Submitted out of mesh order on purpose: the plan groups them by mesh, and each palette must follow its own instance.
    // The order drawn is A (mesh 0), then B and C (mesh 1), so the palettes start at matrix 0, 3 and 8: offsets that no
    // fixed stride between characters would give.
    const std::vector<SkinnedInstance> submitted = {InstanceOf(1, {0, 0, -5}, paletteB), InstanceOf(0, {1, 0, -5}, paletteA),
                                                    InstanceOf(1, {2, 0, -5}, paletteC)};
    skinning::FramePlan plan;
    skinning::PlanFrame(submitted, meshes, CameraLookingDownMinusZ(), SeesEverythingNearby(), plan);

    Check(plan.instances.size() == 3, "three characters are drawn");
    Check(plan.palettes.size() == 3 + 5 + 5, "the palettes hold 13 matrices in all");
    Check(plan.ranges.size() == 2, "one range for each mesh");
    if (plan.instances.size() != 3 || plan.ranges.size() != 2) {
        return;
    }
    Check(plan.ranges[0].mesh == 0 && plan.ranges[0].firstInstance == 0 && plan.ranges[0].totalCount == 1, "mesh 0 has one instance first");
    Check(plan.ranges[1].mesh == 1 && plan.ranges[1].firstInstance == 1 && plan.ranges[1].totalCount == 2, "mesh 1 has two instances after it");

    Check(plan.instances[0].paletteOffset == 0 && plan.instances[1].paletteOffset == 3 && plan.instances[2].paletteOffset == 8,
          "palette offsets count matrices: 0, 3, 8");
    const std::vector<float> expectedTags = {1.0f, 2.0f, 3.0f};
    const std::vector<size_t> jointCounts = {3, 5, 5};
    for (size_t i = 0; i < 3; ++i) {
        const uint32_t offset = plan.instances[i].paletteOffset;
        for (size_t joint = 0; joint < jointCounts[i]; ++joint) {
            const glm::mat4& matrix = plan.palettes[offset + joint];
            Check(matrix[3][0] == expectedTags[i] && matrix[3][1] == static_cast<float>(joint),
                  "instance " + std::to_string(i) + " joint " + std::to_string(joint) + " reads its own matrix");
        }
    }
    Check(plan.instances[0].world[3][0] == 1.0f && plan.instances[1].world[3][0] == 0.0f && plan.instances[2].world[3][0] == 2.0f,
          "each instance keeps its own world matrix");
}

void InstancesAreDroppedForTheRightReasons() {
    const std::vector<skinning::MeshInfo> meshes = TwoMeshes();
    const std::vector<glm::mat4> shortPalette = TaggedPalette(2, 1.0f);  // mesh 0 has 3 joints
    const std::vector<glm::mat4> goodPalette = TaggedPalette(3, 1.0f);
    const std::vector<glm::mat4> longPalette = TaggedPalette(8, 1.0f);  // more than needed is fine; only 3 are used
    const std::vector<SkinnedInstance> submitted = {InstanceOf(0, {0, 0, -5}, shortPalette), InstanceOf(7, {0, 0, -5}, goodPalette),
                                                    InstanceOf(0, {0, 0, -5}, longPalette)};
    skinning::FramePlan plan;
    uint32_t dropped = 0;
    skinning::PlanFrame(submitted, meshes, CameraLookingDownMinusZ(), SeesEverythingNearby(), plan, &dropped);
    Check(plan.instances.size() == 1, "only the character with enough matrices is drawn");
    Check(dropped == 1, "the short palette is counted; the unknown handle is not a palette problem");
    Check(plan.palettes.size() == 3, "only as many matrices as the mesh has joints are copied");
}

void CharactersOutOfViewAreCulledAndOnesSeenOnlyByTheSunStillCastShadows() {
    const std::vector<skinning::MeshInfo> meshes = TwoMeshes();
    const std::vector<glm::mat4> palette = TaggedPalette(3, 0.0f);
    // The sun here sees only a thin slab around x = 50; the camera looks down -Z from the origin.
    const Frustum sunSeesOnlyFarRight(glm::ortho(40.0f, 60.0f, -100.0f, 100.0f, -100.0f, 100.0f));
    const std::vector<SkinnedInstance> submitted = {
        InstanceOf(0, {50, 0, 50}, palette),   // behind the camera, in the sun's view: a shadow caster only
        InstanceOf(0, {0, 1, -10}, palette),   // in front of the camera: seen by the camera (and not by this sun)
        InstanceOf(0, {-80, 0, 50}, palette),  // seen by neither
        InstanceOf(0, {0, 1, -20}, palette),   // in front of the camera
    };
    skinning::FramePlan plan;
    skinning::PlanFrame(submitted, meshes, CameraLookingDownMinusZ(), sunSeesOnlyFarRight, plan);
    Check(plan.instances.size() == 3, "the character seen by nothing is dropped");
    Check(plan.ranges.size() == 1, "one mesh");
    if (plan.ranges.size() != 1 || plan.instances.size() != 3) {
        return;
    }
    Check(plan.ranges[0].cameraCount == 2 && plan.ranges[0].totalCount == 3, "two for the camera, three for the sun");
    // The camera's instances come first, in the order they were submitted; the shadow caster last.
    Check(plan.instances[0].world[3][2] == -10.0f && plan.instances[1].world[3][2] == -20.0f && plan.instances[2].world[3][2] == 50.0f,
          "the instances the camera sees come first");
}

// ---------------------------------------------------------------------------
// Bounds
// ---------------------------------------------------------------------------

void TheAnimatedBoundsHoldTheWholeSkinnedBody() {
    const CharacterMesh& body = CoarseBody();
    const skinning::MeshData data = skinning::BuildMeshData(body);
    for (uint32_t seed = 0; seed <= 8; ++seed) {
        // Seed 0 is the rest pose; the others bend every joint, which is more than any walk does.
        const Pose pose = seed == 0 ? Pose{} : RandomPose(seed);
        const std::vector<glm::mat4> palette = PaletteOf(body.skeleton, pose);
        const skinning::Box box = skinning::AnimatedBounds(data.info.bounds, palette);
        size_t outside = 0;
        for (const skinning::Vertex& vertex : data.vertices) {
            const glm::vec3 posed = skinning::SkinPosition(vertex, palette);
            const bool inside = glm::all(glm::greaterThanEqual(posed, box.min - 1e-4f)) && glm::all(glm::lessThanEqual(posed, box.max + 1e-4f));
            outside += inside ? 0 : 1;
        }
        Check(outside == 0, "pose " + std::to_string(seed) + ": " + std::to_string(outside) + " vertices lie outside the bounds");
    }
}

void EveryJointSphereReachesAllTheVerticesItPullsOn() {
    // This is what makes the bounds hold in every pose: the posed vertex is a blend of points that each lie within their joint's
    // sphere, so the spheres have to reach every vertex with any weight on the joint, however small.
    const skinning::MeshData data = skinning::BuildMeshData(CoarseBody());
    size_t unreached = 0;
    for (const skinning::Vertex& vertex : data.vertices) {
        for (int slot = 0; slot < 4; ++slot) {
            if (vertex.weights[slot] <= 0.0f) {
                continue;
            }
            const uint8_t joint = vertex.joints[slot];
            const float distance = glm::length(vertex.position - data.info.bounds.restPositions[joint]);
            unreached += distance > data.info.bounds.radii[joint] + 1e-5f ? 1 : 0;
        }
    }
    Check(unreached == 0, std::to_string(unreached) + " vertex weights lie outside the sphere of their joint");
}

void TheBoundsAreTightEnoughToCullWith() {
    const CharacterMesh& body = CoarseBody();
    const skinning::MeshData data = skinning::BuildMeshData(body);
    const std::vector<glm::mat4> palette = PaletteOf(body.skeleton, Pose{});
    const skinning::Box box = skinning::AnimatedBounds(data.info.bounds, palette);
    const glm::vec3 size = box.max - box.min;
    // A person is about as tall as the stature and no wider than a couple of metres; a box much bigger than that would
    // keep characters that are long out of view in the draw lists.
    Check(size.y > body.skeleton.statureMeters * 0.95f && size.y < body.skeleton.statureMeters * 1.4f,
          "the box is about as tall as the character: " + std::to_string(size.y));
    Check(size.x < 2.5f && size.z < 2.5f, "the box is not wider than a couple of metres");
}

void TheWorldSphereFollowsTheWorldMatrix() {
    const skinning::Box box{{-1.0f, 0.0f, -1.0f}, {1.0f, 2.0f, 1.0f}};
    const skinning::Sphere inPlace = skinning::WorldSphere(box, glm::mat4(1.0f));
    Check(Near(inPlace.center, {0.0f, 1.0f, 0.0f}, 1e-6f), "the sphere is centred on the box");
    Check(std::abs(inPlace.radius - std::sqrt(3.0f)) < 1e-5f, "the sphere reaches the corners of the box");

    glm::mat4 world = Translation({10.0f, 0.0f, -4.0f});
    world = glm::rotate(world, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    world = glm::scale(world, glm::vec3(2.0f));
    const skinning::Sphere placed = skinning::WorldSphere(box, world);
    Check(Near(placed.center, {10.0f, 2.0f, -4.0f}, 1e-4f), "the centre moves with the world matrix");
    Check(std::abs(placed.radius - 2.0f * std::sqrt(3.0f)) < 1e-4f, "the radius follows the scale of the world matrix");
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
        {"AnIdentityPaletteGivesBackTheRestMesh", AnIdentityPaletteGivesBackTheRestMesh},
        {"TheRestPoseOfTheSkeletonGivesBackTheMesh", TheRestPoseOfTheSkeletonGivesBackTheMesh},
        {"TheCpuBlendMatchesTheSkinningOfTheCharacterModule", TheCpuBlendMatchesTheSkinningOfTheCharacterModule},
        {"ARotatedElbowMovesTheForearmAndLeavesTheRest", ARotatedElbowMovesTheForearmAndLeavesTheRest},
        {"MeshDataHasWeightsThatAddUpToOne", MeshDataHasWeightsThatAddUpToOne},
        {"TheRealMeshHasNormalisedWeightsAndKnownJoints", TheRealMeshHasNormalisedWeightsAndKnownJoints},
        {"TheLayoutsMatchTheShader", TheLayoutsMatchTheShader},
        {"EveryInstanceFindsItsOwnPalette", EveryInstanceFindsItsOwnPalette},
        {"InstancesAreDroppedForTheRightReasons", InstancesAreDroppedForTheRightReasons},
        {"CharactersOutOfViewAreCulledAndOnesSeenOnlyByTheSunStillCastShadows",
         CharactersOutOfViewAreCulledAndOnesSeenOnlyByTheSunStillCastShadows},
        {"TheAnimatedBoundsHoldTheWholeSkinnedBody", TheAnimatedBoundsHoldTheWholeSkinnedBody},
        {"EveryJointSphereReachesAllTheVerticesItPullsOn", EveryJointSphereReachesAllTheVerticesItPullsOn},
        {"TheBoundsAreTightEnoughToCullWith", TheBoundsAreTightEnoughToCullWith},
        {"TheWorldSphereFollowsTheWorldMatrix", TheWorldSphereFollowsTheWorldMatrix},
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
    std::cout << (failureCount == 0 ? "all skinning tests passed\n" : "skinning tests failed\n");
    return failureCount == 0 ? 0 : 1;
}
