#include "demo/CrowdCharacters.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <iostream>

#include <glm/gtc/matrix_transform.hpp>

#include "character/CharacterGenerator.h"
#include "character/CharacterParameters.h"
#include "core/JobSystem.h"

namespace demo {
namespace {

constexpr uint32_t kDistinctBodies = 4;
// Voxel sizes of the coarser meshes of each body for the middle and the far distance (the full mesh uses 0.024).
constexpr float kCoarserVoxelMeters[] = {0.05f, 0.10f};

// Walk cycle amplitudes in degrees at full stride.
constexpr float kThighSwing = 28.0f;
constexpr float kKneeFlex = 55.0f;
constexpr float kArmSwing = 22.0f;

// Makes the same person twice more from coarser grids, without the fine head and hands, and chains them below the full mesh.
void UploadCoarserVersions(IRenderModule& renderer, const character::CharacterParameters& parameters, CharacterHandle full) {
    CharacterHandle finer = full;
    for (const float voxelMeters : kCoarserVoxelMeters) {
        character::MeshSettings settings;
        settings.voxelMeters = voxelMeters;
        settings.faceAndHands = false;
        const character::CharacterMesh mesh = character::GenerateCharacter(parameters, settings, &JobSystem::Shared());
        const CharacterHandle coarser = renderer.UploadCharacterMesh(skinning::BuildMeshData(mesh));
        if (!coarser.IsValid() || !renderer.SetCharacterLowerDetail(finer, coarser)) {
            std::cerr << "A coarser version of a character was not accepted; it is drawn at full detail from afar\n";
            return;
        }
        finer = coarser;
    }
}

// A number that belongs to one person and stays the same as they walk, so that they keep their body from frame to frame: the
// crowd gives people no identity in what it hands out for drawing, but their clothes and height are chosen once and stay.
uint32_t PersonKey(const NpcInstance& person) {
    const auto bits = [](float value) { return std::bit_cast<uint32_t>(value); };
    uint32_t key = bits(person.heightScale) * 2654435761u;
    key = (key ^ bits(person.clothingColor.r)) * 2246822519u;
    key = (key ^ bits(person.clothingColor.g)) * 3266489917u;
    key = (key ^ bits(person.clothingColor.b)) * 668265263u;
    return key ^ (key >> 15);
}

}  // namespace

void CrowdCharacters::MakeBodies(IRenderModule& renderer, uint32_t seed) {
    bodies.clear();
    for (uint32_t i = 0; i < kDistinctBodies; ++i) {
        const character::CharacterParameters parameters = character::RandomParameters(seed * 131u + i);
        const character::CharacterMesh mesh = character::GenerateCharacter(parameters, {}, &JobSystem::Shared());
        Body body;
        body.skeleton = mesh.skeleton;
        body.handle = renderer.UploadCharacterMesh(skinning::BuildMeshData(mesh));
        if (!body.handle.IsValid()) {
            std::cerr << "The renderer did not accept character body " << i << "\n";
            continue;
        }
        UploadCoarserVersions(renderer, parameters, body.handle);
        bodies.push_back(std::move(body));
    }
}

// A walk made from the cycle the crowd keeps for each person: the legs swing against each other with the knee bending as the
// leg passes forward, the arms swing against the legs, and the body bobs twice in a cycle. At zero stride the person stands.
character::Pose CrowdCharacters::WalkPose(float phase, float stride, float variation) {
    using character::Joint;
    const float swingLeft = std::sin(phase);
    const float swingRight = -swingLeft;
    // A knee bends while its leg swings forward, which is while the swing is rising.
    const float rateLeft = std::cos(phase);
    const float rateRight = -rateLeft;
    const auto flex = [&](float rate) { return stride * kKneeFlex * std::max(0.0f, rate); };

    character::Pose pose;
    pose.rootTranslation.y = -0.012f - stride * 0.02f * std::abs(std::sin(phase));
    pose.rotation[character::Index(Joint::ThighL)] = character::FromDegrees(-stride * kThighSwing * swingLeft, 0.0f, 0.0f);
    pose.rotation[character::Index(Joint::ThighR)] = character::FromDegrees(-stride * kThighSwing * swingRight, 0.0f, 0.0f);
    pose.rotation[character::Index(Joint::CalfL)] = character::FromDegrees(5.0f + flex(rateLeft), 0.0f, 0.0f);
    pose.rotation[character::Index(Joint::CalfR)] = character::FromDegrees(5.0f + flex(rateRight), 0.0f, 0.0f);
    // The arms swing against the legs of their own side, a little less for some people than for others.
    const float armSwing = stride * kArmSwing * (0.8f + 0.4f * variation);
    pose.rotation[character::Index(Joint::UpperArmL)] = character::FromDegrees(armSwing * swingLeft, 0.0f, 6.0f);
    pose.rotation[character::Index(Joint::UpperArmR)] = character::FromDegrees(armSwing * swingRight, 0.0f, -6.0f);
    pose.rotation[character::Index(Joint::Spine2)] = character::FromDegrees(0.0f, stride * 4.0f * swingLeft, 0.0f);
    pose.rotation[character::Index(Joint::Head)] = character::FromDegrees(0.0f, -stride * 3.0f * swingLeft, 0.0f);
    return pose;
}

size_t CrowdCharacters::ChooseNearest(std::vector<NpcInstance>& people, glm::vec3 camera, uint32_t limit, float reach) {
    const auto distanceSquared = [&](const NpcInstance& person) { return glm::dot(person.position - camera, person.position - camera); };
    // The people within reach go to the front, then the nearest `limit` of those go first among them.
    const auto withinReach = std::partition(people.begin(), people.end(),
                                            [&](const NpcInstance& person) { return distanceSquared(person) <= reach * reach; });
    const size_t found = static_cast<size_t>(withinReach - people.begin());
    const size_t chosen = std::min<size_t>(found, limit);
    std::nth_element(people.begin(), people.begin() + static_cast<std::ptrdiff_t>(chosen), people.begin() + static_cast<std::ptrdiff_t>(found),
                     [&](const NpcInstance& a, const NpcInstance& b) { return distanceSquared(a) < distanceSquared(b); });
    return chosen;
}

void CrowdCharacters::TakeNearest(std::vector<NpcInstance>& people, glm::vec3 camera, uint32_t limit, float reach, FrameInput& frame) {
    instances.clear();
    palettes.clear();
    if (bodies.empty() || limit == 0 || people.empty()) {
        return;
    }
    const size_t chosen = ChooseNearest(people, camera, limit, reach);

    // The palettes are written first, all of them, because the instances point into the list and it must not move afterwards.
    for (size_t i = 0; i < chosen; ++i) {
        const NpcInstance& person = people[i];
        const Body& body = bodies[PersonKey(person) % bodies.size()];
        const std::array<glm::mat4, character::kJointCount> skin = character::SkinMatrices(
            body.skeleton, WalkPose(person.walkPhase, person.stride, static_cast<float>((PersonKey(person) >> 16) & 0xFFu) / 255.0f));
        palettes.insert(palettes.end(), skin.begin(), skin.end());
    }
    for (size_t i = 0; i < chosen; ++i) {
        const NpcInstance& person = people[i];
        SkinnedInstance instance;
        instance.mesh = bodies[PersonKey(person) % bodies.size()].handle;
        instance.world = glm::scale(glm::rotate(glm::translate(glm::mat4(1.0f), person.position), person.heading, glm::vec3(0.0f, 1.0f, 0.0f)),
                                    glm::vec3(person.heightScale));
        instance.palette = std::span<const glm::mat4>(palettes).subspan(i * character::kJointCount, character::kJointCount);
        instance.tint = person.clothingColor;
        instance.seed = static_cast<float>(PersonKey(person) & 0xFFFFu) / 65536.0f;
        instances.push_back(instance);
    }
    people.erase(people.begin(), people.begin() + static_cast<std::ptrdiff_t>(chosen));  // they are drawn as characters now
    frame.characters = instances;
}

}  // namespace demo
