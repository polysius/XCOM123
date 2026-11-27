#include "demo/CharacterShowcase.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <iterator>
#include <span>

#include <glm/gtc/matrix_transform.hpp>

#include "character/CharacterGenerator.h"
#include "character/CharacterParameters.h"
#include "core/JobSystem.h"

namespace demo {

namespace {

constexpr uint32_t kDistinctBodies = 4;
constexpr float kSpacingMeters = 1.7f;
constexpr float kEyeHeightMeters = 1.6f;
// Grid sizes of the coarser meshes of each body, for the middle and the far distance (the full mesh uses 0.024).
constexpr float kCoarserVoxelMeters[] = {0.05f, 0.10f};

// Skin tones from light to dark. The character shader takes one colour for the whole body, so this is the colour of everything.
const glm::vec3 kTints[] = {{0.80f, 0.60f, 0.46f}, {0.62f, 0.44f, 0.33f}, {0.45f, 0.30f, 0.22f}, {0.72f, 0.52f, 0.40f}};

// The tallest thing a character may stand on. Trees and roofs are above this.
constexpr float kHighestFloorMeters = 3.0f;

// The boxes of the city that reach within `radius` metres of a point, so that finding the floor under each character does not
// have to look at every box of the city.
std::vector<size_t> BoxesNear(const CityLayout& city, glm::vec2 point, float radius) {
    std::vector<size_t> found;
    for (size_t i = 0; i < city.boxes.size(); ++i) {
        const BoxInstance& box = city.boxes[i];
        const float reach = radius + glm::length(glm::vec2(box.halfSize.x, box.halfSize.z));
        if (glm::length(glm::vec2(box.center.x, box.center.z) - point) <= reach) {
            found.push_back(i);
        }
    }
    return found;
}

// The height of the floor under a point: the top of the highest solid thing there that is not too tall to stand on.
float GroundHeightAt(const CityLayout& city, std::span<const size_t> candidates, glm::vec2 point) {
    float height = 0.0f;
    for (const size_t i : candidates) {
        const BoxInstance& box = city.boxes[i];
        const float top = box.center.y + box.halfSize.y;
        if (city.boxParts[i] == CityPart::Water || top > kHighestFloorMeters || top <= height) {
            continue;
        }
        const glm::vec2 offset = point - glm::vec2(box.center.x, box.center.z);
        const float cosine = std::cos(box.yaw);
        const float sine = std::sin(box.yaw);
        const float alongX = offset.x * cosine - offset.y * sine;
        const float alongZ = offset.x * sine + offset.y * cosine;
        if (std::abs(alongX) <= box.halfSize.x && std::abs(alongZ) <= box.halfSize.z) {
            height = top;
        }
    }
    return height;
}

// The largest square of the city proper. The squares of the satellite districts are left out: they may stand on terraces at other
// heights, and the camera of this demo is meant to look at the city around the characters.
OrientedRect LargestSquare(const CityLayout& city) {
    const OrientedRect* best = nullptr;
    for (const OrientedRect& plaza : city.plazas) {
        const glm::vec2 centre = plaza.Center();
        const bool inCore = centre.x >= city.coreBounds.min.x && centre.x <= city.coreBounds.max.x &&
                            centre.y >= city.coreBounds.min.y && centre.y <= city.coreBounds.max.y;
        if (inCore && (best == nullptr || plaza.Area() > best->Area())) {
            best = &plaza;
        }
    }
    return best != nullptr ? *best : OrientedRect(Rect{{-20.0f, -20.0f}, {20.0f, 20.0f}});
}

// A camera in front of the grid (the characters face +Z, so it stands on the +Z side and looks along -Z), far enough back to
// see the whole width of the grid with the 60 degree field of view of the fly camera.
CameraPlacement ViewOfGrid(glm::vec3 gridCentre, float gridWidth, float gridDepth) {
    const float halfWidthOfView = std::tan(glm::radians(60.0f) * 0.5f) * 16.0f / 9.0f;
    const float distance = 0.5f * gridWidth / halfWidthOfView + 0.5f * gridDepth + 3.0f;
    CameraPlacement view;
    view.position = gridCentre + glm::vec3(0.0f, kEyeHeightMeters - 0.2f, distance);
    view.yawDegrees = 180.0f;
    view.pitchDegrees = -4.0f;
    return view;
}

// Makes the same person twice more from coarser grids, without the fine head and hands (which hold most of the triangles), and
// chains them below the full mesh. The skeleton depends on the parameters alone, so all three share it.
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

}  // namespace

character::Pose IdleSwayPose(const character::Skeleton& skeleton, float seconds, uint32_t characterIndex) {
    using character::Joint;
    // Every character is a little out of step with the next, so that the crowd does not move as one.
    const float t = seconds + 0.37f * static_cast<float>(characterIndex);
    const float breath = std::sin(t * 1.6f);

    character::Pose pose;
    pose.rootTranslation.y = -0.012f + 0.006f * breath;
    pose.rotation[character::Index(Joint::Spine1)] = character::FromDegrees(1.5f * breath, 3.0f * std::sin(t * 0.7f), 3.0f * std::sin(t * 0.9f));
    pose.rotation[character::Index(Joint::Spine2)] = character::FromDegrees(1.0f * breath, 4.0f * std::sin(t * 0.7f), 2.0f * std::sin(t * 0.9f));
    pose.rotation[character::Index(Joint::Head)] = character::FromDegrees(4.0f * std::sin(t * 1.1f), 14.0f * std::sin(t * 0.6f), 0.0f);

    // The left elbow bends and straightens, and the right arm lifts away from the body and back.
    pose.rotation[character::Index(Joint::ForearmL)] = character::HingeRotation(skeleton, Joint::ForearmL, 55.0f + 40.0f * std::sin(t * 1.3f));
    pose.rotation[character::Index(Joint::ForearmR)] = character::HingeRotation(skeleton, Joint::ForearmR, 25.0f);
    pose.rotation[character::Index(Joint::UpperArmR)] = character::FromDegrees(0.0f, 0.0f, 25.0f + 25.0f * std::sin(t * 1.3f + 1.0f));

    // The knees give a little with every breath.
    const float kneeBend = 5.0f + 3.0f * breath;
    pose.rotation[character::Index(Joint::CalfL)] = character::FromDegrees(kneeBend, 0.0f, 0.0f);
    pose.rotation[character::Index(Joint::CalfR)] = character::FromDegrees(kneeBend, 0.0f, 0.0f);
    pose.rotation[character::Index(Joint::ThighL)] = character::FromDegrees(-0.5f * kneeBend, 0.0f, 0.0f);
    pose.rotation[character::Index(Joint::ThighR)] = character::FromDegrees(-0.5f * kneeBend, 0.0f, 0.0f);
    return pose;
}

void CharacterShowcase::Spawn(IRenderModule& renderer, const CityLayout& city, uint32_t count, uint32_t seed) {
    bodies.clear();
    placements.clear();
    viewPoint.reset();
    if (count == 0) {
        return;
    }

    const uint32_t distinct = std::min(count, kDistinctBodies);
    for (uint32_t i = 0; i < distinct; ++i) {
        const character::CharacterParameters parameters = character::RandomParameters(seed * 131u + i);
        const character::CharacterMesh mesh = character::GenerateCharacter(parameters, {}, &JobSystem::Shared());
        Body body;
        body.skeleton = mesh.skeleton;
        const skinning::MeshData data = skinning::BuildMeshData(mesh);
        body.jointCount = data.info.jointCount;
        body.handle = renderer.UploadCharacterMesh(data);
        if (!body.handle.IsValid()) {
            std::cerr << "The renderer did not accept character body " << i << "\n";
            continue;
        }
        UploadCoarserVersions(renderer, parameters, body.handle);
        bodies.push_back(body);
    }
    if (bodies.empty()) {
        return;
    }

    // A grid of characters, wider than deep, in the largest square and facing +Z. Squares often have a fountain or a monument in the
    // middle, so the grid stands in front of it, towards the camera.
    const OrientedRect square = LargestSquare(city);
    const glm::vec2 centre = square.Center() + glm::vec2(0.0f, 0.3f * square.Size().y);
    const uint32_t columns = std::max(1u, static_cast<uint32_t>(std::ceil(std::sqrt(1.6f * static_cast<float>(count)))));
    const uint32_t rows = (count + columns - 1) / columns;
    const float gridRadius = kSpacingMeters * 0.5f * static_cast<float>(std::max(columns, rows) + 1) * 1.5f;
    const std::vector<size_t> nearby = BoxesNear(city, centre, gridRadius);
    const float ground = GroundHeightAt(city, nearby, centre);  // for the camera
    for (uint32_t i = 0; i < count; ++i) {
        const float column = static_cast<float>(i % columns) - 0.5f * static_cast<float>(columns - 1);
        const float row = static_cast<float>(i / columns) - 0.5f * static_cast<float>(rows - 1);
        const glm::vec2 spot(centre.x + column * kSpacingMeters, centre.y - row * kSpacingMeters);
        Placement placement;
        placement.body = i % static_cast<uint32_t>(bodies.size());
        placement.world = glm::translate(glm::mat4(1.0f), glm::vec3(spot.x, GroundHeightAt(city, nearby, spot), spot.y));
        placement.tint = kTints[(i / bodies.size() + i) % std::size(kTints)];
        placements.push_back(placement);
    }
    viewPoint = ViewOfGrid({centre.x, ground, centre.y}, static_cast<float>(columns) * kSpacingMeters, static_cast<float>(rows) * kSpacingMeters);
    std::cout << "Characters: " << count << " standing in the largest square; view them with --camera "
              << viewPoint->position.x << "," << viewPoint->position.y << "," << viewPoint->position.z << "," << viewPoint->yawDegrees
              << "," << viewPoint->pitchDegrees << "\n";
}

void CharacterShowcase::Update(float worldSeconds, FrameInput& frame) {
    if (placements.empty()) {
        return;
    }
    // The palettes are written first, all of them, because the instances point into the list and it must not move afterwards.
    palettes.clear();
    paletteStart.clear();
    for (uint32_t i = 0; i < placements.size(); ++i) {
        const Body& body = bodies[placements[i].body];
        const character::Pose pose = poseProvider(body.skeleton, worldSeconds, i);
        const std::array<glm::mat4, character::kJointCount> skin = character::SkinMatrices(body.skeleton, pose);
        paletteStart.push_back(palettes.size());
        palettes.insert(palettes.end(), skin.begin(), skin.end());
    }

    instances.clear();
    for (uint32_t i = 0; i < placements.size(); ++i) {
        const Placement& placement = placements[i];
        SkinnedInstance instance;
        instance.mesh = bodies[placement.body].handle;
        instance.world = placement.world;
        instance.palette = std::span<const glm::mat4>(palettes).subspan(paletteStart[i], character::kJointCount);
        instance.tint = placement.tint;
        instance.seed = static_cast<float>(i) * 0.37f;
        instances.push_back(instance);
    }
    frame.characters = instances;
}

}  // namespace demo
