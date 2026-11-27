#include "render/SkinningMath.h"

#include <algorithm>
#include <cmath>

#include "character/CharacterGenerator.h"

namespace skinning {

namespace {

// The blend of the joint matrices that pull on a vertex. A weight of zero leaves its joint out.
glm::mat4 BlendedMatrix(const Vertex& vertex, std::span<const glm::mat4> palette) {
    glm::mat4 blended(0.0f);
    for (int slot = 0; slot < 4; ++slot) {
        blended += vertex.weights[slot] * palette[vertex.joints[slot]];
    }
    return blended;
}

// How much a matrix scales at most: the longest of its three axes. Joint matrices are rigid, so this is 1 unless a pose scales;
// a world matrix may stretch one axis more than the others, and a sphere must follow the longest stretch to hold the character.
float MatrixScale(const glm::mat4& matrix) {
    return std::max({glm::length(glm::vec3(matrix[0])), glm::length(glm::vec3(matrix[1])), glm::length(glm::vec3(matrix[2]))});
}

// A weight list made to add up to 1. Weights that are all zero put the whole vertex on the first joint of the list.
std::array<float, 4> Normalised(const std::array<float, 4>& weights) {
    float sum = 0.0f;
    for (float weight : weights) {
        sum += std::max(weight, 0.0f);
    }
    if (sum <= 0.0f) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }
    std::array<float, 4> result{};
    for (int slot = 0; slot < 4; ++slot) {
        result[slot] = std::max(weights[slot], 0.0f) / sum;
    }
    return result;
}

// Gives each joint a sphere around its rest position that reaches every vertex the joint pulls on. This works on the vertices as
// they will be drawn, with their weights already normalised, so a vertex that had no weight at all counts for the joint it was
// given and cannot fall outside the spheres.
JointBounds MakeJointBounds(const character::CharacterMesh& character, std::span<const Vertex> vertices) {
    JointBounds bounds;
    bounds.restPositions.reserve(character::kJointCount);
    for (const character::JointInfo& joint : character.skeleton.joints) {
        bounds.restPositions.push_back(joint.restPosition);
    }
    bounds.radii.assign(bounds.restPositions.size(), 0.0f);
    for (const Vertex& vertex : vertices) {
        for (int slot = 0; slot < 4; ++slot) {
            if (vertex.weights[slot] > 0.0f && vertex.joints[slot] < bounds.radii.size()) {
                const uint8_t joint = vertex.joints[slot];
                const float distance = glm::length(vertex.position - bounds.restPositions[joint]);
                bounds.radii[joint] = std::max(bounds.radii[joint], distance);
            }
        }
    }
    return bounds;
}

}  // namespace

MeshData BuildMeshData(const character::CharacterMesh& character) {
    MeshData data;
    data.vertices.resize(character.positions.size());
    for (size_t v = 0; v < data.vertices.size(); ++v) {
        Vertex& vertex = data.vertices[v];
        vertex.position = character.positions[v];
        vertex.normal = character.normals[v];
        vertex.joints = character.boneIndices[v];
        vertex.weights = Normalised(character.boneWeights[v]);
        for (int slot = 0; slot < 4; ++slot) {
            // The shader reads every slot, so one with no weight must still name a joint of the palette.
            if (vertex.weights[slot] == 0.0f && vertex.joints[slot] >= character::kJointCount) {
                vertex.joints[slot] = 0;
            }
        }
    }
    data.indices = character.indices;
    data.info.bounds = MakeJointBounds(character, data.vertices);
    data.info.jointCount = static_cast<uint32_t>(data.info.bounds.restPositions.size());
    return data;
}

bool IsDrawable(const MeshData& mesh) {
    constexpr uint32_t kMostJoints = 256;  // a joint is named by one byte
    if (mesh.vertices.empty() || mesh.indices.empty() || mesh.indices.size() % 3 != 0 || mesh.info.jointCount == 0 ||
        mesh.info.jointCount > kMostJoints) {
        return false;
    }
    const bool indicesAreVertices = std::all_of(mesh.indices.begin(), mesh.indices.end(),
                                                [&](uint32_t index) { return index < mesh.vertices.size(); });
    const bool jointsExist = std::all_of(mesh.vertices.begin(), mesh.vertices.end(), [&](const Vertex& vertex) {
        return std::all_of(vertex.joints.begin(), vertex.joints.end(), [&](uint8_t joint) { return joint < mesh.info.jointCount; });
    });
    return indicesAreVertices && jointsExist;
}

bool LinkLowerDetail(std::span<MeshInfo> meshes, uint32_t body, uint32_t lower) {
    if (body >= meshes.size() || lower >= meshes.size() || body == lower || meshes[body].jointCount != meshes[lower].jointCount) {
        return false;
    }
    // The chain below `lower` must not lead back to `body`.
    for (uint32_t mesh = lower; mesh != CharacterHandle::kInvalid; mesh = meshes[mesh].lowerDetail) {
        if (mesh == body) {
            return false;
        }
    }
    meshes[body].lowerDetail = lower;
    return true;
}

glm::vec3 SkinPosition(const Vertex& vertex, std::span<const glm::mat4> palette) {
    return glm::vec3(BlendedMatrix(vertex, palette) * glm::vec4(vertex.position, 1.0f));
}

glm::vec3 SkinNormal(const Vertex& vertex, std::span<const glm::mat4> palette) {
    return glm::normalize(glm::mat3(BlendedMatrix(vertex, palette)) * vertex.normal);
}

Box AnimatedBounds(const JointBounds& bounds, std::span<const glm::mat4> palette) {
    Box box{glm::vec3(INFINITY), glm::vec3(-INFINITY)};
    for (size_t joint = 0; joint < bounds.restPositions.size(); ++joint) {
        const glm::vec3 center = glm::vec3(palette[joint] * glm::vec4(bounds.restPositions[joint], 1.0f));
        const glm::vec3 reach(bounds.radii[joint] * MatrixScale(palette[joint]));
        box.min = glm::min(box.min, center - reach);
        box.max = glm::max(box.max, center + reach);
    }
    return box;
}

Sphere WorldSphere(const Box& box, const glm::mat4& world) {
    const glm::vec3 localCenter = 0.5f * (box.min + box.max);
    const float localRadius = glm::length(0.5f * (box.max - box.min));
    return {glm::vec3(world * glm::vec4(localCenter, 1.0f)), localRadius * MatrixScale(world)};
}

uint32_t ChooseLevel(uint32_t previous, float distance, const LodSettings& settings) {
    const uint32_t levels = static_cast<uint32_t>(settings.switchDistances.size());
    uint32_t level = std::min(previous, levels);
    while (level < levels && distance > settings.switchDistances[level] * (1.0f + settings.hysteresis)) {
        ++level;
    }
    while (level > 0 && distance < settings.switchDistances[level - 1] * (1.0f - settings.hysteresis)) {
        --level;
    }
    return level;
}

uint32_t MeshAtLevel(std::span<const MeshInfo> meshes, uint32_t mesh, uint32_t level) {
    for (; level > 0 && meshes[mesh].lowerDetail != CharacterHandle::kInvalid; --level) {
        mesh = meshes[mesh].lowerDetail;
    }
    return mesh;
}

bool ShadowCanTouchView(const Sphere& body, float feetHeight, const glm::vec3& towardsSun, const Frustum& camera) {
    constexpr float kLongestShadowMeters = 60.0f;
    const float height = std::max(body.center.y + body.radius - feetHeight, 0.0f);
    const float length = std::min(height / std::max(towardsSun.y, 0.1f), kLongestShadowMeters);
    const glm::vec3 middleOfShadow = body.center - towardsSun * (0.5f * length);
    return camera.ContainsSphere(middleOfShadow, body.radius + 0.5f * length);
}

namespace {

GpuInstance MakeGpuInstance(const SkinnedInstance& submitted, uint32_t paletteOffset) {
    GpuInstance instance{};
    instance.world = submitted.world;
    instance.color = submitted.tint;
    instance.roughness = submitted.roughness;
    instance.paletteOffset = paletteOffset;
    instance.material = static_cast<float>(submitted.material);
    instance.seed = submitted.seed;
    return instance;
}

// Lays out the instances of one pass: sorted by mesh, with one range for each mesh. `meshOf` says which mesh a pick has in this pass.
template <typename MeshOf>
void LayOutPass(std::span<const SkinnedInstance> submitted, std::vector<Pick>& picks, MeshOf meshOf, FramePlan& plan,
                std::vector<MeshRange>& ranges) {
    // Ties are broken by the submitted order, so the result does not depend on how the sort goes about it (and no memory is needed).
    std::sort(picks.begin(), picks.end(), [&](const Pick& a, const Pick& b) {
        return meshOf(a) != meshOf(b) ? meshOf(a) < meshOf(b) : a.submittedIndex < b.submittedIndex;
    });
    for (const Pick& pick : picks) {
        const uint32_t mesh = meshOf(pick);
        if (mesh == kNoMesh) {
            break;  // the ones with no mesh in this pass sort last
        }
        if (ranges.empty() || ranges.back().mesh != mesh) {
            ranges.push_back({mesh, static_cast<uint32_t>(plan.instances.size()), 0});
        }
        ++ranges.back().count;
        plan.instances.push_back(MakeGpuInstance(submitted[pick.submittedIndex], pick.paletteOffset));
    }
}

}  // namespace

void PlanFrame(std::span<const SkinnedInstance> submitted, std::span<const MeshInfo> meshes, const FrameView& view, FramePlan& plan,
               uint32_t* dropped) {
    plan.Clear();
    plan.lodLevels.resize(submitted.size(), 0);

    for (uint32_t index = 0; index < submitted.size(); ++index) {
        const SkinnedInstance& instance = submitted[index];
        if (instance.mesh.index >= meshes.size()) {
            continue;
        }
        const MeshInfo& body = meshes[instance.mesh.index];
        if (instance.palette.size() < body.jointCount) {
            if (dropped != nullptr) {
                ++*dropped;
            }
            continue;
        }
        const Sphere sphere = WorldSphere(AnimatedBounds(body.bounds, instance.palette), instance.world);
        const float distance = glm::length(sphere.center - view.cameraPosition);
        const bool seenByCamera = view.camera.ContainsSphere(sphere.center, sphere.radius);
        const bool castsShadow = distance <= view.lod.shadowDistance && view.sun.ContainsSphere(sphere.center, sphere.radius) &&
                                 ShadowCanTouchView(sphere, instance.world[3][1], view.towardsSun, view.camera);
        if (!seenByCamera && !castsShadow) {
            continue;
        }

        const uint32_t level = ChooseLevel(plan.lodLevels[index], distance, view.lod);
        plan.lodLevels[index] = static_cast<uint8_t>(level);
        Pick pick;
        pick.submittedIndex = index;
        pick.paletteOffset = static_cast<uint32_t>(plan.palettes.size());
        pick.cameraMesh = seenByCamera ? MeshAtLevel(meshes, instance.mesh.index, level) : kNoMesh;
        // The shadow of a character is soft and small, so it is cast by a mesh one level coarser than the one that is seen.
        pick.sunMesh = castsShadow ? MeshAtLevel(meshes, instance.mesh.index, level + 1) : kNoMesh;
        plan.picks.push_back(pick);
        plan.palettes.insert(plan.palettes.end(), instance.palette.begin(), instance.palette.begin() + body.jointCount);
    }

    LayOutPass(submitted, plan.picks, [](const Pick& pick) { return pick.cameraMesh; }, plan, plan.cameraRanges);
    LayOutPass(submitted, plan.picks, [](const Pick& pick) { return pick.sunMesh; }, plan, plan.sunRanges);
}

}  // namespace skinning
