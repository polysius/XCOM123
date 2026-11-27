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

// How much a joint matrix scales, taken from its first column. Joint matrices are rigid, so this is 1 unless the pose scales.
float MatrixScale(const glm::mat4& matrix) {
    return glm::length(glm::vec3(matrix[0]));
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

// Gives each joint a sphere around its rest position that reaches every vertex the joint pulls on.
JointBounds MakeJointBounds(const character::CharacterMesh& character) {
    JointBounds bounds;
    bounds.restPositions.reserve(character::kJointCount);
    for (const character::JointInfo& joint : character.skeleton.joints) {
        bounds.restPositions.push_back(joint.restPosition);
    }
    bounds.radii.assign(bounds.restPositions.size(), 0.0f);
    for (size_t v = 0; v < character.positions.size(); ++v) {
        for (int slot = 0; slot < 4; ++slot) {
            if (character.boneWeights[v][slot] > 0.0f) {
                const uint8_t joint = character.boneIndices[v][slot];
                const float distance = glm::length(character.positions[v] - bounds.restPositions[joint]);
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
    }
    data.indices = character.indices;
    data.info.bounds = MakeJointBounds(character);
    data.info.jointCount = static_cast<uint32_t>(data.info.bounds.restPositions.size());
    return data;
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

namespace {

// The place of an instance in the lists of its mesh, once it is known to be drawn.
struct Chosen {
    uint32_t submittedIndex = 0;
    uint32_t mesh = 0;
    bool seenByCamera = false;
};

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

}  // namespace

void PlanFrame(std::span<const SkinnedInstance> submitted, std::span<const MeshInfo> meshes, const Frustum& camera,
               const Frustum& sun, FramePlan& plan, uint32_t* dropped) {
    plan.Clear();

    std::vector<Chosen> chosen;
    chosen.reserve(submitted.size());
    for (uint32_t index = 0; index < submitted.size(); ++index) {
        const SkinnedInstance& instance = submitted[index];
        if (instance.mesh.index >= meshes.size()) {
            continue;
        }
        const MeshInfo& mesh = meshes[instance.mesh.index];
        if (instance.palette.size() < mesh.jointCount) {
            if (dropped != nullptr) {
                ++*dropped;
            }
            continue;
        }
        const Sphere sphere = WorldSphere(AnimatedBounds(mesh.bounds, instance.palette), instance.world);
        const bool seenByCamera = camera.ContainsSphere(sphere.center, sphere.radius);
        if (seenByCamera || sun.ContainsSphere(sphere.center, sphere.radius)) {
            chosen.push_back({index, instance.mesh.index, seenByCamera});
        }
    }

    // One range per mesh, with the instances the camera sees first. The sort is stable, so submitted order is kept within a group.
    std::stable_sort(chosen.begin(), chosen.end(), [](const Chosen& a, const Chosen& b) {
        return a.mesh != b.mesh ? a.mesh < b.mesh : a.seenByCamera > b.seenByCamera;
    });

    for (const Chosen& pick : chosen) {
        const SkinnedInstance& instance = submitted[pick.submittedIndex];
        const uint32_t jointCount = meshes[pick.mesh].jointCount;
        if (plan.ranges.empty() || plan.ranges.back().mesh != pick.mesh) {
            plan.ranges.push_back({pick.mesh, static_cast<uint32_t>(plan.instances.size()), 0, 0});
        }
        MeshRange& range = plan.ranges.back();
        ++range.totalCount;
        range.cameraCount += pick.seenByCamera ? 1 : 0;

        plan.instances.push_back(MakeGpuInstance(instance, static_cast<uint32_t>(plan.palettes.size())));
        plan.palettes.insert(plan.palettes.end(), instance.palette.begin(), instance.palette.begin() + jointCount);
    }
}

}  // namespace skinning
