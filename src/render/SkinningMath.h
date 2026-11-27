#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "scene/Frustum.h"
#include "scene/SceneTypes.h"

namespace character {
struct CharacterMesh;
}

// The CPU side of drawing skinned characters, with no Vulkan in it so that tests can run it without a GPU.
//
// The skinning itself is done by shaders/skinned.vert, and SkinPosition and SkinNormal below are the same arithmetic written
// for the CPU. Keep the two in step: tests/SkinningTests.cpp checks the CPU version against the skinning of the character module,
// and the shader is a line by line copy of the CPU version.
//
// A palette is the list of skinning matrices of one character in one frame, one per joint: the matrix carries a point of the
// rest pose to its place in the pose, in the character's own space (feet on y = 0, facing +Z). The instance's world matrix
// then places that space in the world. All the palettes of a frame are packed one after the other into one buffer, and an
// instance finds its own by an offset counted in matrices.
namespace skinning {

// One vertex as the GPU reads it: where it is in the rest pose, which way it faces, and up to four joints that pull on it.
struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    std::array<uint8_t, 4> joints;
    std::array<float, 4> weights;  // add up to 1
};
static_assert(sizeof(Vertex) == 44, "skinning::Vertex must match the vertex layout of the skinned pipelines");

// The part of a character the culling needs. Every joint owns a sphere around its rest position, big enough to hold every
// vertex that the joint pulls on. A joint matrix moves its sphere rigidly, so wherever a pose puts the joints, the spheres hold
// the whole skinned body.
struct JointBounds {
    std::vector<glm::vec3> restPositions;
    std::vector<float> radii;
};

// What the renderer keeps of a mesh on the CPU once its triangles are on the GPU.
struct MeshInfo {
    JointBounds bounds;
    uint32_t jointCount = 0;  // the number of matrices in a palette of this mesh
};

// A character mesh as it is handed to the renderer.
struct MeshData {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    MeshInfo info;
};

// Turns a generated character into renderer data. The weights of each vertex are made to add up to 1 here, so that a
// vertex never ends up scaled by the shader.
MeshData BuildMeshData(const character::CharacterMesh& character);

// The vertex shader's blend, on the CPU: the weighted sum of the joint matrices, applied to the position.
glm::vec3 SkinPosition(const Vertex& vertex, std::span<const glm::mat4> palette);
// The normal turned the same way. Joint matrices are rigid, so the upper 3x3 of the blend needs no inverse transpose.
glm::vec3 SkinNormal(const Vertex& vertex, std::span<const glm::mat4> palette);

// A box with sides along the axes.
struct Box {
    glm::vec3 min{0.0f};
    glm::vec3 max{0.0f};
};

// A box around the posed character, in the character's own space.
Box AnimatedBounds(const JointBounds& bounds, std::span<const glm::mat4> palette);

// A sphere in the world around the posed character: the box above, moved by the world matrix. The world matrix may scale
// but not shear, which is all a character is placed with.
struct Sphere {
    glm::vec3 center{0.0f};
    float radius = 0.0f;
};
Sphere WorldSphere(const Box& box, const glm::mat4& world);

// What the shaders read for each instance. Matches the instance inputs of shaders/skinned.vert.
struct GpuInstance {
    glm::mat4 world;
    glm::vec3 color;
    float roughness;
    uint32_t paletteOffset;  // of the first matrix of this instance's palette, counted in matrices
    float material;          // a SurfaceMaterial, stored as float for the shader
    float seed;
    float unused = 0.0f;
};
static_assert(sizeof(GpuInstance) == 96, "skinning::GpuInstance must match the instance layout of the skinned pipelines");

// The instances of one mesh in a frame. They are stored in one list for all meshes: first the ones the camera can see, then the
// ones only the sun can see, so the scene pass draws the first `cameraCount` and the shadow pass all `totalCount`.
struct MeshRange {
    uint32_t mesh = 0;
    uint32_t firstInstance = 0;
    uint32_t cameraCount = 0;
    uint32_t totalCount = 0;
};

// Everything the GPU needs for the characters of one frame.
struct FramePlan {
    std::vector<GpuInstance> instances;
    std::vector<glm::mat4> palettes;  // every drawn instance's palette, one after the other
    std::vector<MeshRange> ranges;    // in the order of the meshes; only meshes with something to draw

    void Clear() {
        instances.clear();
        palettes.clear();
        ranges.clear();
    }
};

// Decides which of the submitted characters to draw, and lays out their instances and palettes. A character is dropped when
// neither the camera nor the sun can see it, or when it is not something that can be drawn: a handle that names no mesh, or a
// palette that does not have a matrix for every joint of its mesh (reading past the palette would pull on other characters'
// joints). `dropped`, when given, counts the ones dropped for that second reason.
void PlanFrame(std::span<const SkinnedInstance> submitted, std::span<const MeshInfo> meshes, const Frustum& camera,
               const Frustum& sun, FramePlan& plan, uint32_t* dropped = nullptr);

}  // namespace skinning
