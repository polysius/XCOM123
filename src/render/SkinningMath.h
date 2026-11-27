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
//
// Level of detail. A body may have lower-detail meshes of the same skeleton, linked as a chain (full, coarser, coarsest). Which
// level a character is drawn at follows its distance to the camera, and the shadow pass draws one level coarser than the camera
// does, and nothing at all for characters whose shadow cannot reach the view.
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
    uint32_t lowerDetail = CharacterHandle::kInvalid;  // index of the next coarser mesh of the same body, if there is one
};

// A character mesh as it is handed to the renderer.
struct MeshData {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    MeshInfo info;
};

// Turns a generated character into renderer data. The weights of each vertex are made to add up to 1 here, so that a
// vertex never ends up scaled by the shader, and a slot with no weight is pointed at joint 0 so that it names a joint that exists.
MeshData BuildMeshData(const character::CharacterMesh& character);

// Whether a mesh can be drawn safely: it has triangles, every index names a vertex, the palette is not larger than a joint
// byte can name, and every joint slot of every vertex, including the ones with no weight (the shader reads all four), names a
// joint of the palette.
bool IsDrawable(const MeshData& mesh);

// Makes `lower` the next coarser mesh of `body`. Refused (false) when either is not a mesh, when they are the same mesh, when
// their palettes differ in size (they would not share a skeleton), or when it would close a loop in the chain.
bool LinkLowerDetail(std::span<MeshInfo> meshes, uint32_t body, uint32_t lower);

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

// A sphere in the world around the posed character: the box above, moved by the world matrix. The radius follows the largest
// scale of the matrix, so a character stretched along one axis is still held. The world matrix may not shear.
struct Sphere {
    glm::vec3 center{0.0f};
    float radius = 0.0f;
};
Sphere WorldSphere(const Box& box, const glm::mat4& world);

// How level of detail is chosen.
struct LodSettings {
    // Distances in metres from the camera at which a character goes to the next coarser level.
    std::array<float, 2> switchDistances{25.0f, 70.0f};
    // A character goes back to a finer level only once it is this fraction closer than the distance it went coarse at, so that
    // one standing on the edge does not flicker between two meshes.
    float hysteresis = 0.1f;
    // Characters farther than this from the camera cast no shadow at all.
    float shadowDistance = 90.0f;
};

// The level for a character at `distance` that was drawn at level `previous` the frame before.
uint32_t ChooseLevel(uint32_t previous, float distance, const LodSettings& settings);

// The mesh `level` steps down the chain from `mesh`, or the end of the chain if it is shorter than that.
uint32_t MeshAtLevel(std::span<const MeshInfo> meshes, uint32_t mesh, uint32_t level);

// Whether the shadow of a character could fall within the camera's view. The shadow runs from the character along the direction
// away from the sun until it reaches the height of the character's feet; everything it can touch lies within a sphere around
// the middle of that run, and the shadow of a very low sun is cut off at 60 m.
bool ShadowCanTouchView(const Sphere& body, float feetHeight, const glm::vec3& towardsSun, const Frustum& camera);

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

// A run of instances in the plan that are drawn with one mesh.
struct MeshRange {
    uint32_t mesh = 0;
    uint32_t firstInstance = 0;
    uint32_t count = 0;
};

// Where a drawn character stands in the lists, before they are laid out. Kept in the plan so that it allocates only once.
struct Pick {
    uint32_t submittedIndex = 0;
    uint32_t paletteOffset = 0;
    uint32_t cameraMesh = 0;  // the mesh the scene pass draws, or kNoMesh
    uint32_t sunMesh = 0;     // the mesh the shadow pass draws, or kNoMesh
};
constexpr uint32_t kNoMesh = 0xFFFFFFFFu;

// Everything the GPU needs for the characters of one frame.
struct FramePlan {
    std::vector<GpuInstance> instances;  // the instances of the scene pass, then those of the shadow pass
    std::vector<glm::mat4> palettes;     // every drawn character's palette once, whichever passes draw it
    std::vector<MeshRange> cameraRanges; // what the scene pass draws, one range for each mesh
    std::vector<MeshRange> sunRanges;    // what the shadow pass draws
    std::vector<Pick> picks;
    // The level each submitted character was drawn at last frame, by position in the submitted list. It is what makes the
    // choice of level steady, so callers should submit characters in the same order from one frame to the next.
    std::vector<uint8_t> lodLevels;

    void Clear() {
        instances.clear();
        palettes.clear();
        cameraRanges.clear();
        sunRanges.clear();
        picks.clear();
    }
};

// What PlanFrame needs to know about the view.
struct FrameView {
    const Frustum& camera;
    const Frustum& sun;
    glm::vec3 cameraPosition{0.0f};
    glm::vec3 towardsSun{0.0f, 1.0f, 0.0f};  // unit vector
    LodSettings lod;
};

// Decides which of the submitted characters to draw, with which mesh in which pass, and lays out their instances and palettes.
// A character is dropped when neither the camera nor its shadow can be seen, or when it is not something that can be drawn: a
// handle that names no mesh, or a palette that does not have a matrix for every joint of its mesh (reading past the palette
// would pull on other characters' joints). `dropped`, when given, counts the ones dropped for that second reason.
void PlanFrame(std::span<const SkinnedInstance> submitted, std::span<const MeshInfo> meshes, const FrameView& view, FramePlan& plan,
               uint32_t* dropped = nullptr);

}  // namespace skinning
