#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "character/CharacterParameters.h"
#include "character/Skeleton.h"

class JobSystem;

// A character as a skinned triangle mesh with its skeleton, made from nothing but the parameters. The surface is a
// body of smoothly merged parts (see BodyShape.h) turned into triangles with marching tetrahedra, and the girth of the body is
// solved so that the volume of the mesh, at the density of a living body, comes out at the requested mass.
//
// Each vertex follows up to four bones with weights that add up to 1. The weights come from the distance to the parts
// of the bones near the vertex, and only a bone, its parent and its children may share a vertex, so that the torso
// never pulls on the inside of an upper arm. In the rest pose the skinned mesh is the mesh itself.

namespace character {

struct MeshSettings {
    float voxelMeters = 0.024f;  // the size of the grid the body is cut from, 6 mm to 4 cm; 2.4 cm gives 16000 to 50000 triangles of body
    // The head and the hands are cut from finer grids, as pieces of their own (see DetailShapes.h). Without them the body
    // is made whole, with a plain head and mitten hands.
    bool faceAndHands = true;
    float headVoxelMeters = 0.005f;
    float handVoxelMeters = 0.005f;
};

// The surfaces that a character is made of. They overlap a little at the neck and the wrists, and each is closed.
enum class MeshPieceKind : uint8_t {
    Body,  // the trunk and limbs, with the head and the hands too when the character has no finer ones
    Head,
    HandLeft,
    HandRight,
};

// Where a piece lies in the vertex and index lists of the mesh. The indices of a piece count from the first vertex of the
// whole mesh, not of the piece.
struct MeshPiece {
    MeshPieceKind kind = MeshPieceKind::Body;
    uint32_t firstVertex = 0;
    uint32_t vertexCount = 0;
    uint32_t firstIndex = 0;
    uint32_t indexCount = 0;
};

struct CharacterMesh {
    CharacterParameters parameters;  // the sanitised parameters the character was made from
    Skeleton skeleton;
    std::vector<glm::vec3> positions;  // in the rest pose, standing on y = 0
    std::vector<glm::vec3> normals;
    std::vector<std::array<uint8_t, 4>> boneIndices;
    std::vector<std::array<float, 4>> boneWeights;
    std::vector<uint32_t> indices;  // three to a triangle, counter-clockwise seen from outside
    std::vector<MeshPiece> pieces;  // the closed surfaces the lists above are made of, in order
    float girth = 1.0f;             // the factor on the radii of the trunk and limbs that gave the mass
    float volumeCubicMeters = 0.0f;
    // The volume at the density of a living body. The head and the hands overlap the body a little at the neck and the wrists,
    // and the overlap is counted twice, which is less than 1 percent of the mass.
    float massKg = 0.0f;
};

// Makes a character. `jobs` may be null, which runs on the calling thread; the result is the same either way.
CharacterMesh GenerateCharacter(const CharacterParameters& parameters, const MeshSettings& settings = {}, JobSystem* jobs = nullptr);

// The positions of the vertices in a pose, by linear blend skinning.
std::vector<glm::vec3> SkinPositions(const CharacterMesh& mesh, const Pose& pose);

// The shape of one piece of a mesh as a surface. A piece is a closed sphere: no edge in other than two triangles, one
// connected surface, and an Euler characteristic (vertices minus edges plus triangles) of 2. A handle, a tunnel
// through it, makes it 0.
struct MeshTopology {
    int badEdges = 0;
    int pieces = 0;
    long long eulerCharacteristic = 0;

    bool IsClosedSphere() const { return badEdges == 0 && pieces == 1 && eulerCharacteristic == 2; }
};
MeshTopology MeasureTopology(const CharacterMesh& mesh, size_t piece);

// The signed volume enclosed by a closed triangle mesh: positive when the triangles face outwards.
float MeshVolume(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& indices);

}  // namespace character
