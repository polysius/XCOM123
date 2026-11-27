#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "character/CharacterGenerator.h"

// Small tools on triangle meshes that the skin, the garments and the hair share: a vertex that carries everything a
// skinned and textured vertex has, cutting a mesh along a field, normals, and distance and ray queries.

namespace character {

// One skinned vertex. `attribute` is a number per vertex that follows the vertex through cuts and splits (the skin tone,
// or the distance of a garment from its edge); what it means is up to the caller.
struct MeshVertex {
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    glm::vec2 uv{0.0f};
    std::array<uint8_t, 4> bones{};
    std::array<float, 4> weights{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec4 attribute{0.0f};
};

// A vertex a fraction `t` of the way from `a` to `b`. The two may follow different bones: the result follows the four
// heaviest of them.
MeshVertex Blend(const MeshVertex& a, const MeshVertex& b, float t);

// The bone with the most weight on a vertex.
int DominantBone(const MeshVertex& vertex);

struct TriangleMesh {
    std::vector<MeshVertex> vertices;
    std::vector<uint32_t> indices;
};

// The triangles of one piece of a character mesh as a mesh of its own.
TriangleMesh ExtractPiece(const CharacterMesh& mesh, size_t piece);

// Keeps what is on the positive side of `field` (one number per vertex), cutting the triangles that cross zero so that
// the new edge follows the zero line smoothly. A cut vertex is a blend of the two vertices of the edge it lies on.
// Triangles keep their winding.
TriangleMesh ClipByField(const TriangleMesh& mesh, const std::vector<float>& field);

// Normals from the triangles, each weighted by the angle at the vertex, pointing out of counter-clockwise triangles.
std::vector<glm::vec3> VertexNormals(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& indices);

// Distance and ray queries on a fixed set of triangles, with a grid of cells that hold the triangles near them.
class TriangleGrid {
public:
    struct Nearest {
        float distance = 1e9f;  // to the surface, 1e9 when no triangle is within the search radius
        // Negative inside the surface. The side is read from the normal of the surface at the nearest point, interpolated between the
        // smooth vertex normals of its triangle, which stays right where the nearest point is on an edge or a corner.
        float signedDistance = 1e9f;
        glm::vec3 point{0.0f};
        glm::vec3 faceNormal{0.0f};
        glm::vec3 surfaceNormal{0.0f, 1.0f, 0.0f};  // unit, pointing out of the surface, at `point`
        uint32_t triangle = 0;
    };

    TriangleGrid(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& indices, float cellMeters = 0.03f);

    Nearest FindNearest(glm::vec3 point, float searchRadius) const;

    // The first hit of a ray (a unit `direction`) within `maxDistance`; the distance along the ray, or a negative number
    // if nothing is hit. Triangles are hit from either side.
    float Raycast(glm::vec3 origin, glm::vec3 direction, float maxDistance) const;

private:
    void FinishNearest(Nearest& nearest, glm::vec3 point) const;
    glm::ivec3 CellOf(glm::vec3 point) const;
    size_t CellIndex(glm::ivec3 cell) const;

    const std::vector<glm::vec3>& positions;
    const std::vector<uint32_t>& indices;
    glm::vec3 origin{0.0f};
    glm::ivec3 cellCount{1};
    float cellSize = 0.03f;
    std::vector<glm::vec3> vertexNormals;  // smooth, for the side of a point
    std::vector<char> onBoundary;          // per vertex: on an edge that only one triangle has, the rim of an open surface
    std::vector<uint32_t> cellStart;      // cell -> first entry of `cellTriangles`, one extra at the end
    std::vector<uint32_t> cellTriangles;  // the triangles of every cell one after the other
};

// The closest point of the triangle abc to p.
glm::vec3 ClosestPointOnTriangle(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c);

}  // namespace character
