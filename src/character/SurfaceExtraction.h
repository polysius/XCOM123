#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

class JobSystem;

// Turning an implicit shape (a function that is negative inside, positive outside and about zero on the surface) into a
// triangle mesh, for the body, the head and the hands alike.

namespace character {

class ImplicitShape {
public:
    virtual ~ImplicitShape() = default;
    // The signed distance (approximately) from the surface.
    virtual float Distance(glm::vec3 point) const = 0;
    // A box that holds the whole shape.
    virtual glm::vec3 BoundsMin() const = 0;
    virtual glm::vec3 BoundsMax() const = 0;
};

struct Surface {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<uint32_t> indices;  // three to a triangle, counter-clockwise seen from outside
};

// How the grid is cut into triangles.
enum class CuttingMethod {
    // Marching tetrahedra: about nine triangles for every square of surface a cell wide, and always a closed surface
    // in which every edge has exactly two triangles, however thin the shape is. For shapes that are thinner than about
    // two cells.
    Tetrahedra,
    // Surface nets: one vertex per cell the surface passes through, a third of the triangles and better shaped ones, but
    // two sheets in one cell make a bad edge. For shapes that are thicker than about three cells.
    SurfaceNets,
};

// The volume of the shape found by counting the cells of a grid of 2.5 cm whose centre is inside it. Quick, and good to
// a percent or two.
float CoarseVolume(const ImplicitShape& shape, JobSystem* jobs);

// Cuts the surface from the shape on a grid of `voxel` metres. If the grid did not catch it well (a thin gap that
// became a tunnel, a part that broke off) the cut is tried again on a grid moved by a fraction of a cell, up to a few
// times, and the first sound result is kept, else the least bad. The pieces of the surface that are only specks are
// dropped. `jobs` may be null; the result is the same either way.
Surface ExtractSoundSurface(const ImplicitShape& shape, float voxel, CuttingMethod method, JobSystem* jobs);

}  // namespace character
