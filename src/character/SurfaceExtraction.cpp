#include "character/SurfaceExtraction.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "character/ParallelEach.h"
#include "core/JobSystem.h"

namespace character {
namespace {

// The grid the surface is cut from: nodes at `origin + (i, j, k) * voxel`.
struct Grid {
    glm::vec3 origin{0.0f};
    float voxel = 0.01f;
    int nx = 0;
    int ny = 0;
    int nz = 0;

    size_t NodeIndex(int i, int j, int k) const { return (static_cast<size_t>(k) * ny + j) * nx + i; }
    size_t CellIndex(int i, int j, int k) const { return (static_cast<size_t>(k) * (ny - 1) + j) * (nx - 1) + i; }
    glm::vec3 NodePosition(int i, int j, int k) const { return origin + glm::vec3(i, j, k) * voxel; }
};

// `shift` moves the grid by a fraction of a cell (0 to 1 in each direction).
Grid MakeGrid(const ImplicitShape& shape, float voxel, glm::vec3 shift) {
    constexpr int kPadding = 3;  // empty nodes all round, so that the surface never touches the edge of the grid
    Grid grid;
    grid.voxel = voxel;
    grid.origin = shape.BoundsMin() - glm::vec3(voxel * kPadding) - shift * voxel;
    const glm::vec3 size = shape.BoundsMax() - shape.BoundsMin();
    grid.nx = static_cast<int>(std::ceil(size.x / voxel)) + 2 * kPadding + 1;
    grid.ny = static_cast<int>(std::ceil(size.y / voxel)) + 2 * kPadding + 1;
    grid.nz = static_cast<int>(std::ceil(size.z / voxel)) + 2 * kPadding + 1;
    return grid;
}

glm::vec3 Gradient(const ImplicitShape& shape, glm::vec3 point) {
    constexpr float kStep = 0.0015f;
    return glm::vec3(shape.Distance(point + glm::vec3(kStep, 0, 0)) - shape.Distance(point - glm::vec3(kStep, 0, 0)),
                     shape.Distance(point + glm::vec3(0, kStep, 0)) - shape.Distance(point - glm::vec3(0, kStep, 0)),
                     shape.Distance(point + glm::vec3(0, 0, kStep)) - shape.Distance(point - glm::vec3(0, 0, kStep))) /
           (2.0f * kStep);
}

// The volume of the body found by counting the cells of a coarse grid whose centre is inside it. Quick, and good to a
// percent or two, which is enough to find the girth that gives a mass.
float CoarseVolumeOf(const ImplicitShape& shape, JobSystem* jobs) {
    constexpr float kVoxel = 0.025f;
    const glm::vec3 low = shape.BoundsMin();
    const glm::vec3 size = shape.BoundsMax() - low;
    const int nx = static_cast<int>(std::ceil(size.x / kVoxel));
    const int ny = static_cast<int>(std::ceil(size.y / kVoxel));
    const int nz = static_cast<int>(std::ceil(size.z / kVoxel));
    std::vector<int> insidePerRow(static_cast<size_t>(ny) * nz, 0);
    ParallelEach(jobs, ny * nz, [&](int row) {
        const int j = row % ny;
        const int k = row / ny;
        int count = 0;
        for (int i = 0; i < nx; ++i) {
            const glm::vec3 centre = low + (glm::vec3(i, j, k) + 0.5f) * kVoxel;
            count += shape.Distance(centre) < 0.0f ? 1 : 0;
        }
        insidePerRow[row] = count;
    });
    long long inside = 0;
    for (const int count : insidePerRow) {
        inside += count;
    }
    return static_cast<float>(inside) * kVoxel * kVoxel * kVoxel;
}

// Marching tetrahedra: every cube of the grid is cut into six tetrahedra that share the cube's long diagonal, and the
// surface crosses each tetrahedron in nothing, one triangle or two. The cuts of two neighbouring cubes agree on the
// face they share, and a vertex is made once for each edge of the grid it sits on, so the triangles always join up into
// a closed surface in which every edge has exactly two triangles, however thin the shape is. Each vertex is then moved
// onto the true surface of the function, and its normal is the gradient there. Cubes alternate between two mirrored
// ways of cutting them into five tetrahedra, which still agree on every shared face; a single way of cutting would give
// a thin part pointing across its diagonals a worse hold than one pointing along them.
Surface CutTetrahedra(const Grid& grid, const std::vector<float>& values) {
    // The corners of a cube are numbered by their bits: 1 for +x, 2 for +y, 4 for +z. A cube is cut into four tetrahedra at
    // its corners and one in the middle; the cubes of odd position are cut the mirrored way (corner c becomes c ^ 1).
    static const int kTetrahedra[5][4] = {{0, 1, 2, 4}, {3, 1, 2, 7}, {5, 1, 4, 7}, {6, 2, 4, 7}, {1, 2, 4, 7}};

    Surface surface;
    std::unordered_map<uint64_t, uint32_t> edgeVertex;  // the vertex on the edge between two nodes, by their indices
    const auto vertexOnEdge = [&](size_t nodeA, glm::vec3 positionA, float valueA, size_t nodeB, glm::vec3 positionB, float valueB) {
        if (nodeA > nodeB) {
            std::swap(nodeA, nodeB);
            std::swap(positionA, positionB);
            std::swap(valueA, valueB);
        }
        const uint64_t key = (static_cast<uint64_t>(nodeA) << 32) | static_cast<uint64_t>(nodeB);
        const auto found = edgeVertex.find(key);
        if (found != edgeVertex.end()) {
            return found->second;
        }
        const float t = valueA / (valueA - valueB);
        const uint32_t id = static_cast<uint32_t>(surface.positions.size());
        surface.positions.push_back(positionA + (positionB - positionA) * t);
        edgeVertex.emplace(key, id);
        return id;
    };

    for (int k = 0; k + 1 < grid.nz; ++k) {
        for (int j = 0; j + 1 < grid.ny; ++j) {
            for (int i = 0; i + 1 < grid.nx; ++i) {
                size_t node[8];
                glm::vec3 position[8];
                float value[8];
                int inside = 0;
                for (int c = 0; c < 8; ++c) {
                    const int ci = i + (c & 1);
                    const int cj = j + ((c >> 1) & 1);
                    const int ck = k + ((c >> 2) & 1);
                    node[c] = grid.NodeIndex(ci, cj, ck);
                    position[c] = grid.NodePosition(ci, cj, ck);
                    value[c] = values[node[c]];
                    inside += value[c] < 0.0f ? 1 : 0;
                }
                if (inside == 0 || inside == 8) {
                    continue;
                }
                const int mirror = ((i + j + k) & 1) != 0 ? 1 : 0;
                for (const auto& cutTetrahedron : kTetrahedra) {
                    int tetrahedron[4];
                    for (int corner = 0; corner < 4; ++corner) {
                        tetrahedron[corner] = cutTetrahedron[corner] ^ mirror;
                    }
                    int in[4];
                    int out[4];
                    int insideCount = 0;
                    int outsideCount = 0;
                    for (int corner = 0; corner < 4; ++corner) {
                        const int c = tetrahedron[corner];
                        if (value[c] < 0.0f) {
                            in[insideCount++] = c;
                        } else {
                            out[outsideCount++] = c;
                        }
                    }
                    if (insideCount == 0 || insideCount == 4) {
                        continue;
                    }
                    const auto edge = [&](int a, int b) { return vertexOnEdge(node[a], position[a], value[a], node[b], position[b], value[b]); };
                    uint32_t triangle[2][3];
                    int triangles = 0;
                    if (insideCount == 1 || insideCount == 3) {
                        const int lone = insideCount == 1 ? in[0] : out[0];
                        const int* others = insideCount == 1 ? out : in;
                        triangle[0][0] = edge(lone, others[0]);
                        triangle[0][1] = edge(lone, others[1]);
                        triangle[0][2] = edge(lone, others[2]);
                        triangles = 1;
                    } else {
                        const uint32_t ac = edge(in[0], out[0]);
                        const uint32_t ad = edge(in[0], out[1]);
                        const uint32_t bd = edge(in[1], out[1]);
                        const uint32_t bc = edge(in[1], out[0]);
                        triangle[0][0] = ac;
                        triangle[0][1] = ad;
                        triangle[0][2] = bd;
                        triangle[1][0] = ac;
                        triangle[1][1] = bd;
                        triangle[1][2] = bc;
                        triangles = 2;
                    }
                    // Face outwards: the way the function grows, which is the gradient of its linear fit in this tetrahedron.
                    const int c0 = tetrahedron[0];
                    const glm::mat3 edges(position[tetrahedron[1]] - position[c0], position[tetrahedron[2]] - position[c0], position[tetrahedron[3]] - position[c0]);
                    const glm::vec3 differences(value[tetrahedron[1]] - value[c0], value[tetrahedron[2]] - value[c0], value[tetrahedron[3]] - value[c0]);
                    const glm::vec3 growth = glm::inverse(glm::transpose(edges)) * differences;
                    for (int t = 0; t < triangles; ++t) {
                        const glm::vec3 a = surface.positions[triangle[t][0]];
                        const glm::vec3 b = surface.positions[triangle[t][1]];
                        const glm::vec3 c = surface.positions[triangle[t][2]];
                        if (glm::dot(glm::cross(b - a, c - a), growth) < 0.0f) {
                            std::swap(triangle[t][1], triangle[t][2]);
                        }
                        surface.indices.insert(surface.indices.end(), {triangle[t][0], triangle[t][1], triangle[t][2]});
                    }
                }
            }
        }
    }
    return surface;
}

// Surface nets: one vertex in every cell of the grid that the surface passes through, and a quad across every grid edge
// that it crosses.
Surface CutSurfaceNets(const Grid& grid, const std::vector<float>& values) {
    // The corner of a cell is numbered by its bits: 1 for +x, 2 for +y, 4 for +z.
    static const int kEdges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    std::vector<int> cellVertex(static_cast<size_t>(grid.nx - 1) * (grid.ny - 1) * (grid.nz - 1), -1);
    Surface surface;
    for (int k = 0; k + 1 < grid.nz; ++k) {
        for (int j = 0; j + 1 < grid.ny; ++j) {
            for (int i = 0; i + 1 < grid.nx; ++i) {
                float corner[8];
                int inside = 0;
                for (int c = 0; c < 8; ++c) {
                    corner[c] = values[grid.NodeIndex(i + (c & 1), j + ((c >> 1) & 1), k + ((c >> 2) & 1))];
                    inside += corner[c] < 0.0f ? 1 : 0;
                }
                if (inside == 0 || inside == 8) {
                    continue;
                }
                glm::vec3 sum(0.0f);
                int crossings = 0;
                for (const auto& edge : kEdges) {
                    const float a = corner[edge[0]];
                    const float b = corner[edge[1]];
                    if ((a < 0.0f) == (b < 0.0f)) {
                        continue;
                    }
                    const float t = a / (a - b);
                    const glm::vec3 pa(edge[0] & 1, (edge[0] >> 1) & 1, (edge[0] >> 2) & 1);
                    const glm::vec3 pb(edge[1] & 1, (edge[1] >> 1) & 1, (edge[1] >> 2) & 1);
                    sum += pa + (pb - pa) * t;
                    ++crossings;
                }
                cellVertex[grid.CellIndex(i, j, k)] = static_cast<int>(surface.positions.size());
                surface.positions.push_back(grid.NodePosition(i, j, k) + sum / static_cast<float>(crossings) * grid.voxel);
            }
        }
    }

    const auto addQuad = [&](int c0, int c1, int c2, int c3, glm::vec3 outward) {
        if (c0 < 0 || c1 < 0 || c2 < 0 || c3 < 0) {
            return;
        }
        const glm::vec3 p0 = surface.positions[c0];
        const glm::vec3 p1 = surface.positions[c1];
        const glm::vec3 p2 = surface.positions[c2];
        if (glm::dot(glm::cross(p1 - p0, p2 - p0), outward) < 0.0f) {
            std::swap(c1, c3);  // turn the quad round so that it faces out
        }
        const glm::vec3 q0 = surface.positions[c0];
        const glm::vec3 q1 = surface.positions[c1];
        const glm::vec3 q2 = surface.positions[c2];
        const glm::vec3 q3 = surface.positions[c3];
        // Cut along the shorter diagonal, which keeps the triangles from turning into slivers.
        const uint32_t v0 = static_cast<uint32_t>(c0);
        const uint32_t v1 = static_cast<uint32_t>(c1);
        const uint32_t v2 = static_cast<uint32_t>(c2);
        const uint32_t v3 = static_cast<uint32_t>(c3);
        if (glm::distance(q0, q2) <= glm::distance(q1, q3)) {
            surface.indices.insert(surface.indices.end(), {v0, v1, v2, v0, v2, v3});
        } else {
            surface.indices.insert(surface.indices.end(), {v0, v1, v3, v1, v2, v3});
        }
    };
    for (int k = 1; k + 1 < grid.nz; ++k) {
        for (int j = 1; j + 1 < grid.ny; ++j) {
            for (int i = 1; i + 1 < grid.nx; ++i) {
                const bool here = values[grid.NodeIndex(i, j, k)] < 0.0f;
                if (here != (values[grid.NodeIndex(i + 1, j, k)] < 0.0f)) {
                    addQuad(cellVertex[grid.CellIndex(i, j - 1, k - 1)], cellVertex[grid.CellIndex(i, j, k - 1)], cellVertex[grid.CellIndex(i, j, k)],
                            cellVertex[grid.CellIndex(i, j - 1, k)], glm::vec3(here ? 1.0f : -1.0f, 0.0f, 0.0f));
                }
                if (here != (values[grid.NodeIndex(i, j + 1, k)] < 0.0f)) {
                    addQuad(cellVertex[grid.CellIndex(i - 1, j, k - 1)], cellVertex[grid.CellIndex(i, j, k - 1)], cellVertex[grid.CellIndex(i, j, k)],
                            cellVertex[grid.CellIndex(i - 1, j, k)], glm::vec3(0.0f, here ? 1.0f : -1.0f, 0.0f));
                }
                if (here != (values[grid.NodeIndex(i, j, k + 1)] < 0.0f)) {
                    addQuad(cellVertex[grid.CellIndex(i - 1, j - 1, k)], cellVertex[grid.CellIndex(i, j - 1, k)], cellVertex[grid.CellIndex(i, j, k)],
                            cellVertex[grid.CellIndex(i - 1, j, k)], glm::vec3(0.0f, 0.0f, here ? 1.0f : -1.0f));
                }
            }
        }
    }
    return surface;
}

// Moves every vertex onto the true surface of the function, and takes its normal from the gradient there.
void ProjectToSurface(const ImplicitShape& shape, Surface& surface, float voxel, JobSystem* jobs) {
    surface.normals.resize(surface.positions.size());
    ParallelEach(jobs, static_cast<int>(surface.positions.size()), [&](int v) {
        glm::vec3 point = surface.positions[v];
        for (int step = 0; step < 3; ++step) {
            const float distance = shape.Distance(point);
            const glm::vec3 gradient = Gradient(shape, point);
            const float lengthSquared = glm::dot(gradient, gradient);
            if (lengthSquared < 1e-8f) {
                break;
            }
            glm::vec3 move = gradient * (-distance / lengthSquared);
            const float length = glm::length(move);
            if (length > 0.6f * voxel) {
                move *= 0.6f * voxel / length;  // never far from where the cell put it
            }
            point += move;
        }
        surface.positions[v] = point;
        const glm::vec3 gradient = Gradient(shape, point);
        const float length = glm::length(gradient);
        surface.normals[v] = length > 1e-6f ? gradient / length : glm::vec3(0.0f, 1.0f, 0.0f);
    });
}


// Cuts the surface from the shape on one grid, moved by `shift` cells.
Surface ExtractSurface(const ImplicitShape& shape, float voxel, glm::vec3 shift, CuttingMethod method, JobSystem* jobs) {
    const Grid grid = MakeGrid(shape, voxel, shift);
    std::vector<float> values(static_cast<size_t>(grid.nx) * grid.ny * grid.nz);
    ParallelEach(jobs, grid.nz, [&](int k) {
        for (int j = 0; j < grid.ny; ++j) {
            for (int i = 0; i < grid.nx; ++i) {
                values[grid.NodeIndex(i, j, k)] = shape.Distance(grid.NodePosition(i, j, k));
            }
        }
    });
    Surface surface = method == CuttingMethod::Tetrahedra ? CutTetrahedra(grid, values) : CutSurfaceNets(grid, values);
    ProjectToSurface(shape, surface, voxel, jobs);
    return surface;
}

// Keeps the largest connected piece of a surface and drops the rest. The tip of a thin finger can be thinner than a grid
// cell, and then it is cut off as a speck of a few dozen triangles; losing it costs less than a stray piece would.
void KeepLargestPiece(Surface& surface) {
    const size_t vertexCount = surface.positions.size();
    std::vector<int> parent(vertexCount);
    for (size_t v = 0; v < vertexCount; ++v) {
        parent[v] = static_cast<int>(v);
    }
    const auto find = [&](int v) {
        while (parent[v] != v) {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        return v;
    };
    for (size_t t = 0; t + 2 < surface.indices.size(); t += 3) {
        parent[find(static_cast<int>(surface.indices[t]))] = find(static_cast<int>(surface.indices[t + 1]));
        parent[find(static_cast<int>(surface.indices[t + 1]))] = find(static_cast<int>(surface.indices[t + 2]));
    }
    std::vector<int> triangleCount(vertexCount, 0);
    for (size_t t = 0; t + 2 < surface.indices.size(); t += 3) {
        ++triangleCount[find(static_cast<int>(surface.indices[t]))];
    }
    int largest = 0;
    for (size_t v = 1; v < vertexCount; ++v) {
        if (triangleCount[v] > triangleCount[largest]) {
            largest = static_cast<int>(v);
        }
    }

    std::vector<int> newIndex(vertexCount, -1);
    Surface kept;
    for (size_t v = 0; v < vertexCount; ++v) {
        if (find(static_cast<int>(v)) == largest) {
            newIndex[v] = static_cast<int>(kept.positions.size());
            kept.positions.push_back(surface.positions[v]);
            kept.normals.push_back(surface.normals[v]);
        }
    }
    for (size_t t = 0; t + 2 < surface.indices.size(); t += 3) {
        if (newIndex[surface.indices[t]] >= 0) {
            for (int corner = 0; corner < 3; ++corner) {
                kept.indices.push_back(static_cast<uint32_t>(newIndex[surface.indices[t + corner]]));
            }
        }
    }
    surface = std::move(kept);
}

// How well a surface came out: closed without a bad edge, a sphere, and not much thrown away.
struct SurfaceQuality {
    int badEdges = 0;          // edges that are not shared by exactly two triangles
    int eulerError = 0;        // how far the Euler characteristic is from 2: a handle makes it 0, a hole or a bubble more
    size_t droppedTriangles = 0;  // what KeepLargestPiece threw away

    bool IsSound() const { return badEdges == 0 && eulerError == 0 && droppedTriangles <= kMostThatMayBeDropped; }
    long long Badness() const { return badEdges * 1000LL + eulerError * 1000LL + static_cast<long long>(droppedTriangles); }

    // A speck at the tip of a finger is tolerated; the loss of a hand, or a foot, is not.
    static constexpr size_t kMostThatMayBeDropped = 100;
};

SurfaceQuality MeasureSurface(const Surface& surface, size_t droppedTriangles) {
    std::vector<uint64_t> edges;
    edges.reserve(surface.indices.size());
    for (size_t t = 0; t + 2 < surface.indices.size(); t += 3) {
        for (int corner = 0; corner < 3; ++corner) {
            uint32_t a = surface.indices[t + corner];
            uint32_t b = surface.indices[t + (corner + 1) % 3];
            if (a > b) {
                std::swap(a, b);
            }
            edges.push_back((static_cast<uint64_t>(a) << 32) | b);
        }
    }
    std::sort(edges.begin(), edges.end());
    SurfaceQuality quality;
    quality.droppedTriangles = droppedTriangles;
    long long distinct = 0;
    for (size_t i = 0; i < edges.size();) {
        size_t end = i;
        while (end < edges.size() && edges[end] == edges[i]) {
            ++end;
        }
        ++distinct;
        quality.badEdges += (end - i) != 2 ? 1 : 0;
        i = end;
    }
    const long long euler = static_cast<long long>(surface.positions.size()) - distinct + static_cast<long long>(surface.indices.size() / 3);
    quality.eulerError = static_cast<int>(std::abs(euler - 2));
    return quality;
}

}  // namespace

float CoarseVolume(const ImplicitShape& shape, JobSystem* jobs) {
    return CoarseVolumeOf(shape, jobs);
}

Surface ExtractSoundSurface(const ImplicitShape& shape, float voxel, CuttingMethod method, JobSystem* jobs) {
    static const glm::vec3 kShifts[6] = {{0.0f, 0.0f, 0.0f},   {0.5f, 0.5f, 0.5f},    {0.25f, 0.75f, 0.5f},
                                         {0.75f, 0.25f, 0.5f}, {0.33f, 0.17f, 0.83f}, {0.6f, 0.4f, 0.15f}};
    Surface best;
    long long bestBadness = -1;
    // Surface nets that will not come out sound on any grid fall back to tetrahedra on a slightly coarser one, which are
    // closed whatever the shape, so a thin feature never leaves a handle or a bad edge in a piece.
    const CuttingMethod methods[2] = {method, CuttingMethod::Tetrahedra};
    const int methodCount = method == CuttingMethod::SurfaceNets ? 2 : 1;
    for (int attempt = 0; attempt < methodCount; ++attempt) {
        const float cell = attempt == 0 ? voxel : voxel * 1.25f;
        for (const glm::vec3& shift : kShifts) {
            Surface surface = ExtractSurface(shape, cell, shift, methods[attempt], jobs);
            const size_t before = surface.indices.size();
            KeepLargestPiece(surface);
            const SurfaceQuality quality = MeasureSurface(surface, (before - surface.indices.size()) / 3);
            if (bestBadness < 0 || quality.Badness() < bestBadness) {
                best = std::move(surface);
                bestBadness = quality.Badness();
            }
            if (quality.IsSound()) {
                return best;
            }
        }
    }
    return best;
}

}  // namespace character
