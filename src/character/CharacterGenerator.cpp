#include "character/CharacterGenerator.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "character/Anthropometry.h"
#include "character/BodyShape.h"
#include "core/JobSystem.h"

namespace character {
namespace {

// Runs body(index) for every index in [0, count), on the job system when there is one.
template <typename Body>
void ParallelEach(JobSystem* jobs, int count, const Body& body) {
    const auto run = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            body(static_cast<int>(i));
        }
    };
    if (jobs != nullptr) {
        jobs->ParallelFor(static_cast<size_t>(count), 2, run);
    } else {
        run(0, static_cast<size_t>(count));
    }
}

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
Grid MakeGrid(const BodyShape& shape, float voxel, glm::vec3 shift) {
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

glm::vec3 Gradient(const BodyShape& shape, glm::vec3 point) {
    constexpr float kStep = 0.0015f;
    return glm::vec3(shape.Distance(point + glm::vec3(kStep, 0, 0)) - shape.Distance(point - glm::vec3(kStep, 0, 0)),
                     shape.Distance(point + glm::vec3(0, kStep, 0)) - shape.Distance(point - glm::vec3(0, kStep, 0)),
                     shape.Distance(point + glm::vec3(0, 0, kStep)) - shape.Distance(point - glm::vec3(0, 0, kStep))) /
           (2.0f * kStep);
}

// The volume of the body found by counting the cells of a coarse grid whose centre is inside it. Quick, and good to a
// percent or two, which is enough to find the girth that gives a mass.
float CoarseVolume(const BodyShape& shape, JobSystem* jobs) {
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

struct Surface {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<uint32_t> indices;
};

// Marching tetrahedra: every cube of the grid is cut into six tetrahedra that share the cube's long diagonal, and the
// surface crosses each tetrahedron in nothing, one triangle or two. The cuts of two neighbouring cubes agree on the
// face they share, and a vertex is made once for each edge of the grid it sits on, so the triangles always join up into
// a closed surface in which every edge has exactly two triangles, however thin the shape is. Each vertex is then moved
// onto the true surface of the function, and its normal is the gradient there. Cubes alternate between two mirrored
// ways of cutting them into five tetrahedra, which still agree on every shared face; a single way of cutting would give
// a thin part pointing across its diagonals a worse hold than one pointing along them.
Surface ExtractSurface(const BodyShape& shape, float voxel, glm::vec3 shift, JobSystem* jobs) {
    const Grid grid = MakeGrid(shape, voxel, shift);
    std::vector<float> values(static_cast<size_t>(grid.nx) * grid.ny * grid.nz);
    ParallelEach(jobs, grid.nz, [&](int k) {
        for (int j = 0; j < grid.ny; ++j) {
            for (int i = 0; i < grid.nx; ++i) {
                values[grid.NodeIndex(i, j, k)] = shape.Distance(grid.NodePosition(i, j, k));
            }
        }
    });

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

    // Move every vertex onto the surface of the function and take its normal from the gradient.
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

// The weights of every vertex: the four nearest bones among the nearest one, its parent and its children, smoothed
// over the surface so that one bone gives way to the next over a few centimetres and not at a line.
void ComputeSkinWeights(const BodyShape& shape, const Skeleton& skeleton, CharacterMesh& mesh, JobSystem* jobs) {
    constexpr float kBlendWidth = 0.040f;  // metres of the surface over which one bone gives way to the next, before smoothing
    constexpr int kSmoothingPasses = 4;
    const size_t vertexCount = mesh.positions.size();
    mesh.boneIndices.resize(vertexCount);
    mesh.boneWeights.resize(vertexCount);

    // One weight for every bone of every vertex, and which bones each vertex may follow.
    std::vector<std::array<float, kJointCount>> dense(vertexCount);
    std::vector<uint32_t> allowed(vertexCount, 0u);
    ParallelEach(jobs, static_cast<int>(vertexCount), [&](int v) {
        const glm::vec3 point = mesh.positions[v];
        float boneDistance[kJointCount];
        std::fill(boneDistance, boneDistance + kJointCount, 1e9f);
        for (size_t part = 0; part < shape.PartCount(); ++part) {
            const int bone = Index(shape.PartBone(part));
            boneDistance[bone] = std::min(boneDistance[bone], shape.PartDistance(part, point));
        }
        int nearest = 0;
        for (int bone = 1; bone < kJointCount; ++bone) {
            if (boneDistance[bone] < boneDistance[nearest]) {
                nearest = bone;
            }
        }
        // Candidates: the nearest bone, its parent and its children.
        for (int bone = 0; bone < kJointCount; ++bone) {
            const bool neighbour = bone == nearest || bone == skeleton.joints[nearest].parent || skeleton.joints[bone].parent == nearest;
            dense[v][bone] = 0.0f;
            if (neighbour) {
                dense[v][bone] = std::exp(-(boneDistance[bone] - boneDistance[nearest]) / kBlendWidth);
                allowed[v] |= 1u << bone;
            }
        }
    });

    // The neighbours of each vertex are the other corners of its triangles.
    std::vector<std::vector<uint32_t>> neighbours(vertexCount);
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        for (int corner = 0; corner < 3; ++corner) {
            neighbours[mesh.indices[t + corner]].push_back(mesh.indices[t + (corner + 1) % 3]);
            neighbours[mesh.indices[t + corner]].push_back(mesh.indices[t + (corner + 2) % 3]);
        }
    }
    std::vector<std::array<float, kJointCount>> smoothed(vertexCount);
    for (int pass = 0; pass < kSmoothingPasses; ++pass) {
        ParallelEach(jobs, static_cast<int>(vertexCount), [&](int v) {
            std::array<float, kJointCount> average{};
            for (const uint32_t other : neighbours[v]) {
                for (int bone = 0; bone < kJointCount; ++bone) {
                    average[bone] += dense[other][bone];
                }
            }
            const float share = neighbours[v].empty() ? 0.0f : 1.0f / static_cast<float>(neighbours[v].size());
            float total = 0.0f;
            for (int bone = 0; bone < kJointCount; ++bone) {
                // Only the bones this vertex may follow keep any weight: the torso still never pulls on an upper arm.
                const bool permitted = (allowed[v] >> bone & 1u) != 0u;
                smoothed[v][bone] = permitted ? 0.5f * dense[v][bone] + 0.5f * average[bone] * share : 0.0f;
                total += smoothed[v][bone];
            }
            for (float& value : smoothed[v]) {
                value /= std::max(total, 1e-9f);
            }
        });
        dense.swap(smoothed);
    }

    // The four heaviest bones of every vertex, scaled to add up to 1.
    ParallelEach(jobs, static_cast<int>(vertexCount), [&](int v) {
        std::array<float, kJointCount> weight = dense[v];
        std::array<uint8_t, 4> indices{};
        std::array<float, 4> weights{};
        float total = 0.0f;
        for (int slot = 0; slot < 4; ++slot) {
            int best = -1;
            for (int bone = 0; bone < kJointCount; ++bone) {
                if (weight[bone] > 0.0f && (best < 0 || weight[bone] > weight[best])) {
                    best = bone;
                }
            }
            if (best < 0) {
                break;
            }
            indices[slot] = static_cast<uint8_t>(best);
            weights[slot] = weight[best];
            total += weight[best];
            weight[best] = 0.0f;
        }
        for (float& value : weights) {
            value /= total;
        }
        mesh.boneIndices[v] = indices;
        mesh.boneWeights[v] = weights;
    });
}

}  // namespace

MeshTopology MeasureTopology(const CharacterMesh& mesh) {
    MeshTopology topology;
    const size_t vertexCount = mesh.positions.size();

    // The pieces: groups of vertices joined by triangles.
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
    std::vector<bool> used(vertexCount, false);
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        parent[find(static_cast<int>(mesh.indices[t]))] = find(static_cast<int>(mesh.indices[t + 1]));
        parent[find(static_cast<int>(mesh.indices[t + 1]))] = find(static_cast<int>(mesh.indices[t + 2]));
        for (int corner = 0; corner < 3; ++corner) {
            used[mesh.indices[t + corner]] = true;
        }
    }
    long long usedVertices = 0;
    for (size_t v = 0; v < vertexCount; ++v) {
        usedVertices += used[v] ? 1 : 0;
        topology.pieces += (used[v] && find(static_cast<int>(v)) == static_cast<int>(v)) ? 1 : 0;
    }

    // The edges: each should be in exactly two triangles.
    std::vector<uint64_t> edges;
    edges.reserve(mesh.indices.size());
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        for (int corner = 0; corner < 3; ++corner) {
            uint32_t a = mesh.indices[t + corner];
            uint32_t b = mesh.indices[t + (corner + 1) % 3];
            if (a > b) {
                std::swap(a, b);
            }
            edges.push_back((static_cast<uint64_t>(a) << 32) | b);
        }
    }
    std::sort(edges.begin(), edges.end());
    long long distinctEdges = 0;
    for (size_t i = 0; i < edges.size();) {
        size_t end = i;
        while (end < edges.size() && edges[end] == edges[i]) {
            ++end;
        }
        ++distinctEdges;
        topology.badEdges += (end - i) != 2 ? 1 : 0;
        i = end;
    }
    topology.eulerCharacteristic = usedVertices - distinctEdges + static_cast<long long>(mesh.indices.size() / 3);
    return topology;
}

float MeshVolume(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& indices) {
    double volume = 0.0;
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        const glm::dvec3 a(positions[indices[i]]);
        const glm::dvec3 b(positions[indices[i + 1]]);
        const glm::dvec3 c(positions[indices[i + 2]]);
        volume += glm::dot(a, glm::cross(b, c)) / 6.0;
    }
    return static_cast<float>(volume);
}

namespace {

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

// Cuts the surface from the body, and if the grid did not catch it well (a thin gap that became a tunnel, a hand that
// broke off) tries again with the grid moved by a fraction of a cell, up to a few times; the first sound result is kept,
// else the least bad. Where the grid lies is the only thing that changes between the tries, so the result is the same
// every time for the same body.
Surface ExtractSoundSurface(const BodyShape& shape, float voxel, JobSystem* jobs) {
    static const glm::vec3 kShifts[6] = {{0.0f, 0.0f, 0.0f},   {0.5f, 0.5f, 0.5f},    {0.25f, 0.75f, 0.5f},
                                         {0.75f, 0.25f, 0.5f}, {0.33f, 0.17f, 0.83f}, {0.6f, 0.4f, 0.15f}};
    Surface best;
    long long bestBadness = -1;
    for (const glm::vec3& shift : kShifts) {
        Surface surface = ExtractSurface(shape, voxel, shift, jobs);
        const size_t before = surface.indices.size();
        KeepLargestPiece(surface);
        const SurfaceQuality quality = MeasureSurface(surface, (before - surface.indices.size()) / 3);
        if (bestBadness < 0 || quality.Badness() < bestBadness) {
            best = std::move(surface);
            bestBadness = quality.Badness();
        }
        if (quality.IsSound()) {
            break;
        }
    }
    return best;
}

}  // namespace

CharacterMesh GenerateCharacter(const CharacterParameters& parameters, const MeshSettings& settings, JobSystem* jobs) {
    CharacterMesh mesh;
    mesh.parameters = Sanitised(parameters);
    mesh.skeleton = BuildSkeleton(mesh.parameters);
    const float requestedVoxel = std::isfinite(settings.voxelMeters) ? settings.voxelMeters : MeshSettings{}.voxelMeters;
    const float voxel = std::clamp(requestedVoxel, 0.006f, 0.04f);
    const float minimumRadius = 0.55f * voxel;  // nothing thinner than a cell and a bit, or it breaks up
    const float targetVolume = mesh.parameters.massKg / anthropometry::kBodyDensity;

    // Find the girth that gives the volume. The volume grows about with the square of the girth, since the lengths of
    // the limbs and trunk are fixed, so a few secant steps on the coarse volume are enough.
    float girth = 1.0f;
    float previousGirth = 0.0f;
    float previousVolume = 0.0f;
    for (int step = 0; step < 6; ++step) {
        const float volume = CoarseVolume(BuildBodyShape(mesh.skeleton, mesh.parameters, girth, minimumRadius), jobs);
        if (std::abs(volume / targetVolume - 1.0f) < 0.01f) {
            break;
        }
        float next = girth * std::sqrt(targetVolume / std::max(volume, 1e-4f));
        if (previousGirth > 0.0f && std::abs(volume - previousVolume) > 1e-5f) {
            next = girth + (targetVolume - volume) * (girth - previousGirth) / (volume - previousVolume);
        }
        previousGirth = girth;
        previousVolume = volume;
        girth = std::clamp(next, 0.45f, 2.2f);
    }

    // Cut the surface, then correct the girth with the volume of the mesh itself, which the coarse count only estimates.
    Surface surface;
    BodyShape shape;
    for (int pass = 0; pass < 3; ++pass) {
        shape = BuildBodyShape(mesh.skeleton, mesh.parameters, girth, minimumRadius);
        surface = ExtractSoundSurface(shape, voxel, jobs);
        mesh.volumeCubicMeters = MeshVolume(surface.positions, surface.indices);
        const float error = mesh.volumeCubicMeters / targetVolume;
        if (pass == 2 || std::abs(error - 1.0f) < 0.01f) {
            break;
        }
        girth = std::clamp(girth / std::sqrt(error), 0.45f, 2.2f);
    }
    mesh.girth = girth;
    mesh.massKg = mesh.volumeCubicMeters * anthropometry::kBodyDensity;
    mesh.positions = std::move(surface.positions);
    mesh.normals = std::move(surface.normals);
    mesh.indices = std::move(surface.indices);
    ComputeSkinWeights(shape, mesh.skeleton, mesh, jobs);
    return mesh;
}

std::vector<glm::vec3> SkinPositions(const CharacterMesh& mesh, const Pose& pose) {
    const std::array<glm::mat4, kJointCount> skin = SkinMatrices(mesh.skeleton, pose);
    std::vector<glm::vec3> result(mesh.positions.size());
    for (size_t v = 0; v < mesh.positions.size(); ++v) {
        glm::vec3 sum(0.0f);
        for (int slot = 0; slot < 4; ++slot) {
            const float weight = mesh.boneWeights[v][slot];
            if (weight > 0.0f) {
                sum += weight * glm::vec3(skin[mesh.boneIndices[v][slot]] * glm::vec4(mesh.positions[v], 1.0f));
            }
        }
        result[v] = sum;
    }
    return result;
}

}  // namespace character
