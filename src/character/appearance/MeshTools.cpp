#include "character/appearance/MeshTools.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace character {

MeshVertex Blend(const MeshVertex& a, const MeshVertex& b, float t) {
    MeshVertex result;
    result.position = glm::mix(a.position, b.position, t);
    const glm::vec3 normal = glm::mix(a.normal, b.normal, t);
    result.normal = glm::length(normal) > 1e-6f ? glm::normalize(normal) : a.normal;
    result.uv = glm::mix(a.uv, b.uv, t);
    result.attribute = glm::mix(a.attribute, b.attribute, t);

    // Add the weights of both vertices by bone, then keep the four heaviest.
    std::array<float, kJointCount> sum{};
    for (int slot = 0; slot < 4; ++slot) {
        sum[a.bones[slot]] += (1.0f - t) * a.weights[slot];
        sum[b.bones[slot]] += t * b.weights[slot];
    }
    float total = 0.0f;
    result.weights = {0.0f, 0.0f, 0.0f, 0.0f};
    result.bones = {0, 0, 0, 0};
    for (int slot = 0; slot < 4; ++slot) {
        const auto heaviest = std::max_element(sum.begin(), sum.end());
        if (*heaviest <= 0.0f) {
            break;
        }
        result.bones[slot] = static_cast<uint8_t>(heaviest - sum.begin());
        result.weights[slot] = *heaviest;
        total += *heaviest;
        *heaviest = 0.0f;
    }
    for (float& weight : result.weights) {
        weight /= std::max(total, 1e-9f);
    }
    return result;
}

int DominantBone(const MeshVertex& vertex) {
    int best = 0;
    for (int slot = 1; slot < 4; ++slot) {
        if (vertex.weights[slot] > vertex.weights[best]) {
            best = slot;
        }
    }
    return vertex.bones[best];
}

TriangleMesh ExtractPiece(const CharacterMesh& mesh, size_t piece) {
    const MeshPiece& range = mesh.pieces[piece];
    TriangleMesh result;
    result.vertices.resize(range.vertexCount);
    for (uint32_t i = 0; i < range.vertexCount; ++i) {
        const uint32_t v = range.firstVertex + i;
        MeshVertex& vertex = result.vertices[i];
        vertex.position = mesh.positions[v];
        vertex.normal = mesh.normals[v];
        vertex.bones = mesh.boneIndices[v];
        vertex.weights = mesh.boneWeights[v];
    }
    result.indices.reserve(range.indexCount);
    for (uint32_t i = 0; i < range.indexCount; ++i) {
        result.indices.push_back(mesh.indices[range.firstIndex + i] - range.firstVertex);
    }
    return result;
}

TriangleMesh ClipByField(const TriangleMesh& mesh, const std::vector<float>& field) {
    TriangleMesh result;
    std::unordered_map<uint32_t, uint32_t> kept;       // old vertex -> new vertex
    std::unordered_map<uint64_t, uint32_t> crossings;  // edge (low, high) -> new vertex on it

    const auto keep = [&](uint32_t old) {
        const auto found = kept.find(old);
        if (found != kept.end()) {
            return found->second;
        }
        const uint32_t index = static_cast<uint32_t>(result.vertices.size());
        result.vertices.push_back(mesh.vertices[old]);
        kept.emplace(old, index);
        return index;
    };
    // The vertex where the edge from `from` (inside) to `to` (outside) crosses zero. The crossing is found from the
    // inside end in both triangles that share the edge, so that they agree on it.
    const auto cross = [&](uint32_t from, uint32_t to) {
        const uint32_t low = std::min(from, to);
        const uint32_t high = std::max(from, to);
        const uint64_t key = (static_cast<uint64_t>(low) << 32) | high;
        const auto found = crossings.find(key);
        if (found != crossings.end()) {
            return found->second;
        }
        const uint32_t inside = field[from] >= 0.0f ? from : to;
        const uint32_t outside = inside == from ? to : from;
        // Not within 1 percent of an end of the edge: a cut that close to a vertex would only make a sliver of a triangle.
        const float t = glm::clamp(field[inside] / (field[inside] - field[outside]), 0.01f, 0.99f);
        const uint32_t index = static_cast<uint32_t>(result.vertices.size());
        result.vertices.push_back(Blend(mesh.vertices[inside], mesh.vertices[outside], t));
        crossings.emplace(key, index);
        return index;
    };

    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const uint32_t id[3] = {mesh.indices[t], mesh.indices[t + 1], mesh.indices[t + 2]};
        bool inside[3];
        int insideCount = 0;
        for (int c = 0; c < 3; ++c) {
            inside[c] = field[id[c]] >= 0.0f;
            insideCount += inside[c] ? 1 : 0;
        }
        if (insideCount == 3) {
            result.indices.insert(result.indices.end(), {keep(id[0]), keep(id[1]), keep(id[2])});
        } else if (insideCount == 1) {
            const int a = inside[0] ? 0 : (inside[1] ? 1 : 2);
            const int b = (a + 1) % 3;
            const int c = (a + 2) % 3;
            result.indices.insert(result.indices.end(), {keep(id[a]), cross(id[a], id[b]), cross(id[a], id[c])});
        } else if (insideCount == 2) {
            const int outsideCorner = !inside[0] ? 0 : (!inside[1] ? 1 : 2);
            const int a = (outsideCorner + 1) % 3;
            const int b = (outsideCorner + 2) % 3;
            const uint32_t ab = cross(id[b], id[outsideCorner]);  // on the edge from b to the outside corner
            const uint32_t aa = cross(id[a], id[outsideCorner]);
            result.indices.insert(result.indices.end(), {keep(id[a]), keep(id[b]), ab});
            result.indices.insert(result.indices.end(), {keep(id[a]), ab, aa});
        }
    }
    return result;
}

std::vector<glm::vec3> VertexNormals(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& indices) {
    std::vector<glm::vec3> normals(positions.size(), glm::vec3(0.0f));
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        const uint32_t id[3] = {indices[t], indices[t + 1], indices[t + 2]};
        const glm::vec3 faceNormal = glm::cross(positions[id[1]] - positions[id[0]], positions[id[2]] - positions[id[0]]);
        if (glm::dot(faceNormal, faceNormal) < 1e-18f) {
            continue;
        }
        const glm::vec3 unit = glm::normalize(faceNormal);
        for (int c = 0; c < 3; ++c) {
            const glm::vec3 toNext = glm::normalize(positions[id[(c + 1) % 3]] - positions[id[c]] + glm::vec3(1e-12f));
            const glm::vec3 toPrevious = glm::normalize(positions[id[(c + 2) % 3]] - positions[id[c]] + glm::vec3(1e-12f));
            const float angle = std::acos(std::clamp(glm::dot(toNext, toPrevious), -1.0f, 1.0f));
            normals[id[c]] += unit * angle;
        }
    }
    for (glm::vec3& normal : normals) {
        normal = glm::length(normal) > 1e-9f ? glm::normalize(normal) : glm::vec3(0.0f, 1.0f, 0.0f);
    }
    return normals;
}

glm::vec3 ClosestPointOnTriangle(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c) {
    // After Ericson, Real-Time Collision Detection: find the Voronoi region of the triangle that holds the point.
    const glm::vec3 ab = b - a;
    const glm::vec3 ac = c - a;
    const glm::vec3 ap = p - a;
    const float d1 = glm::dot(ab, ap);
    const float d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) {
        return a;
    }
    const glm::vec3 bp = p - b;
    const float d3 = glm::dot(ab, bp);
    const float d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) {
        return b;
    }
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        return a + ab * (d1 / (d1 - d3));
    }
    const glm::vec3 cp = p - c;
    const float d5 = glm::dot(ab, cp);
    const float d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) {
        return c;
    }
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        return a + ac * (d2 / (d2 - d6));
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    const float denominator = 1.0f / (va + vb + vc);
    return a + ab * (vb * denominator) + ac * (vc * denominator);
}

TriangleGrid::TriangleGrid(const std::vector<glm::vec3>& positionList, const std::vector<uint32_t>& indexList, float cellMeters)
    : positions(positionList), indices(indexList), cellSize(cellMeters) {
    glm::vec3 low(1e9f);
    glm::vec3 high(-1e9f);
    for (const glm::vec3& p : positions) {
        low = glm::min(low, p);
        high = glm::max(high, p);
    }
    if (positions.empty()) {
        low = high = glm::vec3(0.0f);
    }
    origin = low - glm::vec3(cellSize);
    cellCount = glm::ivec3(glm::ceil((high + glm::vec3(cellSize) - origin) / cellSize)) + glm::ivec3(1);

    // Two passes over the triangles: count how many fall in each cell, then fill the lists.
    const size_t cells = static_cast<size_t>(cellCount.x) * cellCount.y * cellCount.z;
    cellStart.assign(cells + 1, 0u);
    const auto forEachCell = [&](size_t triangle, const auto& visit) {
        const glm::vec3 a = positions[indices[triangle * 3]];
        const glm::vec3 b = positions[indices[triangle * 3 + 1]];
        const glm::vec3 c = positions[indices[triangle * 3 + 2]];
        const glm::ivec3 lowCell = CellOf(glm::min(a, glm::min(b, c)));
        const glm::ivec3 highCell = CellOf(glm::max(a, glm::max(b, c)));
        for (int z = lowCell.z; z <= highCell.z; ++z) {
            for (int y = lowCell.y; y <= highCell.y; ++y) {
                for (int x = lowCell.x; x <= highCell.x; ++x) {
                    visit(CellIndex({x, y, z}));
                }
            }
        }
    };
    const size_t triangleCount = indices.size() / 3;
    for (size_t t = 0; t < triangleCount; ++t) {
        forEachCell(t, [&](size_t cell) { ++cellStart[cell + 1]; });
    }
    for (size_t cell = 0; cell < cells; ++cell) {
        cellStart[cell + 1] += cellStart[cell];
    }
    cellTriangles.resize(cellStart[cells]);
    std::vector<uint32_t> fill(cellStart.begin(), cellStart.end() - 1);
    for (size_t t = 0; t < triangleCount; ++t) {
        forEachCell(t, [&](size_t cell) { cellTriangles[fill[cell]++] = static_cast<uint32_t>(t); });
    }
}

glm::ivec3 TriangleGrid::CellOf(glm::vec3 point) const {
    const glm::ivec3 cell(glm::floor((point - origin) / cellSize));
    return glm::clamp(cell, glm::ivec3(0), cellCount - glm::ivec3(1));
}

size_t TriangleGrid::CellIndex(glm::ivec3 cell) const {
    return (static_cast<size_t>(cell.z) * cellCount.y + cell.y) * cellCount.x + cell.x;
}

TriangleGrid::Nearest TriangleGrid::FindNearest(glm::vec3 point, float searchRadius) const {
    Nearest best;
    const glm::ivec3 lowCell = CellOf(point - glm::vec3(searchRadius));
    const glm::ivec3 highCell = CellOf(point + glm::vec3(searchRadius));
    for (int z = lowCell.z; z <= highCell.z; ++z) {
        for (int y = lowCell.y; y <= highCell.y; ++y) {
            for (int x = lowCell.x; x <= highCell.x; ++x) {
                const size_t cell = CellIndex({x, y, z});
                for (uint32_t i = cellStart[cell]; i < cellStart[cell + 1]; ++i) {
                    const uint32_t t = cellTriangles[i];
                    const glm::vec3& a = positions[indices[t * 3]];
                    const glm::vec3& b = positions[indices[t * 3 + 1]];
                    const glm::vec3& c = positions[indices[t * 3 + 2]];
                    const glm::vec3 closest = ClosestPointOnTriangle(point, a, b, c);
                    const float distance = glm::distance(closest, point);
                    if (distance < best.distance && distance <= searchRadius) {
                        best.distance = distance;
                        best.point = closest;
                        best.triangle = t;
                        const glm::vec3 normal = glm::cross(b - a, c - a);
                        best.faceNormal = glm::length(normal) > 1e-12f ? glm::normalize(normal) : glm::vec3(0.0f, 1.0f, 0.0f);
                    }
                }
            }
        }
    }
    return best;
}

float TriangleGrid::Raycast(glm::vec3 rayOrigin, glm::vec3 direction, float maxDistance) const {
    // March along the ray in steps of half a cell, testing the triangles of each cell reached. A triangle may be tested
    // twice, which costs a little time and gives the same answer.
    float nearest = -1.0f;
    const float step = cellSize * 0.5f;
    for (float travelled = 0.0f; travelled <= maxDistance + step; travelled += step) {
        const glm::vec3 point = rayOrigin + direction * std::min(travelled, maxDistance);
        const glm::ivec3 cell = CellOf(point);
        const size_t index = CellIndex(cell);
        for (uint32_t i = cellStart[index]; i < cellStart[index + 1]; ++i) {
            const uint32_t t = cellTriangles[i];
            const glm::vec3& a = positions[indices[t * 3]];
            const glm::vec3 edge1 = positions[indices[t * 3 + 1]] - a;
            const glm::vec3 edge2 = positions[indices[t * 3 + 2]] - a;
            // Moller and Trumbore.
            const glm::vec3 p = glm::cross(direction, edge2);
            const float determinant = glm::dot(edge1, p);
            if (std::abs(determinant) < 1e-12f) {
                continue;
            }
            const glm::vec3 toOrigin = rayOrigin - a;
            const float u = glm::dot(toOrigin, p) / determinant;
            const glm::vec3 q = glm::cross(toOrigin, edge1);
            const float v = glm::dot(direction, q) / determinant;
            const float distance = glm::dot(edge2, q) / determinant;
            if (u >= 0.0f && v >= 0.0f && u + v <= 1.0f && distance > 1e-6f && distance <= maxDistance && (nearest < 0.0f || distance < nearest)) {
                nearest = distance;
            }
        }
        if (nearest >= 0.0f && nearest <= travelled) {
            break;  // every cell that could hold a nearer hit has been searched
        }
    }
    return nearest;
}

}  // namespace character
