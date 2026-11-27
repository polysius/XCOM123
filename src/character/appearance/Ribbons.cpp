#include "character/appearance/Ribbons.h"

#include <algorithm>
#include <cmath>

namespace character {

void AddRibbon(RibbonSet& set, const std::vector<glm::vec3>& points, glm::vec3 faceDirection, float rootWidth, float tipWidth, glm::vec3 rootColour, glm::vec3 tipColour,
               Joint bone, float uOffset) {
    const size_t count = points.size();
    if (count < 2) {
        return;
    }
    RenderMesh& mesh = set.mesh;
    const uint32_t first = static_cast<uint32_t>(mesh.positions.size());
    set.roots.push_back(points.front());
    set.firstVertex.push_back(first);

    glm::vec3 previousSide(0.0f);
    for (size_t i = 0; i < count; ++i) {
        const glm::vec3 tangent = glm::normalize(points[std::min(i + 1, count - 1)] - points[i == 0 ? 0 : i - 1]);
        glm::vec3 side = glm::cross(tangent, faceDirection);
        // A strip that runs along the face direction has no side of its own: keep the one of the point before.
        side = glm::length(side) > 1e-4f ? glm::normalize(side) : (glm::length(previousSide) > 0.0f ? previousSide : glm::vec3(1.0f, 0.0f, 0.0f));
        previousSide = side;
        const glm::vec3 normal = glm::normalize(glm::cross(side, tangent));
        const float along = static_cast<float>(i) / static_cast<float>(count - 1);
        const float halfWidth = 0.5f * glm::mix(rootWidth, tipWidth, along);
        const glm::vec3 colour = glm::mix(rootColour, tipColour, along);
        for (int edge = 0; edge < 2; ++edge) {
            const float sign = edge == 0 ? -1.0f : 1.0f;
            mesh.positions.push_back(points[i] + side * (sign * halfWidth));
            mesh.normals.push_back(normal);
            mesh.uvs.emplace_back(uOffset + static_cast<float>(edge), along);
            mesh.boneIndices.push_back({static_cast<uint8_t>(Index(bone)), 0, 0, 0});
            mesh.boneWeights.push_back({1.0f, 0.0f, 0.0f, 0.0f});
            set.colours.push_back(colour);
        }
    }
    for (uint32_t i = 0; i + 1 < count; ++i) {
        const uint32_t negative = first + i * 2;  // the edge on the -side of the strip
        const uint32_t positive = negative + 1;
        mesh.indices.insert(mesh.indices.end(), {negative, positive, negative + 2});
        mesh.indices.insert(mesh.indices.end(), {positive, positive + 2, negative + 2});
    }
}

void RemoveRibbons(RibbonSet& set, const std::vector<char>& drop) {
    RibbonSet kept;
    kept.strandTexture = set.strandTexture;
    const size_t ribbons = set.RibbonCount();
    size_t indexCursor = 0;
    for (size_t r = 0; r < ribbons; ++r) {
        const uint32_t first = set.firstVertex[r];
        const uint32_t end = r + 1 < ribbons ? set.firstVertex[r + 1] : static_cast<uint32_t>(set.mesh.positions.size());
        const uint32_t vertexCount = end - first;
        // A ribbon of n points has 2n vertices and 6 (n - 1) indices, and its triangles follow those of the ribbon before it.
        const size_t indexCount = static_cast<size_t>(vertexCount / 2 - 1) * 6;
        if (r < drop.size() && drop[r]) {
            indexCursor += indexCount;
            continue;
        }
        const uint32_t newFirst = static_cast<uint32_t>(kept.mesh.positions.size());
        kept.firstVertex.push_back(newFirst);
        kept.roots.push_back(set.roots[r]);
        for (uint32_t v = first; v < end; ++v) {
            kept.mesh.positions.push_back(set.mesh.positions[v]);
            kept.mesh.normals.push_back(set.mesh.normals[v]);
            kept.mesh.uvs.push_back(set.mesh.uvs[v]);
            kept.mesh.boneIndices.push_back(set.mesh.boneIndices[v]);
            kept.mesh.boneWeights.push_back(set.mesh.boneWeights[v]);
            kept.colours.push_back(set.colours[v]);
        }
        for (size_t i = 0; i < indexCount; ++i) {
            kept.mesh.indices.push_back(set.mesh.indices[indexCursor + i] - first + newFirst);
        }
        indexCursor += indexCount;
    }
    set = std::move(kept);
}

}  // namespace character
