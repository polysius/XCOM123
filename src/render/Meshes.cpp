#include "render/Meshes.h"

#include <cmath>

namespace {
// Appends an axis-aligned box built from six outward-facing quads, wound counter-clockwise
// when seen from outside (the renderer culls the back sides).
void AddBox(MeshData& mesh, glm::vec3 center, glm::vec3 halfSize, BodyPart part) {
    const glm::vec3 faceNormals[] = {
        {1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
        {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f},
    };

    for (const glm::vec3& normal : faceNormals) {
        // Two axes spanning the face, chosen so that cross(u, v) == normal. Listing the corners
        // as (-u-v), (+u-v), (+u+v), (-u+v) then runs counter-clockwise seen from outside.
        const glm::vec3 v = std::abs(normal.y) > 0.5f ? glm::vec3(0.0f, 0.0f, -normal.y) : glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::vec3 u = glm::cross(v, normal);

        const glm::vec3 faceCenter = center + normal * glm::dot(glm::abs(normal), halfSize);
        const glm::vec3 uExtent = u * glm::dot(glm::abs(u), halfSize);
        const glm::vec3 vExtent = v * glm::dot(glm::abs(v), halfSize);

        const uint32_t first = static_cast<uint32_t>(mesh.vertices.size());
        const float partId = static_cast<float>(part);
        mesh.vertices.push_back({faceCenter - uExtent - vExtent, normal, partId});
        mesh.vertices.push_back({faceCenter + uExtent - vExtent, normal, partId});
        mesh.vertices.push_back({faceCenter + uExtent + vExtent, normal, partId});
        mesh.vertices.push_back({faceCenter - uExtent + vExtent, normal, partId});
        mesh.indices.insert(mesh.indices.end(), {first, first + 1, first + 2, first, first + 2, first + 3});
    }
}
}  // namespace

MeshData CreateUnitCube() {
    MeshData mesh;
    AddBox(mesh, glm::vec3(0.0f), glm::vec3(1.0f), BodyPart::Torso);
    return mesh;
}

MeshData CreatePersonMesh() {
    MeshData mesh;
    // Legs hang from the hips at 0.9 m; npc.vert swings them around that height.
    // The person faces +Z, so their left side is +X.
    AddBox(mesh, {0.11f, 0.45f, 0.0f}, {0.085f, 0.45f, 0.10f}, BodyPart::LeftLeg);
    AddBox(mesh, {-0.11f, 0.45f, 0.0f}, {0.085f, 0.45f, 0.10f}, BodyPart::RightLeg);
    // Hips (or a coat tail) and the upper body.
    AddBox(mesh, {0.0f, 0.92f, 0.0f}, {0.20f, 0.10f, 0.12f}, BodyPart::Torso);
    AddBox(mesh, {0.0f, 1.20f, 0.0f}, {0.22f, 0.27f, 0.13f}, BodyPart::Torso);
    // Arms hang from the shoulders at 1.45 m.
    AddBox(mesh, {0.29f, 1.16f, 0.0f}, {0.06f, 0.29f, 0.07f}, BodyPart::LeftArm);
    AddBox(mesh, {-0.29f, 1.16f, 0.0f}, {0.06f, 0.29f, 0.07f}, BodyPart::RightArm);
    AddBox(mesh, {0.0f, 1.62f, 0.01f}, {0.10f, 0.12f, 0.11f}, BodyPart::Head);
    return mesh;
}
