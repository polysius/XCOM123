#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

// The vertex format shared by every mesh. Matches input locations 0-2 in box.vert and npc.vert.
struct MeshVertex {
    glm::vec3 position;
    glm::vec3 normal;
    float part;  // which body part the vertex belongs to (see BodyPart); 0 for other meshes
};
static_assert(sizeof(MeshVertex) == 28, "MeshVertex must match the shader vertex layout");

struct MeshData {
    std::vector<MeshVertex> vertices;
    std::vector<uint32_t> indices;
};

// Body part ids that shaders/npc.vert uses to animate the walk cycle.
enum class BodyPart : int {
    Torso = 0,
    Head = 1,
    LeftLeg = 2,
    RightLeg = 3,
    LeftArm = 4,
    RightArm = 5,
};

// A cube spanning -1..1 on every axis. Box instances stretch it to their own size.
MeshData CreateUnitCube();

// A simple low-poly person, about 1.75 m tall, standing on the origin and facing +Z.
MeshData CreatePersonMesh();
