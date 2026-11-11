#pragma once

#include <cstdint>
#include <span>

#include <glm/glm.hpp>

// Plain data shared by the world systems that produce it and the renderer that draws it.
// BoxInstance and NpcInstance are copied straight into GPU instance buffers, so their
// layouts must match the per-instance inputs in shaders/box.vert and shaders/npc.vert.

// Material ids understood by shaders/lit.frag.
enum class SurfaceMaterial : uint32_t {
    Ground = 0,     // roads, pavements and squares
    Facade = 1,     // building walls with procedural windows
    Plain = 2,      // anything else: roofs, ledges, fountains
    Character = 3,  // people (set by the NPC shader)
    Water = 4,      // rivers, canals, ponds and fountain basins, with moving ripples
    Stone = 5,      // dressed stone blocks: quays, bridges, churches, monuments
    RoofTile = 6,   // slate or clay tiles in courses; dormer windows when flagged in the seed
    Foliage = 7,    // tree crowns and hedges
    Grass = 8,      // lawns
    Glass = 9,      // leaded or framed glazing, stained glass when flagged in the seed
    Metal = 10,     // painted iron and lead: railings, lamp posts, domes
    Lamp = 11,      // street lamp lanterns that glow after sunset
};

// A box turned about the vertical axis. Almost all static city geometry is made of these.
// `halfSize` is measured along the box's own axes, so it stays the same however the box turns.
struct BoxInstance {
    glm::vec3 center;
    float material;    // a SurfaceMaterial, stored as float for the shader
    glm::vec3 halfSize;
    float roughness;   // 0 = mirror-like, 1 = fully matte
    glm::vec3 color;   // base colour (albedo), linear RGB
    float seed;        // per-object value that varies procedural detail
    float yaw = 0.0f;  // radians about +Y, the same convention as NpcInstance::heading
    float unused0 = 0.0f;  // pads the instance to four vec4s, as the shader reads it
    float unused1 = 0.0f;
    float unused2 = 0.0f;
};
static_assert(sizeof(BoxInstance) == 64, "BoxInstance must match the shader instance layout");

// One visible person.
struct NpcInstance {
    glm::vec3 position;  // feet position in world space
    float heading;       // radians, 0 faces +Z
    glm::vec3 clothingColor;
    float heightScale;
    float walkPhase;     // radians, drives the leg and arm swing
    float stride;        // 0 = standing still, 1 = full walking swing
    float unused0;
    float unused1;
};
static_assert(sizeof(NpcInstance) == 48, "NpcInstance must match the shader instance layout");

// Everything the renderer needs to draw one frame.
struct FrameInput {
    glm::mat4 view{1.0f};
    glm::mat4 projection{1.0f};
    glm::vec3 cameraPosition{0.0f};
    glm::vec3 sunDirection{0.0f, 1.0f, 0.0f};  // unit vector pointing towards the sun
    float timeSeconds = 0.0f;
    std::span<const NpcInstance> npcs;
};
