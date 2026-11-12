#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <volk.h>

#include "render/Meshes.h"
#include "render/VulkanUtils.h"
#include "scene/SceneTypes.h"

class VulkanContext;

// The meshes and instance buffers that the shadow and scene passes draw: the city as instanced
// boxes, and the crowd as instanced people.
//
// The visible people change every frame, so each frame slot has its own people buffer. A slot's
// buffer is only written after the GPU has finished the slot's previous frame, which also means
// it can grow right there without waiting for the whole GPU to go idle.
class SceneGeometry {
public:
    void Create(const VulkanContext& context, uint32_t frameSlots);
    void Destroy(VkDevice device);

    // Replaces the city boxes. The GPU must not be using the old ones.
    void UploadBoxes(const VulkanContext& context, std::span<const BoxInstance> boxes);
    // Grows every slot's people buffer to hold at least `npcCount` people. The GPU must be idle.
    void ReserveNpcs(const VulkanContext& context, size_t npcCount);
    // Copies this frame's visible people into the slot's buffer, growing it first when needed.
    // The GPU must have finished the slot's previous frame.
    void WriteNpcs(const VulkanContext& context, uint32_t frameSlot, std::span<const NpcInstance> npcs);

    // Copies this frame's props into the slot's buffers, growing them first when needed.
    // The GPU must have finished the slot's previous frame.
    void WriteBodies(const VulkanContext& context, uint32_t frameSlot, std::span<const BodyInstance> boxes,
                     std::span<const BodyInstance> spheres);

    void DrawBoxes(VkCommandBuffer cmd, VkPipeline pipeline) const;
    void DrawNpcs(VkCommandBuffer cmd, VkPipeline pipeline, uint32_t frameSlot) const;
    // Draws the props of the slot: the boxes, then the spheres, with the body pipeline.
    void DrawBodies(VkCommandBuffer cmd, VkPipeline pipeline, uint32_t frameSlot) const;

private:
    struct Mesh {
        GpuBuffer vertices;
        GpuBuffer indices;
        uint32_t indexCount = 0;
    };

    struct NpcSlot {
        GpuBuffer instances;
        size_t capacity = 0;
        uint32_t count = 0;  // people written for the frame being drawn from this slot
    };

    // The props of one frame, as boxes and as spheres, each in an instance buffer of its own.
    struct BodySlot {
        GpuBuffer boxInstances;
        GpuBuffer sphereInstances;
        size_t boxCapacity = 0;
        size_t sphereCapacity = 0;
        uint32_t boxCount = 0;
        uint32_t sphereCount = 0;
    };

    static void GrowBodyBuffer(const VulkanContext& context, GpuBuffer& buffer, size_t& capacity, size_t count);
    static Mesh UploadMesh(const VulkanContext& context, const MeshData& data);
    static void DestroyMesh(VkDevice device, Mesh& mesh);
    static void GrowNpcSlot(const VulkanContext& context, NpcSlot& slot, size_t npcCount);

    Mesh cubeMesh;
    Mesh personMesh;
    Mesh sphereMesh;
    GpuBuffer boxInstances;
    uint32_t boxInstanceCount = 0;
    std::vector<NpcSlot> npcSlots;
    std::vector<BodySlot> bodySlots;
};
