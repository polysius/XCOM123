#include "render/SceneGeometry.h"

#include <algorithm>
#include <cstring>

#include "render/VulkanContext.h"

namespace {
constexpr size_t kInitialNpcCapacity = 16384;
constexpr size_t kInitialBodyCapacity = 1024;
}  // namespace

void SceneGeometry::Create(const VulkanContext& context, uint32_t frameSlots) {
    cubeMesh = UploadMesh(context, CreateUnitCube());
    personMesh = UploadMesh(context, CreatePersonMesh());
    sphereMesh = UploadMesh(context, CreateUnitSphere());
    bodySlots.resize(frameSlots);
    for (BodySlot& slot : bodySlots) {
        GrowBodyBuffer(context, slot.boxInstances, slot.boxCapacity, kInitialBodyCapacity);
        GrowBodyBuffer(context, slot.sphereInstances, slot.sphereCapacity, kInitialBodyCapacity);
    }
    npcSlots.resize(frameSlots);
    for (NpcSlot& slot : npcSlots) {
        GrowNpcSlot(context, slot, kInitialNpcCapacity);
    }
}

void SceneGeometry::Destroy(VkDevice device) {
    DestroyMesh(device, cubeMesh);
    DestroyMesh(device, personMesh);
    DestroyMesh(device, sphereMesh);
    for (BodySlot& slot : bodySlots) {
        DestroyBuffer(device, slot.boxInstances);
        DestroyBuffer(device, slot.sphereInstances);
    }
    bodySlots.clear();
    DestroyBuffer(device, boxInstances);
    boxInstanceCount = 0;
    for (NpcSlot& slot : npcSlots) {
        DestroyBuffer(device, slot.instances);
    }
    npcSlots.clear();
}

void SceneGeometry::UploadBoxes(const VulkanContext& context, std::span<const BoxInstance> boxes) {
    DestroyBuffer(context.device, boxInstances);
    const VkDeviceSize bytes = std::max<VkDeviceSize>(boxes.size_bytes(), sizeof(BoxInstance));
    boxInstances = CreateMappedBuffer(context.device, context.physicalDevice, bytes, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    if (!boxes.empty()) {
        std::memcpy(boxInstances.mapped, boxes.data(), boxes.size_bytes());
    }
    boxInstanceCount = static_cast<uint32_t>(boxes.size());
}

void SceneGeometry::ReserveNpcs(const VulkanContext& context, size_t npcCount) {
    for (NpcSlot& slot : npcSlots) {
        GrowNpcSlot(context, slot, npcCount);
    }
}

void SceneGeometry::WriteNpcs(const VulkanContext& context, uint32_t frameSlot, std::span<const NpcInstance> npcs) {
    NpcSlot& slot = npcSlots[frameSlot];
    GrowNpcSlot(context, slot, npcs.size());
    if (!npcs.empty()) {
        std::memcpy(slot.instances.mapped, npcs.data(), npcs.size_bytes());
    }
    slot.count = static_cast<uint32_t>(npcs.size());
}

void SceneGeometry::WriteBodies(const VulkanContext& context, uint32_t frameSlot, std::span<const BodyInstance> boxes,
                                std::span<const BodyInstance> spheres) {
    BodySlot& slot = bodySlots[frameSlot];
    GrowBodyBuffer(context, slot.boxInstances, slot.boxCapacity, boxes.size());
    GrowBodyBuffer(context, slot.sphereInstances, slot.sphereCapacity, spheres.size());
    if (!boxes.empty()) {
        std::memcpy(slot.boxInstances.mapped, boxes.data(), boxes.size_bytes());
    }
    if (!spheres.empty()) {
        std::memcpy(slot.sphereInstances.mapped, spheres.data(), spheres.size_bytes());
    }
    slot.boxCount = static_cast<uint32_t>(boxes.size());
    slot.sphereCount = static_cast<uint32_t>(spheres.size());
}

void SceneGeometry::GrowBodyBuffer(const VulkanContext& context, GpuBuffer& buffer, size_t& capacity, size_t count) {
    if (count <= capacity) {
        return;
    }
    capacity = std::max(count, capacity * 2);
    DestroyBuffer(context.device, buffer);
    buffer = CreateMappedBuffer(context.device, context.physicalDevice, capacity * sizeof(BodyInstance),
                                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
}

void SceneGeometry::GrowNpcSlot(const VulkanContext& context, NpcSlot& slot, size_t npcCount) {
    if (npcCount <= slot.capacity) {
        return;
    }
    // Grow by at least doubling, so a slowly rising number of people does not reallocate every frame.
    slot.capacity = std::max(npcCount, slot.capacity * 2);
    DestroyBuffer(context.device, slot.instances);
    slot.instances = CreateMappedBuffer(context.device, context.physicalDevice, slot.capacity * sizeof(NpcInstance),
                                        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
}

void SceneGeometry::DrawBoxes(VkCommandBuffer cmd, VkPipeline pipeline) const {
    if (boxInstanceCount == 0) {
        return;
    }
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    const VkBuffer buffers[] = {cubeMesh.vertices.buffer, boxInstances.buffer};
    const VkDeviceSize offsets[] = {0, 0};
    vkCmdBindVertexBuffers(cmd, 0, 2, buffers, offsets);
    vkCmdBindIndexBuffer(cmd, cubeMesh.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
    // One draw call for the whole city: every box is an instance of the same cube.
    vkCmdDrawIndexed(cmd, cubeMesh.indexCount, boxInstanceCount, 0, 0, 0);
}

void SceneGeometry::DrawNpcs(VkCommandBuffer cmd, VkPipeline pipeline, uint32_t frameSlot) const {
    const NpcSlot& slot = npcSlots[frameSlot];
    if (slot.count == 0) {
        return;
    }
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    const VkBuffer buffers[] = {personMesh.vertices.buffer, slot.instances.buffer};
    const VkDeviceSize offsets[] = {0, 0};
    vkCmdBindVertexBuffers(cmd, 0, 2, buffers, offsets);
    vkCmdBindIndexBuffer(cmd, personMesh.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
    // One draw call for the whole crowd.
    vkCmdDrawIndexed(cmd, personMesh.indexCount, slot.count, 0, 0, 0);
}

void SceneGeometry::DrawBodies(VkCommandBuffer cmd, VkPipeline pipeline, uint32_t frameSlot) const {
    const BodySlot& slot = bodySlots[frameSlot];
    if (slot.boxCount == 0 && slot.sphereCount == 0) {
        return;
    }
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    const VkDeviceSize offsets[] = {0, 0};
    if (slot.boxCount > 0) {
        const VkBuffer buffers[] = {cubeMesh.vertices.buffer, slot.boxInstances.buffer};
        vkCmdBindVertexBuffers(cmd, 0, 2, buffers, offsets);
        vkCmdBindIndexBuffer(cmd, cubeMesh.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, cubeMesh.indexCount, slot.boxCount, 0, 0, 0);
    }
    if (slot.sphereCount > 0) {
        const VkBuffer buffers[] = {sphereMesh.vertices.buffer, slot.sphereInstances.buffer};
        vkCmdBindVertexBuffers(cmd, 0, 2, buffers, offsets);
        vkCmdBindIndexBuffer(cmd, sphereMesh.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, sphereMesh.indexCount, slot.sphereCount, 0, 0, 0);
    }
}

SceneGeometry::Mesh SceneGeometry::UploadMesh(const VulkanContext& context, const MeshData& data) {
    const VkDeviceSize vertexBytes = data.vertices.size() * sizeof(MeshVertex);
    const VkDeviceSize indexBytes = data.indices.size() * sizeof(uint32_t);

    Mesh mesh;
    mesh.vertices = CreateMappedBuffer(context.device, context.physicalDevice, vertexBytes, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    mesh.indices = CreateMappedBuffer(context.device, context.physicalDevice, indexBytes, VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    std::memcpy(mesh.vertices.mapped, data.vertices.data(), vertexBytes);
    std::memcpy(mesh.indices.mapped, data.indices.data(), indexBytes);
    mesh.indexCount = static_cast<uint32_t>(data.indices.size());
    return mesh;
}

void SceneGeometry::DestroyMesh(VkDevice device, Mesh& mesh) {
    DestroyBuffer(device, mesh.vertices);
    DestroyBuffer(device, mesh.indices);
    mesh.indexCount = 0;
}
