#include "render/SkinnedGeometry.h"

#include <algorithm>
#include <cstring>
#include <iostream>

#include "render/VulkanContext.h"

namespace {
// Room for a small crowd to start with; the buffers grow when a frame has more.
constexpr size_t kInitialInstanceCapacity = 64;
constexpr size_t kInitialPaletteCapacity = 64 * 32;
}  // namespace

void SkinnedGeometry::Create(const VulkanContext& context, uint32_t frameSlots) {
    slots.resize(frameSlots);
    // The palette descriptor has to point at a real buffer from the start, even before any character is drawn.
    for (Slot& slot : slots) {
        Grow(context, slot.instances, slot.instanceCapacity, kInitialInstanceCapacity, sizeof(skinning::GpuInstance),
             VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
        Grow(context, slot.palettes, slot.paletteCapacity, kInitialPaletteCapacity, sizeof(glm::mat4),
             VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    }
}

void SkinnedGeometry::Destroy(VkDevice device) {
    for (Mesh& mesh : meshes) {
        DestroyBuffer(device, mesh.vertices);
        DestroyBuffer(device, mesh.indices);
    }
    meshes.clear();
    meshInfos.clear();
    for (Slot& slot : slots) {
        DestroyBuffer(device, slot.instances);
        DestroyBuffer(device, slot.palettes);
    }
    slots.clear();
}

CharacterHandle SkinnedGeometry::UploadMesh(const VulkanContext& context, const skinning::MeshData& data) {
    if (!skinning::IsDrawable(data)) {
        return {};
    }
    const VkDeviceSize vertexBytes = data.vertices.size() * sizeof(skinning::Vertex);
    const VkDeviceSize indexBytes = data.indices.size() * sizeof(uint32_t);

    Mesh mesh;
    mesh.vertices = CreateMappedBuffer(context.device, context.physicalDevice, vertexBytes, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    mesh.indices = CreateMappedBuffer(context.device, context.physicalDevice, indexBytes, VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    std::memcpy(mesh.vertices.mapped, data.vertices.data(), vertexBytes);
    std::memcpy(mesh.indices.mapped, data.indices.data(), indexBytes);
    mesh.indexCount = static_cast<uint32_t>(data.indices.size());

    meshes.push_back(mesh);
    meshInfos.push_back(data.info);
    return CharacterHandle{static_cast<uint32_t>(meshes.size() - 1)};
}

bool SkinnedGeometry::Grow(const VulkanContext& context, GpuBuffer& buffer, size_t& capacity, size_t count, size_t elementBytes,
                           VkBufferUsageFlags usage) {
    if (count <= capacity) {
        return false;
    }
    // Grow by at least doubling, so a slowly rising number of characters does not reallocate every frame.
    const size_t newCapacity = std::max(count, capacity * 2);
    GpuBuffer grown = CreateMappedBuffer(context.device, context.physicalDevice, newCapacity * elementBytes, usage);
    DestroyBuffer(context.device, buffer);
    buffer = grown;
    capacity = newCapacity;
    return true;
}

bool SkinnedGeometry::WriteFrame(const VulkanContext& context, uint32_t frameSlot, std::span<const SkinnedInstance> characters,
                                 const skinning::FrameView& view) {
    Slot& slot = slots[frameSlot];
    uint32_t dropped = 0;
    skinning::PlanFrame(characters, meshInfos, view, plan, &dropped);
    if (dropped > 0 && !warnedAboutShortPalette) {
        // Said once: a palette that is too short is a mistake of the caller, and the frame rate is no place for a message each frame.
        std::cerr << "Skinned characters: " << dropped << " were not drawn because their palette has fewer matrices than the mesh has joints\n";
        warnedAboutShortPalette = true;
    }
    slot.cameraRanges = plan.cameraRanges;
    slot.sunRanges = plan.sunRanges;

    Grow(context, slot.instances, slot.instanceCapacity, plan.instances.size(), sizeof(skinning::GpuInstance),
         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    const bool newPaletteBuffer = Grow(context, slot.palettes, slot.paletteCapacity, plan.palettes.size(), sizeof(glm::mat4),
                                       VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    if (!plan.instances.empty()) {
        std::memcpy(slot.instances.mapped, plan.instances.data(), plan.instances.size() * sizeof(skinning::GpuInstance));
        std::memcpy(slot.palettes.mapped, plan.palettes.data(), plan.palettes.size() * sizeof(glm::mat4));
    }
    return newPaletteBuffer;
}

bool SkinnedGeometry::SetLowerDetail(CharacterHandle body, CharacterHandle lowerDetail) {
    return skinning::LinkLowerDetail(meshInfos, body.index, lowerDetail.index);
}

void SkinnedGeometry::DrawForCamera(VkCommandBuffer cmd, VkPipeline pipeline, uint32_t frameSlot) const {
    Draw(cmd, pipeline, frameSlot, false);
}

void SkinnedGeometry::DrawForSun(VkCommandBuffer cmd, VkPipeline pipeline, uint32_t frameSlot) const {
    Draw(cmd, pipeline, frameSlot, true);
}

void SkinnedGeometry::Draw(VkCommandBuffer cmd, VkPipeline pipeline, uint32_t frameSlot, bool forSun) const {
    const Slot& slot = slots[frameSlot];
    const std::vector<skinning::MeshRange>& ranges = forSun ? slot.sunRanges : slot.cameraRanges;
    if (ranges.empty()) {
        return;
    }
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    for (const skinning::MeshRange& range : ranges) {
        const Mesh& mesh = meshes[range.mesh];
        const VkBuffer buffers[] = {mesh.vertices.buffer, slot.instances.buffer};
        const VkDeviceSize offsets[] = {0, 0};
        vkCmdBindVertexBuffers(cmd, 0, 2, buffers, offsets);
        vkCmdBindIndexBuffer(cmd, mesh.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, mesh.indexCount, range.count, 0, 0, range.firstInstance);
    }
}
