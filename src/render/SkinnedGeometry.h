#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <volk.h>

#include "render/SkinningMath.h"
#include "render/VulkanUtils.h"
#include "scene/SceneTypes.h"

class VulkanContext;

// The skinned characters that the shadow and scene passes draw: the meshes uploaded once, and for every frame slot an instance
// buffer and a palette buffer.
//
// As with the crowd, each frame slot has buffers of its own, written only after the GPU has finished the slot's previous frame,
// which also means they can grow right there without waiting for the whole GPU to go idle. The palette buffer is read through a
// descriptor, so when it grows the renderer has to point the slot's descriptor at the new buffer (see WriteFrame).
class SkinnedGeometry {
public:
    // The shader binding of the palette buffer in the scene descriptor set.
    static constexpr uint32_t kPaletteBinding = 5;

    void Create(const VulkanContext& context, uint32_t frameSlots);
    void Destroy(VkDevice device);

    // Puts the triangles of a character on the GPU. It only adds buffers, so frames in flight are not disturbed.
    CharacterHandle UploadMesh(const VulkanContext& context, const skinning::MeshData& mesh);

    // Chooses which characters to draw and copies their instances and palettes into the slot's buffers. Returns true when the
    // palette buffer is a new one, so that the slot's descriptor must be written again. The GPU must have finished the slot's
    // previous frame.
    bool WriteFrame(const VulkanContext& context, uint32_t frameSlot, std::span<const SkinnedInstance> characters,
                    const skinning::FrameView& view);

    // Makes `lowerDetail` the mesh that `body` is drawn with when it is far away (and, one level down again, in shadows).
    // False when the pair cannot be linked (see skinning::LinkLowerDetail).
    bool SetLowerDetail(CharacterHandle body, CharacterHandle lowerDetail);

    VkBuffer PaletteBuffer(uint32_t frameSlot) const { return slots[frameSlot].palettes.buffer; }

    // Draws what the camera sees, or (for the shadow pass) everything that is in view of the sun. Draws nothing when the frame
    // has no characters, and does not even bind the pipeline then.
    void DrawForCamera(VkCommandBuffer cmd, VkPipeline pipeline, uint32_t frameSlot) const;
    void DrawForSun(VkCommandBuffer cmd, VkPipeline pipeline, uint32_t frameSlot) const;

private:
    struct Mesh {
        GpuBuffer vertices;
        GpuBuffer indices;
        uint32_t indexCount = 0;
    };

    struct Slot {
        GpuBuffer instances;
        GpuBuffer palettes;
        size_t instanceCapacity = 0;
        size_t paletteCapacity = 0;  // in matrices
        std::vector<skinning::MeshRange> cameraRanges;  // what the scene pass draws from this slot's buffers
        std::vector<skinning::MeshRange> sunRanges;     // what the shadow pass draws
    };

    // Grows a buffer to hold at least `count` elements of `elementBytes`, keeping nothing of the old contents. The new buffer is
    // made before the old one is destroyed, so a failure leaves the old one in place. Returns true when a new buffer was made.
    static bool Grow(const VulkanContext& context, GpuBuffer& buffer, size_t& capacity, size_t count, size_t elementBytes,
                     VkBufferUsageFlags usage);
    void Draw(VkCommandBuffer cmd, VkPipeline pipeline, uint32_t frameSlot, bool forSun) const;

    std::vector<Mesh> meshes;
    std::vector<skinning::MeshInfo> meshInfos;  // the CPU side of `meshes`, with the same indices
    std::vector<Slot> slots;
    skinning::FramePlan plan;  // reused every frame, so that it stops allocating once it is big enough
    bool warnedAboutShortPalette = false;
};
