#pragma once

#include <cstdint>

#include <glm/glm.hpp>
#include <volk.h>

class SceneGeometry;
class VulkanContext;
struct RenderTargets;

// Draws the lit city, the crowd and the sky into the HDR render targets.
//
// It draws at `renderExtent`, which can be smaller than the targets when dynamic resolution
// lowers the render scale: only the top-left part of the targets is touched then.
class ScenePass {
public:
    // `sceneLayout` is the pipeline layout shared with the shadow pass.
    void Create(const VulkanContext& context, VkPipelineLayout sceneLayout, VkSampleCountFlagBits samples);
    void Destroy(VkDevice device);

    void Record(VkCommandBuffer cmd, const RenderTargets& targets, VkExtent2D renderExtent,
                const SceneGeometry& geometry, uint32_t frameSlot, const glm::mat4& cameraViewProjection) const;

private:
    void TransitionTargetsForDrawing(VkCommandBuffer cmd, const RenderTargets& targets) const;

    VkPipelineLayout layout = VK_NULL_HANDLE;  // owned by the renderer
    VkPipeline boxPipeline = VK_NULL_HANDLE;
    VkPipeline npcPipeline = VK_NULL_HANDLE;
    VkPipeline bodyPipeline = VK_NULL_HANDLE;
    VkPipeline skyPipeline = VK_NULL_HANDLE;
};
