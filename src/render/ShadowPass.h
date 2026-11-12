#pragma once

#include <cstdint>

#include <glm/glm.hpp>
#include <volk.h>

#include "render/VulkanUtils.h"
#include "scene/SceneTypes.h"

class SceneGeometry;
class VulkanContext;

// Draws the city and the crowd from the sun into a depth map, which the scene pass samples to
// decide what is in shadow.
class ShadowPass {
public:
    static constexpr uint32_t kMapSize = 4096;

    // `sceneLayout` is the pipeline layout shared with the scene pass (per-frame uniforms plus a
    // view-projection push constant).
    void Create(const VulkanContext& context, VkPipelineLayout sceneLayout);
    void Destroy(VkDevice device);

    // The sun's view-projection for this frame: a square around where the camera is looking.
    static glm::mat4 ComputeLightViewProjection(const FrameInput& input);

    void Record(VkCommandBuffer cmd, const SceneGeometry& geometry, uint32_t frameSlot,
                const glm::mat4& lightViewProjection) const;

    VkImageView MapView() const { return shadowMap.view; }
    VkSampler ComparisonSampler() const { return comparisonSampler; }

private:
    GpuImage shadowMap;
    VkSampler comparisonSampler = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;  // owned by the renderer
    VkPipeline boxPipeline = VK_NULL_HANDLE;
    VkPipeline npcPipeline = VK_NULL_HANDLE;
    VkPipeline bodyPipeline = VK_NULL_HANDLE;
};
