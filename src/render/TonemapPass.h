#pragma once

#include <volk.h>

class VulkanContext;
struct RenderTargets;

// Turns the HDR scene into the displayable image: exposure, filmic tone curve, vignette and
// dithering, written straight into the swapchain image.
//
// When the scene was drawn at a lower render scale, this pass also upsamples it to the window
// with bilinear filtering, by reading only the drawn part of the HDR image.
class TonemapPass {
public:
    void Create(const VulkanContext& context, const RenderTargets& targets);
    void Destroy(VkDevice device);
    // Points the pass at new render targets, after they were recreated for a new window size.
    void BindTargets(VkDevice device, const RenderTargets& targets);

    // Draws into `swapchainView`. The image is left in COLOR_ATTACHMENT_OPTIMAL, so the caller
    // can either present it or copy it out first.
    void Record(VkCommandBuffer cmd, VkImage swapchainImage, VkImageView swapchainView, VkExtent2D windowExtent,
                const RenderTargets& targets, VkExtent2D renderExtent) const;

private:
    VkSampler linearSampler = VK_NULL_HANDLE;
    VkDescriptorSetLayout setLayout = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
    VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
    VkPipeline pipeline = VK_NULL_HANDLE;
};
