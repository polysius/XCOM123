#include "render/RenderTargets.h"

#include <algorithm>
#include <cmath>

#include "render/VulkanContext.h"

void RenderTargets::Create(const VulkanContext& context, VkExtent2D windowExtent, VkSampleCountFlagBits sampleCount) {
    fullExtent = windowExtent;
    samples = sampleCount;
    if (UsesMsaa()) {
        // Transient: the samples only live during the scene pass and are resolved at its end.
        hdrMultisampled = CreateImage(context.device, context.physicalDevice,
                                      {.extent = fullExtent,
                                       .format = kHdrFormat,
                                       .usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT,
                                       .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                                       .samples = samples});
    }
    hdrResolved = CreateImage(context.device, context.physicalDevice,
                              {.extent = fullExtent,
                               .format = kHdrFormat,
                               .usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                               .aspect = VK_IMAGE_ASPECT_COLOR_BIT});
    depth = CreateImage(context.device, context.physicalDevice,
                        {.extent = fullExtent,
                         .format = kDepthFormat,
                         .usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                         .aspect = VK_IMAGE_ASPECT_DEPTH_BIT,
                         .samples = samples});
}

void RenderTargets::Destroy(VkDevice device) {
    DestroyImage(device, hdrMultisampled);
    DestroyImage(device, hdrResolved);
    DestroyImage(device, depth);
}

VkExtent2D RenderTargets::ScaledExtent(float scale) const {
    const auto scaleSide = [scale](uint32_t side) {
        const auto scaled = static_cast<uint32_t>(std::lround(static_cast<float>(side) * scale));
        return std::clamp(scaled, 1u, side);
    };
    return {scaleSide(fullExtent.width), scaleSide(fullExtent.height)};
}
