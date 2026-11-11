#pragma once

#include <volk.h>

#include "render/VulkanUtils.h"

class VulkanContext;

inline constexpr VkFormat kHdrFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
inline constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;

// The images the scene pass draws into and the tonemap pass reads: HDR colour (multisampled,
// plus its single-sample resolve) and the depth buffer.
//
// They are allocated once, at the full window size. Dynamic resolution draws into their
// top-left corner only, so changing the render scale never reallocates anything.
struct RenderTargets {
    void Create(const VulkanContext& context, VkExtent2D windowExtent, VkSampleCountFlagBits sampleCount);
    void Destroy(VkDevice device);

    bool UsesMsaa() const { return samples != VK_SAMPLE_COUNT_1_BIT; }
    // The top-left part of the targets that is drawn at `scale` (a fraction of the full size).
    VkExtent2D ScaledExtent(float scale) const;

    GpuImage hdrMultisampled;  // only used when multisampling is on
    GpuImage hdrResolved;      // the single-sample HDR image the tonemap pass reads
    GpuImage depth;
    VkExtent2D fullExtent{0, 0};
    VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT;
};
