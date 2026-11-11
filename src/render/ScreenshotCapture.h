#pragma once

#include <string>

#include <volk.h>

#include "render/VulkanUtils.h"

class VulkanContext;

// Copies a finished swapchain image into CPU-visible memory and saves it as a PNG file.
class ScreenshotCapture {
public:
    void Destroy(VkDevice device);

    // Records a copy of `image` (in COLOR_ATTACHMENT_OPTIMAL, just drawn) into the readback
    // buffer, then leaves the image ready to present.
    void RecordCopy(const VulkanContext& context, VkCommandBuffer cmd, VkImage image, VkExtent2D extent);

    // Saves the copied pixels. Call only after the GPU has finished the frame that copied them.
    bool SavePng(const std::string& path, VkFormat swapchainFormat) const;

private:
    GpuBuffer readback;
    VkExtent2D copiedExtent{0, 0};
};
