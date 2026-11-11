#include "render/ScreenshotCapture.h"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <utility>
#include <vector>

#include "core/PngWriter.h"
#include "render/VulkanContext.h"

namespace {
constexpr uint32_t kBytesPerPixel = 4;

enum class ChannelOrder { Rgba, Bgra, Unsupported };

ChannelOrder ChannelOrderOf(VkFormat format) {
    switch (format) {
        case VK_FORMAT_R8G8B8A8_SRGB:
        case VK_FORMAT_R8G8B8A8_UNORM:
            return ChannelOrder::Rgba;
        case VK_FORMAT_B8G8R8A8_SRGB:
        case VK_FORMAT_B8G8R8A8_UNORM:
            return ChannelOrder::Bgra;
        default:
            return ChannelOrder::Unsupported;
    }
}
}  // namespace

void ScreenshotCapture::Destroy(VkDevice device) {
    DestroyBuffer(device, readback);
    copiedExtent = {0, 0};
}

void ScreenshotCapture::RecordCopy(const VulkanContext& context, VkCommandBuffer cmd, VkImage image, VkExtent2D extent) {
    const VkDeviceSize bytes = static_cast<VkDeviceSize>(extent.width) * extent.height * kBytesPerPixel;
    if (readback.size < bytes) {
        DestroyBuffer(context.device, readback);
        readback = CreateReadbackBuffer(context.device, context.physicalDevice, bytes);
    }
    copiedExtent = extent;

    TransitionImage(cmd, {.image = image,
                          .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                          .oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          .newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                          .srcStage = kColorOutputStage,
                          .srcAccess = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                          .dstStage = VK_PIPELINE_STAGE_2_COPY_BIT,
                          .dstAccess = VK_ACCESS_2_TRANSFER_READ_BIT});

    // Tightly packed rows (bufferRowLength 0), so the buffer reads like a plain pixel array.
    VkBufferImageCopy region{};
    region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    region.imageExtent = {extent.width, extent.height, 1};
    vkCmdCopyImageToBuffer(cmd, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, readback.buffer, 1, &region);

    // Make the copied bytes visible to the CPU once the frame's fence has signalled.
    VkBufferMemoryBarrier2 bufferBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2};
    bufferBarrier.srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
    bufferBarrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    bufferBarrier.dstStageMask = VK_PIPELINE_STAGE_2_HOST_BIT;
    bufferBarrier.dstAccessMask = VK_ACCESS_2_HOST_READ_BIT;
    bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.buffer = readback.buffer;
    bufferBarrier.size = VK_WHOLE_SIZE;
    VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    dependency.bufferMemoryBarrierCount = 1;
    dependency.pBufferMemoryBarriers = &bufferBarrier;
    vkCmdPipelineBarrier2(cmd, &dependency);

    TransitionImage(cmd, {.image = image,
                          .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                          .oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                          .newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                          .srcStage = VK_PIPELINE_STAGE_2_COPY_BIT,
                          .srcAccess = VK_ACCESS_2_NONE,
                          .dstStage = VK_PIPELINE_STAGE_2_NONE,
                          .dstAccess = VK_ACCESS_2_NONE});
}

bool ScreenshotCapture::SavePng(const std::string& path, VkFormat swapchainFormat) const {
    const ChannelOrder order = ChannelOrderOf(swapchainFormat);
    if (order == ChannelOrder::Unsupported || readback.mapped == nullptr) {
        std::cerr << "Screenshot: the swapchain format " << swapchainFormat << " cannot be saved\n";
        return false;
    }

    // The swapchain stores sRGB-encoded bytes, which is exactly what a PNG file holds, so the
    // bytes are only reordered, never converted.
    const size_t pixelCount = static_cast<size_t>(copiedExtent.width) * copiedExtent.height;
    std::vector<uint8_t> rgba(pixelCount * kBytesPerPixel);
    std::memcpy(rgba.data(), readback.mapped, rgba.size());
    if (order == ChannelOrder::Bgra) {
        for (size_t pixel = 0; pixel < pixelCount; ++pixel) {
            std::swap(rgba[pixel * kBytesPerPixel], rgba[pixel * kBytesPerPixel + 2]);
        }
    }
    return WritePng(path, copiedExtent.width, copiedExtent.height, rgba);
}
