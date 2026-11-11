#pragma once

#include <string>

#include <volk.h>

// Throws std::runtime_error naming `operation` if a Vulkan call did not succeed.
void VkCheck(VkResult result, const char* operation);

// A buffer in memory the CPU can write to directly. It stays mapped for its whole life.
struct GpuBuffer {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    void* mapped = nullptr;
    VkDeviceSize size = 0;
};

// An image in GPU memory, plus the view that shaders and render passes use to access it.
struct GpuImage {
    VkImage image = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkImageView view = VK_NULL_HANDLE;
    VkFormat format = VK_FORMAT_UNDEFINED;
    VkExtent2D extent{0, 0};
};

struct ImageDesc {
    VkExtent2D extent;
    VkFormat format;
    VkImageUsageFlags usage;
    VkImageAspectFlags aspect;
    VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT;
};

GpuBuffer CreateMappedBuffer(VkDevice device, VkPhysicalDevice physicalDevice, VkDeviceSize size, VkBufferUsageFlags usage);
void DestroyBuffer(VkDevice device, GpuBuffer& buffer);

GpuImage CreateImage(VkDevice device, VkPhysicalDevice physicalDevice, const ImageDesc& desc);
void DestroyImage(VkDevice device, GpuImage& image);

VkShaderModule LoadShaderModule(VkDevice device, const std::string& path);

// A layout change for an image, plus the ordering between its previous use (src) and next use (dst).
struct ImageTransition {
    VkImage image;
    VkImageAspectFlags aspect;
    VkImageLayout oldLayout;
    VkImageLayout newLayout;
    VkPipelineStageFlags2 srcStage;
    VkAccessFlags2 srcAccess;
    VkPipelineStageFlags2 dstStage;
    VkAccessFlags2 dstAccess;
};
void TransitionImage(VkCommandBuffer commandBuffer, const ImageTransition& transition);
