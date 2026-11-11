#include "render/VulkanUtils.h"

#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <stdexcept>
#include <vector>

void VkCheck(VkResult result, const char* operation) {
    if (result != VK_SUCCESS) {
        throw std::runtime_error(std::string(operation) + " failed (VkResult " +
                                 std::to_string(static_cast<int>(result)) + ")");
    }
}

namespace {
// Picks a memory type allowed by `allowedTypes` that has all `required` properties,
// favouring one that also has the `preferred` properties.
uint32_t FindMemoryType(VkPhysicalDevice physicalDevice, uint32_t allowedTypes,
                        VkMemoryPropertyFlags required, VkMemoryPropertyFlags preferred) {
    VkPhysicalDeviceMemoryProperties properties;
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &properties);

    for (VkMemoryPropertyFlags wanted : {required | preferred, required}) {
        for (uint32_t index = 0; index < properties.memoryTypeCount; ++index) {
            const bool allowed = (allowedTypes & (1u << index)) != 0;
            const bool hasProperties = (properties.memoryTypes[index].propertyFlags & wanted) == wanted;
            if (allowed && hasProperties) {
                return index;
            }
        }
    }
    throw std::runtime_error("No suitable GPU memory type found");
}

VkDeviceMemory AllocateMemory(VkDevice device, VkPhysicalDevice physicalDevice, const VkMemoryRequirements& requirements,
                              VkMemoryPropertyFlags required, VkMemoryPropertyFlags preferred) {
    VkMemoryAllocateInfo allocateInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    allocateInfo.allocationSize = requirements.size;
    allocateInfo.memoryTypeIndex = FindMemoryType(physicalDevice, requirements.memoryTypeBits, required, preferred);

    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkCheck(vkAllocateMemory(device, &allocateInfo, nullptr, &memory), "vkAllocateMemory");
    return memory;
}
}  // namespace

GpuBuffer CreateMappedBuffer(VkDevice device, VkPhysicalDevice physicalDevice, VkDeviceSize size, VkBufferUsageFlags usage) {
    GpuBuffer result;
    result.size = size;

    VkBufferCreateInfo createInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    createInfo.size = size;
    createInfo.usage = usage;
    createInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VkCheck(vkCreateBuffer(device, &createInfo, nullptr, &result.buffer), "vkCreateBuffer");

    VkMemoryRequirements requirements;
    vkGetBufferMemoryRequirements(device, result.buffer, &requirements);
    // The CPU must be able to write it. When the GPU also offers CPU-writable memory on the
    // graphics card itself, prefer that so draws do not have to read across the PCIe bus.
    result.memory = AllocateMemory(device, physicalDevice, requirements,
                                   VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                   VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VkCheck(vkBindBufferMemory(device, result.buffer, result.memory, 0), "vkBindBufferMemory");
    VkCheck(vkMapMemory(device, result.memory, 0, VK_WHOLE_SIZE, 0, &result.mapped), "vkMapMemory");
    return result;
}

void DestroyBuffer(VkDevice device, GpuBuffer& buffer) {
    vkDestroyBuffer(device, buffer.buffer, nullptr);
    vkFreeMemory(device, buffer.memory, nullptr);  // also unmaps it
    buffer = {};
}

GpuImage CreateImage(VkDevice device, VkPhysicalDevice physicalDevice, const ImageDesc& desc) {
    GpuImage result;
    result.format = desc.format;
    result.extent = desc.extent;

    VkImageCreateInfo createInfo{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    createInfo.imageType = VK_IMAGE_TYPE_2D;
    createInfo.format = desc.format;
    createInfo.extent = {desc.extent.width, desc.extent.height, 1};
    createInfo.mipLevels = 1;
    createInfo.arrayLayers = 1;
    createInfo.samples = desc.samples;
    createInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    createInfo.usage = desc.usage;
    createInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    createInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    VkCheck(vkCreateImage(device, &createInfo, nullptr, &result.image), "vkCreateImage");

    VkMemoryRequirements requirements;
    vkGetImageMemoryRequirements(device, result.image, &requirements);
    result.memory = AllocateMemory(device, physicalDevice, requirements, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 0);
    VkCheck(vkBindImageMemory(device, result.image, result.memory, 0), "vkBindImageMemory");

    VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    viewInfo.image = result.image;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = desc.format;
    viewInfo.subresourceRange = {desc.aspect, 0, 1, 0, 1};
    VkCheck(vkCreateImageView(device, &viewInfo, nullptr, &result.view), "vkCreateImageView");
    return result;
}

void DestroyImage(VkDevice device, GpuImage& image) {
    vkDestroyImageView(device, image.view, nullptr);
    vkDestroyImage(device, image.image, nullptr);
    vkFreeMemory(device, image.memory, nullptr);
    image = {};
}

VkShaderModule LoadShaderModule(VkDevice device, const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        throw std::runtime_error("Cannot open shader " + path);
    }
    const std::streamsize byteCount = file.tellg();
    if (byteCount <= 0 || byteCount % sizeof(uint32_t) != 0) {
        throw std::runtime_error("Shader " + path + " is not valid SPIR-V");
    }

    // SPIR-V is a sequence of 32-bit words; reading into uint32_t storage keeps it correctly aligned.
    std::vector<uint32_t> words(static_cast<size_t>(byteCount) / sizeof(uint32_t));
    file.seekg(0);
    file.read(reinterpret_cast<char*>(words.data()), byteCount);

    VkShaderModuleCreateInfo createInfo{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    createInfo.codeSize = words.size() * sizeof(uint32_t);
    createInfo.pCode = words.data();
    VkShaderModule module = VK_NULL_HANDLE;
    VkCheck(vkCreateShaderModule(device, &createInfo, nullptr, &module), "vkCreateShaderModule");
    return module;
}

void TransitionImage(VkCommandBuffer commandBuffer, const ImageTransition& transition) {
    VkImageMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
    barrier.srcStageMask = transition.srcStage;
    barrier.srcAccessMask = transition.srcAccess;
    barrier.dstStageMask = transition.dstStage;
    barrier.dstAccessMask = transition.dstAccess;
    barrier.oldLayout = transition.oldLayout;
    barrier.newLayout = transition.newLayout;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = transition.image;
    barrier.subresourceRange = {transition.aspect, 0, 1, 0, 1};

    VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    dependency.imageMemoryBarrierCount = 1;
    dependency.pImageMemoryBarriers = &barrier;
    vkCmdPipelineBarrier2(commandBuffer, &dependency);
}
