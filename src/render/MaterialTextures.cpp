#include "render/MaterialTextures.h"

#include <algorithm>
#include <cstring>

namespace {

// Copies every mip level of an array of textures from memory into `image`, and leaves it ready to be sampled.
void UploadArray(const VulkanContext& context, const material::MaterialLibrary& library,
                 const std::vector<std::vector<uint8_t>>& levels, const GpuImage& image) {
    VkDeviceSize totalBytes = 0;
    for (const std::vector<uint8_t>& level : levels) {
        totalBytes += level.size();
    }
    GpuBuffer staging = CreateMappedBuffer(context.device, context.physicalDevice, totalBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT);

    // One copy region per mip level; each covers all layers, which are stored one after another.
    std::vector<VkBufferImageCopy> regions;
    VkDeviceSize offset = 0;
    for (int level = 0; level < library.mipLevels; ++level) {
        std::memcpy(static_cast<uint8_t*>(staging.mapped) + offset, levels[level].data(), levels[level].size());
        VkBufferImageCopy region{};
        region.bufferOffset = offset;
        region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, static_cast<uint32_t>(level), 0, static_cast<uint32_t>(library.layers)};
        region.imageExtent = {static_cast<uint32_t>(library.LevelSize(level)), static_cast<uint32_t>(library.LevelSize(level)), 1};
        regions.push_back(region);
        offset += levels[level].size();
    }

    VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    poolInfo.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
    poolInfo.queueFamilyIndex = context.queueFamily;
    VkCommandPool pool = VK_NULL_HANDLE;
    VkCheck(vkCreateCommandPool(context.device, &poolInfo, nullptr, &pool), "vkCreateCommandPool");
    VkCommandBufferAllocateInfo allocateInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocateInfo.commandPool = pool;
    allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocateInfo.commandBufferCount = 1;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    VkCheck(vkAllocateCommandBuffers(context.device, &allocateInfo, &cmd), "vkAllocateCommandBuffers");

    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VkCheck(vkBeginCommandBuffer(cmd, &beginInfo), "vkBeginCommandBuffer");

    const VkImageSubresourceRange everything{VK_IMAGE_ASPECT_COLOR_BIT, 0, static_cast<uint32_t>(library.mipLevels), 0,
                                             static_cast<uint32_t>(library.layers)};
    const auto transition = [&](VkImageLayout from, VkImageLayout to, VkPipelineStageFlags2 srcStage, VkAccessFlags2 srcAccess,
                                VkPipelineStageFlags2 dstStage, VkAccessFlags2 dstAccess) {
        VkImageMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
        barrier.srcStageMask = srcStage;
        barrier.srcAccessMask = srcAccess;
        barrier.dstStageMask = dstStage;
        barrier.dstAccessMask = dstAccess;
        barrier.oldLayout = from;
        barrier.newLayout = to;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image.image;
        barrier.subresourceRange = everything;
        VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
        dependency.imageMemoryBarrierCount = 1;
        dependency.pImageMemoryBarriers = &barrier;
        vkCmdPipelineBarrier2(cmd, &dependency);
    };
    transition(VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_NONE, VK_ACCESS_2_NONE,
               VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
    vkCmdCopyBufferToImage(cmd, staging.buffer, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                           static_cast<uint32_t>(regions.size()), regions.data());
    transition(VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_COPY_BIT,
               VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
    VkCheck(vkEndCommandBuffer(cmd), "vkEndCommandBuffer");

    VkCommandBufferSubmitInfo commandInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO};
    commandInfo.commandBuffer = cmd;
    VkSubmitInfo2 submit{VK_STRUCTURE_TYPE_SUBMIT_INFO_2};
    submit.commandBufferInfoCount = 1;
    submit.pCommandBufferInfos = &commandInfo;
    VkCheck(vkQueueSubmit2(context.queue, 1, &submit, VK_NULL_HANDLE), "vkQueueSubmit2");
    VkCheck(vkQueueWaitIdle(context.queue), "vkQueueWaitIdle");  // start-up only, so waiting is simplest

    vkDestroyCommandPool(context.device, pool, nullptr);
    DestroyBuffer(context.device, staging);
}

GpuImage CreateArrayImage(const VulkanContext& context, const material::MaterialLibrary& library, VkFormat format) {
    ImageDesc desc{};
    desc.extent = {static_cast<uint32_t>(library.size), static_cast<uint32_t>(library.size)};
    desc.format = format;
    desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    desc.aspect = VK_IMAGE_ASPECT_COLOR_BIT;
    desc.mipLevels = static_cast<uint32_t>(library.mipLevels);
    desc.arrayLayers = static_cast<uint32_t>(library.layers);
    return CreateImage(context.device, context.physicalDevice, desc);
}

}  // namespace

void MaterialTextures::Create(const VulkanContext& context, const material::MaterialLibrary& library) {
    albedoRoughness = CreateArrayImage(context, library, VK_FORMAT_R8G8B8A8_SRGB);
    normalOcclusion = CreateArrayImage(context, library, VK_FORMAT_R8G8B8A8_UNORM);
    UploadArray(context, library, library.albedoRoughness, albedoRoughness);
    UploadArray(context, library, library.normalOcclusion, normalOcclusion);
    tileMeters = library.tileMeters;

    VkSamplerCreateInfo samplerInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    samplerInfo.anisotropyEnable = context.supportsSamplerAnisotropy ? VK_TRUE : VK_FALSE;
    samplerInfo.maxAnisotropy = std::min(8.0f, context.deviceProperties.limits.maxSamplerAnisotropy);
    samplerInfo.maxLod = VK_LOD_CLAMP_NONE;
    VkCheck(vkCreateSampler(context.device, &samplerInfo, nullptr, &sampler), "vkCreateSampler");
}

void MaterialTextures::Destroy(VkDevice device) {
    vkDestroySampler(device, sampler, nullptr);
    sampler = VK_NULL_HANDLE;
    if (albedoRoughness.image != VK_NULL_HANDLE) {
        DestroyImage(device, albedoRoughness);
    }
    if (normalOcclusion.image != VK_NULL_HANDLE) {
        DestroyImage(device, normalOcclusion);
    }
}
