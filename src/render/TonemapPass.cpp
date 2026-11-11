#include "render/TonemapPass.h"

#include <glm/glm.hpp>

#include "render/PipelineBuilder.h"
#include "render/RenderTargets.h"
#include "render/VulkanContext.h"
#include "render/VulkanUtils.h"

namespace {
constexpr float kExposure = 0.9f;

// Mirrors the push constant block in shaders/tonemap.frag.
struct TonemapPushConstants {
    glm::vec4 settings;      // x: exposure
    glm::vec4 sourceRegion;  // xy: UV scale of the drawn part of the HDR image, zw: largest UV to sample
};
}  // namespace

void TonemapPass::Create(const VulkanContext& context, const RenderTargets& targets) {
    VkSamplerCreateInfo samplerInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    VkCheck(vkCreateSampler(context.device, &samplerInfo, nullptr, &linearSampler), "vkCreateSampler");

    // The tonemap shader reads only the HDR scene image. All frames share that image, so one
    // descriptor set is enough.
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = &binding;
    VkCheck(vkCreateDescriptorSetLayout(context.device, &layoutInfo, nullptr, &setLayout), "vkCreateDescriptorSetLayout");

    const VkDescriptorPoolSize poolSize{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1};
    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.maxSets = 1;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    VkCheck(vkCreateDescriptorPool(context.device, &poolInfo, nullptr, &descriptorPool), "vkCreateDescriptorPool");

    VkDescriptorSetAllocateInfo allocateInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    allocateInfo.descriptorPool = descriptorPool;
    allocateInfo.descriptorSetCount = 1;
    allocateInfo.pSetLayouts = &setLayout;
    VkCheck(vkAllocateDescriptorSets(context.device, &allocateInfo, &descriptorSet), "vkAllocateDescriptorSets");
    BindTargets(context.device, targets);

    pipelineLayout = CreatePipelineLayout(context.device, setLayout, VK_SHADER_STAGE_FRAGMENT_BIT,
                                          sizeof(TonemapPushConstants));
    PipelineDesc tonemap;
    tonemap.layout = pipelineLayout;
    tonemap.vertexShader = "fullscreen.vert";
    tonemap.fragmentShader = "tonemap.frag";
    tonemap.meshInput = MeshInput::None;
    tonemap.colorFormat = context.swapchainFormat;
    pipeline = CreateGraphicsPipeline(context.device, tonemap);
}

void TonemapPass::Destroy(VkDevice device) {
    vkDestroyPipeline(device, pipeline, nullptr);
    vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
    vkDestroyDescriptorPool(device, descriptorPool, nullptr);
    vkDestroyDescriptorSetLayout(device, setLayout, nullptr);
    vkDestroySampler(device, linearSampler, nullptr);
    pipeline = VK_NULL_HANDLE;
    pipelineLayout = VK_NULL_HANDLE;
    descriptorPool = VK_NULL_HANDLE;
    descriptorSet = VK_NULL_HANDLE;
    setLayout = VK_NULL_HANDLE;
    linearSampler = VK_NULL_HANDLE;
}

void TonemapPass::BindTargets(VkDevice device, const RenderTargets& targets) {
    const VkDescriptorImageInfo hdrInfo{linearSampler, targets.hdrResolved.view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
    VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    write.dstSet = descriptorSet;
    write.dstBinding = 0;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    write.pImageInfo = &hdrInfo;
    vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
}

void TonemapPass::Record(VkCommandBuffer cmd, VkImage swapchainImage, VkImageView swapchainView, VkExtent2D windowExtent,
                         const RenderTargets& targets, VkExtent2D renderExtent) const {
    // The submit waits for the image to be acquired at this stage, so the barrier starts there too.
    TransitionImage(cmd, {.image = swapchainImage,
                          .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                          .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
                          .newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          .srcStage = kColorOutputStage,
                          .srcAccess = VK_ACCESS_2_NONE,
                          .dstStage = kColorOutputStage,
                          .dstAccess = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT});

    VkRenderingAttachmentInfo colorAttachment{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    colorAttachment.imageView = swapchainView;
    colorAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;  // every pixel gets overwritten
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

    // The scene only covers the top-left renderExtent of the HDR image. Scaling the UVs maps
    // the whole window onto that part, and clamping half a texel inside its edge stops the
    // bilinear filter from blending in stale pixels from outside it.
    const glm::vec2 fullSize(static_cast<float>(targets.fullExtent.width), static_cast<float>(targets.fullExtent.height));
    const glm::vec2 drawnSize(static_cast<float>(renderExtent.width), static_cast<float>(renderExtent.height));
    TonemapPushConstants push;
    push.settings = glm::vec4(kExposure, 0.0f, 0.0f, 0.0f);
    push.sourceRegion = glm::vec4(drawnSize / fullSize, (drawnSize - 0.5f) / fullSize);

    BeginRendering(cmd, windowExtent, &colorAttachment, nullptr);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
    vkCmdPushConstants(cmd, pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push), &push);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRendering(cmd);
}
