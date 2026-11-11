#include "render/ScenePass.h"

#include "render/PipelineBuilder.h"
#include "render/RenderTargets.h"
#include "render/SceneGeometry.h"
#include "render/VulkanContext.h"

void ScenePass::Create(const VulkanContext& context, VkPipelineLayout sceneLayout, VkSampleCountFlagBits samples) {
    layout = sceneLayout;

    // Lit geometry. Depth is reversed (1 = near, 0 = far), so nearer surfaces have GREATER depth.
    PipelineDesc lit;
    lit.layout = sceneLayout;
    lit.fragmentShader = "lit.frag";
    lit.colorFormat = kHdrFormat;
    lit.depthFormat = kDepthFormat;
    lit.samples = samples;
    lit.cullMode = VK_CULL_MODE_BACK_BIT;
    lit.depthTest = true;
    lit.depthWrite = true;
    lit.depthCompare = VK_COMPARE_OP_GREATER;
    lit.vertexShader = "box.vert";
    boxPipeline = CreateGraphicsPipeline(context.device, lit);
    lit.vertexShader = "npc.vert";
    npcPipeline = CreateGraphicsPipeline(context.device, lit);

    // The sky is a full-screen triangle at the far plane, drawn only where nothing else was.
    PipelineDesc sky;
    sky.layout = sceneLayout;
    sky.vertexShader = "fullscreen.vert";
    sky.fragmentShader = "sky.frag";
    sky.usesMeshInput = false;
    sky.colorFormat = kHdrFormat;
    sky.depthFormat = kDepthFormat;
    sky.samples = samples;
    sky.depthTest = true;
    sky.depthCompare = VK_COMPARE_OP_GREATER_OR_EQUAL;
    skyPipeline = CreateGraphicsPipeline(context.device, sky);
}

void ScenePass::Destroy(VkDevice device) {
    vkDestroyPipeline(device, boxPipeline, nullptr);
    vkDestroyPipeline(device, npcPipeline, nullptr);
    vkDestroyPipeline(device, skyPipeline, nullptr);
    boxPipeline = VK_NULL_HANDLE;
    npcPipeline = VK_NULL_HANDLE;
    skyPipeline = VK_NULL_HANDLE;
}

void ScenePass::Record(VkCommandBuffer cmd, const RenderTargets& targets, VkExtent2D renderExtent,
                       const SceneGeometry& geometry, uint32_t frameSlot, const glm::mat4& cameraViewProjection) const {
    TransitionTargetsForDrawing(cmd, targets);

    const bool useMsaa = targets.UsesMsaa();
    VkRenderingAttachmentInfo colorAttachment{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    colorAttachment.imageView = useMsaa ? targets.hdrMultisampled.view : targets.hdrResolved.view;
    colorAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = useMsaa ? VK_ATTACHMENT_STORE_OP_DONT_CARE : VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.clearValue.color = {{0.0f, 0.0f, 0.0f, 1.0f}};
    if (useMsaa) {
        // At the end of the pass, average each pixel's samples into the single-sample image.
        colorAttachment.resolveMode = VK_RESOLVE_MODE_AVERAGE_BIT;
        colorAttachment.resolveImageView = targets.hdrResolved.view;
        colorAttachment.resolveImageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    }

    VkRenderingAttachmentInfo depthAttachment{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    depthAttachment.imageView = targets.depth.view;
    depthAttachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.clearValue.depthStencil = {0.0f, 0};  // reversed depth: 0 is the far plane

    // The render area, viewport and scissor all cover only the scaled part of the targets, so
    // clearing, drawing and the MSAA resolve cost less at a lower render scale.
    BeginRendering(cmd, renderExtent, &colorAttachment, &depthAttachment);
    vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4), &cameraViewProjection);
    geometry.DrawBoxes(cmd, boxPipeline);
    geometry.DrawNpcs(cmd, npcPipeline, frameSlot);
    // The sky goes last and only fills the pixels no geometry covered.
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, skyPipeline);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRendering(cmd);

    TransitionImage(cmd, {.image = targets.hdrResolved.image,
                          .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                          .oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          .newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                          .srcStage = kColorOutputStage,
                          .srcAccess = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                          .dstStage = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                          .dstAccess = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT});
}

void ScenePass::TransitionTargetsForDrawing(VkCommandBuffer cmd, const RenderTargets& targets) const {
    if (targets.UsesMsaa()) {
        TransitionImage(cmd, {.image = targets.hdrMultisampled.image,
                              .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                              .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
                              .newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                              .srcStage = kColorOutputStage,
                              .srcAccess = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                              .dstStage = kColorOutputStage,
                              .dstAccess = VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT});
    }
    // The previous frame's tonemap pass may still be reading the resolved image.
    TransitionImage(cmd, {.image = targets.hdrResolved.image,
                          .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                          .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
                          .newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          .srcStage = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | kColorOutputStage,
                          .srcAccess = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                          .dstStage = kColorOutputStage,
                          .dstAccess = VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT});
    TransitionImage(cmd, {.image = targets.depth.image,
                          .aspect = VK_IMAGE_ASPECT_DEPTH_BIT,
                          .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
                          .newLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                          .srcStage = kDepthTestStages,
                          .srcAccess = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                          .dstStage = kDepthTestStages,
                          .dstAccess = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                                       VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT});
}
