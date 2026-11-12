#include "render/ShadowPass.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/matrix_transform.hpp>

#include "render/PipelineBuilder.h"
#include "render/RenderTargets.h"
#include "render/SceneGeometry.h"
#include "render/VulkanContext.h"

void ShadowPass::Create(const VulkanContext& context, VkPipelineLayout sceneLayout) {
    layout = sceneLayout;
    shadowMap = CreateImage(context.device, context.physicalDevice,
                            {.extent = {kMapSize, kMapSize},
                             .format = kDepthFormat,
                             .usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                             .aspect = VK_IMAGE_ASPECT_DEPTH_BIT});

    // A comparison sampler: the GPU compares the stored depth with ours and blends the results
    // of neighbouring texels, which softens shadow edges for free.
    VkSamplerCreateInfo samplerInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;  // outside the map counts as lit
    samplerInfo.compareEnable = VK_TRUE;
    samplerInfo.compareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    VkCheck(vkCreateSampler(context.device, &samplerInfo, nullptr, &comparisonSampler), "vkCreateSampler");

    // Depth only, from the sun. The shadow map uses ordinary depth (0 = near).
    PipelineDesc shadow;
    shadow.layout = sceneLayout;
    shadow.depthFormat = kDepthFormat;
    shadow.depthTest = true;
    shadow.depthWrite = true;
    shadow.depthCompare = VK_COMPARE_OP_LESS_OR_EQUAL;
    shadow.depthBias = true;
    shadow.depthClamp = context.supportsDepthClamp;
    shadow.vertexShader = "box.vert";
    shadow.meshInput = MeshInput::Boxes;
    boxPipeline = CreateGraphicsPipeline(context.device, shadow);
    shadow.vertexShader = "body.vert";
    bodyPipeline = CreateGraphicsPipeline(context.device, shadow);
    shadow.vertexShader = "npc.vert";
    shadow.meshInput = MeshInput::People;
    npcPipeline = CreateGraphicsPipeline(context.device, shadow);
}

void ShadowPass::Destroy(VkDevice device) {
    vkDestroyPipeline(device, boxPipeline, nullptr);
    vkDestroyPipeline(device, npcPipeline, nullptr);
    vkDestroyPipeline(device, bodyPipeline, nullptr);
    vkDestroySampler(device, comparisonSampler, nullptr);
    DestroyImage(device, shadowMap);
    bodyPipeline = VK_NULL_HANDLE;
    boxPipeline = VK_NULL_HANDLE;
    npcPipeline = VK_NULL_HANDLE;
    comparisonSampler = VK_NULL_HANDLE;
}

glm::mat4 ShadowPass::ComputeLightViewProjection(const FrameInput& input) {
    const glm::vec3 camera = input.cameraPosition;
    const glm::vec3 forward = -glm::vec3(input.view[0][2], input.view[1][2], input.view[2][2]);

    // The shadow map covers a square around where the camera is looking. It grows (in 20 m steps)
    // as the camera rises, trading shadow sharpness for coverage.
    const float radius = std::clamp(std::ceil(std::max(camera.y, 0.0f) * 1.5f / 20.0f) * 20.0f, 80.0f, 320.0f);
    glm::vec3 focus = camera;
    if (forward.y < -0.05f) {
        // Looking down: centre on the spot where the view ray meets the ground.
        const float distanceToGround = std::min(std::max(camera.y, 0.0f) / -forward.y, radius * 1.5f);
        focus = camera + forward * distanceToGround;
    } else {
        // Looking at the horizon: centre a little ahead of the camera.
        const glm::vec2 flatForward(forward.x, forward.z);
        if (glm::length(flatForward) > 0.01f) {
            const glm::vec2 ahead = glm::normalize(flatForward) * (radius * 0.6f);
            focus += glm::vec3(ahead.x, 0.0f, ahead.y);
        }
    }
    focus.y = 0.0f;

    const glm::vec3 sun = glm::normalize(input.sunDirection);
    const glm::vec3 up = std::abs(sun.y) > 0.99f ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::mat4 lightRotation = glm::lookAt(glm::vec3(0.0f), -sun, up);

    // Snap the centre to whole shadow-map texels; otherwise shadow edges shimmer as the camera moves.
    const float texelSize = 2.0f * radius / static_cast<float>(kMapSize);
    glm::vec3 center = glm::vec3(lightRotation * glm::vec4(focus, 1.0f));
    center.x = std::floor(center.x / texelSize) * texelSize;
    center.y = std::floor(center.y / texelSize) * texelSize;

    // In light space the sun looks down -Z, so the box spans view depths -(z + range) to -(z - range).
    const float depthRange = 500.0f;
    const glm::mat4 projection = glm::orthoRH_ZO(center.x - radius, center.x + radius, center.y - radius, center.y + radius,
                                                 -center.z - depthRange, -center.z + depthRange);
    return projection * lightRotation;
}

void ShadowPass::Record(VkCommandBuffer cmd, const SceneGeometry& geometry, uint32_t frameSlot,
                        const glm::mat4& lightViewProjection) const {
    // The previous frame's scene pass may still be reading the shadow map; wait for it.
    TransitionImage(cmd, {.image = shadowMap.image,
                          .aspect = VK_IMAGE_ASPECT_DEPTH_BIT,
                          .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
                          .newLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                          .srcStage = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                          .srcAccess = VK_ACCESS_2_NONE,
                          .dstStage = kDepthTestStages,
                          .dstAccess = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                                       VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT});

    VkRenderingAttachmentInfo depthAttachment{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    depthAttachment.imageView = shadowMap.view;
    depthAttachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    depthAttachment.clearValue.depthStencil = {1.0f, 0};

    BeginRendering(cmd, {kMapSize, kMapSize}, nullptr, &depthAttachment);
    vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4), &lightViewProjection);
    geometry.DrawBoxes(cmd, boxPipeline);
    geometry.DrawBodies(cmd, bodyPipeline, frameSlot);
    geometry.DrawNpcs(cmd, npcPipeline, frameSlot);
    vkCmdEndRendering(cmd);

    TransitionImage(cmd, {.image = shadowMap.image,
                          .aspect = VK_IMAGE_ASPECT_DEPTH_BIT,
                          .oldLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                          .newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                          .srcStage = VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                          .srcAccess = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                          .dstStage = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                          .dstAccess = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT});
}
