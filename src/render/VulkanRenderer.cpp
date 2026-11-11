#include "render/VulkanRenderer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <initializer_list>
#include <iostream>
#include <stdexcept>

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

// Registers this renderer under the name "Vulkan", so main() can create it through the
// ModuleRegistry without knowing this class exists.
static AutoRegister<VulkanRenderer> gVulkanRendererRegistration("Vulkan");

namespace {
constexpr VkFormat kHdrFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;
constexpr uint32_t kShadowMapSize = 4096;
constexpr size_t kInitialNpcCapacity = 16384;
constexpr float kExposure = 0.9f;
constexpr float kFogDensity = 0.0022f;

constexpr VkPipelineStageFlags2 kColorOutputStage = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
constexpr VkPipelineStageFlags2 kDepthTestStages =
    VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;

// Mirrors FrameUniforms in shaders/common.glsl. It only holds mat4 and vec4 members, so the
// C++ layout matches the shader's std140 layout without any padding rules to think about.
struct FrameUniforms {
    glm::mat4 viewProjection;
    glm::mat4 inverseViewProjection;
    glm::mat4 lightViewProjection;
    glm::vec4 cameraPosition;  // w: time in seconds
    glm::vec4 sunDirection;
    glm::vec4 sunColor;
    glm::vec4 settings;        // x: shadow map texel size, y: fog density
};

std::string ShaderPath(const char* name) {
    return std::string(XCOM_SHADER_DIR) + "/" + name + ".spv";
}

// Sun colour and brightness by height: warm and dim near the horizon, white and bright higher up.
glm::vec3 SunRadiance(float sunHeight) {
    const glm::vec3 lowSun(1.0f, 0.52f, 0.28f);
    const glm::vec3 highSun(1.0f, 0.95f, 0.88f);
    const float warmth = glm::smoothstep(0.0f, 0.6f, sunHeight);
    const float intensity = 7.0f * glm::smoothstep(-0.04f, 0.12f, sunHeight);
    return glm::mix(lowSun, highSun, warmth) * intensity;
}

VkSampleCountFlagBits ChooseMsaaSamples(const VkPhysicalDeviceProperties& properties) {
    const VkSampleCountFlags supported =
        properties.limits.framebufferColorSampleCounts & properties.limits.framebufferDepthSampleCounts;
    return (supported & VK_SAMPLE_COUNT_4_BIT) ? VK_SAMPLE_COUNT_4_BIT : VK_SAMPLE_COUNT_1_BIT;
}

// Starts dynamic rendering over `extent` and sets a matching viewport and scissor.
void BeginRendering(VkCommandBuffer cmd, VkExtent2D extent, const VkRenderingAttachmentInfo* color,
                    const VkRenderingAttachmentInfo* depth) {
    VkRenderingInfo info{VK_STRUCTURE_TYPE_RENDERING_INFO};
    info.renderArea = {{0, 0}, extent};
    info.layerCount = 1;
    info.colorAttachmentCount = color != nullptr ? 1 : 0;
    info.pColorAttachments = color;
    info.pDepthAttachment = depth;
    vkCmdBeginRendering(cmd, &info);

    const VkViewport viewport{0.0f, 0.0f, static_cast<float>(extent.width), static_cast<float>(extent.height), 0.0f, 1.0f};
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    const VkRect2D scissor{{0, 0}, extent};
    vkCmdSetScissor(cmd, 0, 1, &scissor);
}
}  // namespace

// ---------------------------------------------------------------------------
// Start-up and shutdown
// ---------------------------------------------------------------------------

bool VulkanRenderer::Initialize(EngineContext& engine) {
    try {
        window = engine.window;
        context.Create(engine.window, engine.vsync);
        msaaSamples = ChooseMsaaSamples(context.deviceProperties);
        CreateFrameResources();
        CreateShadowMapAndSamplers();
        CreateDescriptors();
        CreateSizeDependentResources();
        CreatePipelines();
        cubeMesh = UploadMesh(CreateUnitCube());
        personMesh = UploadMesh(CreatePersonMesh());
        return true;
    } catch (const std::exception& error) {
        std::cerr << "Vulkan renderer failed to start: " << error.what() << "\n";
        return false;
    }
}

void VulkanRenderer::Shutdown() {
    if (context.device == VK_NULL_HANDLE) {
        context.Destroy();  // at most the instance exists; nothing else to clean up
        return;
    }
    VkDevice device = context.device;
    vkDeviceWaitIdle(device);

    for (VkPipeline pipeline : {boxPipeline, npcPipeline, boxShadowPipeline, npcShadowPipeline, skyPipeline, tonemapPipeline}) {
        vkDestroyPipeline(device, pipeline, nullptr);
    }
    vkDestroyPipelineLayout(device, scenePipelineLayout, nullptr);
    vkDestroyPipelineLayout(device, tonemapPipelineLayout, nullptr);
    vkDestroyDescriptorPool(device, descriptorPool, nullptr);  // also frees the descriptor sets
    vkDestroyDescriptorSetLayout(device, sceneSetLayout, nullptr);
    vkDestroyDescriptorSetLayout(device, tonemapSetLayout, nullptr);

    DestroySizeDependentResources();
    for (VkSemaphore semaphore : renderFinished) {
        vkDestroySemaphore(device, semaphore, nullptr);
    }
    renderFinished.clear();
    DestroyImage(device, shadowMap);
    vkDestroySampler(device, shadowSampler, nullptr);
    vkDestroySampler(device, linearSampler, nullptr);

    DestroyMesh(cubeMesh);
    DestroyMesh(personMesh);
    DestroyBuffer(device, boxInstances);

    for (FrameResources& frame : frames) {
        DestroyBuffer(device, frame.uniforms);
        DestroyBuffer(device, frame.npcInstances);
        vkDestroyFence(device, frame.inFlight, nullptr);
        vkDestroySemaphore(device, frame.imageAvailable, nullptr);
        vkDestroyCommandPool(device, frame.commandPool, nullptr);  // also frees its command buffer
        frame = {};
    }
    context.Destroy();
}

std::string VulkanRenderer::GetDeviceName() const {
    return context.deviceProperties.deviceName;
}

void VulkanRenderer::CreateFrameResources() {
    for (FrameResources& frame : frames) {
        VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        poolInfo.queueFamilyIndex = context.queueFamily;
        VkCheck(vkCreateCommandPool(context.device, &poolInfo, nullptr, &frame.commandPool), "vkCreateCommandPool");

        VkCommandBufferAllocateInfo allocateInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        allocateInfo.commandPool = frame.commandPool;
        allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocateInfo.commandBufferCount = 1;
        VkCheck(vkAllocateCommandBuffers(context.device, &allocateInfo, &frame.commandBuffer), "vkAllocateCommandBuffers");

        VkSemaphoreCreateInfo semaphoreInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
        VkCheck(vkCreateSemaphore(context.device, &semaphoreInfo, nullptr, &frame.imageAvailable), "vkCreateSemaphore");

        // Created signalled, so waiting on it before the very first frame returns at once.
        VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        VkCheck(vkCreateFence(context.device, &fenceInfo, nullptr, &frame.inFlight), "vkCreateFence");

        frame.uniforms = CreateMappedBuffer(context.device, context.physicalDevice, sizeof(FrameUniforms),
                                            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
        frame.npcInstances = CreateMappedBuffer(context.device, context.physicalDevice,
                                                kInitialNpcCapacity * sizeof(NpcInstance), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    }
    npcCapacity = kInitialNpcCapacity;
}

void VulkanRenderer::CreateShadowMapAndSamplers() {
    shadowMap = CreateImage(context.device, context.physicalDevice,
                            {.extent = {kShadowMapSize, kShadowMapSize},
                             .format = kDepthFormat,
                             .usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                             .aspect = VK_IMAGE_ASPECT_DEPTH_BIT});

    // A comparison sampler: the GPU compares the stored depth with ours and blends the results
    // of neighbouring texels, which softens shadow edges for free.
    VkSamplerCreateInfo shadowInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    shadowInfo.magFilter = VK_FILTER_LINEAR;
    shadowInfo.minFilter = VK_FILTER_LINEAR;
    shadowInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    shadowInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    shadowInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    shadowInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    shadowInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;  // outside the map counts as lit
    shadowInfo.compareEnable = VK_TRUE;
    shadowInfo.compareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    VkCheck(vkCreateSampler(context.device, &shadowInfo, nullptr, &shadowSampler), "vkCreateSampler");

    VkSamplerCreateInfo linearInfo{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    linearInfo.magFilter = VK_FILTER_LINEAR;
    linearInfo.minFilter = VK_FILTER_LINEAR;
    linearInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    linearInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    linearInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    VkCheck(vkCreateSampler(context.device, &linearInfo, nullptr, &linearSampler), "vkCreateSampler");
}

void VulkanRenderer::CreateDescriptors() {
    // Scene shaders read the per-frame uniforms and the shadow map.
    VkDescriptorSetLayoutBinding sceneBindings[2]{};
    sceneBindings[0].binding = 0;
    sceneBindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    sceneBindings[0].descriptorCount = 1;
    sceneBindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    sceneBindings[1].binding = 1;
    sceneBindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    sceneBindings[1].descriptorCount = 1;
    sceneBindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo sceneLayoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    sceneLayoutInfo.bindingCount = 2;
    sceneLayoutInfo.pBindings = sceneBindings;
    VkCheck(vkCreateDescriptorSetLayout(context.device, &sceneLayoutInfo, nullptr, &sceneSetLayout),
            "vkCreateDescriptorSetLayout");

    // The tonemap shader reads the HDR scene image.
    VkDescriptorSetLayoutBinding tonemapBinding{};
    tonemapBinding.binding = 0;
    tonemapBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    tonemapBinding.descriptorCount = 1;
    tonemapBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo tonemapLayoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    tonemapLayoutInfo.bindingCount = 1;
    tonemapLayoutInfo.pBindings = &tonemapBinding;
    VkCheck(vkCreateDescriptorSetLayout(context.device, &tonemapLayoutInfo, nullptr, &tonemapSetLayout),
            "vkCreateDescriptorSetLayout");

    const VkDescriptorPoolSize poolSizes[] = {
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, kFramesInFlight},
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, kFramesInFlight * 2},
    };
    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.maxSets = kFramesInFlight * 2;
    poolInfo.poolSizeCount = 2;
    poolInfo.pPoolSizes = poolSizes;
    VkCheck(vkCreateDescriptorPool(context.device, &poolInfo, nullptr, &descriptorPool), "vkCreateDescriptorPool");

    for (FrameResources& frame : frames) {
        const VkDescriptorSetLayout layouts[] = {sceneSetLayout, tonemapSetLayout};
        VkDescriptorSet sets[2] = {};
        VkDescriptorSetAllocateInfo allocateInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocateInfo.descriptorPool = descriptorPool;
        allocateInfo.descriptorSetCount = 2;
        allocateInfo.pSetLayouts = layouts;
        VkCheck(vkAllocateDescriptorSets(context.device, &allocateInfo, sets), "vkAllocateDescriptorSets");
        frame.sceneDescriptors = sets[0];
        frame.tonemapDescriptors = sets[1];

        const VkDescriptorBufferInfo uniformInfo{frame.uniforms.buffer, 0, sizeof(FrameUniforms)};
        const VkDescriptorImageInfo shadowInfo{shadowSampler, shadowMap.view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkWriteDescriptorSet writes[2]{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = frame.sceneDescriptors;
        writes[0].dstBinding = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writes[0].pBufferInfo = &uniformInfo;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = frame.sceneDescriptors;
        writes[1].dstBinding = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[1].pImageInfo = &shadowInfo;
        vkUpdateDescriptorSets(context.device, 2, writes, 0, nullptr);
    }

    // Scene pipelines also take the view-projection matrix as a push constant: the camera's in
    // the scene pass and the sun's in the shadow pass, so the same vertex shaders serve both.
    const VkPushConstantRange scenePushRange{VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4)};
    VkPipelineLayoutCreateInfo sceneLayout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    sceneLayout.setLayoutCount = 1;
    sceneLayout.pSetLayouts = &sceneSetLayout;
    sceneLayout.pushConstantRangeCount = 1;
    sceneLayout.pPushConstantRanges = &scenePushRange;
    VkCheck(vkCreatePipelineLayout(context.device, &sceneLayout, nullptr, &scenePipelineLayout), "vkCreatePipelineLayout");

    const VkPushConstantRange tonemapPushRange{VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(glm::vec4)};
    VkPipelineLayoutCreateInfo tonemapLayout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    tonemapLayout.setLayoutCount = 1;
    tonemapLayout.pSetLayouts = &tonemapSetLayout;
    tonemapLayout.pushConstantRangeCount = 1;
    tonemapLayout.pPushConstantRanges = &tonemapPushRange;
    VkCheck(vkCreatePipelineLayout(context.device, &tonemapLayout, nullptr, &tonemapPipelineLayout), "vkCreatePipelineLayout");
}

void VulkanRenderer::CreateSizeDependentResources() {
    const VkExtent2D extent = context.swapchainExtent;
    if (msaaSamples != VK_SAMPLE_COUNT_1_BIT) {
        hdrMultisampled = CreateImage(context.device, context.physicalDevice,
                                      {.extent = extent,
                                       .format = kHdrFormat,
                                       .usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT,
                                       .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                                       .samples = msaaSamples});
    }
    hdrResolved = CreateImage(context.device, context.physicalDevice,
                              {.extent = extent,
                               .format = kHdrFormat,
                               .usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                               .aspect = VK_IMAGE_ASPECT_COLOR_BIT});
    depthBuffer = CreateImage(context.device, context.physicalDevice,
                              {.extent = extent,
                               .format = kDepthFormat,
                               .usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                               .aspect = VK_IMAGE_ASPECT_DEPTH_BIT,
                               .samples = msaaSamples});

    // One "render finished" semaphore per swapchain image. Presenting an image can still be
    // waiting on its semaphore when the next frame starts, so they cannot be shared per frame.
    while (renderFinished.size() < context.swapchainImages.size()) {
        VkSemaphoreCreateInfo semaphoreInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
        VkSemaphore semaphore = VK_NULL_HANDLE;
        VkCheck(vkCreateSemaphore(context.device, &semaphoreInfo, nullptr, &semaphore), "vkCreateSemaphore");
        renderFinished.push_back(semaphore);
    }

    for (FrameResources& frame : frames) {
        const VkDescriptorImageInfo hdrInfo{linearSampler, hdrResolved.view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = frame.tonemapDescriptors;
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo = &hdrInfo;
        vkUpdateDescriptorSets(context.device, 1, &write, 0, nullptr);
    }
}

void VulkanRenderer::DestroySizeDependentResources() {
    DestroyImage(context.device, hdrMultisampled);
    DestroyImage(context.device, hdrResolved);
    DestroyImage(context.device, depthBuffer);
}

void VulkanRenderer::CreatePipelines() {
    // Lit geometry. Depth is reversed (1 = near, 0 = far), so nearer surfaces have GREATER depth.
    PipelineDesc lit;
    lit.layout = scenePipelineLayout;
    lit.fragmentShader = "lit.frag";
    lit.colorFormat = kHdrFormat;
    lit.depthFormat = kDepthFormat;
    lit.samples = msaaSamples;
    lit.cullMode = VK_CULL_MODE_BACK_BIT;
    lit.depthTest = true;
    lit.depthWrite = true;
    lit.depthCompare = VK_COMPARE_OP_GREATER;
    lit.vertexShader = "box.vert";
    boxPipeline = CreatePipeline(lit);
    lit.vertexShader = "npc.vert";
    npcPipeline = CreatePipeline(lit);

    // Depth only, from the sun. The shadow map uses ordinary depth (0 = near).
    PipelineDesc shadow;
    shadow.layout = scenePipelineLayout;
    shadow.depthFormat = kDepthFormat;
    shadow.depthTest = true;
    shadow.depthWrite = true;
    shadow.depthCompare = VK_COMPARE_OP_LESS_OR_EQUAL;
    shadow.shadowBias = true;
    shadow.vertexShader = "box.vert";
    boxShadowPipeline = CreatePipeline(shadow);
    shadow.vertexShader = "npc.vert";
    npcShadowPipeline = CreatePipeline(shadow);

    // The sky is a full-screen triangle at the far plane, drawn only where nothing else was.
    PipelineDesc sky;
    sky.layout = scenePipelineLayout;
    sky.vertexShader = "fullscreen.vert";
    sky.fragmentShader = "sky.frag";
    sky.usesMeshInput = false;
    sky.colorFormat = kHdrFormat;
    sky.depthFormat = kDepthFormat;
    sky.samples = msaaSamples;
    sky.depthTest = true;
    sky.depthCompare = VK_COMPARE_OP_GREATER_OR_EQUAL;
    skyPipeline = CreatePipeline(sky);

    PipelineDesc tonemap;
    tonemap.layout = tonemapPipelineLayout;
    tonemap.vertexShader = "fullscreen.vert";
    tonemap.fragmentShader = "tonemap.frag";
    tonemap.usesMeshInput = false;
    tonemap.colorFormat = context.swapchainFormat;
    tonemapPipeline = CreatePipeline(tonemap);
}

VkPipeline VulkanRenderer::CreatePipeline(const PipelineDesc& desc) {
    const VkShaderModule vertexModule = LoadShaderModule(context.device, ShaderPath(desc.vertexShader));
    const VkShaderModule fragmentModule =
        desc.fragmentShader != nullptr ? LoadShaderModule(context.device, ShaderPath(desc.fragmentShader)) : VK_NULL_HANDLE;

    VkPipelineShaderStageCreateInfo stages[2]{};
    uint32_t stageCount = 0;
    stages[stageCount++] = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0,
                            VK_SHADER_STAGE_VERTEX_BIT, vertexModule, "main", nullptr};
    if (fragmentModule != VK_NULL_HANDLE) {
        stages[stageCount++] = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0,
                                VK_SHADER_STAGE_FRAGMENT_BIT, fragmentModule, "main", nullptr};
    }

    // Binding 0 is the mesh (one entry per vertex); binding 1 the instances (one entry per box or
    // person). Both instance types are 48 bytes, read by the shaders as three vec4s.
    static_assert(sizeof(BoxInstance) == sizeof(NpcInstance), "Box and NPC pipelines share one instance layout");
    const VkVertexInputBindingDescription bindings[] = {
        {0, sizeof(MeshVertex), VK_VERTEX_INPUT_RATE_VERTEX},
        {1, sizeof(BoxInstance), VK_VERTEX_INPUT_RATE_INSTANCE},
    };
    const VkVertexInputAttributeDescription attributes[] = {
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(MeshVertex, position)},
        {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(MeshVertex, normal)},
        {2, 0, VK_FORMAT_R32_SFLOAT, offsetof(MeshVertex, part)},
        {3, 1, VK_FORMAT_R32G32B32A32_SFLOAT, 0},
        {4, 1, VK_FORMAT_R32G32B32A32_SFLOAT, 16},
        {5, 1, VK_FORMAT_R32G32B32A32_SFLOAT, 32},
    };
    VkPipelineVertexInputStateCreateInfo vertexInput{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    if (desc.usesMeshInput) {
        vertexInput.vertexBindingDescriptionCount = 2;
        vertexInput.pVertexBindingDescriptions = bindings;
        vertexInput.vertexAttributeDescriptionCount = 6;
        vertexInput.pVertexAttributeDescriptions = attributes;
    }

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    // Viewport and scissor are set while recording, so one pipeline works at any window size.
    VkPipelineViewportStateCreateInfo viewportState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rasterization{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterization.depthClampEnable = (desc.shadowBias && context.supportsDepthClamp) ? VK_TRUE : VK_FALSE;
    rasterization.polygonMode = VK_POLYGON_MODE_FILL;
    rasterization.cullMode = desc.cullMode;
    // Meshes are wound counter-clockwise; the Y flip in the camera projection keeps them that way on screen.
    rasterization.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    // Pushing shadow-map depth slightly away from the sun prevents surfaces from shadowing themselves.
    rasterization.depthBiasEnable = desc.shadowBias ? VK_TRUE : VK_FALSE;
    rasterization.depthBiasConstantFactor = 1.25f;
    rasterization.depthBiasSlopeFactor = 1.75f;
    rasterization.lineWidth = 1.0f;

    VkPipelineMultisampleStateCreateInfo multisample{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisample.rasterizationSamples = desc.samples;

    VkPipelineDepthStencilStateCreateInfo depthStencil{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    depthStencil.depthTestEnable = desc.depthTest ? VK_TRUE : VK_FALSE;
    depthStencil.depthWriteEnable = desc.depthWrite ? VK_TRUE : VK_FALSE;
    depthStencil.depthCompareOp = desc.depthCompare;

    const bool hasColor = desc.colorFormat != VK_FORMAT_UNDEFINED;
    VkPipelineColorBlendAttachmentState blendAttachment{};
    blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                     VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendStateCreateInfo colorBlend{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    colorBlend.attachmentCount = hasColor ? 1 : 0;
    colorBlend.pAttachments = &blendAttachment;

    const VkDynamicState dynamicStates[] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynamicState.dynamicStateCount = 2;
    dynamicState.pDynamicStates = dynamicStates;

    // Dynamic rendering: instead of a render pass object, the pipeline states its attachment formats.
    VkPipelineRenderingCreateInfo rendering{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO};
    rendering.colorAttachmentCount = hasColor ? 1 : 0;
    rendering.pColorAttachmentFormats = &desc.colorFormat;
    rendering.depthAttachmentFormat = desc.depthFormat;

    VkGraphicsPipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipelineInfo.pNext = &rendering;
    pipelineInfo.stageCount = stageCount;
    pipelineInfo.pStages = stages;
    pipelineInfo.pVertexInputState = &vertexInput;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterization;
    pipelineInfo.pMultisampleState = &multisample;
    pipelineInfo.pDepthStencilState = &depthStencil;
    pipelineInfo.pColorBlendState = &colorBlend;
    pipelineInfo.pDynamicState = &dynamicState;
    pipelineInfo.layout = desc.layout;

    VkPipeline pipeline = VK_NULL_HANDLE;
    const VkResult result = vkCreateGraphicsPipelines(context.device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline);
    vkDestroyShaderModule(context.device, vertexModule, nullptr);
    vkDestroyShaderModule(context.device, fragmentModule, nullptr);
    VkCheck(result, "vkCreateGraphicsPipelines");
    return pipeline;
}

VulkanRenderer::Mesh VulkanRenderer::UploadMesh(const MeshData& data) {
    const VkDeviceSize vertexBytes = data.vertices.size() * sizeof(MeshVertex);
    const VkDeviceSize indexBytes = data.indices.size() * sizeof(uint32_t);

    Mesh mesh;
    mesh.vertices = CreateMappedBuffer(context.device, context.physicalDevice, vertexBytes, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    mesh.indices = CreateMappedBuffer(context.device, context.physicalDevice, indexBytes, VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    std::memcpy(mesh.vertices.mapped, data.vertices.data(), vertexBytes);
    std::memcpy(mesh.indices.mapped, data.indices.data(), indexBytes);
    mesh.indexCount = static_cast<uint32_t>(data.indices.size());
    return mesh;
}

void VulkanRenderer::DestroyMesh(Mesh& mesh) {
    DestroyBuffer(context.device, mesh.vertices);
    DestroyBuffer(context.device, mesh.indices);
    mesh.indexCount = 0;
}

// ---------------------------------------------------------------------------
// Scene data and resizing
// ---------------------------------------------------------------------------

void VulkanRenderer::UploadStaticScene(std::span<const BoxInstance> boxes) {
    // The previous buffer may still be in use by frames the GPU has not finished.
    vkDeviceWaitIdle(context.device);
    DestroyBuffer(context.device, boxInstances);

    const VkDeviceSize bytes = std::max<VkDeviceSize>(boxes.size_bytes(), sizeof(BoxInstance));
    boxInstances = CreateMappedBuffer(context.device, context.physicalDevice, bytes, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    if (!boxes.empty()) {
        std::memcpy(boxInstances.mapped, boxes.data(), boxes.size_bytes());
    }
    boxInstanceCount = static_cast<uint32_t>(boxes.size());
}

void VulkanRenderer::EnsureNpcCapacity(size_t npcCount) {
    if (npcCount <= npcCapacity) {
        return;
    }
    // Grow by doubling, so a slowly rising number of people does not reallocate every frame.
    vkDeviceWaitIdle(context.device);
    npcCapacity = std::max(npcCount, npcCapacity * 2);
    for (FrameResources& frame : frames) {
        DestroyBuffer(context.device, frame.npcInstances);
        frame.npcInstances = CreateMappedBuffer(context.device, context.physicalDevice,
                                                npcCapacity * sizeof(NpcInstance), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    }
}

void VulkanRenderer::SetResolution(int /*width*/, int /*height*/) {
    // The swapchain takes its size from the window itself, so only remember to rebuild it.
    resizeRequested = true;
}

bool VulkanRenderer::RecreateSwapchain() {
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    if (width == 0 || height == 0) {
        return false;  // minimised; try again on a later frame
    }
    vkDeviceWaitIdle(context.device);
    DestroySizeDependentResources();
    context.RecreateSwapchain();
    CreateSizeDependentResources();
    resizeRequested = false;
    return true;
}

// ---------------------------------------------------------------------------
// Per-frame work
// ---------------------------------------------------------------------------

glm::mat4 VulkanRenderer::ComputeSunShadowMatrix(const FrameInput& input) const {
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
    const float texelSize = 2.0f * radius / static_cast<float>(kShadowMapSize);
    glm::vec3 center = glm::vec3(lightRotation * glm::vec4(focus, 1.0f));
    center.x = std::floor(center.x / texelSize) * texelSize;
    center.y = std::floor(center.y / texelSize) * texelSize;

    // In light space the sun looks down -Z, so the box spans view depths -(z + range) to -(z - range).
    const float depthRange = 500.0f;
    const glm::mat4 projection = glm::orthoRH_ZO(center.x - radius, center.x + radius, center.y - radius, center.y + radius,
                                                 -center.z - depthRange, -center.z + depthRange);
    return projection * lightRotation;
}

void VulkanRenderer::WriteFrameUniforms(FrameResources& frame, const FrameInput& input,
                                        const glm::mat4& cameraViewProjection, const glm::mat4& lightViewProjection) {
    const glm::vec3 sun = glm::normalize(input.sunDirection);

    FrameUniforms uniforms;
    uniforms.viewProjection = cameraViewProjection;
    uniforms.inverseViewProjection = glm::inverse(cameraViewProjection);
    uniforms.lightViewProjection = lightViewProjection;
    uniforms.cameraPosition = glm::vec4(input.cameraPosition, input.timeSeconds);
    uniforms.sunDirection = glm::vec4(sun, 0.0f);
    uniforms.sunColor = glm::vec4(SunRadiance(sun.y), 0.0f);
    uniforms.settings = glm::vec4(1.0f / static_cast<float>(kShadowMapSize), kFogDensity, 0.0f, 0.0f);
    std::memcpy(frame.uniforms.mapped, &uniforms, sizeof(uniforms));
}

void VulkanRenderer::Render(const FrameInput& input) {
    if (resizeRequested && !RecreateSwapchain()) {
        return;  // the window is minimised
    }

    FrameResources& frame = frames[frameIndex];
    // Wait until the GPU has finished the last frame that used this frame's resources.
    VkCheck(vkWaitForFences(context.device, 1, &frame.inFlight, VK_TRUE, UINT64_MAX), "vkWaitForFences");
    EnsureNpcCapacity(input.npcs.size());

    uint32_t imageIndex = 0;
    const VkResult acquireResult = vkAcquireNextImageKHR(context.device, context.swapchain, UINT64_MAX,
                                                         frame.imageAvailable, VK_NULL_HANDLE, &imageIndex);
    if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR) {
        resizeRequested = true;
        return;
    }
    if (acquireResult != VK_SUBOPTIMAL_KHR) {
        VkCheck(acquireResult, "vkAcquireNextImageKHR");
    }
    VkCheck(vkResetFences(context.device, 1, &frame.inFlight), "vkResetFences");

    const glm::mat4 cameraViewProjection = input.projection * input.view;
    const glm::mat4 lightViewProjection = ComputeSunShadowMatrix(input);
    WriteFrameUniforms(frame, input, cameraViewProjection, lightViewProjection);
    const uint32_t npcCount = static_cast<uint32_t>(input.npcs.size());
    if (npcCount > 0) {
        std::memcpy(frame.npcInstances.mapped, input.npcs.data(), input.npcs.size_bytes());
    }

    RecordCommands(frame, imageIndex, npcCount, cameraViewProjection, lightViewProjection);

    // Start drawing into the swapchain image only once it is free, and signal when we are done.
    VkSemaphoreSubmitInfo waitInfo{VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO};
    waitInfo.semaphore = frame.imageAvailable;
    waitInfo.stageMask = kColorOutputStage;
    VkSemaphoreSubmitInfo signalInfo{VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO};
    signalInfo.semaphore = renderFinished[imageIndex];
    signalInfo.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    VkCommandBufferSubmitInfo commandInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO};
    commandInfo.commandBuffer = frame.commandBuffer;

    VkSubmitInfo2 submit{VK_STRUCTURE_TYPE_SUBMIT_INFO_2};
    submit.waitSemaphoreInfoCount = 1;
    submit.pWaitSemaphoreInfos = &waitInfo;
    submit.commandBufferInfoCount = 1;
    submit.pCommandBufferInfos = &commandInfo;
    submit.signalSemaphoreInfoCount = 1;
    submit.pSignalSemaphoreInfos = &signalInfo;
    VkCheck(vkQueueSubmit2(context.queue, 1, &submit, frame.inFlight), "vkQueueSubmit2");

    VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    present.waitSemaphoreCount = 1;
    present.pWaitSemaphores = &renderFinished[imageIndex];
    present.swapchainCount = 1;
    present.pSwapchains = &context.swapchain;
    present.pImageIndices = &imageIndex;
    const VkResult presentResult = vkQueuePresentKHR(context.queue, &present);
    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR) {
        resizeRequested = true;
    } else {
        VkCheck(presentResult, "vkQueuePresentKHR");
    }

    frameIndex = (frameIndex + 1) % kFramesInFlight;
}

void VulkanRenderer::RecordCommands(FrameResources& frame, uint32_t imageIndex, uint32_t npcCount,
                                    const glm::mat4& cameraViewProjection, const glm::mat4& lightViewProjection) {
    VkCommandBuffer cmd = frame.commandBuffer;
    VkCheck(vkResetCommandBuffer(cmd, 0), "vkResetCommandBuffer");
    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VkCheck(vkBeginCommandBuffer(cmd, &beginInfo), "vkBeginCommandBuffer");

    // The shadow and scene passes share a pipeline layout, so this binding serves both.
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, scenePipelineLayout, 0, 1, &frame.sceneDescriptors, 0, nullptr);
    RecordShadowPass(cmd, frame, npcCount, lightViewProjection);
    RecordScenePass(cmd, frame, npcCount, cameraViewProjection);
    RecordTonemapPass(cmd, frame, imageIndex);

    VkCheck(vkEndCommandBuffer(cmd), "vkEndCommandBuffer");
}

void VulkanRenderer::RecordShadowPass(VkCommandBuffer cmd, const FrameResources& frame, uint32_t npcCount,
                                      const glm::mat4& lightViewProjection) {
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

    BeginRendering(cmd, {kShadowMapSize, kShadowMapSize}, nullptr, &depthAttachment);
    vkCmdPushConstants(cmd, scenePipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4), &lightViewProjection);
    DrawBoxes(cmd, boxShadowPipeline);
    DrawNpcs(cmd, npcShadowPipeline, frame, npcCount);
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

void VulkanRenderer::RecordScenePass(VkCommandBuffer cmd, const FrameResources& frame, uint32_t npcCount,
                                     const glm::mat4& cameraViewProjection) {
    const bool useMsaa = msaaSamples != VK_SAMPLE_COUNT_1_BIT;
    if (useMsaa) {
        TransitionImage(cmd, {.image = hdrMultisampled.image,
                              .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                              .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
                              .newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                              .srcStage = kColorOutputStage,
                              .srcAccess = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                              .dstStage = kColorOutputStage,
                              .dstAccess = VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT});
    }
    // The previous frame's tonemap pass may still be reading the resolved image.
    TransitionImage(cmd, {.image = hdrResolved.image,
                          .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                          .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
                          .newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          .srcStage = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | kColorOutputStage,
                          .srcAccess = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                          .dstStage = kColorOutputStage,
                          .dstAccess = VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT});
    TransitionImage(cmd, {.image = depthBuffer.image,
                          .aspect = VK_IMAGE_ASPECT_DEPTH_BIT,
                          .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
                          .newLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                          .srcStage = kDepthTestStages,
                          .srcAccess = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                          .dstStage = kDepthTestStages,
                          .dstAccess = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
                                       VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT});

    VkRenderingAttachmentInfo colorAttachment{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    colorAttachment.imageView = useMsaa ? hdrMultisampled.view : hdrResolved.view;
    colorAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = useMsaa ? VK_ATTACHMENT_STORE_OP_DONT_CARE : VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.clearValue.color = {{0.0f, 0.0f, 0.0f, 1.0f}};
    if (useMsaa) {
        // At the end of the pass, average each pixel's samples into the single-sample image.
        colorAttachment.resolveMode = VK_RESOLVE_MODE_AVERAGE_BIT;
        colorAttachment.resolveImageView = hdrResolved.view;
        colorAttachment.resolveImageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    }

    VkRenderingAttachmentInfo depthAttachment{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    depthAttachment.imageView = depthBuffer.view;
    depthAttachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
    depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAttachment.clearValue.depthStencil = {0.0f, 0};  // reversed depth: 0 is the far plane

    BeginRendering(cmd, context.swapchainExtent, &colorAttachment, &depthAttachment);
    vkCmdPushConstants(cmd, scenePipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4), &cameraViewProjection);
    DrawBoxes(cmd, boxPipeline);
    DrawNpcs(cmd, npcPipeline, frame, npcCount);
    // The sky goes last and only fills the pixels no geometry covered.
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, skyPipeline);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRendering(cmd);

    TransitionImage(cmd, {.image = hdrResolved.image,
                          .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                          .oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          .newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                          .srcStage = kColorOutputStage,
                          .srcAccess = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                          .dstStage = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                          .dstAccess = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT});
}

void VulkanRenderer::RecordTonemapPass(VkCommandBuffer cmd, const FrameResources& frame, uint32_t imageIndex) {
    const VkImage swapchainImage = context.swapchainImages[imageIndex];
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
    colorAttachment.imageView = context.swapchainViews[imageIndex];
    colorAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;  // every pixel gets overwritten
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

    BeginRendering(cmd, context.swapchainExtent, &colorAttachment, nullptr);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, tonemapPipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, tonemapPipelineLayout, 0, 1, &frame.tonemapDescriptors, 0, nullptr);
    const glm::vec4 tonemapSettings(kExposure, 0.0f, 0.0f, 0.0f);
    vkCmdPushConstants(cmd, tonemapPipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(tonemapSettings), &tonemapSettings);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRendering(cmd);

    TransitionImage(cmd, {.image = swapchainImage,
                          .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                          .oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          .newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                          .srcStage = kColorOutputStage,
                          .srcAccess = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                          .dstStage = VK_PIPELINE_STAGE_2_NONE,
                          .dstAccess = VK_ACCESS_2_NONE});
}

void VulkanRenderer::DrawBoxes(VkCommandBuffer cmd, VkPipeline pipeline) const {
    if (boxInstanceCount == 0) {
        return;
    }
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    const VkBuffer buffers[] = {cubeMesh.vertices.buffer, boxInstances.buffer};
    const VkDeviceSize offsets[] = {0, 0};
    vkCmdBindVertexBuffers(cmd, 0, 2, buffers, offsets);
    vkCmdBindIndexBuffer(cmd, cubeMesh.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
    // One draw call for the whole city: every box is an instance of the same cube.
    vkCmdDrawIndexed(cmd, cubeMesh.indexCount, boxInstanceCount, 0, 0, 0);
}

void VulkanRenderer::DrawNpcs(VkCommandBuffer cmd, VkPipeline pipeline, const FrameResources& frame, uint32_t npcCount) const {
    if (npcCount == 0) {
        return;
    }
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    const VkBuffer buffers[] = {personMesh.vertices.buffer, frame.npcInstances.buffer};
    const VkDeviceSize offsets[] = {0, 0};
    vkCmdBindVertexBuffers(cmd, 0, 2, buffers, offsets);
    vkCmdBindIndexBuffer(cmd, personMesh.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
    // One draw call for the whole crowd.
    vkCmdDrawIndexed(cmd, personMesh.indexCount, npcCount, 0, 0, 0);
}
