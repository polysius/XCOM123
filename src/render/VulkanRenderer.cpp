#include "render/VulkanRenderer.h"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string_view>

#include <GLFW/glfw3.h>

#include "render/PipelineBuilder.h"

// Registers this renderer under the name "Vulkan", so main() can create it through the
// ModuleRegistry without knowing this class exists.
static AutoRegister<VulkanRenderer> gVulkanRendererRegistration("Vulkan");

namespace {
constexpr float kFogDensity = 0.0022f;
constexpr std::array<std::string_view, 3> kPassNames = {"shadow", "scene", "tonemap"};
// A screenshot frame can be skipped while the swapchain is rebuilt; try this many times.
constexpr int kScreenshotAttempts = 4;

// The header of the water field buffer: a vec4 and an ivec4, as the shader's WaterField block starts.
constexpr VkDeviceSize kWaterFieldHeaderBytes = 32;

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
}  // namespace

// ---------------------------------------------------------------------------
// Start-up and shutdown
// ---------------------------------------------------------------------------

bool VulkanRenderer::Initialize(EngineContext& engine) {
    try {
        window = engine.window;
        context.Create(engine.window, engine.vsync);
        CreateFrameResources();
        CreateSceneLayouts();
        targets.Create(context, context.swapchainExtent, ChooseMsaaSamples(context.deviceProperties));
        geometry.Create(context, kFramesInFlight);
        shadowPass.Create(context, scenePipelineLayout);
        scenePass.Create(context, scenePipelineLayout, targets.samples);
        tonemapPass.Create(context, targets);
        CreateSceneDescriptors();
        CreateSwapchainSemaphores();
        profiler.Create(context, kFramesInFlight, kPassNames);
        dynamicResolution.Configure(ResolutionSettings{});
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

    screenshot.Destroy(device);
    profiler.Destroy(device);
    tonemapPass.Destroy(device);
    scenePass.Destroy(device);
    shadowPass.Destroy(device);
    geometry.Destroy(device);
    targets.Destroy(device);
    DestroySwapchainSemaphores();

    vkDestroyPipelineLayout(device, scenePipelineLayout, nullptr);
    vkDestroyDescriptorPool(device, sceneDescriptorPool, nullptr);  // also frees the descriptor sets
    vkDestroyDescriptorSetLayout(device, sceneSetLayout, nullptr);

    for (FrameResources& frame : frames) {
        DestroyBuffer(device, frame.uniforms);
        DestroyBuffer(device, frame.waterField);
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
        frame.waterField = CreateMappedBuffer(context.device, context.physicalDevice, kWaterFieldHeaderBytes,
                                              VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        std::memset(frame.waterField.mapped, 0, kWaterFieldHeaderBytes);  // no simulated river yet
    }
}

void VulkanRenderer::CreateSceneLayouts() {
    // Scene and shadow shaders read the per-frame uniforms and the shadow map.
    VkDescriptorSetLayoutBinding bindings[3]{};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[2].binding = 2;
    bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[2].descriptorCount = 1;
    bindings[2].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = 3;
    layoutInfo.pBindings = bindings;
    VkCheck(vkCreateDescriptorSetLayout(context.device, &layoutInfo, nullptr, &sceneSetLayout),
            "vkCreateDescriptorSetLayout");

    // Scene pipelines also take the view-projection matrix as a push constant: the camera's in
    // the scene pass and the sun's in the shadow pass, so the same vertex shaders serve both.
    scenePipelineLayout =
        CreatePipelineLayout(context.device, sceneSetLayout, VK_SHADER_STAGE_VERTEX_BIT, sizeof(glm::mat4));
}

void VulkanRenderer::CreateSceneDescriptors() {
    const VkDescriptorPoolSize poolSizes[] = {
        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, kFramesInFlight},
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, kFramesInFlight},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, kFramesInFlight},
    };
    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.maxSets = kFramesInFlight;
    poolInfo.poolSizeCount = 3;
    poolInfo.pPoolSizes = poolSizes;
    VkCheck(vkCreateDescriptorPool(context.device, &poolInfo, nullptr, &sceneDescriptorPool), "vkCreateDescriptorPool");

    for (FrameResources& frame : frames) {
        VkDescriptorSetAllocateInfo allocateInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        allocateInfo.descriptorPool = sceneDescriptorPool;
        allocateInfo.descriptorSetCount = 1;
        allocateInfo.pSetLayouts = &sceneSetLayout;
        VkCheck(vkAllocateDescriptorSets(context.device, &allocateInfo, &frame.sceneDescriptors), "vkAllocateDescriptorSets");

        const VkDescriptorBufferInfo uniformInfo{frame.uniforms.buffer, 0, sizeof(FrameUniforms)};
        const VkDescriptorImageInfo shadowInfo{shadowPass.ComparisonSampler(), shadowPass.MapView(),
                                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        const VkDescriptorBufferInfo waterInfo{frame.waterField.buffer, 0, VK_WHOLE_SIZE};
        VkWriteDescriptorSet writes[3]{};
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
        writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[2].dstSet = frame.sceneDescriptors;
        writes[2].dstBinding = 2;
        writes[2].descriptorCount = 1;
        writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[2].pBufferInfo = &waterInfo;
        vkUpdateDescriptorSets(context.device, 3, writes, 0, nullptr);
    }
}

// One "render finished" semaphore per swapchain image rather than per frame in flight: the
// presentation engine may still be waiting on an image's semaphore when the same frame slot
// comes round again, but never once that image has been acquired again.
void VulkanRenderer::CreateSwapchainSemaphores() {
    VkSemaphoreCreateInfo semaphoreInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    renderFinished.resize(context.swapchainImages.size());
    for (VkSemaphore& semaphore : renderFinished) {
        VkCheck(vkCreateSemaphore(context.device, &semaphoreInfo, nullptr, &semaphore), "vkCreateSemaphore");
    }
}

void VulkanRenderer::DestroySwapchainSemaphores() {
    for (VkSemaphore semaphore : renderFinished) {
        vkDestroySemaphore(context.device, semaphore, nullptr);
    }
    renderFinished.clear();
}

// ---------------------------------------------------------------------------
// Scene data and settings
// ---------------------------------------------------------------------------

void VulkanRenderer::UploadStaticScene(std::span<const BoxInstance> boxes) {
    vkDeviceWaitIdle(context.device);  // the old buffer may still be in use
    geometry.UploadBoxes(context, boxes);
}

void VulkanRenderer::ReserveNpcCapacity(size_t npcCount) {
    vkDeviceWaitIdle(context.device);
    geometry.ReserveNpcs(context, npcCount);
}

void VulkanRenderer::SetResolutionSettings(const ResolutionSettings& settings) {
    dynamicResolution.Configure(settings);
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
    DestroySwapchainSemaphores();
    targets.Destroy(context.device);
    context.RecreateSwapchain();
    targets.Create(context, context.swapchainExtent, targets.samples);
    tonemapPass.BindTargets(context.device, targets);
    CreateSwapchainSemaphores();
    resizeRequested = false;
    return true;
}

// OUT_OF_DATE always means a rebuild. SUBOPTIMAL can be reported for reasons that do not
// affect us, and rebuilding waits for the whole GPU, so it only counts when the size changed.
bool VulkanRenderer::PresentResultNeedsNewSwapchain(VkResult result) const {
    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        return true;
    }
    if (result != VK_SUBOPTIMAL_KHR) {
        return false;
    }
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    return static_cast<uint32_t>(width) != context.swapchainExtent.width ||
           static_cast<uint32_t>(height) != context.swapchainExtent.height;
}

RenderStats VulkanRenderer::GetStats() const {
    RenderStats stats;
    stats.gpuTimingAvailable = profiler.IsAvailable();
    stats.passes = profiler.PassTimings();
    stats.gpuFrameMs = profiler.LatestFrameMs();
    stats.smoothedGpuFrameMs = profiler.SmoothedFrameMs();
    stats.gpuMeasuredFrames = profiler.MeasuredFrameCount();
    stats.renderScale = currentRenderScale;
    stats.renderWidth = currentRenderExtent.width;
    stats.renderHeight = currentRenderExtent.height;
    return stats;
}

// ---------------------------------------------------------------------------
// Per-frame work
// ---------------------------------------------------------------------------

void VulkanRenderer::Render(const FrameInput& input) {
    DrawFrame(input, false);
}

bool VulkanRenderer::RenderAndCapture(const FrameInput& input, const std::string& pngPath) {
    if (!context.swapchainSupportsCopy) {
        std::cerr << "Screenshot: this GPU cannot copy from swapchain images\n";
        return false;
    }
    for (int attempt = 0; attempt < kScreenshotAttempts; ++attempt) {
        if (DrawFrame(input, true)) {
            // Wait for exactly this frame, so the copied pixels are complete before reading them.
            const FrameResources& frame = frames[(frameNumber - 1) % kFramesInFlight];
            VkCheck(vkWaitForFences(context.device, 1, &frame.inFlight, VK_TRUE, UINT64_MAX), "vkWaitForFences");
            return screenshot.SavePng(pngPath, context.swapchainFormat);
        }
    }
    std::cerr << "Screenshot: no frame could be drawn\n";
    return false;
}

void VulkanRenderer::WriteWaterField(FrameResources& frame, const WaterField& water) {
    const size_t cells = static_cast<size_t>(std::max(water.width, 0)) * static_cast<size_t>(std::max(water.height, 0));
    const bool present = cells > 0 && water.offsets.size() >= cells && water.cellSize > 0.0f;
    const VkDeviceSize needed = kWaterFieldHeaderBytes + (present ? cells * sizeof(float) : 0);
    if (needed > frame.waterField.size) {
        // The slot's previous frame is finished, so its buffer can be replaced and the descriptor pointed at the new one.
        DestroyBuffer(context.device, frame.waterField);
        frame.waterField = CreateMappedBuffer(context.device, context.physicalDevice, needed, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        const VkDescriptorBufferInfo info{frame.waterField.buffer, 0, VK_WHOLE_SIZE};
        VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet = frame.sceneDescriptors;
        write.dstBinding = 2;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        write.pBufferInfo = &info;
        vkUpdateDescriptorSets(context.device, 1, &write, 0, nullptr);
    }
    // The header is what shaders/lit.frag reads as WaterField: four floats, then four ints.
    auto* header = static_cast<unsigned char*>(frame.waterField.mapped);
    const float info[4] = {water.origin.x, water.origin.y, water.cellSize, 0.0f};
    const int32_t size[4] = {water.width, water.height, present ? 1 : 0, 0};
    std::memcpy(header, info, sizeof(info));
    std::memcpy(header + sizeof(info), size, sizeof(size));
    if (present) {
        std::memcpy(header + kWaterFieldHeaderBytes, water.offsets.data(), cells * sizeof(float));
    }
}

bool VulkanRenderer::DrawFrame(const FrameInput& input, bool captureScreenshot) {
    if (resizeRequested && !RecreateSwapchain()) {
        return false;  // the window is minimised
    }

    const uint32_t frameSlot = static_cast<uint32_t>(frameNumber % kFramesInFlight);
    FrameResources& frame = frames[frameSlot];
    // Wait until the GPU has finished the last frame that used this slot. With two slots this
    // lets the CPU run at most one frame ahead of the GPU.
    VkCheck(vkWaitForFences(context.device, 1, &frame.inFlight, VK_TRUE, UINT64_MAX), "vkWaitForFences");

    // The slot's previous frame is finished, so its timings are ready and its buffers are free.
    profiler.CollectResults(context.device, frameSlot);
    FeedResolutionController();
    geometry.WriteNpcs(context, frameSlot, input.npcs);
    geometry.WriteBodies(context, frameSlot, input.bodyBoxes, input.bodySpheres);
    WriteWaterField(frame, input.water);

    uint32_t imageIndex = 0;
    const VkResult acquireResult = vkAcquireNextImageKHR(context.device, context.swapchain, UINT64_MAX,
                                                         frame.imageAvailable, VK_NULL_HANDLE, &imageIndex);
    if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR) {
        resizeRequested = true;
        return false;
    }
    if (acquireResult != VK_SUBOPTIMAL_KHR) {
        VkCheck(acquireResult, "vkAcquireNextImageKHR");
    }
    VkCheck(vkResetFences(context.device, 1, &frame.inFlight), "vkResetFences");

    currentRenderScale = dynamicResolution.Scale();
    currentRenderExtent = targets.ScaledExtent(currentRenderScale);

    const glm::mat4 cameraViewProjection = input.projection * input.view;
    const glm::mat4 lightViewProjection = ShadowPass::ComputeLightViewProjection(input);
    WriteFrameUniforms(frame, input, cameraViewProjection, lightViewProjection);
    RecordCommands(frameSlot, imageIndex, cameraViewProjection, lightViewProjection, captureScreenshot);
    Submit(frame, imageIndex);
    Present(imageIndex);
    ++frameNumber;
    return true;
}

// Hands each new GPU frame time to the resolution controller exactly once.
void VulkanRenderer::FeedResolutionController() {
    if (profiler.MeasuredFrameCount() == lastFedMeasurement) {
        return;
    }
    lastFedMeasurement = profiler.MeasuredFrameCount();
    dynamicResolution.AddMeasurement(profiler.LatestFrameMs());
}

void VulkanRenderer::WriteFrameUniforms(FrameResources& frame, const FrameInput& input,
                                        const glm::mat4& cameraViewProjection,
                                        const glm::mat4& lightViewProjection) const {
    const glm::vec3 sun = glm::normalize(input.sunDirection);

    FrameUniforms uniforms;
    uniforms.viewProjection = cameraViewProjection;
    uniforms.inverseViewProjection = glm::inverse(cameraViewProjection);
    uniforms.lightViewProjection = lightViewProjection;
    uniforms.cameraPosition = glm::vec4(input.cameraPosition, input.timeSeconds);
    uniforms.sunDirection = glm::vec4(sun, 0.0f);
    uniforms.sunColor = glm::vec4(SunRadiance(sun.y), 0.0f);
    uniforms.settings = glm::vec4(1.0f / static_cast<float>(ShadowPass::kMapSize), kFogDensity, 0.0f, 0.0f);
    std::memcpy(frame.uniforms.mapped, &uniforms, sizeof(uniforms));
}

void VulkanRenderer::RecordCommands(uint32_t frameSlot, uint32_t imageIndex, const glm::mat4& cameraViewProjection,
                                    const glm::mat4& lightViewProjection, bool captureScreenshot) {
    const FrameResources& frame = frames[frameSlot];
    VkCommandBuffer cmd = frame.commandBuffer;
    VkCheck(vkResetCommandBuffer(cmd, 0), "vkResetCommandBuffer");
    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VkCheck(vkBeginCommandBuffer(cmd, &beginInfo), "vkBeginCommandBuffer");
    profiler.BeginFrame(cmd, frameSlot);

    // The shadow and scene passes share a pipeline layout, so this binding serves both.
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, scenePipelineLayout, 0, 1, &frame.sceneDescriptors,
                            0, nullptr);

    profiler.BeginPass(cmd, frameSlot, kShadowPassIndex);
    shadowPass.Record(cmd, geometry, frameSlot, lightViewProjection);
    profiler.EndPass(cmd, frameSlot, kShadowPassIndex);

    profiler.BeginPass(cmd, frameSlot, kScenePassIndex);
    scenePass.Record(cmd, targets, currentRenderExtent, geometry, frameSlot, cameraViewProjection);
    profiler.EndPass(cmd, frameSlot, kScenePassIndex);

    const VkImage swapchainImage = context.swapchainImages[imageIndex];
    profiler.BeginPass(cmd, frameSlot, kTonemapPassIndex);
    tonemapPass.Record(cmd, swapchainImage, context.swapchainViews[imageIndex], context.swapchainExtent, targets,
                       currentRenderExtent);
    profiler.EndPass(cmd, frameSlot, kTonemapPassIndex);

    if (captureScreenshot) {
        screenshot.RecordCopy(context, cmd, swapchainImage, context.swapchainExtent);
    } else {
        TransitionImage(cmd, {.image = swapchainImage,
                              .aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                              .oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                              .newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                              .srcStage = kColorOutputStage,
                              .srcAccess = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                              .dstStage = VK_PIPELINE_STAGE_2_NONE,
                              .dstAccess = VK_ACCESS_2_NONE});
    }

    profiler.EndFrame(cmd, frameSlot);
    VkCheck(vkEndCommandBuffer(cmd), "vkEndCommandBuffer");
}

void VulkanRenderer::Submit(const FrameResources& frame, uint32_t imageIndex) {
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
}

void VulkanRenderer::Present(uint32_t imageIndex) {
    VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    present.waitSemaphoreCount = 1;
    present.pWaitSemaphores = &renderFinished[imageIndex];
    present.swapchainCount = 1;
    present.pSwapchains = &context.swapchain;
    present.pImageIndices = &imageIndex;
    const VkResult presentResult = vkQueuePresentKHR(context.queue, &present);
    if (PresentResultNeedsNewSwapchain(presentResult)) {
        resizeRequested = true;
    } else if (presentResult != VK_SUBOPTIMAL_KHR) {
        VkCheck(presentResult, "vkQueuePresentKHR");
    }
}
