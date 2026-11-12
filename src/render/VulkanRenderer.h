#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "render/DynamicResolution.h"
#include "render/GpuProfiler.h"
#include "render/IRenderModule.h"
#include "render/RenderTargets.h"
#include "render/SceneGeometry.h"
#include "render/ScenePass.h"
#include "render/ScreenshotCapture.h"
#include "render/ShadowPass.h"
#include "render/TonemapPass.h"
#include "render/VulkanContext.h"
#include "render/VulkanUtils.h"

// Vulkan 1.3 implementation of the render module.
//
// Every frame runs three passes, each in its own class:
//   1. ShadowPass:  depth of the city and the crowd, seen from the sun, into a shadow map.
//   2. ScenePass:   lit geometry and the sky into a multisampled HDR image, drawn at the current
//                   render scale into the top-left part of full-size targets.
//   3. TonemapPass: HDR image to the window with exposure, a filmic curve and a vignette,
//                   scaling the drawn part up to the window size.
//
// This class owns what the passes share (frames in flight, the per-frame uniforms and their
// descriptor sets, the swapchain semaphores) and runs the frame: wait, record, submit, present.
// GpuProfiler times the passes, and DynamicResolution turns those times into a render scale.
class VulkanRenderer final : public IRenderModule {
public:
    bool Initialize(EngineContext& engine) override;
    void Shutdown() override;
    const char* GetName() const override { return "Vulkan Renderer"; }
    const char* GetVersion() const override { return "0.2.0"; }

    void SetResolution(int width, int height) override;
    void SetResolutionSettings(const ResolutionSettings& settings) override;
    void UploadStaticScene(std::span<const BoxInstance> boxes) override;
    void ReserveNpcCapacity(size_t npcCount) override;
    void Render(const FrameInput& frame) override;
    bool RenderAndCapture(const FrameInput& frame, const std::string& pngPath) override;
    RenderStats GetStats() const override;
    std::string GetDeviceName() const override;

private:
    // Two frames in flight: the CPU prepares one frame while the GPU draws the previous one.
    // More would add a frame of input latency without making the frame rate any steadier.
    static constexpr uint32_t kFramesInFlight = 2;

    // Order of the passes in the GPU profiler.
    enum PassIndex : uint32_t { kShadowPassIndex, kScenePassIndex, kTonemapPassIndex };

    struct FrameResources {
        VkCommandPool commandPool = VK_NULL_HANDLE;
        VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
        VkSemaphore imageAvailable = VK_NULL_HANDLE;
        VkFence inFlight = VK_NULL_HANDLE;
        GpuBuffer uniforms;
        GpuBuffer waterField;  // the simulated river: a small header, then the offset of every cell
        VkDescriptorSet sceneDescriptors = VK_NULL_HANDLE;
    };

    void CreateFrameResources();
    void CreateSceneLayouts();
    void CreateSceneDescriptors();
    void CreateSwapchainSemaphores();
    void DestroySwapchainSemaphores();
    bool RecreateSwapchain();
    bool PresentResultNeedsNewSwapchain(VkResult result) const;

    // Draws and presents one frame, optionally copying it for a screenshot. Returns false when
    // the frame was skipped (window minimised or the swapchain had to be rebuilt).
    bool DrawFrame(const FrameInput& input, bool captureScreenshot);
    void FeedResolutionController();
    // Copies the simulated river of this frame into the slot's buffer, making the buffer bigger first when needed.
    void WriteWaterField(FrameResources& frame, const WaterField& water);
    void WriteFrameUniforms(FrameResources& frame, const FrameInput& input, const glm::mat4& cameraViewProjection,
                            const glm::mat4& lightViewProjection) const;
    void RecordCommands(uint32_t frameSlot, uint32_t imageIndex, const glm::mat4& cameraViewProjection,
                        const glm::mat4& lightViewProjection, bool captureScreenshot);
    void Submit(const FrameResources& frame, uint32_t imageIndex);
    void Present(uint32_t imageIndex);

    VulkanContext context;
    GLFWwindow* window = nullptr;
    bool resizeRequested = false;
    uint64_t frameNumber = 0;  // frames submitted so far; the frame slot is frameNumber % kFramesInFlight
    std::array<FrameResources, kFramesInFlight> frames{};
    std::vector<VkSemaphore> renderFinished;  // one per swapchain image, see CreateSwapchainSemaphores

    VkDescriptorSetLayout sceneSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool sceneDescriptorPool = VK_NULL_HANDLE;
    VkPipelineLayout scenePipelineLayout = VK_NULL_HANDLE;

    RenderTargets targets;
    SceneGeometry geometry;
    ShadowPass shadowPass;
    ScenePass scenePass;
    TonemapPass tonemapPass;
    GpuProfiler profiler;
    DynamicResolution dynamicResolution;
    ScreenshotCapture screenshot;

    uint64_t lastFedMeasurement = 0;
    float currentRenderScale = 1.0f;
    VkExtent2D currentRenderExtent{0, 0};
};
