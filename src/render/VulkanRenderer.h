#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "render/DynamicResolution.h"
#include "render/GpuProfiler.h"
#include "render/MaterialTextures.h"
#include "render/IRenderModule.h"
#include "render/RenderTargets.h"
#include "render/SceneGeometry.h"
#include "render/ScenePass.h"
#include "render/ScreenshotCapture.h"
#include "render/SkinnedGeometry.h"
#include "render/ShadowPass.h"
#include "render/TonemapPass.h"
#include "render/VulkanContext.h"
#include "render/VulkanUtils.h"

// Vulkan 1.3 implementation of the render module.
//
// Every frame runs three passes, each in its own class:
//   1. ShadowPass:  depth of the city, the crowd and the skinned characters, seen from the sun, into a shadow map.
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
    StreamedChunkHandle AddStreamedChunk(std::shared_ptr<const void> owner, std::span<const BoxInstance> boxes,
                                         const Aabb& bounds) override;
    void RemoveStreamedChunk(StreamedChunkHandle chunk) override;
    void SetDistanceFade(float radius) override;
    void UploadMaterials(const material::MaterialLibrary& library) override;
    void ReserveNpcCapacity(size_t npcCount) override;
    CharacterHandle UploadCharacterMesh(const skinning::MeshData& mesh) override;
    bool SetCharacterLowerDetail(CharacterHandle body, CharacterHandle lowerDetail) override;
    void Render(const FrameInput& frame) override;
    bool RenderAndCapture(const FrameInput& frame, const std::string& pngPath) override;
    RenderStats GetStats() const override;
    std::string GetDeviceName() const override;
    void SetOverlay(IFrameOverlay* newOverlay) override;

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
    // Points a frame's descriptor set at the buffer that holds the skinning palettes of that frame.
    void WritePaletteDescriptor(const FrameResources& frame, uint32_t frameSlot);
    // Points the descriptor sets of every frame at the material textures, and notes their tile sizes for the shaders.
    void WriteMaterialDescriptors();
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
    void RecordOverlay(VkCommandBuffer cmd, VkImage swapchainImage, VkImageView swapchainView);
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

    MaterialTextures materials;
    float scanStrength = 0.0f;                // 0 until UploadMaterials: how much the shaders use the scanned materials
    std::array<glm::vec4, 3> scanTiles{};     // the real size of each material tile, four to a vector
    RenderTargets targets;
    SceneGeometry geometry;
    SkinnedGeometry skinned;
    ShadowPass shadowPass;
    ScenePass scenePass;
    TonemapPass tonemapPass;
    GpuProfiler profiler;
    DynamicResolution dynamicResolution;
    ScreenshotCapture screenshot;
    IFrameOverlay* overlay = nullptr;  // drawn after the tonemap pass; not owned, see SetOverlay

    float distanceFadeRadius = 0.0f;  // see SetDistanceFade
    bool drawsWithDistanceFade = false;  // whether the scene pipelines use the lit_stream.frag variant
    uint64_t lastFedMeasurement = 0;
    float currentRenderScale = 1.0f;
    VkExtent2D currentRenderExtent{0, 0};
};
