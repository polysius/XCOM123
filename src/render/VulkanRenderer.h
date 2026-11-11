#pragma once

#include <array>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "render/IRenderModule.h"
#include "render/Meshes.h"
#include "render/VulkanContext.h"
#include "render/VulkanUtils.h"

// Vulkan 1.3 implementation of the render module.
//
// Every frame runs three passes:
//   1. Shadow pass:  depth of the city and the crowd, seen from the sun, into a shadow map.
//   2. Scene pass:   lit geometry and the sky into a multisampled HDR image (resolved at the end).
//   3. Tonemap pass: HDR image to the window with exposure, a filmic curve and a vignette.
class VulkanRenderer final : public IRenderModule {
public:
    bool Initialize(EngineContext& engine) override;
    void Shutdown() override;
    const char* GetName() const override { return "Vulkan Renderer"; }
    const char* GetVersion() const override { return "0.1.0"; }

    void SetResolution(int width, int height) override;
    void UploadStaticScene(std::span<const BoxInstance> boxes) override;
    void Render(const FrameInput& frame) override;
    std::string GetDeviceName() const override;

private:
    static constexpr uint32_t kFramesInFlight = 2;

    // What one in-flight frame owns, so the CPU can prepare the next frame while the GPU draws this one.
    struct FrameResources {
        VkCommandPool commandPool = VK_NULL_HANDLE;
        VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
        VkSemaphore imageAvailable = VK_NULL_HANDLE;
        VkFence inFlight = VK_NULL_HANDLE;
        GpuBuffer uniforms;
        GpuBuffer npcInstances;
        VkDescriptorSet sceneDescriptors = VK_NULL_HANDLE;
        VkDescriptorSet tonemapDescriptors = VK_NULL_HANDLE;
    };

    struct Mesh {
        GpuBuffer vertices;
        GpuBuffer indices;
        uint32_t indexCount = 0;
    };

    struct PipelineDesc {
        const char* vertexShader = nullptr;
        const char* fragmentShader = nullptr;        // nullptr for depth-only pipelines
        bool usesMeshInput = true;                   // false for full-screen passes
        VkPipelineLayout layout = VK_NULL_HANDLE;
        VkFormat colorFormat = VK_FORMAT_UNDEFINED;  // VK_FORMAT_UNDEFINED when nothing is drawn in colour
        VkFormat depthFormat = VK_FORMAT_UNDEFINED;
        VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT;
        VkCullModeFlags cullMode = VK_CULL_MODE_NONE;
        bool depthTest = false;
        bool depthWrite = false;
        VkCompareOp depthCompare = VK_COMPARE_OP_ALWAYS;
        bool shadowBias = false;                     // depth bias and depth clamping, for the shadow map
    };

    // Set-up, in the order Initialize runs it.
    void CreateFrameResources();
    void CreateShadowMapAndSamplers();
    void CreateDescriptors();
    void CreateSizeDependentResources();
    void CreatePipelines();
    VkPipeline CreatePipeline(const PipelineDesc& desc);
    Mesh UploadMesh(const MeshData& data);

    void DestroySizeDependentResources();
    void DestroyMesh(Mesh& mesh);
    void EnsureNpcCapacity(size_t npcCount);
    bool RecreateSwapchain();

    // Per-frame work.
    glm::mat4 ComputeSunShadowMatrix(const FrameInput& frame) const;
    void WriteFrameUniforms(FrameResources& frame, const FrameInput& input,
                            const glm::mat4& cameraViewProjection, const glm::mat4& lightViewProjection);
    void RecordCommands(FrameResources& frame, uint32_t imageIndex, uint32_t npcCount,
                        const glm::mat4& cameraViewProjection, const glm::mat4& lightViewProjection);
    void RecordShadowPass(VkCommandBuffer cmd, const FrameResources& frame, uint32_t npcCount,
                          const glm::mat4& lightViewProjection);
    void RecordScenePass(VkCommandBuffer cmd, const FrameResources& frame, uint32_t npcCount,
                         const glm::mat4& cameraViewProjection);
    void RecordTonemapPass(VkCommandBuffer cmd, const FrameResources& frame, uint32_t imageIndex);
    void DrawBoxes(VkCommandBuffer cmd, VkPipeline pipeline) const;
    void DrawNpcs(VkCommandBuffer cmd, VkPipeline pipeline, const FrameResources& frame, uint32_t npcCount) const;

    VulkanContext context;
    GLFWwindow* window = nullptr;
    bool resizeRequested = false;
    uint32_t frameIndex = 0;
    VkSampleCountFlagBits msaaSamples = VK_SAMPLE_COUNT_1_BIT;

    std::array<FrameResources, kFramesInFlight> frames{};
    std::vector<VkSemaphore> renderFinished;  // one per swapchain image
    size_t npcCapacity = 0;

    GpuImage shadowMap;
    GpuImage hdrMultisampled;  // only used when multisampling is on
    GpuImage hdrResolved;      // the single-sample HDR image the tonemap pass reads
    GpuImage depthBuffer;
    VkSampler shadowSampler = VK_NULL_HANDLE;
    VkSampler linearSampler = VK_NULL_HANDLE;

    VkDescriptorSetLayout sceneSetLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout tonemapSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
    VkPipelineLayout scenePipelineLayout = VK_NULL_HANDLE;
    VkPipelineLayout tonemapPipelineLayout = VK_NULL_HANDLE;

    VkPipeline boxPipeline = VK_NULL_HANDLE;
    VkPipeline npcPipeline = VK_NULL_HANDLE;
    VkPipeline boxShadowPipeline = VK_NULL_HANDLE;
    VkPipeline npcShadowPipeline = VK_NULL_HANDLE;
    VkPipeline skyPipeline = VK_NULL_HANDLE;
    VkPipeline tonemapPipeline = VK_NULL_HANDLE;

    Mesh cubeMesh;
    Mesh personMesh;
    GpuBuffer boxInstances;
    uint32_t boxInstanceCount = 0;
};
