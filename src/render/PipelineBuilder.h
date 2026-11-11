#pragma once

#include <cstdint>

#include <volk.h>

// What a pipeline reads from the vertex buffers.
enum class MeshInput {
    None,    // full-screen passes: no vertex buffers at all
    Boxes,   // the unit cube, drawn once per BoxInstance
    People,  // a person's body parts, drawn once per NpcInstance
};

// Everything that differs between the engine's graphics pipelines. The rest (triangle lists,
// dynamic viewport and scissor, dynamic rendering) is the same for all of them.
struct PipelineDesc {
    const char* vertexShader = nullptr;
    const char* fragmentShader = nullptr;        // nullptr for depth-only pipelines
    MeshInput meshInput = MeshInput::Boxes;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    VkFormat colorFormat = VK_FORMAT_UNDEFINED;  // VK_FORMAT_UNDEFINED when nothing is drawn in colour
    VkFormat depthFormat = VK_FORMAT_UNDEFINED;
    VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT;
    VkCullModeFlags cullMode = VK_CULL_MODE_NONE;
    bool depthTest = false;
    bool depthWrite = false;
    VkCompareOp depthCompare = VK_COMPARE_OP_ALWAYS;
    bool depthBias = false;   // push depth away from the light, so surfaces do not shadow themselves
    bool depthClamp = false;  // clamp depth instead of clipping, so casters behind the light are kept
};

// Builds a graphics pipeline from compiled shaders in the build's shader folder.
VkPipeline CreateGraphicsPipeline(VkDevice device, const PipelineDesc& desc);

// A pipeline layout with one descriptor set and one push constant range starting at offset 0.
VkPipelineLayout CreatePipelineLayout(VkDevice device, VkDescriptorSetLayout setLayout,
                                      VkShaderStageFlags pushConstantStages, uint32_t pushConstantBytes);
