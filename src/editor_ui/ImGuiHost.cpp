#include "editor_ui/ImGuiHost.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include "editor_ui/EditorTheme.h"

namespace editor_ui {
namespace {

// The backend needs one image sampler descriptor for the font atlas, and asserts (in debug builds) that the pool it is asked to
// make is larger than that minimum, so a few more are allowed for.
constexpr uint32_t kDescriptorPoolSize = IMGUI_IMPL_VULKAN_MINIMUM_IMAGE_SAMPLER_POOL_SIZE + 7;

void ReportVulkanError(VkResult result) {
    if (result != VK_SUCCESS) {
        std::cerr << "ImGui Vulkan backend: error " << static_cast<int>(result) << "\n";
    }
}

bool IsSrgbFormat(VkFormat format) {
    return format == VK_FORMAT_B8G8R8A8_SRGB || format == VK_FORMAT_R8G8B8A8_SRGB;
}

// ImGui colours are meant to be written as they are. The swapchain is sRGB, so the hardware would encode them a second time and
// wash them out; turning each colour of the draw data to linear first makes the screen show the colour that was chosen.
std::array<uint8_t, 256> MakeLinearTable() {
    std::array<uint8_t, 256> table{};
    for (size_t value = 0; value < table.size(); ++value) {
        const float encoded = static_cast<float>(value) / 255.0f;
        const float linear = encoded <= 0.04045f ? encoded / 12.92f : std::pow((encoded + 0.055f) / 1.055f, 2.4f);
        table[value] = static_cast<uint8_t>(linear * 255.0f + 0.5f);
    }
    return table;
}

void ConvertColorsToLinear(ImDrawData* drawData) {
    static const std::array<uint8_t, 256> table = MakeLinearTable();
    for (int list = 0; list < drawData->CmdListsCount; ++list) {
        ImDrawList* drawList = drawData->CmdLists[list];
        for (ImDrawVert& vertex : drawList->VtxBuffer) {
            const ImU32 color = vertex.col;
            const ImU32 red = table[(color >> IM_COL32_R_SHIFT) & 0xFF];
            const ImU32 green = table[(color >> IM_COL32_G_SHIFT) & 0xFF];
            const ImU32 blue = table[(color >> IM_COL32_B_SHIFT) & 0xFF];
            vertex.col = (color & IM_COL32_A_MASK) | (red << IM_COL32_R_SHIFT) | (green << IM_COL32_G_SHIFT) | (blue << IM_COL32_B_SHIFT);
        }
    }
}

}  // namespace

bool ImGuiHost::Create(const OverlayTarget& target) {
    if (created) {
        return true;
    }
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;  // the editor window has a fixed place, and no layout file is wanted next to the program
    ApplyEditorStyle();

    // The GLFW callbacks are not installed here but by SetInputConnected, while the editor is shown, so that nothing queues up in
    // ImGui while it is hidden and no frames are being built.
    window = target.window;
    if (!ImGui_ImplGlfw_InitForVulkan(target.window, false)) {
        std::cerr << "Editor: the ImGui GLFW backend did not start\n";
        ImGui::DestroyContext();
        return false;
    }

    colorFormat = target.colorFormat;
    ImGui_ImplVulkan_InitInfo info{};
    info.ApiVersion = VK_API_VERSION_1_3;
    info.Instance = target.instance;
    info.PhysicalDevice = target.physicalDevice;
    info.Device = target.device;
    info.QueueFamily = target.queueFamily;
    info.Queue = target.queue;
    info.MinImageCount = std::max(target.imageCount, 2u);
    info.ImageCount = info.MinImageCount;
    info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    info.DescriptorPoolSize = kDescriptorPoolSize;
    info.UseDynamicRendering = true;
    info.PipelineRenderingCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
    info.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
    info.PipelineRenderingCreateInfo.pColorAttachmentFormats = &colorFormat;
    info.CheckVkResultFn = ReportVulkanError;
    // The renderer loads Vulkan through volk, and the backend is built to call volk's functions (IMGUI_IMPL_VULKAN_USE_VOLK), so
    // there is nothing to load here: volkLoadInstance and volkLoadDevice ran when the renderer made its device.
    if (!ImGui_ImplVulkan_Init(&info)) {
        std::cerr << "Editor: the ImGui Vulkan backend did not start\n";
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        return false;
    }
    created = true;
    return true;
}

void ImGuiHost::Destroy() {
    if (!created) {
        return;
    }
    SetInputConnected(false);
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    created = false;
}

void ImGuiHost::SetInputConnected(bool connected) {
    if (!created || connected == inputConnected) {
        return;
    }
    inputConnected = connected;
    if (!connected) {
        ImGui_ImplGlfw_RestoreCallbacks(window);
        return;
    }
    // The engine has already set its own GLFW callbacks. ImGui installs after them and calls them in turn, so both see the input.
    ImGui_ImplGlfw_InstallCallbacks(window);
    // Nothing that happened while the editor was hidden may count: a key or button that was down then is not down now.
    ImGuiIO& io = ImGui::GetIO();
    io.ClearInputKeys();
    io.ClearInputMouse();
    io.AddFocusEvent(true);
}

void ImGuiHost::BeginFrame() {
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImGuiHost::EndFrame() {
    ImGui::Render();
    if (IsSrgbFormat(colorFormat)) {
        ConvertColorsToLinear(ImGui::GetDrawData());
    }
}

void ImGuiHost::Record(VkCommandBuffer commandBuffer, VkImageView view, VkExtent2D extent) const {
    ImDrawData* drawData = ImGui::GetDrawData();
    if (!created || drawData == nullptr || drawData->CmdListsCount == 0) {
        return;
    }
    VkRenderingAttachmentInfo colorAttachment{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
    colorAttachment.imageView = view;
    colorAttachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;  // the scene is already there
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

    VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
    rendering.renderArea = {{0, 0}, extent};
    rendering.layerCount = 1;
    rendering.colorAttachmentCount = 1;
    rendering.pColorAttachments = &colorAttachment;
    vkCmdBeginRendering(commandBuffer, &rendering);
    ImGui_ImplVulkan_RenderDrawData(drawData, commandBuffer);
    vkCmdEndRendering(commandBuffer);
}

}  // namespace editor_ui
