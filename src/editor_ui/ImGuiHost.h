#pragma once

#include <volk.h>

#include "render/IFrameOverlay.h"

// Dear ImGui set up on the renderer's Vulkan device and the engine's window: the context, the GLFW backend that feeds it the
// mouse and keyboard, and the Vulkan backend that draws it with dynamic rendering, as the renderer does.
//
// Nothing here exists until Create is called, so a run without an editor never touches ImGui.

namespace editor_ui {

class ImGuiHost {
public:
    ~ImGuiHost() { Destroy(); }

    // Creates the context and both backends. Returns false, and says why on stderr, if the backends could not start.
    bool Create(const OverlayTarget& target);
    // The GPU must be idle. Does nothing if nothing was created.
    void Destroy();
    bool IsCreated() const { return created; }

    // Lets ImGui receive the window's mouse and keyboard events, or stops it. While it is stopped, no event reaches ImGui, so none
    // are held back to be replayed later; on connecting, the state of keys and buttons is cleared.
    void SetInputConnected(bool connected);

    // Starts the ImGui frame; the caller builds the interface after it and then calls Render.
    void BeginFrame();
    // Ends the ImGui frame and keeps its draw data for Record.
    void EndFrame();
    // Draws what EndFrame kept over the colour image, which is in the colour attachment layout and keeps what it holds.
    void Record(VkCommandBuffer commandBuffer, VkImageView view, VkExtent2D extent) const;

private:
    bool created = false;
    bool inputConnected = false;
    GLFWwindow* window = nullptr;
    // The Vulkan backend keeps a pointer to this format for as long as it lives.
    VkFormat colorFormat = VK_FORMAT_UNDEFINED;
};

}  // namespace editor_ui
