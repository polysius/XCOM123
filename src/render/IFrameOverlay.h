#pragma once

#include <cstdint>

#include <volk.h>

struct GLFWwindow;

// The Vulkan objects an overlay needs to draw on top of a finished frame. They stay valid from Attach to Detach.
struct OverlayTarget {
    GLFWwindow* window = nullptr;
    VkInstance instance = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    uint32_t queueFamily = 0;
    VkQueue queue = VK_NULL_HANDLE;
    VkFormat colorFormat = VK_FORMAT_UNDEFINED;  // the format of the swapchain images, which the overlay draws into
    uint32_t imageCount = 0;                     // how many swapchain images there are
};

// Something drawn over the finished frame, such as an editor. The renderer calls Record after the tonemap pass, while the
// swapchain image is being drawn to, and never when there is no overlay, so frames without one are unchanged.
class IFrameOverlay {
public:
    virtual ~IFrameOverlay() = default;

    // The renderer's device exists. An overlay creates what it needs for drawing here, or later, when it is first used.
    virtual void Attach(const OverlayTarget& target) = 0;
    // The device is about to go, or the overlay is being taken away. The GPU is idle. Everything made on the device is destroyed.
    virtual void Detach() = 0;

    // Whether there is anything to draw in this frame.
    virtual bool IsActive() const = 0;
    // Adds the overlay to the frame. The swapchain image is in the colour attachment layout and holds the tonemapped scene,
    // which the overlay draws over.
    virtual void Record(VkCommandBuffer commandBuffer, VkImage image, VkImageView view, VkExtent2D extent) = 0;
};
