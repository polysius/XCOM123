#pragma once

#include <vector>

#include <volk.h>

struct GLFWwindow;

// Owns the Vulkan objects every renderer needs, whatever it draws: the instance, the chosen
// GPU, the logical device and its queue, and the swapchain (the images shown in the window).
class VulkanContext {
public:
    void Create(GLFWwindow* window, bool vsync);
    void Destroy();
    // Rebuilds the swapchain for the window's current size. The GPU must be idle.
    void RecreateSwapchain();

    VkInstance instance = VK_NULL_HANDLE;
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkPhysicalDeviceProperties deviceProperties{};
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;  // one queue does both drawing and presenting
    uint32_t queueFamily = 0;
    bool supportsDepthClamp = false;

    VkSwapchainKHR swapchain = VK_NULL_HANDLE;
    VkFormat swapchainFormat = VK_FORMAT_UNDEFINED;
    VkExtent2D swapchainExtent{0, 0};
    std::vector<VkImage> swapchainImages;
    std::vector<VkImageView> swapchainViews;

private:
    void CreateInstance();
    void PickPhysicalDevice();
    void CreateDevice();
    void CreateSwapchain(VkSwapchainKHR oldSwapchain);
    void DestroySwapchainViews();

    GLFWwindow* window = nullptr;
    bool vsync = true;
    VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;
};
