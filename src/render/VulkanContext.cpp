#include "render/VulkanContext.h"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <optional>
#include <stdexcept>

#include <GLFW/glfw3.h>

#include "render/VulkanUtils.h"

namespace {
#ifdef NDEBUG
constexpr bool kWantValidation = false;
#else
constexpr bool kWantValidation = true;
#endif
constexpr const char* kValidationLayer = "VK_LAYER_KHRONOS_validation";

bool IsInstanceLayerAvailable(const char* name) {
    uint32_t count = 0;
    vkEnumerateInstanceLayerProperties(&count, nullptr);
    std::vector<VkLayerProperties> layers(count);
    vkEnumerateInstanceLayerProperties(&count, layers.data());
    return std::any_of(layers.begin(), layers.end(),
                       [name](const VkLayerProperties& layer) { return std::strcmp(layer.layerName, name) == 0; });
}

bool HasDeviceExtension(VkPhysicalDevice device, const char* name) {
    uint32_t count = 0;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> extensions(count);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &count, extensions.data());
    return std::any_of(extensions.begin(), extensions.end(),
                       [name](const VkExtensionProperties& extension) { return std::strcmp(extension.extensionName, name) == 0; });
}

VKAPI_ATTR VkBool32 VKAPI_CALL OnValidationMessage(VkDebugUtilsMessageSeverityFlagBitsEXT,
                                                   VkDebugUtilsMessageTypeFlagsEXT,
                                                   const VkDebugUtilsMessengerCallbackDataEXT* data, void*) {
    std::cerr << "[Vulkan validation] " << data->pMessage << "\n";
    return VK_FALSE;
}

// A queue family that can both draw and present to the window's surface.
std::optional<uint32_t> FindGraphicsPresentQueue(VkPhysicalDevice device, VkSurfaceKHR surface) {
    uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &count, nullptr);
    std::vector<VkQueueFamilyProperties> families(count);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &count, families.data());

    for (uint32_t index = 0; index < count; ++index) {
        VkBool32 canPresent = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(device, index, surface, &canPresent);
        if ((families[index].queueFlags & VK_QUEUE_GRAPHICS_BIT) && canPresent) {
            return index;
        }
    }
    return std::nullopt;
}

VkSurfaceFormatKHR ChooseSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& formats) {
    // An sRGB format makes the GPU apply gamma encoding when the final image is written.
    for (const VkSurfaceFormatKHR& format : formats) {
        const bool isSrgb = format.format == VK_FORMAT_B8G8R8A8_SRGB || format.format == VK_FORMAT_R8G8B8A8_SRGB;
        if (isSrgb && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            return format;
        }
    }
    return formats.front();
}

VkPresentModeKHR ChoosePresentMode(VkPhysicalDevice device, VkSurfaceKHR surface, bool vsync) {
    if (vsync) {
        return VK_PRESENT_MODE_FIFO_KHR;  // waits for the display's refresh; always available
    }
    uint32_t count = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &count, nullptr);
    std::vector<VkPresentModeKHR> modes(count);
    vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &count, modes.data());

    for (VkPresentModeKHR preferred : {VK_PRESENT_MODE_MAILBOX_KHR, VK_PRESENT_MODE_IMMEDIATE_KHR}) {
        if (std::find(modes.begin(), modes.end(), preferred) != modes.end()) {
            return preferred;
        }
    }
    return VK_PRESENT_MODE_FIFO_KHR;
}
}  // namespace

void VulkanContext::Create(GLFWwindow* targetWindow, bool enableVsync) {
    window = targetWindow;
    vsync = enableVsync;
    CreateInstance();
    VkCheck(glfwCreateWindowSurface(instance, window, nullptr, &surface), "glfwCreateWindowSurface");
    PickPhysicalDevice();
    CreateDevice();
    CreateSwapchain(VK_NULL_HANDLE);
}

void VulkanContext::CreateInstance() {
    // volk finds the Vulkan runtime installed with the GPU driver (vulkan-1.dll on Windows).
    VkCheck(volkInitialize(), "volkInitialize (is a Vulkan-capable GPU driver installed?)");

    uint32_t glfwExtensionCount = 0;
    const char** glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
    if (glfwExtensions == nullptr) {
        throw std::runtime_error("GLFW cannot create Vulkan window surfaces on this system");
    }
    std::vector<const char*> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);
    std::vector<const char*> layers;

    // The validation layer reports API misuse. It ships with the Vulkan SDK, so debug builds
    // turn it on only when the SDK is installed.
    if (kWantValidation && IsInstanceLayerAvailable(kValidationLayer)) {
        layers.push_back(kValidationLayer);
        extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    } else if (kWantValidation) {
        std::cout << "Vulkan validation layer not found. Install the Vulkan SDK to enable it.\n";
    }

    VkApplicationInfo appInfo{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    appInfo.pApplicationName = "XCOM123";
    appInfo.pEngineName = "XCOM123";
    appInfo.apiVersion = VK_API_VERSION_1_3;

    VkInstanceCreateInfo createInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    createInfo.pApplicationInfo = &appInfo;
    createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    createInfo.ppEnabledExtensionNames = extensions.data();
    createInfo.enabledLayerCount = static_cast<uint32_t>(layers.size());
    createInfo.ppEnabledLayerNames = layers.data();
    VkCheck(vkCreateInstance(&createInfo, nullptr, &instance), "vkCreateInstance");
    volkLoadInstance(instance);

    if (!layers.empty()) {
        VkDebugUtilsMessengerCreateInfoEXT messengerInfo{VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT};
        messengerInfo.messageSeverity =
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        messengerInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                                    VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                                    VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        messengerInfo.pfnUserCallback = OnValidationMessage;
        VkCheck(vkCreateDebugUtilsMessengerEXT(instance, &messengerInfo, nullptr, &debugMessenger),
                "vkCreateDebugUtilsMessengerEXT");
    }
}

void VulkanContext::PickPhysicalDevice() {
    uint32_t count = 0;
    vkEnumeratePhysicalDevices(instance, &count, nullptr);
    std::vector<VkPhysicalDevice> devices(count);
    vkEnumeratePhysicalDevices(instance, &count, devices.data());

    int bestScore = -1;
    for (VkPhysicalDevice candidate : devices) {
        VkPhysicalDeviceProperties properties;
        vkGetPhysicalDeviceProperties(candidate, &properties);
        if (properties.apiVersion < VK_API_VERSION_1_3) {
            continue;
        }

        VkPhysicalDeviceVulkan13Features features13{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
        VkPhysicalDeviceFeatures2 features{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
        features.pNext = &features13;
        vkGetPhysicalDeviceFeatures2(candidate, &features);

        const std::optional<uint32_t> family = FindGraphicsPresentQueue(candidate, surface);
        const bool suitable = features13.dynamicRendering && features13.synchronization2 &&
                              HasDeviceExtension(candidate, VK_KHR_SWAPCHAIN_EXTENSION_NAME) && family.has_value();
        if (!suitable) {
            continue;
        }

        // Prefer a dedicated graphics card over integrated graphics (laptops often have both).
        const int score = properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU     ? 2
                          : properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU ? 1
                                                                                            : 0;
        if (score > bestScore) {
            bestScore = score;
            physicalDevice = candidate;
            deviceProperties = properties;
            queueFamily = *family;
            supportsDepthClamp = features.features.depthClamp == VK_TRUE;
        }
    }

    if (physicalDevice == VK_NULL_HANDLE) {
        throw std::runtime_error("No GPU supports Vulkan 1.3 with dynamic rendering");
    }
    std::cout << "GPU: " << deviceProperties.deviceName << "\n";
}

void VulkanContext::CreateDevice() {
    const float queuePriority = 1.0f;
    VkDeviceQueueCreateInfo queueInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    queueInfo.queueFamilyIndex = queueFamily;
    queueInfo.queueCount = 1;
    queueInfo.pQueuePriorities = &queuePriority;

    // Vulkan 1.3 features: drawing without render-pass objects, and the simpler synchronization API.
    VkPhysicalDeviceVulkan13Features features13{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
    features13.dynamicRendering = VK_TRUE;
    features13.synchronization2 = VK_TRUE;
    VkPhysicalDeviceFeatures2 features{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
    features.pNext = &features13;
    // Depth clamping stops shadow casters that sit behind the shadow camera from being cut away.
    features.features.depthClamp = supportsDepthClamp ? VK_TRUE : VK_FALSE;

    const char* extensions[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
    VkDeviceCreateInfo createInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    createInfo.pNext = &features;
    createInfo.queueCreateInfoCount = 1;
    createInfo.pQueueCreateInfos = &queueInfo;
    createInfo.enabledExtensionCount = 1;
    createInfo.ppEnabledExtensionNames = extensions;
    VkCheck(vkCreateDevice(physicalDevice, &createInfo, nullptr, &device), "vkCreateDevice");
    volkLoadDevice(device);
    vkGetDeviceQueue(device, queueFamily, 0, &queue);
}

void VulkanContext::CreateSwapchain(VkSwapchainKHR oldSwapchain) {
    VkSurfaceCapabilitiesKHR capabilities;
    VkCheck(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &capabilities),
            "vkGetPhysicalDeviceSurfaceCapabilitiesKHR");

    uint32_t formatCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(formatCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, formats.data());
    const VkSurfaceFormatKHR surfaceFormat = ChooseSurfaceFormat(formats);

    // The surface either dictates the image size, or reports 0xFFFFFFFF and lets the window decide.
    if (capabilities.currentExtent.width != UINT32_MAX) {
        swapchainExtent = capabilities.currentExtent;
    } else {
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        swapchainExtent.width = std::clamp(static_cast<uint32_t>(width), capabilities.minImageExtent.width,
                                           capabilities.maxImageExtent.width);
        swapchainExtent.height = std::clamp(static_cast<uint32_t>(height), capabilities.minImageExtent.height,
                                            capabilities.maxImageExtent.height);
    }

    // One more image than the minimum, so the CPU rarely waits for the display to release one.
    uint32_t imageCount = capabilities.minImageCount + 1;
    if (capabilities.maxImageCount > 0) {
        imageCount = std::min(imageCount, capabilities.maxImageCount);
    }

    VkSwapchainCreateInfoKHR createInfo{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    createInfo.surface = surface;
    createInfo.minImageCount = imageCount;
    createInfo.imageFormat = surfaceFormat.format;
    createInfo.imageColorSpace = surfaceFormat.colorSpace;
    createInfo.imageExtent = swapchainExtent;
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    createInfo.preTransform = capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = ChoosePresentMode(physicalDevice, surface, vsync);
    createInfo.clipped = VK_TRUE;
    createInfo.oldSwapchain = oldSwapchain;
    VkCheck(vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapchain), "vkCreateSwapchainKHR");
    swapchainFormat = surfaceFormat.format;

    uint32_t count = 0;
    vkGetSwapchainImagesKHR(device, swapchain, &count, nullptr);
    swapchainImages.resize(count);
    vkGetSwapchainImagesKHR(device, swapchain, &count, swapchainImages.data());

    swapchainViews.resize(count);
    for (uint32_t index = 0; index < count; ++index) {
        VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        viewInfo.image = swapchainImages[index];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = swapchainFormat;
        viewInfo.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        VkCheck(vkCreateImageView(device, &viewInfo, nullptr, &swapchainViews[index]), "vkCreateImageView");
    }
}

void VulkanContext::RecreateSwapchain() {
    DestroySwapchainViews();
    const VkSwapchainKHR oldSwapchain = swapchain;
    CreateSwapchain(oldSwapchain);
    vkDestroySwapchainKHR(device, oldSwapchain, nullptr);
}

void VulkanContext::DestroySwapchainViews() {
    for (VkImageView view : swapchainViews) {
        vkDestroyImageView(device, view, nullptr);
    }
    swapchainViews.clear();
    swapchainImages.clear();  // owned by the swapchain, nothing to destroy
}

void VulkanContext::Destroy() {
    if (device != VK_NULL_HANDLE) {
        DestroySwapchainViews();
        vkDestroySwapchainKHR(device, swapchain, nullptr);
        vkDestroyDevice(device, nullptr);
    }
    if (instance != VK_NULL_HANDLE) {
        if (surface != VK_NULL_HANDLE) {
            vkDestroySurfaceKHR(instance, surface, nullptr);
        }
        if (debugMessenger != VK_NULL_HANDLE) {
            vkDestroyDebugUtilsMessengerEXT(instance, debugMessenger, nullptr);
        }
        vkDestroyInstance(instance, nullptr);
    }
    *this = VulkanContext();
}
