#pragma once

#include <vector>

#include <volk.h>

#include "material/MaterialLibrary.h"
#include "render/VulkanContext.h"
#include "render/VulkanUtils.h"

// The scanned materials on the GPU: two texture arrays with one layer per kind of material and a full chain of mip
// levels (see MaterialLibrary for what the channels hold), and the sampler that reads them with trilinear and
// anisotropic filtering. The textures are copied once when they are created and never change.
class MaterialTextures {
public:
    // Copies the library to the GPU and waits for the copy to finish.
    void Create(const VulkanContext& context, const material::MaterialLibrary& library);
    void Destroy(VkDevice device);

    VkImageView AlbedoRoughnessView() const { return albedoRoughness.view; }
    VkImageView NormalOcclusionView() const { return normalOcclusion.view; }
    VkSampler Sampler() const { return sampler; }
    // The real size in metres of one tile of each layer, which the shader needs to scale the textures to the world.
    const std::vector<float>& TileMeters() const { return tileMeters; }

private:
    GpuImage albedoRoughness;  // sRGB: albedo in rgb, roughness in a
    GpuImage normalOcclusion;  // linear: normal x and y, occlusion, metallic
    VkSampler sampler = VK_NULL_HANDLE;
    std::vector<float> tileMeters;
};
