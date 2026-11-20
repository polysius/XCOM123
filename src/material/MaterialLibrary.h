#pragma once

#include <cstdint>
#include <vector>

#include "material/MaterialGenerator.h"

class JobSystem;

// The generated materials packed the way the renderer stores them: two arrays of textures with one layer per kind of
// material and a chain of mip levels, as 8 bit values ready to be copied to the GPU. It holds no Vulkan types, so it
// can be built and tested without a window.
//
// Array 1, `albedoRoughness`: the albedo as sRGB bytes (the hardware converts them to linear when sampling) and the
// roughness in the alpha byte, which is always linear.
// Array 2, `normalOcclusion`, all linear: the x and y of the tangent space normal as 0.5 + 0.5 * value (z follows from
// them, since the normal has length 1), the ambient occlusion, and the metallic value.
//
// A layer is the material of the kind with the same number (see MaterialKind), and every layer is as big as the others.

namespace material {

struct LibrarySettings {
    uint32_t seed = 1;
    int size = 512;            // pixels along each side of a layer, 128 to 2048
    float wear = 0.5f;
    float scanDetail = 0.6f;
};

struct MaterialLibrary {
    int size = 0;       // pixels along each side of the top level
    int layers = 0;     // kMaterialKindCount
    int mipLevels = 0;  // down to a single texel
    std::vector<float> tileMeters;  // the real size of one tile, per layer

    // For each mip level, all layers one after another, 4 bytes per texel, rows from top to bottom.
    std::vector<std::vector<uint8_t>> albedoRoughness;
    std::vector<std::vector<uint8_t>> normalOcclusion;

    int LevelSize(int level) const { return size >> level > 0 ? size >> level : 1; }
};

// A library of one flat grey texel per layer: no pattern, no relief. It stands in until the real materials are built,
// so that the textures can always be bound.
MaterialLibrary MakeNeutralLibrary();

// Generates every kind of material and packs them. `jobs` may be null; the result is the same either way.
MaterialLibrary BuildMaterialLibrary(const LibrarySettings& settings, JobSystem* jobs = nullptr);

}  // namespace material
