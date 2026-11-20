#include "material/MaterialLibrary.h"

#include <algorithm>
#include <cmath>

#include "material/MaterialNoise.h"

namespace material {
namespace {

// One layer's worth of values at one mip level, before they are packed into bytes.
struct Level {
    int size = 0;
    std::vector<glm::vec3> albedo;     // linear
    std::vector<glm::vec3> normal;     // the average of the normals below, not yet scaled to length 1
    std::vector<float> roughness;
    std::vector<float> metallic;
    std::vector<float> occlusion;
};

Level TopLevelOf(const MaterialMaps& maps) {
    Level level;
    level.size = maps.size;
    level.albedo = maps.albedo;
    level.normal = maps.normal;
    level.roughness = maps.roughness;
    level.metallic = maps.metallic;
    level.occlusion = maps.ambientOcclusion;
    return level;
}

// Averages blocks of 2 by 2 texels. A tile repeats, so every level is made from the one above by plain averaging.
Level HalfSizeOf(const Level& above) {
    Level level;
    level.size = std::max(above.size / 2, 1);
    const size_t count = static_cast<size_t>(level.size) * level.size;
    level.albedo.resize(count);
    level.normal.resize(count);
    level.roughness.resize(count);
    level.metallic.resize(count);
    level.occlusion.resize(count);
    for (int y = 0; y < level.size; ++y) {
        for (int x = 0; x < level.size; ++x) {
            glm::vec3 albedo(0.0f);
            glm::vec3 normal(0.0f);
            float roughness = 0.0f;
            float metallic = 0.0f;
            float occlusion = 0.0f;
            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    const int sourceX = std::min(x * 2 + dx, above.size - 1);
                    const int sourceY = std::min(y * 2 + dy, above.size - 1);
                    const size_t source = static_cast<size_t>(sourceY) * above.size + sourceX;
                    albedo += above.albedo[source];
                    normal += above.normal[source];
                    roughness += above.roughness[source];
                    metallic += above.metallic[source];
                    occlusion += above.occlusion[source];
                }
            }
            const size_t target = static_cast<size_t>(y) * level.size + x;
            level.albedo[target] = albedo * 0.25f;
            level.normal[target] = normal * 0.25f;
            level.roughness[target] = roughness * 0.25f;
            level.metallic[target] = metallic * 0.25f;
            level.occlusion[target] = occlusion * 0.25f;
        }
    }
    return level;
}

uint8_t ToByte(float value) {
    return static_cast<uint8_t>(std::lround(std::clamp(value, 0.0f, 1.0f) * 255.0f));
}

float LinearToSrgb(float linear) {
    linear = std::clamp(linear, 0.0f, 1.0f);
    return linear <= 0.0031308f ? 12.92f * linear : 1.055f * std::pow(linear, 1.0f / 2.4f) - 0.055f;
}

// Packs one level of one layer into its place in the two byte arrays.
void PackLevel(const Level& level, uint8_t* albedoRoughness, uint8_t* normalOcclusion) {
    const size_t count = static_cast<size_t>(level.size) * level.size;
    for (size_t i = 0; i < count; ++i) {
        // Where the normals below point different ways their average is shorter than 1, and the surface is rougher
        // than the roughness alone says: the light is spread by the bumps that are now smaller than a texel. Adding
        // the lost length to the squared roughness is a simple form of Toksvig's correction for it.
        const float length = glm::length(level.normal[i]);
        const glm::vec3 normal = length > 1e-4f ? level.normal[i] / length : glm::vec3(0.0f, 0.0f, 1.0f);
        const float roughness = std::sqrt(level.roughness[i] * level.roughness[i] + (1.0f - std::min(length, 1.0f)));

        uint8_t* colour = albedoRoughness + i * 4;
        colour[0] = ToByte(LinearToSrgb(level.albedo[i].r));
        colour[1] = ToByte(LinearToSrgb(level.albedo[i].g));
        colour[2] = ToByte(LinearToSrgb(level.albedo[i].b));
        colour[3] = ToByte(roughness);

        uint8_t* shape = normalOcclusion + i * 4;
        shape[0] = ToByte(0.5f + 0.5f * normal.x);
        shape[1] = ToByte(0.5f + 0.5f * normal.y);
        shape[2] = ToByte(level.occlusion[i]);
        shape[3] = ToByte(level.metallic[i]);
    }
}

}  // namespace

MaterialLibrary MakeNeutralLibrary() {
    MaterialLibrary library;
    library.size = 1;
    library.layers = kMaterialKindCount;
    library.mipLevels = 1;
    library.tileMeters.assign(static_cast<size_t>(library.layers), 1.0f);
    std::vector<uint8_t> colour;
    std::vector<uint8_t> shape;
    for (int layer = 0; layer < library.layers; ++layer) {
        colour.insert(colour.end(), {128, 128, 128, 128});  // mid grey, medium rough
        shape.insert(shape.end(), {128, 128, 255, 0});      // a flat normal, open to the sky, not metallic
    }
    library.albedoRoughness.push_back(colour);
    library.normalOcclusion.push_back(shape);
    return library;
}

MaterialLibrary BuildMaterialLibrary(const LibrarySettings& settings, JobSystem* jobs) {
    MaterialLibrary library;
    library.size = std::clamp(settings.size, 128, 2048);
    library.layers = kMaterialKindCount;
    int levelCount = 1;
    while ((library.size >> levelCount) > 0) {
        ++levelCount;
    }
    library.mipLevels = levelCount;
    library.albedoRoughness.resize(levelCount);
    library.normalOcclusion.resize(levelCount);
    for (int level = 0; level < levelCount; ++level) {
        const size_t bytes = static_cast<size_t>(library.LevelSize(level)) * library.LevelSize(level) * 4 * library.layers;
        library.albedoRoughness[level].resize(bytes);
        library.normalOcclusion[level].resize(bytes);
    }

    for (int layer = 0; layer < library.layers; ++layer) {
        MaterialSettings kindSettings;
        kindSettings.kind = static_cast<MaterialKind>(layer);
        kindSettings.seed = Hash(settings.seed, static_cast<uint32_t>(layer), 0x4D41u);  // each kind gets a seed of its own
        kindSettings.size = library.size;
        kindSettings.wear = settings.wear;
        kindSettings.scanDetail = settings.scanDetail;
        const MaterialMaps maps = GenerateMaterial(kindSettings, jobs);
        library.tileMeters.push_back(maps.tileMeters);

        Level level = TopLevelOf(maps);
        for (int mip = 0; mip < levelCount; ++mip) {
            const size_t texels = static_cast<size_t>(level.size) * level.size;
            PackLevel(level, library.albedoRoughness[mip].data() + texels * 4 * layer,
                      library.normalOcclusion[mip].data() + texels * 4 * layer);
            if (mip + 1 < levelCount) {
                level = HalfSizeOf(level);
            }
        }
    }
    return library;
}

}  // namespace material
