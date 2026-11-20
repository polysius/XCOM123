#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

class JobSystem;

// A generator of tiling PBR materials that read like laser scanned ones: a measured height field with detail from the
// size of the tile down to single pixels, normals taken from that height rather than painted, albedo and roughness that
// follow the geometry (pits are darker and rougher, worn edges are brighter and smoother), and the faint noise of a
// scanner. Everything is made from the seed alone, so a material can be rebuilt instead of stored.

namespace material {

enum class MaterialKind : uint8_t {
    Stone,
    Brick,
    Concrete,
    Plaster,
    Wood,
    Metal,
    Leather,
    Fabric,
    Asphalt,
    Soil,
};

constexpr int kMaterialKindCount = 10;

const char* KindName(MaterialKind kind);
// Finds a kind by its lower case name ("stone", "brick", ...). Returns false for any other text.
bool ParseKind(const std::string& name, MaterialKind& kind);

struct MaterialSettings {
    MaterialKind kind = MaterialKind::Stone;
    uint32_t seed = 1;
    int size = 1024;          // pixels along each side, 32 to 4096
    float wear = 0.5f;        // 0 = new, 1 = old: bright worn edges, dirt in the pits, stains and chips
    float scanDetail = 0.6f;  // 0 = smooth, 1 = the finest grain and the most scanner noise
};

// The maps of a material. Each has size * size values, rows from top to bottom.
struct MaterialMaps {
    int size = 0;
    float tileMeters = 1.0f;    // the real size of one tile
    float reliefMeters = 0.0f;  // the height between the lowest and the highest point
    std::vector<float> height;               // metres above the lowest point
    std::vector<glm::vec3> albedo;           // linear reflectance; for metal, the colour of its reflection
    std::vector<glm::vec3> normal;           // unit vectors in tangent space, +Z out of the surface, +X right, +Y up the image
    std::vector<float> roughness;            // 0 = mirror, 1 = chalk
    std::vector<float> metallic;             // 0 = insulator, 1 = metal
    std::vector<float> ambientOcclusion;     // 1 = open, 0 = a deep crevice
};

// Makes the maps. `jobs` may be null, which runs on the calling thread; the result is the same either way.
MaterialMaps GenerateMaterial(const MaterialSettings& settings, JobSystem* jobs = nullptr);

}  // namespace material
