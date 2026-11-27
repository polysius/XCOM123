#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

// The plain data that the parts of an appearance (skin, eyes, hair, clothes) are made of: a skinned and textured mesh a
// renderer can draw, and 8 bit images it can upload. Nothing here holds a GPU type.

namespace character {

// A triangle mesh with the same skinning as CharacterMesh: up to four bones per vertex, weights that add up to 1, in the
// rest pose. Triangles are counter-clockwise seen from outside.
struct RenderMesh {
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> uvs;
    std::vector<std::array<uint8_t, 4>> boneIndices;
    std::vector<std::array<float, 4>> boneWeights;
    std::vector<uint32_t> indices;

    size_t VertexCount() const { return positions.size(); }
    size_t TriangleCount() const { return indices.size() / 3; }
};

// An 8 bit image, rows from top to bottom, `channels` bytes to a pixel.
struct ByteImage {
    int width = 0;
    int height = 0;
    int channels = 4;
    std::vector<uint8_t> bytes;

    ByteImage() = default;
    ByteImage(int w, int h, int c) : width(w), height(h), channels(c), bytes(static_cast<size_t>(w) * h * c, 0) {}

    uint8_t* Pixel(int x, int y) { return &bytes[(static_cast<size_t>(y) * width + x) * channels]; }
    const uint8_t* Pixel(int x, int y) const { return &bytes[(static_cast<size_t>(y) * width + x) * channels]; }
};

// What a garment is made of: the kind of the procedural material (see material/MaterialGenerator.h) that its texture is
// made from, and how it is tinted.
enum class GarmentMaterial : uint8_t {
    Cotton,
    Denim,
    Leather,
};

}  // namespace character
