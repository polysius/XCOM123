#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

namespace streaming {

// The address of one square of the world grid. x counts chunks along world X and z along world Z, so the chunk (x, z)
// covers the ground from (x * size, z * size) to ((x + 1) * size, (z + 1) * size), where size is the chunk size.
struct ChunkCoord {
    int32_t x = 0;
    int32_t z = 0;

    friend bool operator==(const ChunkCoord& a, const ChunkCoord& b) { return a.x == b.x && a.z == b.z; }
    friend bool operator!=(const ChunkCoord& a, const ChunkCoord& b) { return !(a == b); }
    // An arbitrary but fixed order, so that sets of chunks can be sorted and compared.
    friend bool operator<(const ChunkCoord& a, const ChunkCoord& b) { return a.x != b.x ? a.x < b.x : a.z < b.z; }
};

struct ChunkCoordHash {
    size_t operator()(const ChunkCoord& coord) const {
        const uint64_t packed = (static_cast<uint64_t>(static_cast<uint32_t>(coord.x)) << 32) |
                                static_cast<uint32_t>(coord.z);
        uint64_t mixed = packed * 0x9E3779B97F4A7C15ull;
        mixed ^= mixed >> 29;
        return static_cast<size_t>(mixed);
    }
};

}  // namespace streaming
