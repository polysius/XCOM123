#pragma once

#include <cstdint>
#include <map>
#include <optional>

// Hands out ranges of a numbered space, such as the instances of one big GPU buffer, and takes them back. It is a free
// list: the free space is kept as a map from the start of each free range to its length, and a range that is returned is
// merged with the free ranges next to it, so the space does not fragment when chunks of different sizes come and go.
// It touches no GPU memory and does no locking, which makes it plain to test.
class RangeAllocator {
public:
    explicit RangeAllocator(uint32_t capacity);

    // The start of a free range of `count` units, or nothing when no free range is that long. Takes the smallest range
    // that fits, so large ranges stay whole for large requests. Zero units cannot be allocated.
    std::optional<uint32_t> Allocate(uint32_t count);

    // Gives back a range obtained from Allocate with the same count.
    void Free(uint32_t start, uint32_t count);

    uint32_t Capacity() const { return capacity; }
    uint32_t FreeTotal() const { return freeTotal; }
    uint32_t UsedTotal() const { return capacity - freeTotal; }
    uint32_t LargestFree() const;
    size_t FreeRangeCount() const { return freeRanges.size(); }

private:
    uint32_t capacity;
    uint32_t freeTotal;
    std::map<uint32_t, uint32_t> freeRanges;  // start -> length
};
