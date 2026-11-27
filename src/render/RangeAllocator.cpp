#include "render/RangeAllocator.h"

#include <algorithm>

RangeAllocator::RangeAllocator(uint32_t capacity) : capacity(capacity), freeTotal(capacity) {
    if (capacity > 0) {
        freeRanges.emplace(0u, capacity);
    }
}

std::optional<uint32_t> RangeAllocator::Allocate(uint32_t count) {
    if (count == 0) {
        return std::nullopt;
    }
    auto best = freeRanges.end();
    for (auto it = freeRanges.begin(); it != freeRanges.end(); ++it) {
        if (it->second >= count && (best == freeRanges.end() || it->second < best->second)) {
            best = it;
        }
    }
    if (best == freeRanges.end()) {
        return std::nullopt;
    }
    const uint32_t start = best->first;
    const uint32_t length = best->second;
    freeRanges.erase(best);
    if (length > count) {
        freeRanges.emplace(start + count, length - count);
    }
    freeTotal -= count;
    return start;
}

void RangeAllocator::Free(uint32_t start, uint32_t count) {
    if (count == 0) {
        return;
    }
    freeTotal += count;
    uint32_t mergedStart = start;
    uint32_t mergedLength = count;

    // Merge with the free range that ends where this one starts.
    auto next = freeRanges.lower_bound(start);
    if (next != freeRanges.begin()) {
        auto before = std::prev(next);
        if (before->first + before->second == start) {
            mergedStart = before->first;
            mergedLength += before->second;
            freeRanges.erase(before);
        }
    }
    // And with the one that starts where this one ends.
    if (next != freeRanges.end() && next->first == start + count) {
        mergedLength += next->second;
        freeRanges.erase(next);
    }
    freeRanges.emplace(mergedStart, mergedLength);
}

uint32_t RangeAllocator::LargestFree() const {
    uint32_t largest = 0;
    for (const auto& [start, length] : freeRanges) {
        largest = std::max(largest, length);
    }
    return largest;
}
