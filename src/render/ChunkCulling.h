#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

// Instances of one page of the chunk arena that a pass draws with one draw call.
struct InstanceRange {
    uint32_t page = 0;
    uint32_t first = 0;
    uint32_t count = 0;
};

// Sorts ranges by page and position and joins the ones that touch, so that the chunks which lie next to each other in
// the arena (chunks are often allocated one after the other) are one draw call.
inline void MergeAdjacentRanges(std::vector<InstanceRange>& ranges) {
    std::sort(ranges.begin(), ranges.end(), [](const InstanceRange& a, const InstanceRange& b) {
        return a.page != b.page ? a.page < b.page : a.first < b.first;
    });
    std::vector<InstanceRange> merged;
    for (const InstanceRange& range : ranges) {
        const bool touchesPrevious = !merged.empty() && merged.back().page == range.page &&
                                     merged.back().first + merged.back().count == range.first;
        if (touchesPrevious) {
            merged.back().count += range.count;
        } else {
            merged.push_back(range);
        }
    }
    ranges = std::move(merged);
}
