#include "world/crowd/SpatialHash.h"

#include <atomic>

#include "core/JobSystem.h"

namespace crowd {

void SpatialHash::Reset(glm::vec2 center, float halfSize, float newCellSize) {
    cellSize = newCellSize;
    columns = std::max(1, static_cast<int>(std::ceil(2.0f * halfSize / cellSize)));
    origin = center - 0.5f * cellSize * static_cast<float>(columns);
    // cellStart holds the counts until FinishCounting turns them into ranges.
    cellStart.assign(static_cast<size_t>(columns) * static_cast<size_t>(columns) + 1, 0);
}

uint32_t SpatialHash::CellOf(glm::vec2 point) const {
    const glm::vec2 local = (point - origin) / cellSize;
    if (!(local.x >= 0.0f && local.y >= 0.0f)) {
        return kNoIndex;  // also rejects NaN
    }
    const int x = static_cast<int>(local.x);
    const int y = static_cast<int>(local.y);
    if (x >= columns || y >= columns) {
        return kNoIndex;
    }
    return static_cast<uint32_t>(y * columns + x);
}

void SpatialHash::Count(uint32_t cell) {
    std::atomic_ref<uint32_t>(cellStart[cell]).fetch_add(1, std::memory_order_relaxed);
}

void SpatialHash::FinishCounting() {
    // Exclusive prefix sum: each cell's range starts where the previous one ends.
    uint32_t total = 0;
    for (size_t cell = 0; cell + 1 < cellStart.size(); ++cell) {
        const uint32_t count = cellStart[cell];
        cellStart[cell] = total;
        total += count;
    }
    cellStart.back() = total;
    fillCursor.assign(cellStart.begin(), cellStart.end() - 1);
    agentInSlot.resize(total);
    entries.resize(total);
}

void SpatialHash::Insert(uint32_t cell, uint32_t agent) {
    const uint32_t slot = std::atomic_ref<uint32_t>(fillCursor[cell]).fetch_add(1, std::memory_order_relaxed);
    agentInSlot[slot] = agent;
}

void SpatialHash::Finish(JobSystem& jobs, const std::vector<AgentMotion>& motion) {
    const size_t cellCount = cellStart.size() - 1;
    jobs.ParallelFor(cellCount, 256, [&](size_t begin, size_t end) {
        for (size_t cell = begin; cell < end; ++cell) {
            const auto first = agentInSlot.begin() + cellStart[cell];
            const auto last = agentInSlot.begin() + cellStart[cell + 1];
            std::sort(first, last);
            for (uint32_t slot = cellStart[cell]; slot < cellStart[cell + 1]; ++slot) {
                const uint32_t agent = agentInSlot[slot];
                entries[slot] = {motion[agent].position, motion[agent].velocity, agent};
            }
        }
    });
}

}  // namespace crowd
