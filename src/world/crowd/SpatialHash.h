#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "world/crowd/CrowdTypes.h"

class JobSystem;

namespace crowd {

// A copy of one person's motion, stored in grid order so that neighbour searches read
// memory in long runs instead of jumping between people.
struct NeighbourEntry {
    glm::vec2 position;
    glm::vec2 velocity;
    uint32_t agent;
};

// A uniform grid over a square window of the city that lists the people in each cell. Only
// people near the focus point need neighbour searches, so the window follows the focus and
// stays small however large the city and the crowd are.
//
// It is rebuilt every step in four phases, each safe to run in parallel:
//   1. Reset, then Count every person's cell.
//   2. FinishCounting turns the counts into cell ranges.
//   3. Insert every counted person.
//   4. Finish sorts each cell (so results do not depend on thread timing) and copies the
//      people's motion into the grid.
class SpatialHash {
public:
    void Reset(glm::vec2 center, float halfSize, float cellSize);

    // The cell containing `point`, or kNoIndex outside the window.
    uint32_t CellOf(glm::vec2 point) const;

    void Count(uint32_t cell);
    void FinishCounting();
    void Insert(uint32_t cell, uint32_t agent);
    void Finish(JobSystem& jobs, const std::vector<AgentMotion>& motion);

    // Calls visit(const NeighbourEntry&) for everyone in the cells overlapping the square of
    // half size `reach` around `point`. Callers filter by exact distance themselves.
    template <typename Visit>
    void ForEachNear(glm::vec2 point, float reach, Visit&& visit) const {
        if (columns == 0) {
            return;
        }
        const glm::vec2 low = (point - reach - origin) / cellSize;
        const glm::vec2 high = (point + reach - origin) / cellSize;
        const int x0 = std::max(static_cast<int>(std::floor(low.x)), 0);
        const int y0 = std::max(static_cast<int>(std::floor(low.y)), 0);
        const int x1 = std::min(static_cast<int>(std::floor(high.x)), columns - 1);
        const int y1 = std::min(static_cast<int>(std::floor(high.y)), columns - 1);
        if (x0 > x1 || y0 > y1) {
            return;  // the square lies outside the window
        }
        // The cells of one row are next to each other in `entries`, so each row is one run.
        for (int y = y0; y <= y1; ++y) {
            const size_t rowStart = static_cast<size_t>(y) * static_cast<size_t>(columns);
            const uint32_t first = cellStart[rowStart + static_cast<size_t>(x0)];
            const uint32_t last = cellStart[rowStart + static_cast<size_t>(x1) + 1];
            for (uint32_t i = first; i < last; ++i) {
                visit(entries[i]);
            }
        }
    }

private:
    glm::vec2 origin{0.0f};
    float cellSize = 2.0f;
    int columns = 0;  // the window is square: as many rows as columns
    std::vector<uint32_t> cellStart;   // per cell, the first slot; one extra entry at the end
    std::vector<uint32_t> fillCursor;  // per cell, the next free slot while inserting
    std::vector<uint32_t> agentInSlot;
    std::vector<NeighbourEntry> entries;
};

}  // namespace crowd
