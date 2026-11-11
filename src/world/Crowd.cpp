#include "world/Crowd.h"

#include <algorithm>
#include <chrono>
#include <limits>

#include "core/JobSystem.h"
#include "scene/Frustum.h"
#include "world/crowd/AgentUpdate.h"
#include "world/crowd/CrowdSpawner.h"

namespace {

// Simulation level of detail, by distance from the focus point.
constexpr float kNearTierDistance = 35.0f;
constexpr float kMidTierDistance = 90.0f;

// The neighbour grid covers everyone who avoids others, plus a margin so that people at
// the edge of the mid tier still see their neighbours just outside it.
constexpr float kNeighbourGridHalfSize = kMidTierDistance + 3.0f;
constexpr float kNeighbourCellSize = 2.5f;

// A long hitch (a breakpoint, a window drag) must not teleport people through walls.
constexpr float kMaxStepSeconds = 0.1f;

// The focus point is placed at eye height above the street.
constexpr float kEyeHeight = 1.7f;

// Work sizes for the parallel loops, large enough that scheduling costs little.
constexpr size_t kAgentChunkSize = 4096;
constexpr size_t kUnitsPerJob = 128;

using Clock = std::chrono::steady_clock;

double MillisecondsBetween(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

crowd::SimTier TierAtDistanceSquared(float distanceSquared) {
    if (distanceSquared < kNearTierDistance * kNearTierDistance) {
        return crowd::SimTier::Near;
    }
    if (distanceSquared < kMidTierDistance * kMidTierDistance) {
        return crowd::SimTier::Mid;
    }
    return crowd::SimTier::Far;
}

// The middle of the street network's bounding box.
glm::vec2 StreetNetworkCenter(const StreetGraph& streets) {
    if (streets.nodes.empty()) {
        return glm::vec2(0.0f);
    }
    glm::vec2 low(std::numeric_limits<float>::max());
    glm::vec2 high(std::numeric_limits<float>::lowest());
    for (glm::vec2 node : streets.nodes) {
        low = glm::min(low, node);
        high = glm::max(high, node);
    }
    return (low + high) * 0.5f;
}

}  // namespace

Crowd::Crowd() : Crowd(JobSystem::Shared()) {}

Crowd::Crowd(JobSystem& jobs) : jobs(&jobs) {}

void Crowd::Spawn(const CityLayout& city, const CrowdSettings& settings) {
    walkways.Build(city);
    state.Clear();
    crowd::SpawnCrowd(settings, walkways, state);

    agentCells.assign(state.AgentCount(), crowd::kNoIndex);
    const glm::vec2 center = StreetNetworkCenter(walkways.Streets());
    focusPoint = glm::vec3(center.x, walkways.WalkHeight() + kEyeHeight, center.y);
    simulationTime = 0.0f;
    stepIndex = 0;
}

void Crowd::SetFocusPoint(glm::vec3 point) {
    focusPoint = point;
}

void Crowd::Update(float deltaSeconds) {
    const float stepSeconds = std::min(deltaSeconds, kMaxStepSeconds);
    if (!(stepSeconds > 0.0f) || state.AgentCount() == 0) {
        return;  // also catches a NaN time step
    }
    const Clock::time_point start = Clock::now();
    AssignTiersAndCells();
    const Clock::time_point tiersDone = Clock::now();
    BuildNeighbourGrid();
    const Clock::time_point gridDone = Clock::now();
    StepUnits(stepSeconds);
    const Clock::time_point agentsDone = Clock::now();

    lastTierMs = MillisecondsBetween(start, tiersDone);
    lastGridMs = MillisecondsBetween(tiersDone, gridDone);
    lastAgentMs = MillisecondsBetween(gridDone, agentsDone);
    simulationTime += stepSeconds;
    ++stepIndex;
}

// Decides everyone's simulation tier for this step and counts who goes in which grid cell.
void Crowd::AssignTiersAndCells() {
    const glm::vec2 focus(focusPoint.x, focusPoint.z);
    neighbourGrid.Reset(focus, kNeighbourGridHalfSize, kNeighbourCellSize);

    const size_t agentCount = state.AgentCount();
    const size_t chunkCount = (agentCount + kAgentChunkSize - 1) / kAgentChunkSize;
    chunkTierCounts.assign(chunkCount, {});
    // The height difference counts too, so a camera high above the street simulates less.
    const float heightOffset = focusPoint.y - walkways.WalkHeight();
    const float heightOffsetSquared = heightOffset * heightOffset;

    jobs->ParallelFor(chunkCount, 1, [&](size_t firstChunk, size_t endChunk) {
        for (size_t chunk = firstChunk; chunk < endChunk; ++chunk) {
            // Counted locally and stored once: neighbouring chunks' counters share cache
            // lines, and updating them per person would make the threads fight over them.
            std::array<uint32_t, 3> tierCounts{};
            const size_t end = std::min(agentCount, (chunk + 1) * kAgentChunkSize);
            for (size_t agent = chunk * kAgentChunkSize; agent < end; ++agent) {
                const glm::vec2 position = state.motion[agent].position;
                const glm::vec2 offset = position - focus;
                const crowd::SimTier tier = TierAtDistanceSquared(glm::dot(offset, offset) + heightOffsetSquared);
                state.tiers[agent] = tier;
                ++tierCounts[static_cast<size_t>(tier)];

                const uint32_t cell = neighbourGrid.CellOf(position);
                agentCells[agent] = cell;
                if (cell != crowd::kNoIndex) {
                    neighbourGrid.Count(cell);
                }
            }
            chunkTierCounts[chunk] = tierCounts;
        }
    });
}

void Crowd::BuildNeighbourGrid() {
    neighbourGrid.FinishCounting();
    const size_t agentCount = state.AgentCount();
    jobs->ParallelFor(agentCount, kAgentChunkSize, [&](size_t begin, size_t end) {
        for (size_t agent = begin; agent < end; ++agent) {
            if (agentCells[agent] != crowd::kNoIndex) {
                neighbourGrid.Insert(agentCells[agent], static_cast<uint32_t>(agent));
            }
        }
    });
    neighbourGrid.Finish(*jobs, state.motion);
}

// Units (single people, walking groups and conversation circles) are independent of each
// other: avoidance only reads the neighbour grid's copy of everyone's motion, so each unit
// can be updated on any thread.
void Crowd::StepUnits(float deltaSeconds) {
    const crowd::StepContext context{walkways, neighbourGrid, deltaSeconds, simulationTime, stepIndex};
    jobs->ParallelFor(state.units.size(), kUnitsPerJob, [&](size_t begin, size_t end) {
        for (size_t unit = begin; unit < end; ++unit) {
            crowd::StepUnit(context, state, static_cast<uint32_t>(unit));
        }
    });
}

void Crowd::CollectVisible(const Frustum& frustum, std::vector<NpcInstance>& visible) const {
    visibility.Collect(*jobs, state, walkways.WalkHeight(), frustum, focusPoint, visible);
}

CrowdStats Crowd::Stats() const {
    CrowdStats stats;
    for (const std::array<uint32_t, 3>& tierCounts : chunkTierCounts) {
        stats.nearAgents += tierCounts[static_cast<size_t>(crowd::SimTier::Near)];
        stats.midAgents += tierCounts[static_cast<size_t>(crowd::SimTier::Mid)];
        stats.farAgents += tierCounts[static_cast<size_t>(crowd::SimTier::Far)];
    }
    stats.visibleByBand = visibility.LastBandCounts();
    stats.tierMs = lastTierMs;
    stats.gridMs = lastGridMs;
    stats.agentMs = lastAgentMs;
    return stats;
}
