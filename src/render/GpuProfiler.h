#pragma once

#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include <volk.h>

#include "render/IRenderModule.h"

class VulkanContext;

// Measures how long the GPU spends on each render pass, with timestamp queries.
//
// Every frame slot has its own query pool. A slot's results are read back only after the
// slot's fence has signalled, when they are certain to be ready, so reading them never makes
// the CPU wait for the GPU. The numbers are therefore always one or two frames old.
class GpuProfiler {
public:
    void Create(const VulkanContext& context, uint32_t frameSlots, std::span<const std::string_view> passNames);
    void Destroy(VkDevice device);

    // False when the GPU cannot write timestamps on our queue; all times then stay zero.
    bool IsAvailable() const { return available; }

    // Reads the timestamps the slot's previous frame wrote. Call once the slot's fence has signalled.
    void CollectResults(VkDevice device, uint32_t frameSlot);

    // Recording, in order: BeginFrame, then Begin/EndPass around each pass, then EndFrame.
    void BeginFrame(VkCommandBuffer cmd, uint32_t frameSlot);
    void BeginPass(VkCommandBuffer cmd, uint32_t frameSlot, uint32_t passIndex) const;
    void EndPass(VkCommandBuffer cmd, uint32_t frameSlot, uint32_t passIndex) const;
    void EndFrame(VkCommandBuffer cmd, uint32_t frameSlot) const;

    std::span<const GpuPassTiming> PassTimings() const { return passTimings; }
    double LatestFrameMs() const { return latestFrameMs; }
    double SmoothedFrameMs() const { return smoothedFrameMs; }
    // Counts collected frames, so callers can tell when a new measurement has arrived.
    uint64_t MeasuredFrameCount() const { return measuredFrames; }

private:
    // Query layout in each pool: the frame's start and end, then a start/end pair per pass.
    static constexpr uint32_t kFrameStartQuery = 0;
    static constexpr uint32_t kFrameEndQuery = 1;
    static uint32_t PassStartQuery(uint32_t passIndex) { return 2 + passIndex * 2; }
    static uint32_t PassEndQuery(uint32_t passIndex) { return 3 + passIndex * 2; }

    double TicksToMs(uint64_t startTicks, uint64_t endTicks) const;
    static double Smooth(double previous, double sample, bool isFirstSample);

    bool available = false;
    uint32_t queriesPerFrame = 0;
    uint64_t timestampMask = 0;
    double millisecondsPerTick = 0.0;

    std::vector<VkQueryPool> queryPools;  // one per frame slot
    std::vector<bool> slotHasPendingResults;
    std::vector<uint64_t> readback;  // value and availability word for each query

    std::vector<GpuPassTiming> passTimings;
    double latestFrameMs = 0.0;
    double smoothedFrameMs = 0.0;
    uint64_t measuredFrames = 0;
};
