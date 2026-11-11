#include "render/GpuProfiler.h"

#include "render/VulkanContext.h"
#include "render/VulkanUtils.h"

namespace {
// Weight of the newest sample in the smoothed times: low enough that the window title is
// readable, high enough to follow a real change within a fraction of a second.
constexpr double kSmoothingWeight = 0.1;

// Writing timestamps at "all commands" records when everything before them has finished, so
// each pass's time includes waiting for the work it depends on, and the passes add up to the frame.
constexpr VkPipelineStageFlags2 kTimestampStage = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
}  // namespace

void GpuProfiler::Create(const VulkanContext& context, uint32_t frameSlots, std::span<const std::string_view> passNames) {
    passTimings.clear();
    for (const std::string_view name : passNames) {
        passTimings.push_back({.name = name});
    }

    available = context.timestampValidBits > 0 && context.deviceProperties.limits.timestampPeriod > 0.0f;
    if (!available) {
        return;
    }
    timestampMask = context.timestampValidBits >= 64 ? ~0ull : (1ull << context.timestampValidBits) - 1;
    millisecondsPerTick = static_cast<double>(context.deviceProperties.limits.timestampPeriod) / 1.0e6;
    queriesPerFrame = PassStartQuery(static_cast<uint32_t>(passNames.size()));

    VkQueryPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
    poolInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
    poolInfo.queryCount = queriesPerFrame;
    queryPools.resize(frameSlots, VK_NULL_HANDLE);
    for (VkQueryPool& pool : queryPools) {
        VkCheck(vkCreateQueryPool(context.device, &poolInfo, nullptr, &pool), "vkCreateQueryPool");
    }
    slotHasPendingResults.assign(frameSlots, false);
    readback.resize(static_cast<size_t>(queriesPerFrame) * 2);
}

void GpuProfiler::Destroy(VkDevice device) {
    for (VkQueryPool pool : queryPools) {
        vkDestroyQueryPool(device, pool, nullptr);
    }
    queryPools.clear();
    slotHasPendingResults.clear();
    available = false;
}

void GpuProfiler::CollectResults(VkDevice device, uint32_t frameSlot) {
    if (!available || !slotHasPendingResults[frameSlot]) {
        return;
    }
    slotHasPendingResults[frameSlot] = false;

    // No WAIT flag: the fence has signalled, so the results are ready, and if a driver ever
    // disagreed we would rather skip one measurement than stall the frame.
    const VkResult result = vkGetQueryPoolResults(
        device, queryPools[frameSlot], 0, queriesPerFrame, readback.size() * sizeof(uint64_t), readback.data(),
        2 * sizeof(uint64_t), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WITH_AVAILABILITY_BIT);
    if (result != VK_SUCCESS && result != VK_NOT_READY) {
        VkCheck(result, "vkGetQueryPoolResults");
    }
    for (uint32_t query = 0; query < queriesPerFrame; ++query) {
        const bool isAvailable = readback[query * 2 + 1] != 0;
        if (!isAvailable) {
            return;
        }
    }

    const auto timestamp = [this](uint32_t query) { return readback[query * 2]; };
    const bool isFirstSample = measuredFrames == 0;
    for (uint32_t pass = 0; pass < passTimings.size(); ++pass) {
        GpuPassTiming& timing = passTimings[pass];
        timing.latestMs = TicksToMs(timestamp(PassStartQuery(pass)), timestamp(PassEndQuery(pass)));
        timing.smoothedMs = Smooth(timing.smoothedMs, timing.latestMs, isFirstSample);
    }
    latestFrameMs = TicksToMs(timestamp(kFrameStartQuery), timestamp(kFrameEndQuery));
    smoothedFrameMs = Smooth(smoothedFrameMs, latestFrameMs, isFirstSample);
    ++measuredFrames;
}

void GpuProfiler::BeginFrame(VkCommandBuffer cmd, uint32_t frameSlot) {
    if (!available) {
        return;
    }
    // Queries must be reset before they are written again. Doing it in the command buffer keeps
    // the reset in order with the GPU work that reads and writes them.
    vkCmdResetQueryPool(cmd, queryPools[frameSlot], 0, queriesPerFrame);
    vkCmdWriteTimestamp2(cmd, kTimestampStage, queryPools[frameSlot], kFrameStartQuery);
    slotHasPendingResults[frameSlot] = true;
}

void GpuProfiler::BeginPass(VkCommandBuffer cmd, uint32_t frameSlot, uint32_t passIndex) const {
    if (available) {
        vkCmdWriteTimestamp2(cmd, kTimestampStage, queryPools[frameSlot], PassStartQuery(passIndex));
    }
}

void GpuProfiler::EndPass(VkCommandBuffer cmd, uint32_t frameSlot, uint32_t passIndex) const {
    if (available) {
        vkCmdWriteTimestamp2(cmd, kTimestampStage, queryPools[frameSlot], PassEndQuery(passIndex));
    }
}

void GpuProfiler::EndFrame(VkCommandBuffer cmd, uint32_t frameSlot) const {
    if (available) {
        vkCmdWriteTimestamp2(cmd, kTimestampStage, queryPools[frameSlot], kFrameEndQuery);
    }
}

double GpuProfiler::TicksToMs(uint64_t startTicks, uint64_t endTicks) const {
    // Masking and unsigned subtraction keep the result right even if the counter wrapped around.
    const uint64_t elapsedTicks = ((endTicks & timestampMask) - (startTicks & timestampMask)) & timestampMask;
    return static_cast<double>(elapsedTicks) * millisecondsPerTick;
}

double GpuProfiler::Smooth(double previous, double sample, bool isFirstSample) {
    return isFirstSample ? sample : previous + (sample - previous) * kSmoothingWeight;
}
