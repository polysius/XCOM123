#include "render/ChunkArena.h"

#include <algorithm>
#include <cstring>

#include "render/VulkanContext.h"

void ChunkArena::Create(const VulkanContext& context, uint32_t frameSlots) {
    device = context.device;
    for (uint32_t i = 0; i < kFirstPages; ++i) {
        AddPage(context);
    }
    slots.resize(frameSlots);
    for (Slot& slot : slots) {
        slot.staging = CreateStagingBuffer(context.device, context.physicalDevice, kStagingBytes);
    }
}

void ChunkArena::Destroy(VkDevice destroyingDevice) {
    for (Slot& slot : slots) {
        DestroyBuffer(destroyingDevice, slot.staging);
    }
    slots.clear();
    for (Page& page : pages) {
        DestroyBuffer(destroyingDevice, page.buffer);
    }
    pages.clear();
    chunks.clear();
    freeChunkIds.clear();
    pending.clear();
    liveChunks = 0;
}

void ChunkArena::AddPage(const VulkanContext& context) {
    Page& page = pages.emplace_back();
    page.buffer = CreateDeviceBuffer(context.device, context.physicalDevice,
                                     static_cast<VkDeviceSize>(kInstancesPerPage) * sizeof(BoxInstance),
                                     VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
}

bool ChunkArena::ReserveRange(const VulkanContext& context, uint32_t count, uint32_t& page, uint32_t& first) {
    for (int attempt = 0; attempt < 2; ++attempt) {
        for (uint32_t index = 0; index < pages.size(); ++index) {
            if (const std::optional<uint32_t> start = pages[index].allocator.Allocate(count)) {
                page = index;
                first = *start;
                return true;
            }
        }
        // No page has room, so open another. This is the only allocation of GPU memory the arena makes after start-up.
        AddPage(context);
    }
    return false;
}

ChunkArena::ChunkId ChunkArena::Add(const VulkanContext& context, std::shared_ptr<const void> owner,
                                    std::span<const BoxInstance> boxes, const Aabb& bounds) {
    const uint32_t count = static_cast<uint32_t>(boxes.size());
    if (count == 0 || count > kInstancesPerPage) {
        return kInvalidChunk;
    }
    Chunk chunk;
    if (!ReserveRange(context, count, chunk.page, chunk.first)) {
        return kInvalidChunk;
    }
    chunk.live = true;
    chunk.count = count;
    chunk.bounds = bounds;
    chunk.owner = std::move(owner);
    chunk.boxes = boxes;

    ChunkId id;
    if (!freeChunkIds.empty()) {
        id = freeChunkIds.back();
        freeChunkIds.pop_back();
        chunks[id] = std::move(chunk);
    } else {
        id = static_cast<ChunkId>(chunks.size());
        chunks.push_back(std::move(chunk));
    }
    pending.push_back(id);
    ++liveChunks;
    return id;
}

void ChunkArena::Remove(ChunkId id, uint64_t frameNumber) {
    if (id >= chunks.size() || !chunks[id].live) {
        return;
    }
    Chunk& chunk = chunks[id];
    const FreedRange range{chunk.page, chunk.first, chunk.count};
    if (chunk.uploaded) {
        retiring.Defer(range, frameNumber);  // frames in flight may still draw it
    } else {
        // Never copied, so no frame has used it: the range can be reused at once.
        pages[range.page].allocator.Free(range.first, range.count);
        pending.erase(std::remove(pending.begin(), pending.end(), id), pending.end());
    }
    chunk = {};
    freeChunkIds.push_back(id);
    --liveChunks;
}

void ChunkArena::BeginFrame(uint32_t frameSlot, int64_t completedFrame) {
    for (const FreedRange& range : retiring.Collect(completedFrame)) {
        pages[range.page].allocator.Free(range.first, range.count);
    }

    Slot& slot = slots[frameSlot];
    slot.copies.clear();
    slot.stagedBytes = 0;
    auto* staging = static_cast<unsigned char*>(slot.staging.mapped);
    size_t stagedChunks = 0;
    for (const ChunkId id : pending) {
        Chunk& chunk = chunks[id];
        const VkDeviceSize bytes = static_cast<VkDeviceSize>(chunk.count) * sizeof(BoxInstance);
        if (slot.stagedBytes + bytes > kStagingBytes) {
            break;  // the rest waits for the next frame, and is not drawn until then
        }
        std::memcpy(staging + slot.stagedBytes, chunk.boxes.data(), bytes);
        slot.copies.push_back({slot.stagedBytes, chunk.page, static_cast<VkDeviceSize>(chunk.first) * sizeof(BoxInstance), bytes});
        slot.stagedBytes += bytes;
        chunk.uploaded = true;
        chunk.owner.reset();
        chunk.boxes = {};
        ++stagedChunks;
    }
    pending.erase(pending.begin(), pending.begin() + static_cast<std::ptrdiff_t>(stagedChunks));
    stats.uploadBytesThisFrame = static_cast<size_t>(slot.stagedBytes);
    stats.uploadBytesMaxFrame = std::max(stats.uploadBytesMaxFrame, stats.uploadBytesThisFrame);
}

void ChunkArena::RecordUploads(VkCommandBuffer cmd, uint32_t frameSlot) {
    const Slot& slot = slots[frameSlot];
    if (slot.copies.empty()) {
        return;
    }
    for (uint32_t page = 0; page < pages.size(); ++page) {
        std::vector<VkBufferCopy> regions;
        for (const StagedCopy& copy : slot.copies) {
            if (copy.page == page) {
                regions.push_back({copy.sourceOffset, copy.destinationOffset, copy.bytes});
            }
        }
        if (!regions.empty()) {
            vkCmdCopyBuffer(cmd, slot.staging.buffer, pages[page].buffer.buffer, static_cast<uint32_t>(regions.size()),
                            regions.data());
        }
    }
    // The shadow and scene passes fetch the instances as vertex attributes, so the copies must finish and be visible first.
    VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    barrier.dstStageMask = VK_PIPELINE_STAGE_2_VERTEX_INPUT_BIT;
    barrier.dstAccessMask = VK_ACCESS_2_VERTEX_ATTRIBUTE_READ_BIT;
    VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    dependency.memoryBarrierCount = 1;
    dependency.pMemoryBarriers = &barrier;
    vkCmdPipelineBarrier2(cmd, &dependency);
}

void ChunkArena::Cull(const Frustum& camera, const Frustum& sun) {
    cameraRanges.clear();
    sunRanges.clear();
    uint32_t seenByCamera = 0;
    uint32_t seenBySun = 0;
    for (const Chunk& chunk : chunks) {
        if (!chunk.live || !chunk.uploaded) {
            continue;
        }
        const InstanceRange range{chunk.page, chunk.first, chunk.count};
        if (camera.ContainsBox(chunk.bounds.min, chunk.bounds.max)) {
            cameraRanges.push_back(range);
            ++seenByCamera;
        }
        if (sun.ContainsBox(chunk.bounds.min, chunk.bounds.max)) {
            sunRanges.push_back(range);
            ++seenBySun;
        }
    }
    MergeAdjacentRanges(cameraRanges);
    MergeAdjacentRanges(sunRanges);
    stats.chunksDrawnByCamera = seenByCamera;
    stats.chunksDrawnBySun = seenBySun;
    stats.drawCallsForCamera = static_cast<uint32_t>(cameraRanges.size());
    stats.drawCallsForSun = static_cast<uint32_t>(sunRanges.size());
}

void ChunkArena::Draw(VkCommandBuffer cmd, VkPipeline pipeline, VkBuffer cubeVertices, VkBuffer cubeIndices,
                      uint32_t cubeIndexCount, const std::vector<InstanceRange>& ranges) const {
    if (ranges.empty()) {
        return;
    }
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    vkCmdBindIndexBuffer(cmd, cubeIndices, 0, VK_INDEX_TYPE_UINT32);
    uint32_t boundPage = 0xFFFFFFFFu;
    const VkDeviceSize offsets[] = {0, 0};
    for (const InstanceRange& range : ranges) {
        if (range.page != boundPage) {
            const VkBuffer buffers[] = {cubeVertices, pages[range.page].buffer.buffer};
            vkCmdBindVertexBuffers(cmd, 0, 2, buffers, offsets);
            boundPage = range.page;
        }
        vkCmdDrawIndexed(cmd, cubeIndexCount, range.count, 0, 0, range.first);
    }
}

void ChunkArena::DrawForCamera(VkCommandBuffer cmd, VkPipeline pipeline, VkBuffer cubeVertices, VkBuffer cubeIndices,
                               uint32_t cubeIndexCount) const {
    Draw(cmd, pipeline, cubeVertices, cubeIndices, cubeIndexCount, cameraRanges);
}

void ChunkArena::DrawForSun(VkCommandBuffer cmd, VkPipeline pipeline, VkBuffer cubeVertices, VkBuffer cubeIndices,
                            uint32_t cubeIndexCount) const {
    Draw(cmd, pipeline, cubeVertices, cubeIndices, cubeIndexCount, sunRanges);
}

ChunkArenaStats ChunkArena::Stats() const {
    ChunkArenaStats result = stats;
    result.chunksPending = static_cast<uint32_t>(pending.size());
    result.chunksResident = liveChunks - result.chunksPending;
    result.pageCount = static_cast<uint32_t>(pages.size());
    for (const Page& page : pages) {
        result.instancesUsed += page.allocator.UsedTotal();
        result.instancesCapacity += page.allocator.Capacity();
    }
    return result;
}
