#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include <volk.h>

#include "render/ChunkCulling.h"
#include "render/RangeAllocator.h"
#include "render/RetirementQueue.h"
#include "render/VulkanUtils.h"
#include "scene/Aabb.h"
#include "scene/Frustum.h"
#include "scene/SceneTypes.h"

class VulkanContext;

// What the arena did, for benchmarks.
struct ChunkArenaStats {
    uint32_t chunksResident = 0;        // chunks that are uploaded and can be drawn
    uint32_t chunksPending = 0;         // chunks waiting for their copy
    uint32_t chunksDrawnByCamera = 0;
    uint32_t chunksDrawnBySun = 0;
    uint32_t drawCallsForCamera = 0;
    uint32_t drawCallsForSun = 0;
    size_t uploadBytesThisFrame = 0;
    size_t uploadBytesMaxFrame = 0;
    size_t instancesUsed = 0;
    size_t instancesCapacity = 0;
    uint32_t pageCount = 0;
};

// The boxes of the streamed world, as the instances of a few large device buffers (pages) that chunks take ranges of.
//
// A chunk is added without waiting for the GPU. Its range is reserved at once, its boxes are copied into the staging buffer
// of the frame slot at the start of the next frame, and a copy command at the start of that frame's command buffer moves them
// into the page, ahead of the passes that draw them. A chunk is removed just as lightly: its range is only given back to the
// allocator after the last frame that may have drawn it has completed (see RetirementQueue), so no buffer is ever destroyed or
// overwritten while the GPU may read it, and nothing needs vkDeviceWaitIdle.
//
// Each frame the visible chunks are chosen against the camera and the sun, and their ranges drawn with one draw per run of
// neighbouring chunks.
class ChunkArena {
public:
    using ChunkId = uint32_t;
    static constexpr ChunkId kInvalidChunk = 0xFFFFFFFFu;

    // Boxes per page: 128k boxes, 8 MB. A chunk must fit in a page, so this is far above the biggest chunk.
    static constexpr uint32_t kInstancesPerPage = 131072;
    static constexpr uint32_t kFirstPages = 3;
    // Bytes the copies of one frame may move. The streaming manager budgets deliveries well below this.
    static constexpr VkDeviceSize kStagingBytes = 12u << 20;

    void Create(const VulkanContext& context, uint32_t frameSlots);
    void Destroy(VkDevice device);

    // Reserves room for `boxes` and queues their upload. `owner` keeps whatever `boxes` points into alive until the boxes are
    // copied, which happens in the next BeginFrame that has room in its staging buffer. Returns kInvalidChunk when the chunk
    // is empty or bigger than a page.
    ChunkId Add(const VulkanContext& context, std::shared_ptr<const void> owner, std::span<const BoxInstance> boxes,
                const Aabb& bounds);
    // The chunk is not drawn from the next frame on; its range is reused once the GPU is finished with it.
    // `frameNumber` is the frame being prepared.
    void Remove(ChunkId chunk, uint64_t frameNumber);

    // Call at the start of a frame, after the fence of its slot: gives back the ranges that no frame in flight can use any more
    // (`completedFrame` is the newest frame known to be complete) and copies the pending boxes into the slot's staging buffer.
    void BeginFrame(uint32_t frameSlot, int64_t completedFrame);
    // Records the copies from staging into the pages, and the barrier that makes them visible to vertex fetch.
    void RecordUploads(VkCommandBuffer cmd, uint32_t frameSlot);

    // Chooses the chunks each pass draws. Chunks that were added but not copied yet are left out.
    void Cull(const Frustum& camera, const Frustum& sun);
    void DrawForCamera(VkCommandBuffer cmd, VkPipeline pipeline, VkBuffer cubeVertices, VkBuffer cubeIndices,
                       uint32_t cubeIndexCount) const;
    void DrawForSun(VkCommandBuffer cmd, VkPipeline pipeline, VkBuffer cubeVertices, VkBuffer cubeIndices,
                    uint32_t cubeIndexCount) const;

    bool IsEmpty() const { return liveChunks == 0; }
    ChunkArenaStats Stats() const;

private:
    struct Page {
        GpuBuffer buffer;
        RangeAllocator allocator{kInstancesPerPage};
    };

    struct Chunk {
        bool live = false;
        bool uploaded = false;
        uint32_t page = 0;
        uint32_t first = 0;
        uint32_t count = 0;
        Aabb bounds;
        std::shared_ptr<const void> owner;      // until the boxes are in staging
        std::span<const BoxInstance> boxes;     // valid while `owner` is held
    };

    struct FreedRange {
        uint32_t page;
        uint32_t first;
        uint32_t count;
    };

    struct StagedCopy {
        VkDeviceSize sourceOffset;
        uint32_t page;
        VkDeviceSize destinationOffset;
        VkDeviceSize bytes;
    };

    struct Slot {
        GpuBuffer staging;
        std::vector<StagedCopy> copies;  // what BeginFrame staged, for RecordUploads
        VkDeviceSize stagedBytes = 0;
    };

    // Finds room for `count` instances, adding a page when no page has it.
    bool ReserveRange(const VulkanContext& context, uint32_t count, uint32_t& page, uint32_t& first);
    void AddPage(const VulkanContext& context);
    void Draw(VkCommandBuffer cmd, VkPipeline pipeline, VkBuffer cubeVertices, VkBuffer cubeIndices, uint32_t cubeIndexCount,
              const std::vector<InstanceRange>& ranges) const;

    VkDevice device = VK_NULL_HANDLE;
    std::vector<Page> pages;
    std::vector<Chunk> chunks;
    std::vector<ChunkId> freeChunkIds;
    std::vector<ChunkId> pending;   // added, not yet staged, in the order they were added
    std::vector<Slot> slots;
    RetirementQueue<FreedRange> retiring;
    std::vector<InstanceRange> cameraRanges;
    std::vector<InstanceRange> sunRanges;
    uint32_t liveChunks = 0;
    ChunkArenaStats stats;
};
