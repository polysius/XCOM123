#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <glm/glm.hpp>

#include "streaming/ChunkCoord.h"
#include "streaming/ChunkPayload.h"

// World streaming. The world is a grid of square chunks. Around a moving player the manager generates chunks on its own worker
// threads, hands finished chunks to the main thread under a per frame budget, and lets them go again when the player has left.
// It knows nothing about GPUs or meshes: a chunk is a ChunkPayload made by a generator function the caller supplies.
//
// Positions are on the ground plane: glm::vec2 x is world X and y is world Z.
//
// Threading rules:
//   Update, WaitUntilIdle, ResidentCoords, IsResident and Stats belong to one thread (the main thread).
//   The generator runs on the worker threads, several at a time, so it must be thread safe.
//   The clock is only called from Update, on the main thread.
namespace streaming {

// Called on worker threads. Returning null or throwing marks the chunk as failed: it is counted and not retried until the
// player has left its neighbourhood and come back.
using GeneratorFunction = std::function<std::shared_ptr<const ChunkPayload>(ChunkCoord)>;

// Seconds from a monotonic clock. Injectable so that tests can control time.
using ClockFunction = std::function<double()>;

double SteadyClockSeconds();

struct StreamingSettings {
    float chunkSize = 256.0f;
    // Chunks whose nearest point is within this distance of the player are wanted.
    float loadRadius = 700.0f;
    // A resident chunk is kept until its nearest point is farther than this. It is never smaller than loadRadius, which gives
    // a band in which chunks are neither loaded nor unloaded, so a player standing at a border does not make chunks flicker.
    float unloadRadius = 800.0f;
    // Prefetch: the point where the player will be in lookAheadSeconds also gets chunks, within prefetchRadius.
    float lookAheadSeconds = 3.0f;
    float prefetchRadius = 450.0f;
    // A jump of the player between two Updates larger than this is a teleport.
    float teleportDistance = 1200.0f;
    // Hard limits on what is resident (delivered and not yet unloaded).
    size_t maxResidentChunks = 96;
    size_t memoryBudgetBytes = 512ull << 20;
    // Delivery budget of one Update. At least one ready chunk is delivered per Update even if it alone exceeds the byte
    // budget, so that a big chunk cannot starve.
    size_t uploadBytesPerFrame = 4u << 20;
    uint32_t uploadChunksPerFrame = 2;
    // When above zero, delivery also stops once the clock says this much time passed inside Update.
    double uploadSecondsPerFrame = 0.0;
    // Worker threads. Zero selects synchronous mode: no threads, and Update generates up to uploadChunksPerFrame queued
    // chunks itself in priority order, then delivers them. The resident set is then a pure function of the path of the player.
    // Threaded mode reaches the same resident set once the player stands still, but not the same timing frame by frame.
    unsigned workerThreads = 2;
};

struct ResidentChunk {
    ChunkCoord coord;
    std::shared_ptr<const ChunkPayload> payload;
};

// What one Update tells the caller.
struct StreamingFrame {
    // Chunks that became resident this frame, nearest first. The caller keeps its own copy of the payload for as long as the
    // GPU may use it; the manager drops its copy when the chunk is unloaded.
    std::vector<ResidentChunk> delivered;
    // Chunks that stopped being resident this frame. A chunk is never in both lists of one frame.
    std::vector<ChunkCoord> unloaded;
};

// Counters count since construction; the others are gauges of the moment.
struct StreamingStats {
    uint64_t requested = 0;
    uint64_t generated = 0;
    uint64_t failed = 0;      // generator returned null or threw
    uint64_t delivered = 0;
    uint64_t unloaded = 0;
    uint64_t cancelled = 0;   // requests dropped from the queue and generated results thrown away because nobody wants them
    uint64_t teleports = 0;

    size_t residentChunks = 0;
    size_t residentChunksPeak = 0;
    size_t residentBytes = 0;
    size_t residentBytesPeak = 0;
    size_t queuedChunks = 0;
    size_t inFlightChunks = 0;
    size_t readyChunks = 0;           // generated, not yet delivered
    size_t pendingDestroyChunks = 0;  // unloaded, payload not yet destroyed
    size_t deliveredBytesLastFrame = 0;
    size_t deliveredBytesMaxFrame = 0;

    // Seconds from the first request of a chunk to its delivery, by the injected clock, one sample per delivered chunk.
    std::vector<double> latencies;

    // The sample at rank fraction * (count - 1) after sorting, fraction in [0, 1]. Zero when there are no samples.
    double LatencyPercentile(double fraction) const;
};

class StreamingManager {
public:
    StreamingManager(StreamingSettings settings, GeneratorFunction generator, ClockFunction clock = SteadyClockSeconds);
    // Stops and joins the workers. Chunks still queued are dropped.
    ~StreamingManager();

    StreamingManager(const StreamingManager&) = delete;
    StreamingManager& operator=(const StreamingManager&) = delete;

    // Call once per frame with the player position and velocity (metres per second). Plans which chunks are wanted, delivers
    // finished chunks within the budget and unloads chunks the player has left.
    StreamingFrame Update(glm::vec2 position, glm::vec2 velocity);

    // Blocks until no chunk is queued or being generated. Results may still wait to be delivered by the next Update.
    // Returns at once in synchronous mode.
    void WaitUntilIdle();

    std::vector<ChunkCoord> ResidentCoords() const;  // sorted
    bool IsResident(ChunkCoord coord) const;
    StreamingStats Stats() const;
    const StreamingSettings& Settings() const { return settings_; }

private:
    // Where the player is and where the next seconds will take them. Decides what is wanted and what is kept.
    struct Interest {
        glm::vec2 position{0.0f};
        glm::vec2 velocity{0.0f};
        glm::vec2 predicted{0.0f};
    };
    struct QueuedRequest {
        double priority = 0.0;     // smaller is more urgent
        double requestTime = 0.0;  // clock reading when the chunk was first queued
    };
    struct ReadyChunk {
        std::shared_ptr<const ChunkPayload> payload;
        double requestTime = 0.0;
    };
    struct ResidentEntry {
        std::shared_ptr<const ChunkPayload> payload;
        size_t bytes = 0;
    };

    bool IsWanted(const Interest& interest, ChunkCoord coord) const;
    bool IsKept(const Interest& interest, ChunkCoord coord) const;
    double Priority(const Interest& interest, ChunkCoord coord) const;
    std::vector<ChunkCoord> WantedCoords(const Interest& interest) const;

    void StartWorkers();
    void WorkerLoop();
    // Takes the best queued chunk, generates it with the lock released and stores the result. Needs the lock held on entry.
    void GenerateNext(std::unique_lock<std::mutex>& lock);
    std::shared_ptr<const ChunkPayload> RunGenerator(ChunkCoord coord) const;
    void DestroyGraveyard(std::unique_lock<std::mutex>& lock);

    // The functions below run on the main thread with the lock held.
    void DropTeleportedWork();
    void ReplanQueue(const Interest& interest, double now);
    bool AlreadyHandled(ChunkCoord coord) const;
    void DiscardUnwantedResults(const Interest& interest);
    void UnloadLeftBehind(const Interest& interest, StreamingFrame& frame);
    void DeliverReady(const Interest& interest, double now, StreamingFrame& frame);
    bool MakeRoom(const Interest& interest, ChunkCoord incoming, size_t incomingBytes, StreamingFrame& frame);
    void Unload(std::map<ChunkCoord, ResidentEntry>::iterator entry, StreamingFrame& frame);
    void Bury(std::shared_ptr<const ChunkPayload> payload);
    void RecordDelivery(ChunkCoord coord, ReadyChunk ready, double now, StreamingFrame& frame);

    const StreamingSettings settings_;
    const GeneratorFunction generator_;
    const ClockFunction clock_;

    // One mutex guards everything below; the generator is the only long work and it runs without it.
    mutable std::mutex mutex_;
    std::condition_variable workAvailable_;
    std::condition_variable idle_;
    bool stopping_ = false;

    std::unordered_map<ChunkCoord, QueuedRequest, ChunkCoordHash> queue_;
    std::unordered_map<ChunkCoord, double, ChunkCoordHash> inFlight_;  // coord -> request time
    std::unordered_map<ChunkCoord, ReadyChunk, ChunkCoordHash> ready_;
    std::unordered_set<ChunkCoord, ChunkCoordHash> failed_;
    std::map<ChunkCoord, ResidentEntry> resident_;
    // Unloaded payloads wait here to be destroyed away from the frame that unloaded them.
    std::vector<std::shared_ptr<const ChunkPayload>> graveyard_;
    size_t destroyingChunks_ = 0;  // payloads a thread has taken from the graveyard and is destroying right now
    size_t residentBytes_ = 0;
    StreamingStats stats_;

    bool hasPreviousPosition_ = false;
    glm::vec2 previousPosition_{0.0f};

    std::vector<std::thread> workers_;
};

}  // namespace streaming
