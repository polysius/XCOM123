#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "core/JobSystem.h"
#include "render/IRenderModule.h"
#include "scene/Frustum.h"
#include "streaming/ChunkCoord.h"
#include "streaming/StreamingManager.h"
#include "world/stream/ChunkGenerator.h"

namespace streaming {

struct StreamedWorldSettings {
    uint32_t seed = 1789;
    // People over all the chunks kept loaded. Each chunk gets its share of them when it is generated, so the number of
    // people in the world stays about this as the player moves.
    uint32_t peopleTotal = 10000;
    // Metres around the camera that are kept loaded.
    float loadRadius = 760.0f;
    // Threads that generate chunks. The frame loop and the job system keep their own, so generation never queues behind them.
    unsigned workerThreads = 3;
};

// The streamed world as the frame loop sees it: it keeps the chunks around the camera resident, tells the renderer to add
// and remove each chunk's boxes, and keeps the people of those chunks walking.
//
// Everything that belongs to a chunk (its boxes and its crowd) is made on a worker thread by world::GenerateChunk and
// arrives whole, so the frame loop only hands a finished chunk to the renderer and starts updating its people. When a chunk
// leaves, the renderer is told, and the manager's worker normally frees the chunk and its crowd. The last reference may just
// as well be the frame's, in which case the frame frees it, which costs well under a millisecond for a chunk.
//
// People stay in the chunk they were made in: each chunk has street graph and walkways of its own, and people turn round at
// its edge.
//
// Not streamed yet: physics props (only static crates and barrels are), the simulated river (the water is still, with
// bridges) and the grid of showcase characters stay with the fixed city.
class StreamedWorld {
public:
    StreamedWorld(IRenderModule& renderer, const StreamedWorldSettings& settings);
    ~StreamedWorld();

    StreamedWorld(const StreamedWorld&) = delete;
    StreamedWorld& operator=(const StreamedWorld&) = delete;

    // Once per frame, before drawing: lets the manager plan, deliver and unload, and passes the changes to the renderer.
    // `velocity` is how the camera is moving, in metres per second, so that chunks ahead of it are made first.
    void Update(glm::vec3 cameraPosition, glm::vec3 velocity);

    // Moves the people of every loaded chunk forward. The simulation puts most detail near `focus`.
    void UpdateCrowd(float deltaSeconds, glm::vec3 focus);
    // Replaces the contents of `visible` with the people inside the frustum, from all loaded chunks.
    void CollectVisible(const Frustum& frustum, std::vector<NpcInstance>& visible) const;
    size_t PeopleCount() const;

    // Blocks until everything within the load radius of `position` is loaded. For screenshots and tests, never for a frame.
    void LoadAround(glm::vec3 position);

    size_t LoadedChunkCount() const { return loaded.size(); }
    // What the last Update did, for finding the frames in which chunks came and went.
    size_t DeliveredByLastUpdate() const { return deliveredByLastUpdate; }
    size_t UnloadedByLastUpdate() const { return unloadedByLastUpdate; }
    StreamingStats Stats() const { return manager.Stats(); }
    const StreamingSettings& ManagerSettings() const { return manager.Settings(); }
    uint32_t PeoplePerChunk() const { return peoplePerChunk; }

private:
    struct LoadedChunk {
        std::shared_ptr<const world::ChunkData> data;
        StreamedChunkHandle handle;
    };

    static StreamingSettings MakeManagerSettings(const StreamedWorldSettings& settings);
    static uint32_t ChunksInRadius(float radius);
    void Receive(const StreamingFrame& frame);

    IRenderModule& renderer;
    // Declared before everything that holds a crowd, so that it is destroyed after all of them.
    JobSystem crowdJobs{0u};
    uint32_t peoplePerChunk = 0;
    size_t deliveredByLastUpdate = 0;
    size_t unloadedByLastUpdate = 0;
    StreamingManager manager;
    std::map<ChunkCoord, LoadedChunk> loaded;  // ordered, so the crowds update and are collected in one fixed order
    std::vector<LoadedChunk*> crowded;         // the loaded chunks that have a crowd, rebuilt whenever chunks come or go
    mutable std::vector<std::vector<NpcInstance>> visibleByChunk;  // scratch space of CollectVisible, one list per crowd
};

}  // namespace streaming
