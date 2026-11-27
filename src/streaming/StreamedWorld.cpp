#include "streaming/StreamedWorld.h"

#include <algorithm>
#include <cmath>

namespace streaming {
namespace {

// A chunk generated this long ago and still not here is a problem; LoadAround gives up after this many Update calls.
constexpr int kLoadAroundMaxRounds = 4000;
// How far past the load radius a chunk is kept, so that a camera hovering at the edge does not make chunks flicker.
constexpr float kUnloadMargin = 128.0f;
// Chunks the frame hands to the renderer per Update. The manager enforces it; it is also what keeps a frame that
// delivers chunks within a fraction of a millisecond.
constexpr uint32_t kChunksPerFrame = 2;
constexpr size_t kUploadBytesPerFrame = 2u << 20;

// Crowds are not all simulated every frame. A chunk's crowd is updated every frame when the chunk is near the camera, every
// second frame at middle distance and every fourth frame beyond, with the time it missed, and at most this many crowds are
// updated in one frame whatever their distance (the ones that have waited longest for their interval go first). The work of a
// frame then stays bounded however many chunks are loaded.
constexpr float kNearCrowdDistance = 350.0f;
constexpr float kMidCrowdDistance = 600.0f;
constexpr size_t kMaxCrowdUpdatesPerFrame = 24;

// The nearest distance from `point` to the square of a chunk at (x, z) in chunk units.
float DistanceToChunk(glm::vec2 point, int x, int z) {
    const glm::vec2 low(static_cast<float>(x) * world::kChunkSize, static_cast<float>(z) * world::kChunkSize);
    const glm::vec2 nearest = glm::clamp(point, low, low + glm::vec2(world::kChunkSize));
    return glm::distance(point, nearest);
}

}  // namespace

uint32_t StreamedWorld::ChunksInRadius(float radius) {
    // Count from the middle of a chunk, which is where an average camera stands.
    const glm::vec2 centre(world::kChunkSize * 0.5f);
    const int reach = static_cast<int>(std::ceil(radius / world::kChunkSize)) + 1;
    uint32_t count = 0;
    for (int z = -reach; z <= reach; ++z) {
        for (int x = -reach; x <= reach; ++x) {
            count += DistanceToChunk(centre, x, z) <= radius ? 1u : 0u;
        }
    }
    return count;
}

StreamingSettings StreamedWorld::MakeManagerSettings(const StreamedWorldSettings& settings) {
    StreamingSettings manager;
    manager.chunkSize = world::kChunkSize;
    manager.loadRadius = settings.loadRadius;
    manager.unloadRadius = settings.loadRadius + kUnloadMargin;
    manager.lookAheadSeconds = 3.0f;
    manager.prefetchRadius = settings.loadRadius * 0.55f;
    manager.teleportDistance = settings.loadRadius * 2.0f;
    // Room for the chunks of the load radius, the prefetch area ahead and the hysteresis ring.
    manager.maxResidentChunks = ChunksInRadius(settings.loadRadius + kUnloadMargin) * 2;
    manager.memoryBudgetBytes = 2048ull << 20;
    manager.uploadBytesPerFrame = kUploadBytesPerFrame;
    manager.uploadChunksPerFrame = kChunksPerFrame;
    manager.workerThreads = settings.workerThreads;
    return manager;
}

StreamedWorld::StreamedWorld(IRenderModule& renderer, const StreamedWorldSettings& settings)
    : renderer(renderer),
      peoplePerChunk(world::PeoplePerChunk(settings.peopleTotal, settings.loadRadius)),
      manager(MakeManagerSettings(settings),
              [seed = settings.seed, people = peoplePerChunk,
               jobs = &crowdJobs](ChunkCoord coord) {
                  world::ChunkSettings chunk;
                  chunk.seed = seed;
                  chunk.agentCount = people;
                  chunk.crowdJobs = jobs;
                  return std::shared_ptr<const ChunkPayload>(world::GenerateChunk(chunk, coord));
              }) {}

StreamedWorld::~StreamedWorld() {
    for (auto& [coord, chunk] : loaded) {
        renderer.RemoveStreamedChunk(chunk.handle);
    }
}

void StreamedWorld::Receive(const StreamingFrame& frame) {
    deliveredByLastUpdate = frame.delivered.size();
    unloadedByLastUpdate = frame.unloaded.size();
    for (const ResidentChunk& delivered : frame.delivered) {
        auto data = std::static_pointer_cast<const world::ChunkData>(delivered.payload);
        // The renderer keeps the chunk alive until its boxes are copied to the GPU, so no copy is made here.
        const StreamedChunkHandle handle = renderer.AddStreamedChunk(data, data->layout.boxes, data->bounds);
        loaded[delivered.coord] = {std::move(data), handle};
    }
    for (const ChunkCoord& coord : frame.unloaded) {
        const auto found = loaded.find(coord);
        if (found != loaded.end()) {
            renderer.RemoveStreamedChunk(found->second.handle);
            reaper.Retire(std::move(found->second.data));  // freed on the reaper's thread, not here
            loaded.erase(found);
        }
    }
    if (!frame.delivered.empty() || !frame.unloaded.empty()) {
        crowded.clear();
        for (auto& [coord, chunk] : loaded) {
            if (chunk.data->crowd) {
                crowded.push_back(&chunk);
            }
        }
    }
}

void StreamedWorld::Update(glm::vec3 cameraPosition, glm::vec3 velocity) {
    Receive(manager.Update({cameraPosition.x, cameraPosition.z}, {velocity.x, velocity.z}));
}

void StreamedWorld::LoadAround(glm::vec3 position) {
    for (int round = 0; round < kLoadAroundMaxRounds; ++round) {
        manager.WaitUntilIdle();
        Update(position, glm::vec3(0.0f));
        const StreamingStats stats = manager.Stats();
        if (stats.queuedChunks == 0 && stats.inFlightChunks == 0 && stats.readyChunks == 0) {
            return;
        }
    }
}

namespace {

// How many crowd frames a chunk's crowd may go without an update, by the distance of the chunk from the camera.
uint64_t CrowdInterval(float distance) {
    return distance < kNearCrowdDistance ? 1u : (distance < kMidCrowdDistance ? 2u : 4u);
}

}  // namespace

void StreamedWorld::UpdateCrowd(float deltaSeconds, glm::vec3 focus) {
    ++crowdFrame;
    const glm::vec2 focusOnGround(focus.x, focus.z);
    // The crowds whose interval has run out, and how overdue each is.
    std::vector<std::pair<float, LoadedChunk*>> overdue;
    for (LoadedChunk* chunk : crowded) {
        chunk->pendingSeconds += deltaSeconds;
        const float distance = glm::distance(focusOnGround, world::ChunkRect(chunk->data->coord).Center());
        const uint64_t interval = CrowdInterval(distance);
        const uint64_t waited = crowdFrame - chunk->lastCrowdFrame;
        if (waited >= interval) {
            // Overdue first, and among equals the nearer.
            overdue.emplace_back(static_cast<float>(waited) / static_cast<float>(interval) - distance * 1e-6f, chunk);
        }
    }
    const size_t count = std::min(overdue.size(), kMaxCrowdUpdatesPerFrame);
    std::partial_sort(overdue.begin(), overdue.begin() + static_cast<std::ptrdiff_t>(count), overdue.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });
    dueCrowds.clear();
    for (size_t i = 0; i < count; ++i) {
        dueCrowds.push_back(overdue[i].second);
    }
    // One crowd per task: each crowd runs its own loops on the thread that has it (see ChunkSettings::crowdJobs), and the
    // crowds are spread over the shared pool.
    JobSystem::Shared().ParallelFor(dueCrowds.size(), 1, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            LoadedChunk& chunk = *dueCrowds[i];
            chunk.data->crowd->SetFocusPoint(focus);
            chunk.data->crowd->Update(chunk.pendingSeconds);
            chunk.pendingSeconds = 0.0f;
            chunk.lastCrowdFrame = crowdFrame;
        }
    });
}

void StreamedWorld::CollectVisible(const Frustum& frustum, std::vector<NpcInstance>& visible) const {
    visibleByChunk.resize(crowded.size());
    JobSystem::Shared().ParallelFor(crowded.size(), 1, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            const world::ChunkData& data = *crowded[i]->data;
            visibleByChunk[i].clear();
            if (frustum.ContainsBox(data.bounds.min, data.bounds.max)) {
                data.crowd->CollectVisible(frustum, visibleByChunk[i]);
            }
        }
    });
    visible.clear();
    for (const std::vector<NpcInstance>& fromChunk : visibleByChunk) {
        visible.insert(visible.end(), fromChunk.begin(), fromChunk.end());
    }
}

size_t StreamedWorld::PeopleWithin(glm::vec2 centre, float radius) const {
    size_t people = 0;
    for (const auto& [coord, chunk] : loaded) {
        if (!chunk.data->crowd) {
            continue;
        }
        for (const crowd::AgentMotion& motion : chunk.data->crowd->State().motion) {
            people += glm::distance(motion.position, centre) <= radius ? 1u : 0u;
        }
    }
    return people;
}

size_t StreamedWorld::MissingNear(glm::vec3 position, float radius) const {
    const glm::vec2 point(position.x, position.z);
    const int reach = static_cast<int>(std::ceil(radius / world::kChunkSize)) + 1;
    const ChunkCoord here = world::ChunkAt(point);
    size_t missing = 0;
    for (int z = here.z - reach; z <= here.z + reach; ++z) {
        for (int x = here.x - reach; x <= here.x + reach; ++x) {
            if (DistanceToChunk(point, x, z) <= radius && loaded.count({x, z}) == 0) {
                ++missing;
            }
        }
    }
    return missing;
}

size_t StreamedWorld::PeopleCount() const {
    size_t people = 0;
    for (const auto& [coord, chunk] : loaded) {
        people += chunk.data->crowd ? chunk.data->crowd->AgentCount() : 0u;
    }
    return people;
}

}  // namespace streaming
