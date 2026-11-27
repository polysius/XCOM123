#pragma once

// Builds the content of one chunk of the streamed world from the world seed and the chunk's address alone, with the same
// block, lot, building, park and landmark builders as the fixed city, laid out on the position-pure skeleton of
// WorldSkeleton.h.
//
// Differences from GenerateCity, on purpose:
//   * The street plan is the lattice of arterials with local streets inside each cell, not a global plan with a ring
//     road, boulevards and satellite districts, because those need the whole city to be planned in one go.
//   * Districts are a Voronoi diagram of hashed sites, not ranked by distance from a historic core.
//   * Rivers are bands of cells that run along X for ever and meander inside their band. Water, banks and bridges are
//     built per cell from the river curve, so they continue across chunk borders exactly.
//   * Landmarks are limited to the clock tower and the parish church, which fit a cell.
//   * Ground is flat at height zero, so there are no terraces.
// Everything a chunk builds belongs to the cells it owns. A cell may reach up to the arterial jitter (12 m) past the
// nominal chunk square, which ChunkData::bounds accounts for.

#include <cstddef>
#include <cstdint>
#include <memory>

#include <glm/glm.hpp>

#include "scene/Aabb.h"
#include "scene/SceneTypes.h"
#include "streaming/ChunkCoord.h"
#include "streaming/ChunkPayload.h"
#include "world/CityGenerator.h"
#include "world/Crowd.h"
#include "world/stream/WorldSkeleton.h"

class JobSystem;

namespace world {

struct ChunkSettings {
    uint32_t seed = 1789;
    // People spawned in the chunk and kept in a crowd of its own. Zero builds no crowd and no walkways.
    uint32_t agentCount = 0;
    // The job system the chunk's crowd runs its own parallel loops on. A streamed world has dozens of small crowds, and
    // waking the shared pool for each loop of each crowd costs more than the loops do, so it gives them a pool with no
    // workers (everything runs on the caller) and spreads the crowds themselves over the shared pool. Null means the
    // shared pool.
    JobSystem* crowdJobs = nullptr;
};

// The ground square a chunk nominally covers.
Rect ChunkRect(streaming::ChunkCoord coord);
// The chunk that holds a ground position.
streaming::ChunkCoord ChunkAt(glm::vec2 position);

class ChunkData final : public streaming::ChunkPayload {
public:
    streaming::ChunkCoord coord;
    // The boxes, the streets, squares, trees and lights of the chunk's cells. boxes and boxParts are in step.
    CityLayout layout;
    // Everything drawn, as a box in the world.
    Aabb bounds;
    // The people of the chunk walking its streets. Null when ChunkSettings::agentCount is zero. The pointer is
    // shared and the crowd is updated by whoever holds the chunk, from one thread at a time.
    std::shared_ptr<Crowd> crowd;

    size_t ResidentBytes() const override;
    size_t UploadBytes() const override { return layout.boxes.size() * sizeof(BoxInstance); }
};

// The number of people each chunk gets so that about `peopleTotal` people stand within `loadRadius` of the camera: the density
// of a disc of that radius, times the area of a chunk. Sizing by density, not by how many chunks are loaded, keeps the number
// of people near the camera steady however many chunks the prefetch and the hysteresis keep around. Zero people give zero.
uint32_t PeoplePerChunk(uint32_t peopleTotal, float loadRadius);

std::shared_ptr<ChunkData> GenerateChunk(const ChunkSettings& settings, streaming::ChunkCoord coord);

}  // namespace world
