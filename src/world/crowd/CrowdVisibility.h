#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "scene/SceneTypes.h"

class Frustum;
class JobSystem;

namespace crowd {

struct AgentLook;
struct AgentMotion;
struct CrowdState;

// The visible list is ordered by distance from the viewer in these bands, nearest first, so
// the renderer can draw each band with its own level of detail (and near people first, which
// also helps the depth test). A band holds people closer than its limit; the last band holds
// everyone further away.
constexpr size_t kDistanceBandCount = 4;
constexpr float kDistanceBandLimits[kDistanceBandCount - 1] = {30.0f, 80.0f, 200.0f};

// Builds the per-frame list of people to draw: frustum culling, distance banding and
// packing into NpcInstance, all in parallel.
class CrowdVisibility {
public:
    using BandCounts = std::array<uint32_t, kDistanceBandCount>;

    // Replaces `visible` with everyone inside `frustum`, grouped by band around `viewPoint`.
    void Collect(JobSystem& jobs, const CrowdState& state, float walkHeight, const Frustum& frustum,
                 glm::vec3 viewPoint, std::vector<NpcInstance>& visible);

    // How many people the last Collect put in each band.
    const BandCounts& LastBandCounts() const { return bandCounts; }

private:
    void ClassifyChunk(size_t chunk, const CrowdState& state, float walkHeight, const Frustum& frustum,
                       glm::vec3 viewPoint);
    void ComputeChunkOffsets();
    void WriteChunk(size_t chunk, const CrowdState& state, float walkHeight, std::vector<NpcInstance>& visible) const;

    // Per person, its band, or kInvisible when it is outside the frustum.
    std::vector<uint8_t> agentBand;
    // Per chunk of people, first how many it has in each band, then where in the output its
    // people of each band go.
    std::vector<BandCounts> chunkBands;
    BandCounts bandCounts{};
    size_t agentCount = 0;
};

// Packs one person for the renderer. A person whose last update was `lagSeconds` ago is
// moved on along its velocity, so people updated every few steps still move smoothly.
// See shaders/npc.vert for how unused0 and unused1 are read.
NpcInstance MakeNpcInstance(const AgentMotion& motion, const AgentLook& look, float gaitLength, float lagSeconds,
                            float walkHeight);

}  // namespace crowd
