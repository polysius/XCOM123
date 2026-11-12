#include "world/crowd/CrowdVisibility.h"

#include <cmath>

#include "core/JobSystem.h"
#include "scene/Frustum.h"
#include "world/crowd/CrowdMath.h"
#include "world/crowd/CrowdState.h"

namespace crowd {

namespace {

constexpr size_t kChunkSize = 4096;
constexpr uint8_t kInvisible = 0xFF;

// A person fits in a sphere of this radius around a point this high above the feet. Sized
// for the tallest person so culling does not need to read everyone's height.
constexpr float kBoundsCenterHeight = 0.9f * kMaxHeightScale;
constexpr float kBoundsRadius = 1.0f * kMaxHeightScale;

// Gesture weights are stored in steps of 1/64, enough for a smooth fade.
constexpr float kGestureWeightSteps = 64.0f;
// Keeps a packed fraction below 1 so it never carries into the integer part.
constexpr float kBelowOne = 0.999f;

uint8_t BandOf(float distanceSquared) {
    uint8_t band = 0;
    while (band < kDistanceBandCount - 1 && distanceSquared >= kDistanceBandLimits[band] * kDistanceBandLimits[band]) {
        ++band;
    }
    return band;
}

}  // namespace

NpcInstance MakeNpcInstance(const AgentMotion& motion, const AgentLook& look, float gaitLength, float lagSeconds,
                            const Walkways& walkways) {
    // A person who is behind in time is drawn a little ahead along its velocity. That point is not
    // always clear: walking towards a pew or a stall it can lie inside it. Such a person is drawn
    // where the simulation put it, which is never inside anything.
    glm::vec2 position = motion.position + motion.velocity * lagSeconds;
    float extraDistance = glm::length(motion.velocity) * lagSeconds;
    if (lagSeconds > 0.0f && walkways.IsInsideObstacle(position)) {
        position = motion.position;
        extraDistance = 0.0f;
    }

    NpcInstance instance;
    // The height is that of the ground where the person is drawn, which is a little past the stored
    // position: next to the edge of a step the two can stand a step apart.
    instance.position = glm::vec3(position.x, walkways.WalkHeightAt(position), position.y);
    instance.heading = look.heading;
    instance.clothingColor = look.clothingColor;
    instance.heightScale = look.heightScale;
    instance.walkPhase = look.walkPhase + extraDistance / gaitLength * kTwoPi;
    instance.stride = look.stride;
    // unused0: gesture kind in the integer part, the person's variation in the fraction.
    instance.unused0 = static_cast<float>(look.gestureKind) + look.variation * kBelowOne;
    // unused1: gesture weight in 64ths in the integer part, the gesture clock in the fraction.
    instance.unused1 = std::floor(look.gestureWeight * kGestureWeightSteps + 0.5f) + look.gesturePhase * kBelowOne;
    return instance;
}

void CrowdVisibility::Collect(JobSystem& jobs, const CrowdState& state, const Walkways& walkways,
                              const Frustum& frustum, glm::vec3 viewPoint, std::vector<NpcInstance>& visible) {
    agentCount = state.AgentCount();
    agentBand.resize(agentCount);
    const size_t chunkCount = (agentCount + kChunkSize - 1) / kChunkSize;
    chunkBands.assign(chunkCount, BandCounts{});

    // Two passes: first find who is visible and in which band, then, once every chunk knows
    // where its people go, write them out in band order.
    jobs.ParallelFor(chunkCount, 1, [&](size_t begin, size_t end) {
        for (size_t chunk = begin; chunk < end; ++chunk) {
            ClassifyChunk(chunk, state, walkways, frustum, viewPoint);
        }
    });
    ComputeChunkOffsets();

    uint32_t total = 0;
    for (uint32_t count : bandCounts) {
        total += count;
    }
    visible.resize(total);
    jobs.ParallelFor(chunkCount, 1, [&](size_t begin, size_t end) {
        for (size_t chunk = begin; chunk < end; ++chunk) {
            WriteChunk(chunk, state, walkways, visible);
        }
    });
}

void CrowdVisibility::ClassifyChunk(size_t chunk, const CrowdState& state, const Walkways& walkways,
                                    const Frustum& frustum, glm::vec3 viewPoint) {
    BandCounts& counts = chunkBands[chunk];
    const size_t end = std::min(agentCount, (chunk + 1) * kChunkSize);
    for (size_t agent = chunk * kChunkSize; agent < end; ++agent) {
        const glm::vec2 position = state.motion[agent].position;
        const glm::vec3 feet(position.x, walkways.WalkHeightAt(position), position.y);
        const glm::vec3 center = feet + glm::vec3(0.0f, kBoundsCenterHeight, 0.0f);
        // Far people are drawn a little ahead of their stored position (about 10 cm at most
        // at 60 steps per second); the slack in the bounds covers that.
        if (!frustum.ContainsSphere(center, kBoundsRadius)) {
            agentBand[agent] = kInvisible;
            continue;
        }
        const glm::vec3 offset = feet - viewPoint;
        const uint8_t band = BandOf(glm::dot(offset, offset));
        agentBand[agent] = band;
        ++counts[band];
    }
}

// Turns the per-chunk counts into output positions: band by band, and within a band chunk
// by chunk, so the list is in band order and the same every run.
void CrowdVisibility::ComputeChunkOffsets() {
    bandCounts = {};
    for (const BandCounts& counts : chunkBands) {
        for (size_t band = 0; band < kDistanceBandCount; ++band) {
            bandCounts[band] += counts[band];
        }
    }
    BandCounts nextSlot{};
    uint32_t bandStart = 0;
    for (size_t band = 0; band < kDistanceBandCount; ++band) {
        nextSlot[band] = bandStart;
        bandStart += bandCounts[band];
    }
    for (BandCounts& counts : chunkBands) {
        for (size_t band = 0; band < kDistanceBandCount; ++band) {
            const uint32_t count = counts[band];
            counts[band] = nextSlot[band];
            nextSlot[band] += count;
        }
    }
}

void CrowdVisibility::WriteChunk(size_t chunk, const CrowdState& state, const Walkways& walkways,
                                 std::vector<NpcInstance>& visible) const {
    BandCounts slot = chunkBands[chunk];
    const size_t end = std::min(agentCount, (chunk + 1) * kChunkSize);
    for (size_t agent = chunk * kChunkSize; agent < end; ++agent) {
        const uint8_t band = agentBand[agent];
        if (band != kInvisible) {
            visible[slot[band]++] = MakeNpcInstance(state.motion[agent], state.looks[agent],
                                                    state.behaviour[agent].gaitLength, state.lagSeconds[agent], walkways);
        }
    }
}

}  // namespace crowd
