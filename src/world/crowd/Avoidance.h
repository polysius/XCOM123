#pragma once

#include <cstdint>

#include <glm/glm.hpp>

// Local avoidance: how a person reacts to the people right around it.

namespace crowd {

class SpatialHash;

struct AvoidanceResult {
    // A change to the walking velocity, in m/s, that keeps personal space and steers around
    // people who are about to walk into this one.
    glm::vec2 steering{0.0f};
    // A small direct move, in metres, that separates bodies which already overlap.
    glm::vec2 push{0.0f};
};

// The person avoiding others, and how much effort to spend.
struct AvoidanceQuery {
    uint32_t self = 0;
    // The members of self's unit (a walking group or a conversation circle), indices
    // [firstCompanion, firstCompanion + companionCount). Companions keep closer to each other
    // than strangers do: between them only a smaller personal space counts, and they do not
    // sidestep each other.
    uint32_t firstCompanion = 0;
    uint32_t companionCount = 1;
    glm::vec2 position{0.0f};
    glm::vec2 velocity{0.0f};
    // The unit direction the person wants to walk in, or zero when standing.
    glm::vec2 forward{0.0f};
    // Pushing overlapping bodies apart is only worth its cost near the viewer.
    bool resolveOverlap = false;
};

// Looks at the neighbours of a person in `hash` and works out how to keep clear of them.
//
// Three rules, from cheap to clever:
//  - Separation: people inside each other's personal space drift apart.
//  - Anticipation: people predict where each neighbour will be in the next few seconds and
//    sidestep anyone they would pass too close to. On a head-on course both sides also step
//    to their own right, so oncoming flows pass each other side by side.
//  - Overlap: bodies that already touch are pushed apart.
AvoidanceResult ComputeAvoidance(const SpatialHash& hash, const AvoidanceQuery& query, float deltaSeconds);

}  // namespace crowd
