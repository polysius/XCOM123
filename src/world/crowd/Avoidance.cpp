#include "world/crowd/Avoidance.h"

#include <cmath>

#include "world/crowd/CrowdMath.h"
#include "world/crowd/CrowdTypes.h"
#include "world/crowd/SpatialHash.h"

namespace crowd {

namespace {

// Neighbours further away than this are ignored.
constexpr float kNeighbourRadius = 2.5f;
// The neighbour search is centred this far ahead of the person, where most of the people
// that matter are.
constexpr float kLookAhead = 0.75f;

// Separation: centre distance below which people start to drift apart, and how strongly.
constexpr float kPersonalSpace = 0.8f;
constexpr float kSeparationStrength = 2.5f;
// Members of the same group walk side by side on purpose, so between them only this smaller
// space counts. It still keeps them apart when their paths cross, for example on a crosswalk.
constexpr float kCompanionSpace = 0.55f;

// Anticipation: how far ahead people predict, how close a pass counts as too close, and how
// strongly they sidestep.
constexpr float kPredictionSeconds = 2.5f;
constexpr float kComfortableMiss = 0.75f;
constexpr float kAnticipationStrength = 1.4f;
// Courses whose directions differ by more than about 120 degrees count as head-on.
constexpr float kHeadOnCosine = -0.5f;
constexpr float kKeepRightStrength = 0.6f;

// Overlap: centre distance at which bodies touch, the speed below which a neighbour counts
// as standing (and so does not move out of the way itself), and the fastest push.
constexpr float kContactDistance = 2.0f * kBodyRadius;
constexpr float kStandingSpeed = 0.05f;
constexpr float kMaxPushSpeed = 1.0f;

constexpr float kMaxSteering = 1.6f;

// A direction to split two people who stand at exactly the same point. It only has to be
// different for the two of them, so it is taken from the order of their indices.
glm::vec2 SplitDirection(uint32_t self, uint32_t other) {
    const float angle = 0.7f + (self < other ? 0.0f : kPi);
    return {std::cos(angle), std::sin(angle)};
}

// The person's own right, which head-on passes keep to.
glm::vec2 RightOf(glm::vec2 forward) {
    return -LeftOf(forward);
}

// The sidestep that avoids a neighbour moving relative to us, or zero when the predicted
// pass is comfortable.
glm::vec2 AnticipationSteering(glm::vec2 offsetToOther, glm::vec2 relativeVelocity, glm::vec2 forward,
                               glm::vec2 otherVelocity) {
    const float closingSpeedSquared = glm::dot(relativeVelocity, relativeVelocity);
    if (closingSpeedSquared < 1e-4f) {
        return glm::vec2(0.0f);  // moving together, nothing will change
    }
    const float timeOfClosestPass = glm::dot(offsetToOther, relativeVelocity) / closingSpeedSquared;
    if (timeOfClosestPass <= 0.0f || timeOfClosestPass >= kPredictionSeconds) {
        return glm::vec2(0.0f);  // moving apart, or too far in the future to matter
    }

    // Where the neighbour will be, seen from us, when we are closest.
    const glm::vec2 offsetAtPass = offsetToOther - relativeVelocity * timeOfClosestPass;
    const float missDistance = glm::length(offsetAtPass);
    if (missDistance >= kComfortableMiss) {
        return glm::vec2(0.0f);
    }

    // Sooner and closer passes call for a stronger sidestep.
    const float urgency = (1.0f - timeOfClosestPass / kPredictionSeconds) * (1.0f - missDistance / kComfortableMiss);
    glm::vec2 away = SafeNormalize(-offsetAtPass);
    const bool headOn = glm::dot(forward, SafeNormalize(otherVelocity)) < kHeadOnCosine;
    if (headOn || away == glm::vec2(0.0f)) {
        // Both people keep right, so they pick opposite sides and never mirror each other.
        away = SafeNormalize(away + RightOf(forward) * kKeepRightStrength);
    }
    return away * (urgency * kAnticipationStrength);
}

}  // namespace

AvoidanceResult ComputeAvoidance(const SpatialHash& hash, const AvoidanceQuery& query, float deltaSeconds) {
    AvoidanceResult result;
    const glm::vec2 position = query.position;
    hash.ForEachNear(position + query.forward * kLookAhead, kNeighbourRadius, [&](const NeighbourEntry& other) {
        if (other.agent == query.self) {
            return;
        }
        const glm::vec2 offsetToOther = other.position - position;
        const float distanceSquared = glm::dot(offsetToOther, offsetToOther);
        if (distanceSquared > kNeighbourRadius * kNeighbourRadius) {
            return;
        }

        const float distance = std::sqrt(distanceSquared);
        const glm::vec2 awayFromOther =
            distance > 1e-4f ? -offsetToOther / distance : SplitDirection(query.self, other.agent);
        // Unsigned wrap-around makes this one comparison cover both ends of the range.
        const bool isCompanion = other.agent - query.firstCompanion < query.companionCount;

        const float personalSpace = isCompanion ? kCompanionSpace : kPersonalSpace;
        if (distance < personalSpace) {
            const float closeness = (personalSpace - distance) / personalSpace;
            result.steering += awayFromOther * (closeness * kSeparationStrength);
        }
        if (query.resolveOverlap && distance < kContactDistance) {
            // Each of two walkers moves out of the way by half; a standing neighbour does not
            // move, so the walker takes the whole step.
            const bool otherStands = glm::dot(other.velocity, other.velocity) < kStandingSpeed * kStandingSpeed;
            const float share = otherStands ? 1.0f : 0.5f;
            result.push += awayFromOther * ((kContactDistance - distance) * share);
        }
        if (!isCompanion) {
            result.steering += AnticipationSteering(offsetToOther, query.velocity - other.velocity, query.forward,
                                                    other.velocity);
        }
    });

    result.steering = ClampLength(result.steering, kMaxSteering);
    result.push = ClampLength(result.push, kMaxPushSpeed * deltaSeconds);
    return result;
}

}  // namespace crowd
