#pragma once

#include <cstdint>

#include <glm/glm.hpp>

#include "world/CityGenerator.h"

// One simulation step for the people of one unit, and the behaviour rules the spawner shares.

namespace crowd {

class SpatialHash;
class Walkways;
struct CrowdState;
struct CrowdUnit;

// Everything a step reads that is the same for every person.
struct StepContext {
    const Walkways& walkways;
    const SpatialHash& neighbours;  // everyone's motion at the start of the step
    float deltaSeconds;
    float timeSeconds;    // simulation time, the clock of conversations
    uint32_t stepIndex;   // counts steps, used to spread level of detail work over steps
};

// Advances the members of unit `unitIndex` by one step: moves and animates them, or, for a
// unit far from the focus point that is not due this step, only notes the time it owes.
// Units are independent, so different units can be stepped on different threads at once.
void StepUnit(const StepContext& context, CrowdState& state, uint32_t unitIndex);

// Behaviour rules shared with the spawner -----------------------------------------------------

// Seconds a street walker walks before stopping to look around.
float NextLookAroundDelay(uint32_t& random);

// A random spot in `plaza` where a person can stand: clear of obstacles, roads and reserved
// space. Returns false if none was found in `tries` attempts.
bool PickPlazaSpot(const Walkways& walkways, const OrientedRect& plaza, uint32_t tries, uint32_t& random,
                   glm::vec2& spot);

}  // namespace crowd
