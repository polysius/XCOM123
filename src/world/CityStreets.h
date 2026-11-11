#pragma once

// Everything people walk and drive on: the ground, the river and canal with their quays, the
// bridges, the pavements around every block and the street lamps. Internal to src/world.

#include "world/CityPlan.h"

namespace city {

// Expects city.streets to hold the graph built from plan.segments.
void BuildStreetsAndWater(const CityPlan& plan, uint32_t seed, BoxWriter& boxes, CityLayout& city);

}  // namespace city
