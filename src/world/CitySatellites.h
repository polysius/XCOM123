#pragma once

// Satellite districts: smaller districts built on their own, turned to an angle of their own
// and placed in the open land around the finished city, each joined to the city by a street.
// Internal to src/world.

#include "world/CityPlan.h"

namespace city {

// Adds up to `settings.satelliteCount` districts to `city`, which holds the finished core city
// planned by `plan`. A district that finds no room that clears the river, the canal, the core
// and the districts placed before it is left out, so fewer than the wanted number may result.
//
// Each district is joined to the core at a T junction of its ring road (never at a corner) by a
// street that leaves and enters the two ring roads at right angles and bends gently between.
// The street graph, the bounds and every other part of the layout are extended to match.
void AddSatellites(CityLayout& city, const CitySettings& settings, const CityPlan& plan);

}  // namespace city
