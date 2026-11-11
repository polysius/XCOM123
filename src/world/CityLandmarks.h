#pragma once

// The city's procedural landmarks: a cathedral, a palace, a market hall, a clock tower and parish
// churches, each built on its own block together with the square in front of it. Internal to src/world.

#include "world/CityBuild.h"

namespace city {

// Builds `kind` on `land`, the block inside its pavements, and records it in city.landmarks.
// Squares in front of landmarks are exported in city.plazas and stay free of buildings.
void BuildLandmark(BoxWriter& boxes, CityLayout& city, LandmarkKind kind, const Rect& land, District district,
                   Random& random);

}  // namespace city
