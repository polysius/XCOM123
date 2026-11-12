#pragma once

// Everything people walk and drive on: the ground, the river and canal with their quays, the
// bridges, the pavements around every block and the street lamps. Internal to src/world.

#include "world/CityPlan.h"

namespace city {

// Expects city.streets to hold the graph built from plan.segments.
void BuildStreetsAndWater(const CityPlan& plan, uint32_t seed, BoxWriter& boxes, CityLayout& city);

// The road, pavements and lamps of a street laid across open land along `path`, a polyline
// whose first and last points lie on the centre lines of existing streets. It meets those
// streets at right angles and bends only gently in between: the first and last legs should be
// perpendicular to the streets they join. `roadHalfWidth` and `streetHalfWidth` (road plus one
// pavement) are those of the rest of the city. The graph is not touched here.
void BuildConnectorStreet(const std::vector<glm::vec2>& path, float roadHalfWidth, float streetHalfWidth,
                          Random& random, BoxWriter& boxes, CityLayout& city);

// A bridge for a joining street where it crosses a reach of the river: the street runs from `origin`
// along `direction`, graph edge `edge`, and the bridge spans `start` to `end` metres along it, from
// land to land across the water and the levees. Its road and pavements are already laid by
// BuildConnectorStreet. The levees are lowered to a stone sill level with the ground along the whole
// crossing, so the street runs over them without a step, and the layout gains the bridge and the
// crossing. The bridge may meet the river at any angle.
void BuildRiverBridge(glm::vec2 origin, glm::vec2 direction, float start, float end, uint32_t edge, float roadHalfWidth,
                      float streetHalfWidth, Random& random, BoxWriter& boxes, CityLayout& city);

}  // namespace city
