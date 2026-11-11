#pragma once

// Open spaces and the things standing in them: trees, benches, fountains, squares, parks,
// boulevard medians and courtyard gardens. Internal to src/world.

#include "world/CityBuild.h"

namespace city {

// A tree of `height` metres whose trunk stands at `position` on ground at height `ground`.
// The crown never starts below kOverhangClearance, so it is safe to plant beside walkways.
void AddTree(BoxWriter& boxes, CityLayout& city, glm::vec2 position, float ground, float height, Random& random);

// A park bench whose seat faces `facing`, which must point along X or Z.
void AddBench(BoxWriter& boxes, CityLayout& city, glm::vec2 center, float ground, glm::vec2 facing);

// A round-looking fountain basin with a tiered centre piece. `radius` should stay at or below
// 4.5 m: the crowd keeps 5.5 m away from the centre of every square.
void AddFountain(BoxWriter& boxes, CityLayout& city, glm::vec2 center, float ground, float radius, Random& random);

// A statue on a plinth, for squares too small or too busy for a fountain.
void AddStatue(BoxWriter& boxes, glm::vec2 center, float ground, Random& random);

// A market stall: counter, back board and a cloth canopy. `facing` points to the customers and
// must point along X or Z.
void AddMarketStall(BoxWriter& boxes, CityLayout& city, glm::vec2 center, glm::vec2 facing, float ground, Random& random);

// A paved square filling `land`. People walk on the inner part, which is exported in
// CityLayout::plazas; trees, benches and market stalls line the edges.
void BuildPlaza(BoxWriter& boxes, CityLayout& city, const Rect& land, District district, Random& random);

// A lawn with gravel paths, trees, hedges, benches and a fountain or pond.
void BuildPark(BoxWriter& boxes, CityLayout& city, const Rect& land, Random& random);

// The planted strip between the two carriageways of a boulevard.
void BuildMedian(BoxWriter& boxes, CityLayout& city, const Rect& land, Random& random);

// The yard inside a perimeter block: gravel, cobbles, or grass with a tree or two kept clear of
// the low annexes built into it.
void PlantCourtyard(BoxWriter& boxes, CityLayout& city, const Rect& courtyard, const std::vector<Rect>& annexes,
                    Random& random);

}  // namespace city
