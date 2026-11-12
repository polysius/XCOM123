#pragma once

// Ordinary town buildings: blocks cut into lots, each lot a facade with ledges, cornice, roof,
// chimneys and, depending on the district, balconies, awnings and attic storeys. Internal to src/world.

#include "world/CityBuild.h"

namespace city {

// A plot of land for one building. Street sides are the sides that face a pavement.
struct Lot {
    Rect area;
    uint8_t streetSides = 0;
};

enum class RoofShape : uint8_t {
    Mansard,  // steep slate slopes with dormers, then a low top
    Pitched,  // tiled, approximated by three receding steps
    Flat,     // a low parapet roof
};

// Everything that decides how one building looks.
struct BuildingDesign {
    glm::vec3 wallColor{0.8f};
    glm::vec3 trimColor{0.85f};  // ledges and cornices
    glm::vec3 roofColor{0.2f};
    uint32_t facadeFlags = 0;
    int upperFloors = 4;
    RoofShape roof = RoofShape::Mansard;
    float corniceDepth = 0.35f;
    bool balconies = false;
    bool awnings = false;
    bool attic = false;  // a set-back storey above the cornice
    bool belvedere = false;  // a small tower turned 45 degrees, with a copper spire, on a mansard roof
};

// Height of the top of a building's walls.
float WallHeight(int upperFloors);

// Adds one building with the writer's current part tag, so landmarks can reuse it.
void AddBuilding(BoxWriter& boxes, const Lot& lot, const BuildingDesign& design, Random& random);

// Fills `land` with buildings in the manner of `district`: a ring of buildings around a planted
// courtyard where the block is deep enough, back-to-back rows where it is not.
void BuildBlockOfBuildings(BoxWriter& boxes, CityLayout& city, const Rect& land, District district, Random& random);

}  // namespace city
