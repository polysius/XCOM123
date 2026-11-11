#pragma once

// The street plan: where the river, canal, boulevards and streets go, which district every place
// belongs to, and which land between the streets becomes a block. Internal to src/world.

#include <vector>

#include "world/CityBuild.h"

namespace city {

// The character of a district, used to plan its streets and its buildings.
struct DistrictStyle {
    float minBlock;     // street spacing range, centre line to centre line
    float maxBlock;
    float mergeChance;  // chance that a block up to 1.7 times the maximum stays whole
    float plazaChance;
    float parkChance;
    float minLotWidth;
    float maxLotWidth;
    float minDepth;     // depth of the buildings around a block, from the street to the courtyard
    float maxDepth;
    int minUpperFloors;
    int maxUpperFloors;
};

const DistrictStyle& StyleOf(District district);

// Splits the city into districts around a few seed points (a Voronoi diagram). The historic core
// on the river becomes the old quarter, the ring around it the grand district.
class DistrictMap {
public:
    DistrictMap() = default;
    DistrictMap(uint32_t seed, float halfSize, glm::vec2 historicCore);

    District At(glm::vec2 point) const;

private:
    struct Site {
        glm::vec2 position;
        District district;
    };
    std::vector<Site> sites;
};

// A straight street centre line, from one intersection or bend to the next.
struct Segment {
    glm::vec2 a;
    glm::vec2 b;
};

enum class GapKind : uint8_t {
    River,
    Canal,
    Median,  // the planted strip of a boulevard
};

// A long strip between two parallel streets that holds no ordinary blocks.
struct Gap {
    GapKind kind;
    bool alongX;                   // true when the strip runs along X, between streets at constant Z
    Rect area;                     // between the centre lines of the two streets
    std::vector<float> crossings;  // where streets cross it: X for strips along X, Z otherwise
};

// A place reserved for a landmark before the ordinary streets are laid out.
struct LandmarkSite {
    LandmarkKind kind;
    Rect area;            // between the centre lines of the streets around it
    bool placed = false;  // false when no block could be found for it
};

struct PlannedBlock {
    Rect area;            // between the centre lines of the streets around it
    BlockUse use;
    int site = -1;        // index into CityPlan::sites for landmark blocks
};

struct Waterway {
    Rect water;           // the water surface
    float wallTop;        // height of the quay walls along it
};

// The river beyond its straight part. Through the city the river runs straight, between the
// blocks the planner lays out along it; beyond the ring road it bends. A reach is one side of that:
// a polyline running out to the edge of the open land, always moving away from the city, so that
// it can never cross itself.
struct RiverReach {
    std::vector<glm::vec2> points;  // the centre line, from where the straight river ends
    float halfWidth;                // the water reaches this far to either side
    float wallTop;                  // height of the bank walls
};

struct CityPlan {
    float halfSize = 0.0f;         // half the distance between the ring road centre lines
    float roadHalfWidth = 0.0f;
    float streetHalfWidth = 0.0f;  // road plus one pavement: the centre line to the edge of the land
    float farDistance = 0.0f;      // how far the open land and the river reach beyond the centre
    bool districtOnly = false;     // see CitySettings::districtOnly
    glm::vec2 historicCore{0.0f};
    DistrictMap districts;
    std::vector<Segment> segments;
    std::vector<Gap> gaps;
    std::vector<PlannedBlock> blocks;
    std::vector<LandmarkSite> sites;
    std::vector<Waterway> waterways;  // the river first, then the canal
    float riverStraightEnd = 0.0f;    // the straight river reaches this far from the centre, east and west
    std::vector<RiverReach> reaches;  // the river beyond that, one reach each side (none for a district)
};

inline constexpr float kQuayWallThickness = 0.8f;
// Beyond the ring road the banks of the river are thick levees, because the grass around a bend is
// cut away in coarse cells and the levee hides the jagged edge. It must be wider than a cell's
// diagonal (see LayRiverBankGrass in CityStreets.cpp).
inline constexpr float kReachWallThickness = 4.0f;
// The straight river continues this far past the outer pavement of the ring road.
inline constexpr float kRiverStraightMargin = 40.0f;

CityPlan PlanCity(const CitySettings& settings);

// Turns street centre lines into a graph. Segments meeting at T junctions are split there.
StreetGraph BuildStreetGraph(const std::vector<Segment>& segments);

// The water surface plus the quay walls around it.
Rect WaterZone(const Waterway& waterway);

}  // namespace city
