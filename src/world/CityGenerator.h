#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "scene/SceneTypes.h"

// An axis-aligned rectangle on the ground plane. x is world X, y is world Z.
struct Rect {
    glm::vec2 min{0.0f};
    glm::vec2 max{0.0f};

    glm::vec2 Center() const { return (min + max) * 0.5f; }
    glm::vec2 Size() const { return max - min; }
    Rect Expanded(float amount) const { return {min - amount, max + amount}; }
    float Area() const { return glm::max(max.x - min.x, 0.0f) * glm::max(max.y - min.y, 0.0f); }
    bool IsEmpty() const { return max.x <= min.x || max.y <= min.y; }
    bool Contains(glm::vec2 point) const {
        return point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y;
    }
    // True when the two rectangles share some area; touching edges do not count.
    bool Overlaps(const Rect& other) const {
        return min.x < other.max.x && other.min.x < max.x && min.y < other.max.y && other.min.y < max.y;
    }
};

// The street network as a graph: intersections are nodes, street segments between them are edges.
// Every edge is a straight, axis-aligned segment with pavements on both sides.
struct StreetGraph {
    struct Edge {
        uint32_t from;
        uint32_t to;
        float length;
    };
    std::vector<glm::vec2> nodes;
    std::vector<Edge> edges;
    std::vector<std::vector<uint32_t>> edgesAtNode;  // indices into `edges`, for each node
};

struct CitySettings {
    uint32_t seed = 1789;
    float citySize = 1800.0f;      // metres between the centre lines of the ring road on opposite sides
    float roadWidth = 8.0f;        // the carriageway between the two pavements, the same on every street
    float sidewalkWidth = 3.5f;
    float riverWidth = 60.0f;      // width of the water between the two embankment walls
    float canalWidth = 16.0f;
    bool canal = true;             // a canal branching off the river through one bank
    int boulevardCount = 3;        // wide streets with a planted median, 0 to 3
    int extraBridgeCount = 3;      // river bridges besides the ones carrying the main avenues
};

// Every box is tagged with what it belongs to, so tools and tests can tell a building from a lamp.
enum class CityPart : uint8_t {
    Ground,      // asphalt and the open land outside the city
    Pavement,    // pavements and kerbs along streets
    Plaza,       // paved squares
    Park,        // lawns, park paths and hedges
    Water,
    Embankment,  // quay walls and riverside promenades
    Bridge,
    Building,
    Landmark,    // churches, the palace, the market hall and the clock tower
    Tree,
    StreetLamp,
    Bench,
    Fountain,    // fountains and statues
    MarketStall,
    Count,
};

// Districts give neighbourhoods a distinct character: street spacing, lot sizes, heights and colours.
enum class District : uint8_t {
    OldQuarter,   // narrow lots, 4 to 6 storeys, colourful plaster, tiled roofs
    Grand,        // big courtyard blocks behind uniform limestone fronts and mansard roofs
    Market,       // brick and cream houses with shop awnings, more squares
    Residential,  // ordinary apartment blocks
    Parkland,     // large parks with a few villas
    Count,
};

enum class BlockUse : uint8_t {
    Buildings,
    Plaza,
    Park,
    Median,    // the planted strip down the middle of a boulevard
    Landmark,
};

enum class LandmarkKind : uint8_t {
    Cathedral,
    Palace,
    MarketHall,
    ClockTower,
    ParishChurch,
    Count,
};

// The land between streets. `land` excludes the pavements around it.
struct CityBlock {
    Rect land;
    District district;
    BlockUse use;
};

struct Landmark {
    LandmarkKind kind;
    Rect footprint;          // the block it occupies
    glm::vec2 entrance;      // where people would gather in front of it
};

// A street edge that crosses water, and the deck that carries it.
struct Bridge {
    uint32_t edge;   // index into StreetGraph::edges
    Rect deck;       // the part of the street over the water, pavements included
};

struct Tree {
    glm::vec3 base;  // bottom of the trunk
    float height;
    float crownRadius;
};

enum class PointOfInterestKind : uint8_t {
    Fountain,
    Bench,
    LandmarkEntrance,
    MarketStall,
    BridgeView,  // the middle of a bridge, for people who stop to look at the river
};

struct PointOfInterest {
    glm::vec3 position;
    glm::vec2 facing;  // unit direction a person standing here would face
    PointOfInterestKind kind;
};

// Everything the generator produces: geometry to draw, and the walkable spaces people use.
struct CityLayout {
    std::vector<BoxInstance> boxes;
    std::vector<CityPart> boxParts;     // what each box in `boxes` belongs to, same order
    StreetGraph streets;
    std::vector<Rect> plazas;           // open squares people wander in, free of buildings
    std::vector<Rect> parks;
    std::vector<Rect> water;            // water surfaces of the river and the canal
    std::vector<Bridge> bridges;
    std::vector<CityBlock> blocks;
    std::vector<Landmark> landmarks;
    std::vector<glm::vec3> streetLights;  // light positions of the street lamp lanterns
    std::vector<Tree> trees;
    std::vector<PointOfInterest> pointsOfInterest;
    Rect bounds;                        // outer edge of the pavements along the ring road
    float roadWidth = 0.0f;
    float sidewalkCenterOffset = 0.0f;  // distance from a street's centre line to the middle of its pavements
    float sidewalkWidth = 0.0f;
    float walkHeight = 0.0f;            // height of the pavement surface people stand on
    float waterLevel = 0.0f;            // height of the river surface
};

// Builds a city from `settings`. The same seed always produces the same city.
CityLayout GenerateCity(const CitySettings& settings);

const char* DistrictName(District district);
const char* CityPartName(CityPart part);
