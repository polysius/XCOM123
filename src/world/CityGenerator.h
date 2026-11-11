#pragma once

#include <algorithm>
#include <array>
#include <cmath>
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

// A rectangle on the ground plane that may be turned. It is `area`, an ordinary rectangle, turned
// by `yaw` about its own centre, with the same sign convention as BoxInstance::yaw (a positive
// angle turns +Z towards +X). A plain Rect converts to an OrientedRect that is not turned, and
// every operation then reduces to the Rect arithmetic, so axis-aligned cities are unchanged.
//
// Code that works in the rectangle's own frame (sampling a point inside it, clamping a person
// to it) moves points with ToFrame and FromFrame, which are the identity when yaw is zero.
struct OrientedRect {
    Rect area;
    float yaw = 0.0f;

    OrientedRect() = default;
    OrientedRect(const Rect& unturned, float turn = 0.0f) : area(unturned), yaw(turn) {}

    glm::vec2 Center() const { return area.Center(); }
    glm::vec2 Size() const { return area.Size(); }
    float Area() const { return area.Area(); }
    bool IsEmpty() const { return area.IsEmpty(); }
    OrientedRect Expanded(float amount) const { return {area.Expanded(amount), yaw}; }

    // A world point expressed in the rectangle's own frame, where `area` is axis-aligned.
    glm::vec2 ToFrame(glm::vec2 point) const {
        if (yaw == 0.0f) {
            return point;
        }
        const glm::vec2 center = Center();
        const glm::vec2 offset = point - center;
        const float cosine = std::cos(yaw);
        const float sine = std::sin(yaw);
        return center + glm::vec2(offset.x * cosine - offset.y * sine, offset.x * sine + offset.y * cosine);
    }
    // The reverse: a point of the rectangle's own frame placed in the world.
    glm::vec2 FromFrame(glm::vec2 point) const {
        if (yaw == 0.0f) {
            return point;
        }
        const glm::vec2 center = Center();
        const glm::vec2 offset = point - center;
        const float cosine = std::cos(yaw);
        const float sine = std::sin(yaw);
        return center + glm::vec2(offset.x * cosine + offset.y * sine, -offset.x * sine + offset.y * cosine);
    }

    // `tolerance` widens the rectangle by that many metres on every side.
    bool Contains(glm::vec2 point, float tolerance = 0.0f) const {
        const glm::vec2 local = ToFrame(point);
        return local.x >= area.min.x - tolerance && local.x <= area.max.x + tolerance &&
               local.y >= area.min.y - tolerance && local.y <= area.max.y + tolerance;
    }

    // The smallest axis-aligned rectangle around it.
    Rect Bounds() const {
        if (yaw == 0.0f) {
            return area;
        }
        const glm::vec2 half = Size() * 0.5f;
        const float cosine = std::abs(std::cos(yaw));
        const float sine = std::abs(std::sin(yaw));
        const glm::vec2 reach(cosine * half.x + sine * half.y, sine * half.x + cosine * half.y);
        return {Center() - reach, Center() + reach};
    }

    // The four corners in the world, counter-clockwise from the minimum corner of the frame.
    std::array<glm::vec2, 4> Corners() const {
        return {FromFrame(area.min), FromFrame({area.max.x, area.min.y}), FromFrame(area.max),
                FromFrame({area.min.x, area.max.y})};
    }

    // True when the two share some area; touching edges do not count. Uses the separating axis
    // test: two convex shapes are apart exactly when some edge direction of either separates them.
    bool Overlaps(const OrientedRect& other) const {
        if (yaw == 0.0f && other.yaw == 0.0f) {
            return area.Overlaps(other.area);
        }
        const std::array<glm::vec2, 4> mine = Corners();
        const std::array<glm::vec2, 4> theirs = other.Corners();
        const glm::vec2 axes[] = {mine[1] - mine[0], mine[3] - mine[0], theirs[1] - theirs[0], theirs[3] - theirs[0]};
        for (glm::vec2 axis : axes) {
            float lowMine = INFINITY, highMine = -INFINITY, lowTheirs = INFINITY, highTheirs = -INFINITY;
            for (int i = 0; i < 4; ++i) {
                const float a = glm::dot(mine[i], axis);
                const float b = glm::dot(theirs[i], axis);
                lowMine = std::min(lowMine, a);
                highMine = std::max(highMine, a);
                lowTheirs = std::min(lowTheirs, b);
                highTheirs = std::max(highTheirs, b);
            }
            if (highMine <= lowTheirs || highTheirs <= lowMine) {
                return false;
            }
        }
        return true;
    }
};

// The street network as a graph: intersections are nodes, street segments between them are edges.
// Every edge is a straight segment with pavements on both sides. Edges may run at any angle.
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

    // A district is the streets and blocks inside a ring road and nothing else: no river, canal,
    // landmarks or open land around it. Satellite districts are built this way, and their asphalt
    // lies a centimetre above the ground so that it does not fight with the land they stand on.
    bool districtOnly = false;

    // Smaller districts built on their own, turned to their own angles and placed in the open land
    // around the city, each joined to it by a street. Fewer are made when there is no room.
    int satelliteCount = 4;
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
    OrientedRect land;
    District district;
    BlockUse use;
};

struct Landmark {
    LandmarkKind kind;
    OrientedRect footprint;  // the block it occupies
    glm::vec2 entrance;      // where people would gather in front of it
};

// A street edge that crosses water, and the deck that carries it.
struct Bridge {
    uint32_t edge;   // index into StreetGraph::edges
    OrientedRect deck;  // the part of the street over the water, pavements included
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
    std::vector<OrientedRect> plazas;   // open squares people wander in, free of buildings
    std::vector<OrientedRect> parks;
    std::vector<OrientedRect> water;    // water surfaces of the river and the canal
    std::vector<Bridge> bridges;
    std::vector<CityBlock> blocks;
    std::vector<Landmark> landmarks;
    std::vector<glm::vec3> streetLights;  // light positions of the street lamp lanterns
    std::vector<Tree> trees;
    std::vector<PointOfInterest> pointsOfInterest;
    Rect bounds;                        // everything: the core's ring road and any satellite districts
    Rect coreBounds;                    // outer edge of the pavements along the core's ring road
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
