#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "world/CityGenerator.h"
#include "world/TerrainField.h"

namespace crowd {

// The walkable space of a city, built once from a CityLayout: the pavements along every
// street, the crosswalks at intersections, the open squares, and a grid of the obstacles
// (buildings, fountains and other props) that stand on them.
//
// It relies only on the CityLayout contract: streets are straight edges of a connected
// graph, each street has a pavement on both sides centred sidewalkCenterOffset from its
// centre line, and squares are building-free rectangles. Any street graph works, not just
// a grid.
//
// Intersections. The streets leaving a node are its arms, numbered counter-clockwise by
// direction. Corner i of a node is the pavement corner between arm i and arm i + 1 (wrapping
// around), so a node with k arms has k corners. Crossing arm i means walking from corner
// i - 1 to corner i over the crosswalk laid across that street near the node.
class Walkways {
public:
    void Build(const CityLayout& city);

    const StreetGraph& Streets() const { return streets; }
    const std::vector<OrientedRect>& Plazas() const { return plazas; }

    // A way from a square into a room inside a landmark and back. A person walks to `outsideSpot`
    // or `insideSpot` as an ordinary stroller, then along `passage` (a strip clear of obstacles that
    // runs between the two spots through the doorways) to the other spot, and carries on strolling
    // in the square at the other end. The passage is walkable ground like a square.
    struct Portal {
        uint32_t outsidePlaza;
        uint32_t insidePlaza;
        OrientedRect passage;
        glm::vec2 outsideSpot;
        glm::vec2 insideSpot;
    };
    const std::vector<Portal>& Portals() const { return portals; }
    // The portals that lead out of or into `plaza`.
    const std::vector<uint32_t>& PortalsOfPlaza(uint32_t plaza) const { return portalsOfPlaza[plaza]; }

    // Distances from a street's centre line: the road ends and the pavement starts at
    // RoadHalfWidth, the middle of the pavement is LaneCenter, and it ends at OuterEdge.
    float RoadHalfWidth() const { return roadHalfWidth; }
    float LaneCenter() const { return laneCenter; }
    float OuterEdge() const { return outerEdge; }
    float SidewalkWidth() const { return sidewalkWidth; }
    // The height of the pavement at the level of the city. People stand higher on a terrace or a stair.
    float WalkHeight() const { return walkHeight; }
    // True when the whole city is level, so that WalkHeightAt is WalkHeight everywhere.
    bool IsLevel() const { return terrain.IsLevel(); }
    // The height of the walking surface at `point`: WalkHeight, plus the lift of the terrain there.
    float WalkHeightAt(glm::vec2 point) const { return walkHeight + terrain.LiftAt(point); }

    // Intersections ---------------------------------------------------------------------

    uint32_t ArmCount(uint32_t node) const { return armStart[node + 1] - armStart[node]; }
    uint32_t ArmEdge(uint32_t node, uint32_t arm) const { return arms[armStart[node] + arm].edge; }
    // The index of `edge` among the arms of `node`, which must be one of its two ends.
    uint32_t ArmOfEdge(uint32_t edge, uint32_t node) const;

    // The point of a corner at `laneOffset` metres from both streets that meet there.
    glm::vec2 CornerPoint(uint32_t node, uint32_t corner, float laneOffset) const;
    // How far from the node, measured along either street of the corner, that point lies.
    float CornerSetback(uint32_t node, uint32_t corner, float laneOffset) const;

    // Obstacles and reserved space ---------------------------------------------------------

    // True inside the footprint of a building or prop.
    bool IsInsideObstacle(glm::vec2 point) const { return (CellFlags(point) & kObstacleFlag) != 0; }
    // True if a person can stand with its centre at `point` without its body touching an
    // obstacle.
    bool CanStandAt(glm::vec2 point) const { return (CellFlags(point) & kNoStandingFlag) == 0; }
    // As CanStandAt, and the space is not reserved either (see Reserve).
    bool CanStandAtUnreserved(glm::vec2 point) const {
        return (CellFlags(point) & (kNoStandingFlag | kReservedFlag)) == 0;
    }
    // Marks a disc as taken, for example by a conversation circle, so that people strolling in
    // squares do not walk through it. Street walkers ignore reservations.
    void Reserve(glm::vec2 center, float radius);
    // True if a person can stand anywhere in the disc and none of it is reserved. A disc on a
    // portal's passage is never free.
    bool IsDiscFree(glm::vec2 center, float radius) const;
    // True if a person can walk the straight line from a to b without touching an obstacle
    // or reserved space.
    bool IsPathFree(glm::vec2 a, glm::vec2 b) const;
    // How much of a square's area people can stand on, in square metres.
    float FreeArea(const OrientedRect& area) const;

    // Queries over the whole network (slower; for spawning and checking) -------------------

    // The distance from `point` to the nearest street centre line.
    float DistanceToNearestStreet(glm::vec2 point) const;
    // True if a person may stand at `point`: on a pavement, a crosswalk or in a square, and not
    // inside an obstacle. `tolerance` widens the walkable areas by that many metres.
    bool IsWalkable(glm::vec2 point, float tolerance) const;

private:
    struct Arm {
        uint32_t edge;
        glm::vec2 direction;  // unit vector pointing away from the node
    };
    // How a corner's point and setback scale with the distance from the streets.
    struct CornerShape {
        glm::vec2 bisector;  // unit vector from the node into the corner
        float offsetScale;   // distance from the node per metre of lane offset
        float setbackScale;  // setback along either street per metre of lane offset
    };
    struct Segment {
        glm::vec2 start;
        glm::vec2 end;
    };
    // A uniform grid over the city that lists, for each cell, the segments near it.
    struct SegmentBuckets {
        glm::vec2 origin{0.0f};
        float cellSize = 25.0f;
        int columns = 0;
        int rows = 0;
        std::vector<uint32_t> cellStart;  // per cell, the first entry in `items`; one extra at the end
        std::vector<uint32_t> items;      // segment indices
    };

    static constexpr uint8_t kObstacleFlag = 1;    // inside an obstacle's footprint
    static constexpr uint8_t kNoStandingFlag = 2;  // within a body radius of an obstacle
    static constexpr uint8_t kReservedFlag = 4;

    void BuildArms();
    void BuildCrosswalks();
    void BuildObstacleGrid(const CityLayout& city);
    void BuildPortals(const CityLayout& city);
    void RasteriseObstacle(const BoxInstance& box, float grow, uint8_t flag);
    uint8_t CellFlags(glm::vec2 point) const;
    glm::ivec2 CellOf(glm::vec2 point) const;
    static void FillBuckets(SegmentBuckets& buckets, const std::vector<Segment>& segments, float reach,
                            glm::vec2 boundsMin, glm::vec2 boundsMax);
    template <typename Visit>
    static void ForEachNearbySegment(const SegmentBuckets& buckets, glm::vec2 point, Visit&& visit);

    StreetGraph streets;
    std::vector<OrientedRect> plazas;
    std::vector<Portal> portals;
    std::vector<std::vector<uint32_t>> portalsOfPlaza;  // per square, indices into `portals`
    float roadHalfWidth = 0.0f;
    float laneCenter = 0.0f;
    float outerEdge = 0.0f;
    float sidewalkWidth = 0.0f;
    float walkHeight = 0.0f;
    TerrainField terrain;

    // Arms of every node in counter-clockwise order, node n's in [armStart[n], armStart[n + 1]).
    std::vector<uint32_t> armStart;
    std::vector<Arm> arms;
    std::vector<CornerShape> corners;  // indexed like `arms`: corner i of node n at armStart[n] + i

    std::vector<Segment> streetSegments;  // one per edge, along its centre line
    std::vector<Segment> crosswalks;      // centre lines of the crosswalks
    SegmentBuckets streetBuckets;
    SegmentBuckets crosswalkBuckets;

    // Obstacle grid: one byte of flags per cell. Outside the grid counts as an obstacle.
    glm::vec2 gridOrigin{0.0f};
    float gridCellSize = 0.5f;
    int gridColumns = 0;
    int gridRows = 0;
    std::vector<uint8_t> gridFlags;
};

}  // namespace crowd
