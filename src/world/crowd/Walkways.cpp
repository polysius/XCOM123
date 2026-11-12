#include "world/crowd/Walkways.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "world/crowd/CrowdMath.h"
#include "world/crowd/CrowdTypes.h"

namespace crowd {

namespace {

// Boxes that reach into this height range above the walking surface are in people's way.
// Lower boxes are kerbs and paving; higher ones are ledges and balconies overhead.
constexpr float kObstacleLowest = 0.25f;
constexpr float kObstacleHighest = 1.6f;
// Obstacle footprints are shrunk by this much before they are drawn into the grid, so that
// a person standing right against a wall never counts as inside it.
constexpr float kObstacleShrink = 0.05f;
// The obstacle grid reaches this far past the street network.
constexpr float kGridMargin = 10.0f;
// The flattest corner angle used when placing corner points. Sharper corners would put the
// point very far from the node.
constexpr float kMinCornerSine = 1.0f / 3.0f;

float SegmentDistance(glm::vec2 point, glm::vec2 start, glm::vec2 end) {
    const glm::vec2 span = end - start;
    const float lengthSquared = glm::dot(span, span);
    const float t = lengthSquared > 1e-12f ? std::clamp(glm::dot(point - start, span) / lengthSquared, 0.0f, 1.0f) : 0.0f;
    return glm::length(point - (start + span * t));
}

}  // namespace

void Walkways::Build(const CityLayout& city) {
    streets = city.streets;
    plazas = city.plazas;
    sidewalkWidth = city.sidewalkWidth;
    laneCenter = city.sidewalkCenterOffset;
    roadHalfWidth = city.sidewalkCenterOffset - city.sidewalkWidth * 0.5f;
    outerEdge = city.sidewalkCenterOffset + city.sidewalkWidth * 0.5f;
    walkHeight = city.walkHeight;
    terrain.Build(city.terrain);

    BuildArms();
    BuildCrosswalks();
    BuildObstacleGrid(city);
}

void Walkways::BuildArms() {
    const uint32_t nodeCount = static_cast<uint32_t>(streets.nodes.size());
    armStart.assign(nodeCount + 1, 0);
    arms.clear();
    corners.clear();

    for (uint32_t node = 0; node < nodeCount; ++node) {
        armStart[node] = static_cast<uint32_t>(arms.size());
        const size_t first = arms.size();
        for (uint32_t edgeIndex : streets.edgesAtNode[node]) {
            const StreetGraph::Edge& edge = streets.edges[edgeIndex];
            const uint32_t other = edge.from == node ? edge.to : edge.from;
            arms.push_back({edgeIndex, SafeNormalize(streets.nodes[other] - streets.nodes[node])});
        }
        // Counter-clockwise order, starting from the +X direction.
        std::sort(arms.begin() + static_cast<std::ptrdiff_t>(first), arms.end(), [](const Arm& a, const Arm& b) {
            return std::atan2(a.direction.y, a.direction.x) < std::atan2(b.direction.y, b.direction.x);
        });

        const size_t armCount = arms.size() - first;
        for (size_t i = 0; i < armCount; ++i) {
            const glm::vec2 current = arms[first + i].direction;
            if (armCount == 1) {
                // A dead end has no real corner: people turn around on their own pavement.
                corners.push_back({current, 1.0f, 0.0f});
                continue;
            }
            const glm::vec2 next = arms[first + (i + 1) % armCount].direction;
            float opening = std::atan2(current.x * next.y - current.y * next.x, glm::dot(current, next));
            if (opening <= 0.0f) {
                opening += kTwoPi;  // the corner is the counter-clockwise turn from `current` to `next`
            }
            const float halfOpening = opening * 0.5f;
            const glm::vec2 bisector(current.x * std::cos(halfOpening) - current.y * std::sin(halfOpening),
                                     current.x * std::sin(halfOpening) + current.y * std::cos(halfOpening));
            // A point on the bisector at distance d from the node is d * sin(half opening) from
            // both streets, and d * cos(half opening) along each of them.
            const float offsetScale = 1.0f / std::max(std::sin(halfOpening), kMinCornerSine);
            corners.push_back({bisector, offsetScale, offsetScale * std::cos(halfOpening)});
        }
    }
    armStart[nodeCount] = static_cast<uint32_t>(arms.size());
}

void Walkways::BuildCrosswalks() {
    streetSegments.clear();
    for (const StreetGraph::Edge& edge : streets.edges) {
        streetSegments.push_back({streets.nodes[edge.from], streets.nodes[edge.to]});
    }

    // One crosswalk over every arm of every intersection, from corner arm - 1 to corner arm.
    crosswalks.clear();
    for (uint32_t node = 0; node < static_cast<uint32_t>(streets.nodes.size()); ++node) {
        const uint32_t armCount = ArmCount(node);
        if (armCount < 2) {
            continue;
        }
        for (uint32_t arm = 0; arm < armCount; ++arm) {
            const uint32_t previousCorner = (arm + armCount - 1) % armCount;
            crosswalks.push_back({CornerPoint(node, previousCorner, laneCenter), CornerPoint(node, arm, laneCenter)});
        }
    }

    glm::vec2 boundsMin(0.0f);
    glm::vec2 boundsMax(0.0f);
    if (!streets.nodes.empty()) {
        boundsMin = boundsMax = streets.nodes.front();
        for (glm::vec2 node : streets.nodes) {
            boundsMin = glm::min(boundsMin, node);
            boundsMax = glm::max(boundsMax, node);
        }
    }
    const float reach = outerEdge + sidewalkWidth;
    FillBuckets(streetBuckets, streetSegments, reach, boundsMin - reach, boundsMax + reach);
    FillBuckets(crosswalkBuckets, crosswalks, reach, boundsMin - reach, boundsMax + reach);
}

void Walkways::FillBuckets(SegmentBuckets& buckets, const std::vector<Segment>& segments, float reach,
                           glm::vec2 boundsMin, glm::vec2 boundsMax) {
    buckets.origin = boundsMin;
    buckets.columns = std::max(1, static_cast<int>(std::ceil((boundsMax.x - boundsMin.x) / buckets.cellSize)));
    buckets.rows = std::max(1, static_cast<int>(std::ceil((boundsMax.y - boundsMin.y) / buckets.cellSize)));
    const size_t cellCount = static_cast<size_t>(buckets.columns) * static_cast<size_t>(buckets.rows);

    // Each segment goes into every cell its bounding box (grown by `reach`) touches. Two passes:
    // count per cell, then fill, so the result is one flat array.
    const auto forEachCell = [&](const Segment& segment, auto&& visit) {
        const glm::vec2 low = glm::min(segment.start, segment.end) - reach - buckets.origin;
        const glm::vec2 high = glm::max(segment.start, segment.end) + reach - buckets.origin;
        const int x0 = std::clamp(static_cast<int>(std::floor(low.x / buckets.cellSize)), 0, buckets.columns - 1);
        const int x1 = std::clamp(static_cast<int>(std::floor(high.x / buckets.cellSize)), 0, buckets.columns - 1);
        const int y0 = std::clamp(static_cast<int>(std::floor(low.y / buckets.cellSize)), 0, buckets.rows - 1);
        const int y1 = std::clamp(static_cast<int>(std::floor(high.y / buckets.cellSize)), 0, buckets.rows - 1);
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                visit(static_cast<size_t>(y) * static_cast<size_t>(buckets.columns) + static_cast<size_t>(x));
            }
        }
    };

    std::vector<uint32_t> counts(cellCount, 0);
    for (const Segment& segment : segments) {
        forEachCell(segment, [&](size_t cell) { ++counts[cell]; });
    }
    buckets.cellStart.assign(cellCount + 1, 0);
    std::partial_sum(counts.begin(), counts.end(), buckets.cellStart.begin() + 1);
    buckets.items.assign(buckets.cellStart.back(), 0);
    std::vector<uint32_t> fill(buckets.cellStart.begin(), buckets.cellStart.end() - 1);
    for (uint32_t index = 0; index < static_cast<uint32_t>(segments.size()); ++index) {
        forEachCell(segments[index], [&](size_t cell) { buckets.items[fill[cell]++] = index; });
    }
}

template <typename Visit>
void Walkways::ForEachNearbySegment(const SegmentBuckets& buckets, glm::vec2 point, Visit&& visit) {
    const glm::vec2 local = (point - buckets.origin) / buckets.cellSize;
    const int x = static_cast<int>(std::floor(local.x));
    const int y = static_cast<int>(std::floor(local.y));
    if (x < 0 || y < 0 || x >= buckets.columns || y >= buckets.rows) {
        return;
    }
    const size_t cell = static_cast<size_t>(y) * static_cast<size_t>(buckets.columns) + static_cast<size_t>(x);
    for (uint32_t i = buckets.cellStart[cell]; i < buckets.cellStart[cell + 1]; ++i) {
        visit(buckets.items[i]);
    }
}

void Walkways::BuildObstacleGrid(const CityLayout& city) {
    glm::vec2 low = city.bounds.min;
    glm::vec2 high = city.bounds.max;
    for (glm::vec2 node : streets.nodes) {
        low = glm::min(low, node - outerEdge);
        high = glm::max(high, node + outerEdge);
    }
    for (const OrientedRect& plaza : plazas) {
        const Rect bounds = plaza.Bounds();
        low = glm::min(low, bounds.min);
        high = glm::max(high, bounds.max);
    }
    gridOrigin = low - kGridMargin;
    gridColumns = static_cast<int>(std::ceil((high.x - low.x + 2.0f * kGridMargin) / gridCellSize));
    gridRows = static_cast<int>(std::ceil((high.y - low.y + 2.0f * kGridMargin) / gridCellSize));
    gridFlags.assign(static_cast<size_t>(gridColumns) * static_cast<size_t>(gridRows), 0);

    for (const BoxInstance& box : city.boxes) {
        const float bottom = box.center.y - box.halfSize.y;
        const float top = box.center.y + box.halfSize.y;
        // Whether a box is in people's way depends on how high it stands above the ground under it.
        const float ground = WalkHeightAt({box.center.x, box.center.z});
        if (bottom < ground + kObstacleHighest && top > ground + kObstacleLowest) {
            RasteriseObstacle(box, -kObstacleShrink, kObstacleFlag);
            RasteriseObstacle(box, kBodyRadius - kObstacleShrink, kNoStandingFlag);
        }
    }
}

// Sets `flag` on every cell whose centre lies inside the box's footprint grown by `grow`. The
// footprint is a rectangle turned by the box's yaw.
void Walkways::RasteriseObstacle(const BoxInstance& box, float grow, uint8_t flag) {
    const float cosine = std::cos(box.yaw);
    const float sine = std::sin(box.yaw);
    // A cell is judged by its centre alone. Along an edge that runs across the grid at an angle the
    // centre can be further from a person standing in the cell than along an axis-aligned edge:
    // the cell reaches (|cos| + |sin|) * half a cell across a turned edge, against half a cell
    // across an axis-aligned one. The footprint grows by the difference, which is nothing for a
    // box that is not turned.
    // That is exact, which leaves no margin for a small box (a lamp base is about a third of a cell
    // wide) in the worst position, so a turned box gets a couple of centimetres more.
    constexpr float kTurnedMargin = 0.02f;
    const float turnedCellReach = 0.5f * gridCellSize * (std::abs(cosine) + std::abs(sine) - 1.0f) +
                                  (std::abs(sine) > 1e-6f ? kTurnedMargin : 0.0f);
    const glm::vec2 reach = glm::vec2(box.halfSize.x, box.halfSize.z) + grow + turnedCellReach;  // half size along the box's axes
    if (reach.x <= 0.0f || reach.y <= 0.0f) {
        return;
    }
    // The cells to test are those in the axis-aligned rectangle that encloses the turned one.
    const glm::vec2 enclosingReach(std::abs(cosine) * reach.x + std::abs(sine) * reach.y,
                                   std::abs(sine) * reach.x + std::abs(cosine) * reach.y);
    const glm::vec2 boxCenter(box.center.x, box.center.z);
    const glm::vec2 low = boxCenter - enclosingReach;
    const glm::vec2 high = boxCenter + enclosingReach;
    const glm::vec2 first = glm::ceil((low - gridOrigin) / gridCellSize - 0.5f);
    const glm::vec2 last = glm::floor((high - gridOrigin) / gridCellSize - 0.5f);
    const int x0 = std::max(0, static_cast<int>(first.x));
    const int y0 = std::max(0, static_cast<int>(first.y));
    const int x1 = std::min(gridColumns - 1, static_cast<int>(last.x));
    const int y1 = std::min(gridRows - 1, static_cast<int>(last.y));
    for (int y = y0; y <= y1; ++y) {
        uint8_t* row = gridFlags.data() + static_cast<size_t>(y) * static_cast<size_t>(gridColumns);
        for (int x = x0; x <= x1; ++x) {
            // Express the cell's centre in the box's own axes, where the footprint is axis-aligned.
            const glm::vec2 offset = gridOrigin + (glm::vec2(x, y) + 0.5f) * gridCellSize - boxCenter;
            const float alongX = offset.x * cosine - offset.y * sine;
            const float alongZ = offset.x * sine + offset.y * cosine;
            if (std::abs(alongX) <= reach.x && std::abs(alongZ) <= reach.y) {
                row[x] |= flag;
            }
        }
    }
}

glm::ivec2 Walkways::CellOf(glm::vec2 point) const {
    const glm::vec2 local = (point - gridOrigin) / gridCellSize;
    return {static_cast<int>(std::floor(local.x)), static_cast<int>(std::floor(local.y))};
}

uint8_t Walkways::CellFlags(glm::vec2 point) const {
    const glm::ivec2 cell = CellOf(point);
    if (cell.x < 0 || cell.y < 0 || cell.x >= gridColumns || cell.y >= gridRows) {
        return kObstacleFlag | kNoStandingFlag;  // nobody may leave the city
    }
    return gridFlags[static_cast<size_t>(cell.y) * static_cast<size_t>(gridColumns) + static_cast<size_t>(cell.x)];
}

void Walkways::Reserve(glm::vec2 center, float radius) {
    const glm::ivec2 low = CellOf(center - radius);
    const glm::ivec2 high = CellOf(center + radius);
    for (int y = std::max(low.y, 0); y <= std::min(high.y, gridRows - 1); ++y) {
        for (int x = std::max(low.x, 0); x <= std::min(high.x, gridColumns - 1); ++x) {
            const glm::vec2 cellCenter = gridOrigin + (glm::vec2(x, y) + 0.5f) * gridCellSize;
            if (glm::distance(cellCenter, center) <= radius) {
                gridFlags[static_cast<size_t>(y) * static_cast<size_t>(gridColumns) + static_cast<size_t>(x)] |= kReservedFlag;
            }
        }
    }
}

bool Walkways::IsDiscFree(glm::vec2 center, float radius) const {
    const float reach = radius + gridCellSize * 0.5f;
    const glm::ivec2 low = CellOf(center - reach);
    const glm::ivec2 high = CellOf(center + reach);
    for (int y = low.y; y <= high.y; ++y) {
        for (int x = low.x; x <= high.x; ++x) {
            const glm::vec2 cellCenter = gridOrigin + (glm::vec2(x, y) + 0.5f) * gridCellSize;
            if (glm::distance(cellCenter, center) <= reach && !CanStandAtUnreserved(cellCenter)) {
                return false;
            }
        }
    }
    return true;
}

bool Walkways::IsPathFree(glm::vec2 a, glm::vec2 b) const {
    const float length = glm::distance(a, b);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / (gridCellSize * 0.5f))));
    for (int step = 0; step <= steps; ++step) {
        if (!CanStandAtUnreserved(glm::mix(a, b, static_cast<float>(step) / static_cast<float>(steps)))) {
            return false;
        }
    }
    return true;
}

float Walkways::FreeArea(const OrientedRect& area) const {
    int freeCells = 0;
    const Rect bounds = area.Bounds();
    const glm::ivec2 low = CellOf(bounds.min);
    const glm::ivec2 high = CellOf(bounds.max);
    for (int y = low.y; y <= high.y; ++y) {
        for (int x = low.x; x <= high.x; ++x) {
            const glm::vec2 cellCenter = gridOrigin + (glm::vec2(x, y) + 0.5f) * gridCellSize;
            if (area.Contains(cellCenter) && CanStandAtUnreserved(cellCenter)) {
                ++freeCells;
            }
        }
    }
    return static_cast<float>(freeCells) * gridCellSize * gridCellSize;
}

uint32_t Walkways::ArmOfEdge(uint32_t edge, uint32_t node) const {
    for (uint32_t arm = 0; arm < ArmCount(node); ++arm) {
        if (arms[armStart[node] + arm].edge == edge) {
            return arm;
        }
    }
    return 0;
}

glm::vec2 Walkways::CornerPoint(uint32_t node, uint32_t corner, float laneOffset) const {
    const CornerShape& shape = corners[armStart[node] + corner];
    return streets.nodes[node] + shape.bisector * (laneOffset * shape.offsetScale);
}

float Walkways::CornerSetback(uint32_t node, uint32_t corner, float laneOffset) const {
    return laneOffset * corners[armStart[node] + corner].setbackScale;
}

float Walkways::DistanceToNearestStreet(glm::vec2 point) const {
    float nearest = std::numeric_limits<float>::max();
    ForEachNearbySegment(streetBuckets, point, [&](uint32_t index) {
        nearest = std::min(nearest, SegmentDistance(point, streetSegments[index].start, streetSegments[index].end));
    });
    return nearest;
}

bool Walkways::IsWalkable(glm::vec2 point, float tolerance) const {
    if (IsInsideObstacle(point)) {
        return false;
    }
    for (const OrientedRect& plaza : plazas) {
        if (plaza.Contains(point, tolerance)) {
            return true;
        }
    }
    bool onCrosswalk = false;
    ForEachNearbySegment(crosswalkBuckets, point, [&](uint32_t index) {
        const Segment& crosswalk = crosswalks[index];
        // Crosswalks reach half a pavement width past their corner points, into the pavement.
        const glm::vec2 along = SafeNormalize(crosswalk.end - crosswalk.start) * (sidewalkWidth * 0.5f);
        if (SegmentDistance(point, crosswalk.start - along, crosswalk.end + along) <= sidewalkWidth * 0.5f + tolerance) {
            onCrosswalk = true;
        }
    });
    if (onCrosswalk) {
        return true;
    }
    // On a pavement: close enough to some street, but not on any street's road.
    const float distance = DistanceToNearestStreet(point);
    return distance >= roadHalfWidth - tolerance && distance <= outerEdge + tolerance;
}

}  // namespace crowd
