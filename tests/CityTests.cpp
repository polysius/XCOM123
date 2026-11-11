// Console tests for the procedural city generator, plus a tool that draws a top-down map.
//
//   CityTests                         checks several seeds and prints a summary
//   CityTests --map city.png          also writes a map of the default city
//   CityTests --map city.png --seed 7 --window -200 -200 200 200 --scale 4
//
// The checks follow the contract the crowd simulation relies on: every street has a pavement,
// the street graph is connected, nothing blocks the pavements, squares are free of buildings,
// lamps stand on pavements and bridges carry streets across the water.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "PngImage.h"
#include "world/CityComposition.h"
#include "world/CityGenerator.h"

namespace {

// ---------------------------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------------------------

int totalFailures = 0;
std::map<std::string, int> failuresPerCheck;
bool quiet = false;  // set while checks run on geometry that is broken on purpose

// Records a failed check. Only the first few failures of each kind are printed, so one broken
// rule does not flood the output.
void Fail(const std::string& check, const std::string& detail) {
    ++totalFailures;
    const int count = ++failuresPerCheck[check];
    if (quiet) {
        return;
    }
    if (count <= 5) {
        std::printf("  FAIL %s: %s\n", check.c_str(), detail.c_str());
    } else if (count == 6) {
        std::printf("  FAIL %s: (more failures not shown)\n", check.c_str());
    }
}

void Expect(bool condition, const std::string& check, const std::string& detail) {
    if (!condition) {
        Fail(check, detail);
    }
}

// Runs `checks` without printing or keeping their failures, and returns how many they found.
// Used to make sure the checks do notice geometry that is broken on purpose.
template <typename Checks>
int CountFailuresOf(Checks&& checks) {
    const int before = totalFailures;
    const std::map<std::string, int> savedPerCheck = failuresPerCheck;
    quiet = true;
    checks();
    quiet = false;
    const int found = totalFailures - before;
    totalFailures = before;
    failuresPerCheck = savedPerCheck;
    return found;
}

std::string Format(const char* format, double a = 0, double b = 0, double c = 0, double d = 0) {
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer), format, a, b, c, d);
    return buffer;
}

// ---------------------------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------------------------

constexpr float kTolerance = 0.01f;

// Clearance under projecting parts of buildings (cornices, balconies, awnings) above a pavement.
constexpr float kOverhangClearance = 3.0f;

// The crowd keeps people this far (half a square) from the middle of a square, where fountains
// and statues stand.
constexpr float kFountainKeepOut = 5.5f;

// How high above the pavement the space must be free for people to walk.
constexpr float kWalkingClearance = 2.0f;

// The ground footprint of a box, turned by its yaw.
OrientedRect FootprintOf(const BoxInstance& box) {
    return {Rect{{box.center.x - box.halfSize.x, box.center.z - box.halfSize.z},
                 {box.center.x + box.halfSize.x, box.center.z + box.halfSize.z}},
            box.yaw};
}

float BottomOf(const BoxInstance& box) { return box.center.y - box.halfSize.y; }
float TopOf(const BoxInstance& box) { return box.center.y + box.halfSize.y; }

// True when the two rectangles share more than a sliver of area.
bool OverlapsBeyondTolerance(const OrientedRect& a, const OrientedRect& b) {
    if (a.yaw == 0.0f && b.yaw == 0.0f) {
        return a.area.min.x < b.area.max.x - kTolerance && b.area.min.x < a.area.max.x - kTolerance &&
               a.area.min.y < b.area.max.y - kTolerance && b.area.min.y < a.area.max.y - kTolerance;
    }
    // With turned rectangles each one is shrunk by half the tolerance before the overlap test.
    return a.Expanded(-kTolerance * 0.5f).Overlaps(b.Expanded(-kTolerance * 0.5f));
}

bool ContainsWithTolerance(const OrientedRect& rect, glm::vec2 point) {
    return rect.Contains(point, kTolerance);
}

// The strip beside a street segment a to b: from `lateralLow` to `lateralHigh` metres to the side
// of its centre line (positive is to the left of the direction a to b), and `extend` metres past
// both ends. The strip is turned to follow the segment, whichever way it runs.
OrientedRect SegmentStrip(glm::vec2 a, glm::vec2 b, float lateralLow, float lateralHigh, float extend) {
    const float length = glm::length(b - a);
    const glm::vec2 direction = (b - a) / length;
    const glm::vec2 side{-direction.y, direction.x};
    if (a.x == b.x || a.y == b.y) {
        // An axis-aligned segment gives an ordinary rectangle, the bounds of the strip's corners.
        const glm::vec2 corners[] = {a - direction * extend + side * lateralLow, a - direction * extend + side * lateralHigh,
                                     b + direction * extend + side * lateralLow, b + direction * extend + side * lateralHigh};
        glm::vec2 low = corners[0];
        glm::vec2 high = corners[0];
        for (glm::vec2 corner : corners) {
            low = glm::min(low, corner);
            high = glm::max(high, corner);
        }
        return Rect{low, high};
    }
    // Otherwise the strip's own frame has +X along the segment, which is a yaw of atan2(-y, x).
    const glm::vec2 center = (a + b) * 0.5f + side * ((lateralLow + lateralHigh) * 0.5f);
    const glm::vec2 half(length * 0.5f + extend, (lateralHigh - lateralLow) * 0.5f);
    return {Rect{center - half, center + half}, std::atan2(-direction.y, direction.x)};
}

// The strip covered by a street segment widened by `halfWidth` on both sides.
OrientedRect StreetBand(const StreetGraph& streets, const StreetGraph::Edge& edge, float halfWidth) {
    return SegmentStrip(streets.nodes[edge.from], streets.nodes[edge.to], -halfWidth, halfWidth, halfWidth);
}

// The smallest axis-aligned rectangle, in the frame of `frame`, around `shape`.
Rect BoundsInFrameOf(const OrientedRect& frame, const OrientedRect& shape) {
    glm::vec2 low(INFINITY);
    glm::vec2 high(-INFINITY);
    for (glm::vec2 corner : shape.Corners()) {
        const glm::vec2 local = frame.ToFrame(corner);
        low = glm::min(low, local);
        high = glm::max(high, local);
    }
    return {low, high};
}

bool IsBuildingPart(CityPart part) { return part == CityPart::Building || part == CityPart::Landmark; }

// A uniform grid of box indices, so tests can find the boxes near a point quickly.
class BoxGrid {
public:
    BoxGrid(const Rect& area, float cellSize) : area(area), cellSize(cellSize) {
        columns = static_cast<int>(std::ceil(area.Size().x / cellSize)) + 1;
        rows = static_cast<int>(std::ceil(area.Size().y / cellSize)) + 1;
        cells.resize(static_cast<size_t>(columns) * static_cast<size_t>(rows));
    }

    void Insert(uint32_t index, const OrientedRect& turnedFootprint) {
        const Rect footprint = turnedFootprint.Bounds();
        const int x0 = Column(footprint.min.x);
        const int x1 = Column(footprint.max.x);
        const int z0 = Row(footprint.min.y);
        const int z1 = Row(footprint.max.y);
        for (int z = z0; z <= z1; ++z) {
            for (int x = x0; x <= x1; ++x) {
                cells[Cell(x, z)].push_back(index);
            }
        }
    }

    // Calls `visit` for every box inserted into a cell that `region` touches. A box can be
    // visited more than once.
    template <typename Visitor>
    void Visit(const Rect& region, Visitor&& visit) const {
        for (int z = Row(region.min.y); z <= Row(region.max.y); ++z) {
            for (int x = Column(region.min.x); x <= Column(region.max.x); ++x) {
                for (uint32_t index : cells[Cell(x, z)]) {
                    visit(index);
                }
            }
        }
    }

private:
    int Column(float x) const { return std::clamp(static_cast<int>((x - area.min.x) / cellSize), 0, columns - 1); }
    int Row(float z) const { return std::clamp(static_cast<int>((z - area.min.y) / cellSize), 0, rows - 1); }
    size_t Cell(int x, int z) const { return static_cast<size_t>(z) * static_cast<size_t>(columns) + static_cast<size_t>(x); }

    Rect area;
    float cellSize;
    int columns = 0;
    int rows = 0;
    std::vector<std::vector<uint32_t>> cells;
};

// Boxes sorted into two grids: walkable surfaces at pavement height, and everything that stands
// above the pavement low enough to get in someone's way.
struct CityIndex {
    BoxGrid pavements;
    BoxGrid obstacles;
};

CityIndex BuildIndex(const CityLayout& city) {
    const Rect area = city.bounds.Expanded(50.0f);
    CityIndex index{BoxGrid(area, 8.0f), BoxGrid(area, 8.0f)};
    for (uint32_t i = 0; i < city.boxes.size(); ++i) {
        const BoxInstance& box = city.boxes[i];
        const OrientedRect footprint = FootprintOf(box);
        if (std::abs(TopOf(box) - city.walkHeight) < kTolerance) {
            index.pavements.Insert(i, footprint);
        }
        const bool risesAbovePavement = TopOf(box) > city.walkHeight + 0.06f;
        const bool reachesDown = BottomOf(box) < city.walkHeight + kWalkingClearance;
        if (risesAbovePavement && reachesDown && footprint.Size().x < 600.0f && footprint.Size().y < 600.0f) {
            index.obstacles.Insert(i, footprint);
        }
    }
    return index;
}

bool IsOnPavement(const CityLayout& city, const CityIndex& index, glm::vec2 point) {
    bool found = false;
    index.pavements.Visit({point, point}, [&](uint32_t i) {
        found = found || ContainsWithTolerance(FootprintOf(city.boxes[i]), point);
    });
    return found;
}

// ---------------------------------------------------------------------------------------------
// Checks
// ---------------------------------------------------------------------------------------------

void CheckBoxes(const CityLayout& city) {
    Expect(city.boxes.size() == city.boxParts.size(), "boxes", "every box needs a part tag");
    Expect(city.boxes.size() < 150000, "boxes", Format("%.0f boxes is over the 150k budget", double(city.boxes.size())));
    for (size_t i = 0; i < city.boxes.size(); ++i) {
        const BoxInstance& box = city.boxes[i];
        const bool finite = std::isfinite(box.center.x) && std::isfinite(box.center.y) && std::isfinite(box.center.z);
        const bool positive = box.halfSize.x > 0.0f && box.halfSize.y > 0.0f && box.halfSize.z > 0.0f;
        const bool knownMaterial = box.material >= 0.0f && box.material <= static_cast<float>(SurfaceMaterial::Lamp);
        Expect(finite && positive && knownMaterial, "boxes",
               Format("box %.0f is malformed (half size %.3f %.3f %.3f)", double(i), box.halfSize.x, box.halfSize.y,
                      box.halfSize.z));
    }
}

void CheckStreetGraph(const CityLayout& city) {
    const StreetGraph& streets = city.streets;
    Expect(!streets.edges.empty(), "graph", "the city has no streets");
    Expect(streets.edgesAtNode.size() == streets.nodes.size(), "graph", "edgesAtNode must have one list per node");
    if (streets.edgesAtNode.size() != streets.nodes.size()) {
        return;
    }
    std::vector<int> degree(streets.nodes.size(), 0);
    for (uint32_t e = 0; e < streets.edges.size(); ++e) {
        const StreetGraph::Edge& edge = streets.edges[e];
        if (edge.from >= streets.nodes.size() || edge.to >= streets.nodes.size()) {
            Fail("graph", Format("edge %.0f refers to a missing node", e));
            continue;
        }
        const glm::vec2 a = streets.nodes[edge.from];
        const glm::vec2 b = streets.nodes[edge.to];
        const float length = glm::length(b - a);
        Expect(length > 0.5f, "graph", Format("edge %.0f is only %.3f m long", e, length));
        Expect(std::abs(length - edge.length) < kTolerance, "graph", Format("edge %.0f stores a wrong length", e));
        for (uint32_t node : {edge.from, edge.to}) {
            const auto& list = streets.edgesAtNode[node];
            Expect(std::find(list.begin(), list.end(), e) != list.end(), "graph",
                   Format("edge %.0f is missing from edgesAtNode of node %.0f", e, node));
            ++degree[node];
        }
    }
    for (size_t node = 0; node < streets.nodes.size(); ++node) {
        Expect(degree[node] > 0, "graph", Format("node %.0f is not used by any edge", double(node)));
        Expect(degree[node] == static_cast<int>(streets.edgesAtNode[node].size()), "graph",
               Format("edgesAtNode of node %.0f lists edges that do not touch it", double(node)));
    }

    // Every node must be reachable from the first one.
    std::vector<bool> reached(streets.nodes.size(), false);
    std::vector<uint32_t> pending = {0};
    reached[0] = true;
    size_t reachedCount = 1;
    while (!pending.empty()) {
        const uint32_t node = pending.back();
        pending.pop_back();
        for (uint32_t e : streets.edgesAtNode[node]) {
            const StreetGraph::Edge& edge = streets.edges[e];
            const uint32_t other = edge.from == node ? edge.to : edge.from;
            if (!reached[other]) {
                reached[other] = true;
                ++reachedCount;
                pending.push_back(other);
            }
        }
    }
    Expect(reachedCount == streets.nodes.size(), "graph",
           Format("only %.0f of %.0f nodes are connected", double(reachedCount), double(streets.nodes.size())));
}

// Walks both pavements of every street at the offsets the crowd uses and checks that there is
// a pavement surface underfoot and nothing in the way.
void CheckPavements(const CityLayout& city, const CityIndex& index) {
    const float roadHalf = city.roadWidth * 0.5f;
    const float spread = 0.35f * city.sidewalkWidth;  // the crowd varies its lane by this much
    const float offsets[] = {city.sidewalkCenterOffset - spread, city.sidewalkCenterOffset, city.sidewalkCenterOffset + spread};
    for (uint32_t e = 0; e < city.streets.edges.size(); ++e) {
        const StreetGraph::Edge& edge = city.streets.edges[e];
        const glm::vec2 a = city.streets.nodes[edge.from];
        const glm::vec2 b = city.streets.nodes[edge.to];
        const glm::vec2 direction = (b - a) / edge.length;
        const glm::vec2 side{-direction.y, direction.x};
        // Near the ends the pavement crosses the road of the side street, which is expected.
        for (float along = roadHalf + 0.5f; along <= edge.length - roadHalf - 0.5f; along += 1.5f) {
            for (float sign : {-1.0f, 1.0f}) {
                for (float offset : offsets) {
                    const glm::vec2 point = a + direction * along + side * (sign * offset);
                    if (!IsOnPavement(city, index, point)) {
                        Fail("pavement", Format("no pavement under (%.1f, %.1f) beside edge %.0f", point.x, point.y, e));
                    }
                }
            }
        }

        // The whole walking band on each side must be free of obstacles.
        for (float sign : {-1.0f, 1.0f}) {
            const float inner = sign * offsets[0];
            const float outer = sign * offsets[2];
            const OrientedRect band = SegmentStrip(a, b, std::min(inner, outer), std::max(inner, outer), 0.0f);
            index.obstacles.Visit(band.Bounds(), [&](uint32_t i) {
                const BoxInstance& box = city.boxes[i];
                if (OverlapsBeyondTolerance(FootprintOf(box), band)) {
                    Fail("pavement", Format("a box at (%.1f, %.1f) blocks the pavement of edge %.0f", box.center.x,
                                            box.center.z, e) +
                                         " [" + CityPartName(city.boxParts[i]) + "]");
                }
            });
        }
    }
}

// Buildings must keep out of streets, pavements, squares and water, except for parts that
// project overhead with enough clearance.
void CheckBuildingsKeepClear(const CityLayout& city) {
    const float streetHalf = city.roadWidth * 0.5f + city.sidewalkWidth;
    BoxGrid streetBands(city.bounds.Expanded(50.0f), 16.0f);
    std::vector<OrientedRect> bands;
    for (const StreetGraph::Edge& edge : city.streets.edges) {
        bands.push_back(StreetBand(city.streets, edge, streetHalf));
        streetBands.Insert(static_cast<uint32_t>(bands.size() - 1), bands.back());
    }
    for (size_t i = 0; i < city.boxes.size(); ++i) {
        if (!IsBuildingPart(city.boxParts[i])) {
            continue;
        }
        const BoxInstance& box = city.boxes[i];
        const OrientedRect footprint = FootprintOf(box);
        const bool overhead = BottomOf(box) >= kOverhangClearance;
        if (!overhead) {
            streetBands.Visit(footprint.Bounds(), [&](uint32_t band) {
                if (OverlapsBeyondTolerance(footprint, bands[band])) {
                    Fail("clear of streets", Format("building box at (%.1f, %.1f) stands in a street", box.center.x, box.center.z));
                }
            });
            for (const OrientedRect& plaza : city.plazas) {
                if (OverlapsBeyondTolerance(footprint, plaza)) {
                    Fail("clear of squares", Format("building box at (%.1f, %.1f) stands in a square", box.center.x, box.center.z));
                }
            }
        }
        for (const OrientedRect& water : city.water) {
            if (OverlapsBeyondTolerance(footprint, water)) {
                Fail("clear of water", Format("building box at (%.1f, %.1f) stands in the water", box.center.x, box.center.z));
            }
        }
    }
}

// People wander anywhere in a square except close to its middle, so nothing may stand in a
// square outside that keep-out area.
void CheckSquares(const CityLayout& city, const CityIndex& index) {
    for (const OrientedRect& plaza : city.plazas) {
        Expect(plaza.Size().x > 1.0f && plaza.Size().y > 1.0f, "squares", "a square is too small to walk in");
        bool cornersInBounds = true;
        for (glm::vec2 corner : plaza.Corners()) {
            cornersInBounds = cornersInBounds && city.bounds.Contains(corner);
        }
        Expect(cornersInBounds, "squares", "a square is out of bounds");
        // The keep-out area and the part of a box inside the square are worked out in the square's
        // own frame. For a box turned against the square its bounds there are slightly generous,
        // which can only make this check stricter.
        const glm::vec2 center = plaza.Center();
        const Rect keepOut{center - glm::vec2(kFountainKeepOut), center + glm::vec2(kFountainKeepOut)};
        index.obstacles.Visit(plaza.Bounds(), [&](uint32_t i) {
            const OrientedRect footprint = FootprintOf(city.boxes[i]);
            if (!OverlapsBeyondTolerance(footprint, plaza)) {
                return;
            }
            const Rect footprintInFrame = BoundsInFrameOf(plaza, footprint);
            const Rect insideSquare{glm::max(footprintInFrame.min, plaza.area.min),
                                    glm::min(footprintInFrame.max, plaza.area.max)};
            const bool withinKeepOut = insideSquare.min.x >= keepOut.min.x - kTolerance &&
                                       insideSquare.max.x <= keepOut.max.x + kTolerance &&
                                       insideSquare.min.y >= keepOut.min.y - kTolerance &&
                                       insideSquare.max.y <= keepOut.max.y + kTolerance;
            if (!withinKeepOut) {
                Fail("squares", Format("a box at (%.1f, %.1f) stands in a square away from its centre",
                                       city.boxes[i].center.x, city.boxes[i].center.z) +
                                    " [" + CityPartName(city.boxParts[i]) + "]");
            }
        });
    }
}

void CheckStreetLights(const CityLayout& city, const CityIndex& index) {
    Expect(!city.streetLights.empty(), "lamps", "the city has no street lamps");
    const float roadHalf = city.roadWidth * 0.5f;
    // Each street's road and pavement strips, built once instead of once per lamp.
    std::vector<OrientedRect> pavements;
    std::vector<OrientedRect> roads;
    for (const StreetGraph::Edge& edge : city.streets.edges) {
        pavements.push_back(StreetBand(city.streets, edge, roadHalf + city.sidewalkWidth));
        roads.push_back(StreetBand(city.streets, edge, roadHalf));
    }
    for (const glm::vec3& light : city.streetLights) {
        const glm::vec2 position{light.x, light.z};
        Expect(city.bounds.Contains(position), "lamps", Format("lamp at (%.1f, %.1f) is out of bounds", light.x, light.z));
        Expect(IsOnPavement(city, index, position), "lamps", Format("lamp at (%.1f, %.1f) is not on a pavement", light.x, light.z));
        Expect(light.y > 3.0f && light.y < 8.0f, "lamps", Format("lamp at (%.1f, %.1f) hangs at %.1f m", light.x, light.z, light.y));
        // The lamp must stand on the pavement of some street, not in its road.
        bool besideStreet = false;
        for (size_t edge = 0; edge < roads.size(); ++edge) {
            const OrientedRect& pavement = pavements[edge];
            const OrientedRect& road = roads[edge];
            if (pavement.Contains(position) && !road.Contains(position)) {
                besideStreet = true;
            }
            if (road.Expanded(-kTolerance).Contains(position)) {
                Fail("lamps", Format("lamp at (%.1f, %.1f) stands in a road", light.x, light.z));
            }
        }
        Expect(besideStreet, "lamps", Format("lamp at (%.1f, %.1f) is not beside a street", light.x, light.z));
    }
}

// Streets may only cross water on bridges, and every bridge must reach land on both banks.
void CheckWaterAndBridges(const CityLayout& city, bool expectRiver) {
    if (!expectRiver) {
        Expect(city.water.empty() && city.bridges.empty(), "water", "a district must have no water and no bridges");
    } else {
        Expect(!city.water.empty(), "water", "the city has no river");
        Expect(city.bridges.size() >= 2, "bridges", "the river needs at least two bridges");
    }
    std::vector<bool> isBridge(city.streets.edges.size(), false);
    for (const Bridge& bridge : city.bridges) {
        if (bridge.edge >= city.streets.edges.size()) {
            Fail("bridges", "a bridge refers to a missing street");
            continue;
        }
        isBridge[bridge.edge] = true;
        const StreetGraph::Edge& edge = city.streets.edges[bridge.edge];
        const glm::vec2 a = city.streets.nodes[edge.from];
        const glm::vec2 b = city.streets.nodes[edge.to];
        // Everything is measured in the deck's own frame, where the deck is axis-aligned.
        const glm::vec2 frameA = bridge.deck.ToFrame(a);
        const glm::vec2 frameB = bridge.deck.ToFrame(b);
        const bool alongX = std::abs(frameA.y - frameB.y) < kTolerance;
        bool spansWater = false;
        for (const OrientedRect& water : city.water) {
            if (!OverlapsBeyondTolerance(bridge.deck, water)) {
                continue;
            }
            // The deck must cover the water from bank to bank, and the street must start and
            // end on land beyond it.
            const Rect deck = bridge.deck.area;
            const Rect waterInFrame = BoundsInFrameOf(bridge.deck, water);
            const float deckLow = alongX ? deck.min.x : deck.min.y;
            const float deckHigh = alongX ? deck.max.x : deck.max.y;
            const float waterLow = alongX ? waterInFrame.min.x : waterInFrame.min.y;
            const float waterHigh = alongX ? waterInFrame.max.x : waterInFrame.max.y;
            const float streetLow = alongX ? std::min(frameA.x, frameB.x) : std::min(frameA.y, frameB.y);
            const float streetHigh = alongX ? std::max(frameA.x, frameB.x) : std::max(frameA.y, frameB.y);
            spansWater = deckLow <= waterLow + kTolerance && deckHigh >= waterHigh - kTolerance && streetLow < waterLow &&
                         streetHigh > waterHigh;
        }
        Expect(spansWater, "bridges", Format("the bridge on edge %.0f does not join both banks", bridge.edge));
        for (const OrientedRect& water : city.water) {
            Expect(!water.Contains(a) && !water.Contains(b), "bridges", Format("edge %.0f ends in the water", bridge.edge));
        }
    }
    const float streetHalf = city.roadWidth * 0.5f + city.sidewalkWidth;
    for (uint32_t e = 0; e < city.streets.edges.size(); ++e) {
        if (isBridge[e]) {
            continue;
        }
        const OrientedRect band = StreetBand(city.streets, city.streets.edges[e], streetHalf);
        for (const OrientedRect& water : city.water) {
            Expect(!OverlapsBeyondTolerance(band, water), "water", Format("street edge %.0f runs into the water", e));
        }
    }
}

void CheckDeterminism(const CitySettings& settings, const CityLayout& first) {
    const CityLayout second = GenerateCity(settings);
    const bool sameBoxes = first.boxes.size() == second.boxes.size() &&
                           std::memcmp(first.boxes.data(), second.boxes.data(), first.boxes.size() * sizeof(BoxInstance)) == 0;
    const bool sameLights = first.streetLights == second.streetLights;
    const bool sameNodes = first.streets.nodes == second.streets.nodes;
    Expect(sameBoxes && sameLights && sameNodes && first.boxParts == second.boxParts, "determinism",
           "the same seed produced a different city");

    CitySettings other = settings;
    other.seed = settings.seed + 1;
    const CityLayout different = GenerateCity(other);
    const bool differs = different.boxes.size() != first.boxes.size() ||
                         std::memcmp(different.boxes.data(), first.boxes.data(), first.boxes.size() * sizeof(BoxInstance)) != 0;
    Expect(differs, "determinism", "different seeds produced the same city");
}

// ---------------------------------------------------------------------------------------------
// Summary
// ---------------------------------------------------------------------------------------------

void PrintSummary(const CityLayout& city, double milliseconds) {
    const glm::vec2 size = city.bounds.Size();
    std::printf("  %.0f x %.0f m, %zu boxes, %zu street edges, %zu squares, %zu parks, %zu bridges, %zu lamps, %zu trees, "
                "generated in %.0f ms\n",
                double(size.x), double(size.y), city.boxes.size(), city.streets.edges.size(), city.plazas.size(),
                city.parks.size(), city.bridges.size(), city.streetLights.size(), city.trees.size(), milliseconds);

    std::map<CityPart, size_t> boxesPerPart;
    for (CityPart part : city.boxParts) {
        ++boxesPerPart[part];
    }
    std::printf("  boxes:");
    for (const auto& [part, count] : boxesPerPart) {
        std::printf(" %s %zu,", CityPartName(part), count);
    }
    std::printf("\n");

    std::map<District, size_t> blocksPerDistrict;
    size_t buildingBlocks = 0;
    for (const CityBlock& block : city.blocks) {
        ++blocksPerDistrict[block.district];
        buildingBlocks += block.use == BlockUse::Buildings ? 1 : 0;
    }
    std::printf("  blocks:");
    for (const auto& [district, count] : blocksPerDistrict) {
        std::printf(" %s %zu,", DistrictName(district), count);
    }
    std::printf(" (%zu with houses)\n", buildingBlocks);

    static const char* kLandmarkNames[] = {"cathedral", "palace", "market hall", "clock tower", "parish church"};
    std::printf("  landmarks:");
    for (const Landmark& landmark : city.landmarks) {
        std::printf(" %s,", kLandmarkNames[static_cast<int>(landmark.kind)]);
    }
    std::printf("\n");
}

// ---------------------------------------------------------------------------------------------
// Map
// ---------------------------------------------------------------------------------------------

uint8_t ToSrgbByte(float linear) {
    const float clamped = std::clamp(linear, 0.0f, 1.0f);
    const float srgb = clamped <= 0.0031308f ? clamped * 12.92f : 1.055f * std::pow(clamped, 1.0f / 2.4f) - 0.055f;
    return static_cast<uint8_t>(std::lround(srgb * 255.0f));
}

struct MapView {
    Rect window;
    float pixelsPerMetre;

    int X(float x) const { return static_cast<int>(std::floor((x - window.min.x) * pixelsPerMetre)); }
    int Y(float z) const { return static_cast<int>(std::floor((window.max.y - z) * pixelsPerMetre)); }  // north is up
    // The world point at the middle of pixel (x, y).
    glm::vec2 WorldAt(int x, int y) const {
        return {window.min.x + (static_cast<float>(x) + 0.5f) / pixelsPerMetre,
                window.max.y - (static_cast<float>(y) + 0.5f) / pixelsPerMetre};
    }
};

// Draws box footprints from the lowest top to the highest, coloured by each box's albedo and
// brightened with height so tall buildings and towers stand out.
void DrawBoxes(RgbImage& image, const MapView& view, const CityLayout& city) {
    std::vector<uint32_t> order(city.boxes.size());
    for (uint32_t i = 0; i < order.size(); ++i) {
        order[i] = i;
    }
    std::stable_sort(order.begin(), order.end(),
                     [&](uint32_t a, uint32_t b) { return TopOf(city.boxes[a]) < TopOf(city.boxes[b]); });
    for (uint32_t i : order) {
        const BoxInstance& box = city.boxes[i];
        const float heightShade = 0.75f + 0.5f * std::clamp(TopOf(box) / 40.0f, 0.0f, 1.0f);
        const glm::vec3 color = box.color * heightShade;
        const OrientedRect footprint = FootprintOf(box);
        const Rect bounds = footprint.Bounds();
        const int x0 = view.X(bounds.min.x);
        const int y0 = view.Y(bounds.max.y);
        const int x1 = view.X(bounds.max.x) - 1;
        const int y1 = view.Y(bounds.min.y) - 1;
        const uint8_t red = ToSrgbByte(color.r);
        const uint8_t green = ToSrgbByte(color.g);
        const uint8_t blue = ToSrgbByte(color.b);
        if (footprint.yaw == 0.0f) {
            image.FillRect(x0, y0, x1, y1, red, green, blue);
            continue;
        }
        // A turned box covers the pixels whose centres lie inside it.
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                if (footprint.Contains(view.WorldAt(x, y))) {
                    image.Set(x, y, red, green, blue);
                }
            }
        }
    }
}

void DrawStreetGraph(RgbImage& image, const MapView& view, const CityLayout& city) {
    for (const StreetGraph::Edge& edge : city.streets.edges) {
        const glm::vec2 a = city.streets.nodes[edge.from];
        const glm::vec2 b = city.streets.nodes[edge.to];
        image.DrawLine(view.X(a.x), view.Y(a.y), view.X(b.x), view.Y(b.y), 220, 40, 40);
    }
    for (const Bridge& bridge : city.bridges) {
        const StreetGraph::Edge& edge = city.streets.edges[bridge.edge];
        const glm::vec2 a = city.streets.nodes[edge.from];
        const glm::vec2 b = city.streets.nodes[edge.to];
        image.DrawLine(view.X(a.x), view.Y(a.y), view.X(b.x), view.Y(b.y), 255, 160, 0);
    }
    for (const OrientedRect& plaza : city.plazas) {
        // The outline follows the square's corners, so a turned square is drawn turned.
        const std::array<glm::vec2, 4> corners = plaza.Corners();
        for (size_t i = 0; i < corners.size(); ++i) {
            const glm::vec2 from = corners[i];
            const glm::vec2 to = corners[(i + 1) % corners.size()];
            image.DrawLine(view.X(from.x), view.Y(from.y), view.X(to.x), view.Y(to.y), 40, 90, 230);
        }
    }
    for (const glm::vec3& light : city.streetLights) {
        image.FillRect(view.X(light.x) - 1, view.Y(light.z) - 1, view.X(light.x) + 1, view.Y(light.z) + 1, 255, 230, 60);
    }
}

bool WriteMaps(const CityLayout& city, const std::string& path, const Rect& window, float pixelsPerMetre) {
    const MapView view{window, pixelsPerMetre};
    const int width = static_cast<int>(std::ceil(window.Size().x * pixelsPerMetre));
    const int height = static_cast<int>(std::ceil(window.Size().y * pixelsPerMetre));
    RgbImage image(width, height, 20, 20, 20);
    DrawBoxes(image, view, city);
    bool saved = image.SavePng(path);

    // A second copy with the street graph, squares and lamps drawn on top.
    DrawStreetGraph(image, view, city);
    const size_t dot = path.find_last_of('.');
    const std::string overlayPath = (dot == std::string::npos ? path : path.substr(0, dot)) + "_graph.png";
    saved = image.SavePng(overlayPath) && saved;
    std::printf("map written to %s and %s (%d x %d pixels)\n", path.c_str(), overlayPath.c_str(), width, height);
    return saved;
}

// ---------------------------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------------------------

struct Options {
    std::string mapPath;
    uint32_t mapSeed = CitySettings{}.seed;
    int mapSatellites = -1;  // negative keeps the generator's default
    bool hasWindow = false;
    Rect window;
    float pixelsPerMetre = 1.0f;
};

Options ParseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        const bool hasValue = i + 1 < argc;
        if (argument == "--map" && hasValue) {
            options.mapPath = argv[++i];
        } else if (argument == "--seed" && hasValue) {
            options.mapSeed = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
        } else if (argument == "--satellites" && hasValue) {
            options.mapSatellites = std::atoi(argv[++i]);
        } else if (argument == "--scale" && hasValue) {
            options.pixelsPerMetre = std::strtof(argv[++i], nullptr);
        } else if (argument == "--window" && i + 4 < argc) {
            options.hasWindow = true;
            options.window.min = {std::strtof(argv[i + 1], nullptr), std::strtof(argv[i + 2], nullptr)};
            options.window.max = {std::strtof(argv[i + 3], nullptr), std::strtof(argv[i + 4], nullptr)};
            i += 4;
        } else {
            std::printf("unknown or incomplete option %s\n", argument.c_str());
        }
    }
    return options;
}

CityLayout GenerateTimed(const CitySettings& settings, double& milliseconds) {
    const auto start = std::chrono::steady_clock::now();
    CityLayout city = GenerateCity(settings);
    milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return city;
}

// The crowd places pavement corners where two streets meet, and it stops being able to when two
// streets leave a node at nearly the same angle, so no two arms may be closer than this.
constexpr float kMinJunctionAngle = 0.5235988f;  // 30 degrees

void CheckJunctionAngles(const CityLayout& city) {
    const StreetGraph& streets = city.streets;
    for (size_t node = 0; node < streets.nodes.size(); ++node) {
        std::vector<float> angles;
        for (uint32_t e : streets.edgesAtNode[node]) {
            const StreetGraph::Edge& edge = streets.edges[e];
            const glm::vec2 other = streets.nodes[edge.from == node ? edge.to : edge.from];
            const glm::vec2 direction = other - streets.nodes[node];
            angles.push_back(std::atan2(direction.y, direction.x));
        }
        std::sort(angles.begin(), angles.end());
        for (size_t i = 0; i + 1 < angles.size() + (angles.size() > 2 ? 1 : 0); ++i) {
            // Consecutive arms counter-clockwise, wrapping around from the last to the first.
            const float next = i + 1 < angles.size() ? angles[i + 1] : angles[0] + 6.2831855f;
            Expect(next - angles[i] >= kMinJunctionAngle - 1e-4f, "junctions",
                   Format("two streets leave node %.0f at only %.1f degrees", double(node), double(next - angles[i]) * 57.29578));
        }
    }
}

void RunContractChecks(const CityLayout& city, bool expectRiver) {
    const CityIndex index = BuildIndex(city);
    CheckBoxes(city);
    CheckStreetGraph(city);
    CheckJunctionAngles(city);
    CheckPavements(city, index);
    CheckBuildingsKeepClear(city);
    CheckSquares(city, index);
    CheckStreetLights(city, index);
    CheckWaterAndBridges(city, expectRiver);
}

void TestSeed(uint32_t seed, bool checkDeterminism, bool districtOnly = false, int satellites = 0) {
    std::printf("seed %u%s", seed, districtOnly ? " (district only)" : "");
    if (satellites > 0) {
        std::printf(" with %d satellites", satellites);
    }
    std::printf("\n");
    CitySettings settings;
    settings.seed = seed;
    settings.districtOnly = districtOnly;
    settings.satelliteCount = satellites;
    if (districtOnly) {
        settings.citySize = 600.0f;
    }
    double milliseconds = 0.0;
    const CityLayout city = GenerateTimed(settings, milliseconds);
    // The summary shows how long generation took. It is not checked against a limit: wall-clock
    // time depends on the build type and on how busy the machine is, not on the generator's logic.
    PrintSummary(city, milliseconds);

    RunContractChecks(city, !districtOnly);
    if (checkDeterminism) {
        CheckDeterminism(settings, city);
    }
}

// A district built on its own, then turned and moved into a city through AppendPart. The whole
// contract must still hold, which proves the checks and the transform on turned geometry.
void TestPlacedDistrict(uint32_t seed, const city::Placement& placement) {
    std::printf("district %u placed at (%.0f, %.0f) turned %.2f rad\n", seed, double(placement.offset.x),
                double(placement.offset.y), double(placement.yaw));
    CitySettings settings;
    settings.seed = seed;
    settings.districtOnly = true;
    settings.citySize = 600.0f;
    const CityLayout district = GenerateCity(settings);

    CityLayout city;
    city.roadWidth = district.roadWidth;
    city.sidewalkWidth = district.sidewalkWidth;
    city.sidewalkCenterOffset = district.sidewalkCenterOffset;
    city.walkHeight = district.walkHeight;
    city.waterLevel = district.waterLevel;
    city::AppendPart(city, district, placement);
    PrintSummary(city, 0.0);

    Expect(city.boxes.size() == district.boxes.size(), "compose", "every box must be carried over");
    Expect(city.streets.nodes.size() == district.streets.nodes.size(), "compose", "every node must be carried over");
    for (size_t i = 0; i < district.streets.edges.size(); ++i) {
        Expect(std::abs(city.streets.edges[i].length - district.streets.edges[i].length) < kTolerance, "compose",
               "turning a street must not change its length");
    }
    Expect(city.bounds.Contains(placement.Apply(glm::vec2(0.0f))), "compose", "the bounds must hold the part");
    RunContractChecks(city, false);

    // The checks must notice when the geometry stops matching the street graph.
    CityLayout sideways = city;
    for (BoxInstance& box : sideways.boxes) {
        box.center.x += 3.0f;
    }
    Expect(CountFailuresOf([&] { RunContractChecks(sideways, false); }) > 0, "compose",
           "the checks did not notice boxes moved 3 m away from the streets");
    CityLayout twisted = city;
    for (BoxInstance& box : twisted.boxes) {
        box.yaw += 0.5f;
    }
    Expect(CountFailuresOf([&] { RunContractChecks(twisted, false); }) > 0, "compose",
           "the checks did not notice boxes turned away from the streets");
}

}  // namespace

// The checks above rely on the helpers that measure strips, footprints and overlaps. Checking the
// helpers on shapes whose answer is known keeps a mistake in them from hiding a real problem.
void CheckHelpers() {
    // A street running diagonally, from the origin towards (+X, -Z).
    const glm::vec2 a{0.0f, 0.0f};
    const glm::vec2 b{30.0f, -30.0f};
    const OrientedRect band = StreetBand(StreetGraph{{a, b}, {{0, 1, glm::length(b - a)}}, {{0}, {0}}}, {0, 1, glm::length(b - a)}, 5.0f);
    Expect(band.Contains({15.0f, -15.0f}), "helpers", "a diagonal street band must contain its centre line");
    Expect(band.Contains({18.0f, -12.0f}), "helpers", "a diagonal street band must contain points within its width");
    Expect(!band.Contains({15.0f, 0.0f}), "helpers", "a diagonal street band must not contain a point 10 m off its centre line");
    Expect(!band.Contains({-9.0f, 9.0f}), "helpers", "a diagonal street band must stop past its ends");

    // The same strip on one side only.
    const OrientedRect side = SegmentStrip(a, b, 2.0f, 4.0f, 0.0f);
    const glm::vec2 left = glm::normalize(glm::vec2(1.0f, 1.0f));  // to the left of the direction a to b
    Expect(side.Contains(glm::vec2(15.0f, -15.0f) + left * 3.0f), "helpers", "a one sided strip must contain its own side");
    Expect(!side.Contains(glm::vec2(15.0f, -15.0f) - left * 3.0f), "helpers", "a one sided strip must not contain the other side");

    // Overlap between turned boxes looks at the turn, not at the bounding boxes.
    BoxInstance first{};
    first.center = {0.0f, 1.0f, 0.0f};
    first.halfSize = {10.0f, 1.0f, 1.0f};
    first.yaw = 0.7853982f;
    BoxInstance second = first;
    second.center = {2.2f, 1.0f, 2.2f};  // 3.1 m to the side of the first: a gap of 1.1 m
    Expect(!OverlapsBeyondTolerance(FootprintOf(first), FootprintOf(second)), "helpers", "parallel diagonal boxes with a gap must not overlap");
    second.center = {1.1f, 1.0f, 1.1f};  // 1.6 m to the side: they overlap
    Expect(OverlapsBeyondTolerance(FootprintOf(first), FootprintOf(second)), "helpers", "overlapping diagonal boxes must overlap");
}

int main(int argc, char** argv) {
    const Options options = ParseOptions(argc, argv);
    if (!options.mapPath.empty()) {
        CitySettings settings;
        settings.seed = options.mapSeed;
        if (options.mapSatellites >= 0) {
            settings.satelliteCount = options.mapSatellites;
        }
        const CityLayout city = GenerateCity(settings);
        const Rect window = options.hasWindow ? options.window : city.bounds.Expanded(20.0f);
        return WriteMaps(city, options.mapPath, window, options.pixelsPerMetre) ? 0 : 1;
    }

    CheckHelpers();
    const uint32_t seeds[] = {CitySettings{}.seed, 1u, 2u, 7u, 42u, 2024u};
    for (size_t i = 0; i < std::size(seeds); ++i) {
        TestSeed(seeds[i], i == 0);
    }
    // A district on its own, as a satellite of a larger city is built.
    TestSeed(5u, true, true);
    TestSeed(99u, false, true);
    // The city with satellite districts turned to their own angles and joined by streets.
    TestSeed(CitySettings{}.seed, true, false, 4);
    TestSeed(1u, false, false, 3);
    TestSeed(42u, false, false, 4);
    TestPlacedDistrict(5u, {{0.0f, 0.0f}, 0.0f});
    TestPlacedDistrict(5u, {{1234.0f, -567.0f}, 0.6f});
    TestPlacedDistrict(99u, {{-900.0f, 700.0f}, -1.1f});
    TestPlacedDistrict(7u, {{300.0f, 2000.0f}, 2.4f});
    if (totalFailures == 0) {
        std::printf("all city checks passed\n");
        return 0;
    }
    std::printf("%d city checks failed\n", totalFailures);
    for (const auto& [check, count] : failuresPerCheck) {
        std::printf("  %s: %d\n", check.c_str(), count);
    }
    return 1;
}
