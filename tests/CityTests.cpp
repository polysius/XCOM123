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
#include "world/CityGenerator.h"

namespace {

// ---------------------------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------------------------

int totalFailures = 0;
std::map<std::string, int> failuresPerCheck;

// Records a failed check. Only the first few failures of each kind are printed, so one broken
// rule does not flood the output.
void Fail(const std::string& check, const std::string& detail) {
    ++totalFailures;
    const int count = ++failuresPerCheck[check];
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

Rect FootprintOf(const BoxInstance& box) {
    return {{box.center.x - box.halfSize.x, box.center.z - box.halfSize.z},
            {box.center.x + box.halfSize.x, box.center.z + box.halfSize.z}};
}

float BottomOf(const BoxInstance& box) { return box.center.y - box.halfSize.y; }
float TopOf(const BoxInstance& box) { return box.center.y + box.halfSize.y; }

// True when the two rectangles share more than a sliver of area.
bool OverlapsBeyondTolerance(const Rect& a, const Rect& b) {
    return a.min.x < b.max.x - kTolerance && b.min.x < a.max.x - kTolerance && a.min.y < b.max.y - kTolerance &&
           b.min.y < a.max.y - kTolerance;
}

bool ContainsWithTolerance(const Rect& rect, glm::vec2 point) {
    return point.x >= rect.min.x - kTolerance && point.x <= rect.max.x + kTolerance && point.y >= rect.min.y - kTolerance &&
           point.y <= rect.max.y + kTolerance;
}

// The rectangle covered by a street segment widened by `halfWidth` on both sides.
Rect StreetBand(const StreetGraph& streets, const StreetGraph::Edge& edge, float halfWidth) {
    const glm::vec2 a = streets.nodes[edge.from];
    const glm::vec2 b = streets.nodes[edge.to];
    return Rect{glm::min(a, b), glm::max(a, b)}.Expanded(halfWidth);
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

    void Insert(uint32_t index, const Rect& footprint) {
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
        const Rect footprint = FootprintOf(box);
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
        Expect(a.x == b.x || a.y == b.y, "graph", Format("edge %.0f is not axis-aligned", e));
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
            const glm::vec2 inner = side * (sign * offsets[0]);
            const glm::vec2 outer = side * (sign * offsets[2]);
            const Rect band = Rect{glm::min(a + inner, b + outer), glm::max(a + inner, b + outer)};
            index.obstacles.Visit(band, [&](uint32_t i) {
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
    std::vector<Rect> bands;
    for (const StreetGraph::Edge& edge : city.streets.edges) {
        bands.push_back(StreetBand(city.streets, edge, streetHalf));
        streetBands.Insert(static_cast<uint32_t>(bands.size() - 1), bands.back());
    }
    for (size_t i = 0; i < city.boxes.size(); ++i) {
        if (!IsBuildingPart(city.boxParts[i])) {
            continue;
        }
        const BoxInstance& box = city.boxes[i];
        const Rect footprint = FootprintOf(box);
        const bool overhead = BottomOf(box) >= kOverhangClearance;
        if (!overhead) {
            streetBands.Visit(footprint, [&](uint32_t band) {
                if (OverlapsBeyondTolerance(footprint, bands[band])) {
                    Fail("clear of streets", Format("building box at (%.1f, %.1f) stands in a street", box.center.x, box.center.z));
                }
            });
            for (const Rect& plaza : city.plazas) {
                if (OverlapsBeyondTolerance(footprint, plaza)) {
                    Fail("clear of squares", Format("building box at (%.1f, %.1f) stands in a square", box.center.x, box.center.z));
                }
            }
        }
        for (const Rect& water : city.water) {
            if (OverlapsBeyondTolerance(footprint, water)) {
                Fail("clear of water", Format("building box at (%.1f, %.1f) stands in the water", box.center.x, box.center.z));
            }
        }
    }
}

// People wander anywhere in a square except close to its middle, so nothing may stand in a
// square outside that keep-out area.
void CheckSquares(const CityLayout& city, const CityIndex& index) {
    for (const Rect& plaza : city.plazas) {
        Expect(plaza.Size().x > 1.0f && plaza.Size().y > 1.0f, "squares", "a square is too small to walk in");
        Expect(city.bounds.Contains(plaza.min) && city.bounds.Contains(plaza.max), "squares", "a square is out of bounds");
        const glm::vec2 center = plaza.Center();
        const Rect keepOut{center - glm::vec2(kFountainKeepOut), center + glm::vec2(kFountainKeepOut)};
        index.obstacles.Visit(plaza, [&](uint32_t i) {
            const Rect footprint = FootprintOf(city.boxes[i]);
            if (!OverlapsBeyondTolerance(footprint, plaza)) {
                return;
            }
            const Rect insideSquare{glm::max(footprint.min, plaza.min), glm::min(footprint.max, plaza.max)};
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
    for (const glm::vec3& light : city.streetLights) {
        const glm::vec2 position{light.x, light.z};
        Expect(city.bounds.Contains(position), "lamps", Format("lamp at (%.1f, %.1f) is out of bounds", light.x, light.z));
        Expect(IsOnPavement(city, index, position), "lamps", Format("lamp at (%.1f, %.1f) is not on a pavement", light.x, light.z));
        Expect(light.y > 3.0f && light.y < 8.0f, "lamps", Format("lamp at (%.1f, %.1f) hangs at %.1f m", light.x, light.z, light.y));
        // The lamp must stand on the pavement of some street, not in its road.
        bool besideStreet = false;
        for (const StreetGraph::Edge& edge : city.streets.edges) {
            const Rect pavement = StreetBand(city.streets, edge, roadHalf + city.sidewalkWidth);
            const Rect road = StreetBand(city.streets, edge, roadHalf);
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
void CheckWaterAndBridges(const CityLayout& city) {
    Expect(!city.water.empty(), "water", "the city has no river");
    Expect(city.bridges.size() >= 2, "bridges", "the river needs at least two bridges");
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
        const bool alongX = a.y == b.y;
        bool spansWater = false;
        for (const Rect& water : city.water) {
            if (!OverlapsBeyondTolerance(bridge.deck, water)) {
                continue;
            }
            // The deck must cover the water from bank to bank, and the street must start and
            // end on land beyond it.
            const float deckLow = alongX ? bridge.deck.min.x : bridge.deck.min.y;
            const float deckHigh = alongX ? bridge.deck.max.x : bridge.deck.max.y;
            const float waterLow = alongX ? water.min.x : water.min.y;
            const float waterHigh = alongX ? water.max.x : water.max.y;
            const float streetLow = alongX ? std::min(a.x, b.x) : std::min(a.y, b.y);
            const float streetHigh = alongX ? std::max(a.x, b.x) : std::max(a.y, b.y);
            spansWater = deckLow <= waterLow + kTolerance && deckHigh >= waterHigh - kTolerance && streetLow < waterLow &&
                         streetHigh > waterHigh;
        }
        Expect(spansWater, "bridges", Format("the bridge on edge %.0f does not join both banks", bridge.edge));
        for (const Rect& water : city.water) {
            Expect(!water.Contains(a) && !water.Contains(b), "bridges", Format("edge %.0f ends in the water", bridge.edge));
        }
    }
    const float streetHalf = city.roadWidth * 0.5f + city.sidewalkWidth;
    for (uint32_t e = 0; e < city.streets.edges.size(); ++e) {
        if (isBridge[e]) {
            continue;
        }
        const Rect band = StreetBand(city.streets, city.streets.edges[e], streetHalf);
        for (const Rect& water : city.water) {
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
        const Rect footprint = FootprintOf(box);
        image.FillRect(view.X(footprint.min.x), view.Y(footprint.max.y), view.X(footprint.max.x) - 1,
                       view.Y(footprint.min.y) - 1, ToSrgbByte(color.r), ToSrgbByte(color.g), ToSrgbByte(color.b));
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
    for (const Rect& plaza : city.plazas) {
        const int x0 = view.X(plaza.min.x);
        const int x1 = view.X(plaza.max.x);
        const int y0 = view.Y(plaza.max.y);
        const int y1 = view.Y(plaza.min.y);
        image.DrawLine(x0, y0, x1, y0, 40, 90, 230);
        image.DrawLine(x1, y0, x1, y1, 40, 90, 230);
        image.DrawLine(x1, y1, x0, y1, 40, 90, 230);
        image.DrawLine(x0, y1, x0, y0, 40, 90, 230);
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

void TestSeed(uint32_t seed, bool checkDeterminism) {
    std::printf("seed %u\n", seed);
    CitySettings settings;
    settings.seed = seed;
    double milliseconds = 0.0;
    const CityLayout city = GenerateTimed(settings, milliseconds);
    PrintSummary(city, milliseconds);
    // Generation should take well under a second; the limit is loose because the machine may be busy.
    Expect(milliseconds < 3000.0, "speed", Format("generation took %.0f ms", milliseconds));

    const CityIndex index = BuildIndex(city);
    CheckBoxes(city);
    CheckStreetGraph(city);
    CheckPavements(city, index);
    CheckBuildingsKeepClear(city);
    CheckSquares(city, index);
    CheckStreetLights(city, index);
    CheckWaterAndBridges(city);
    if (checkDeterminism) {
        CheckDeterminism(settings, city);
    }
}

}  // namespace

int main(int argc, char** argv) {
    const Options options = ParseOptions(argc, argv);
    if (!options.mapPath.empty()) {
        CitySettings settings;
        settings.seed = options.mapSeed;
        const CityLayout city = GenerateCity(settings);
        const Rect window = options.hasWindow ? options.window : city.bounds.Expanded(20.0f);
        return WriteMaps(city, options.mapPath, window, options.pixelsPerMetre) ? 0 : 1;
    }

    const uint32_t seeds[] = {CitySettings{}.seed, 1u, 2u, 7u, 42u, 2024u};
    for (size_t i = 0; i < std::size(seeds); ++i) {
        TestSeed(seeds[i], i == 0);
    }
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
