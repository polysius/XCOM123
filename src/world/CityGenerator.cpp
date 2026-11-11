#include "world/CityGenerator.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <random>

namespace {
// Floor heights. shaders/lit.frag uses the same values to place the windows.
constexpr float kGroundFloorHeight = 4.5f;
constexpr float kUpperFloorHeight = 3.3f;
constexpr float kPavementHeight = 0.15f;
constexpr float kMaxLotSize = 24.0f;    // blocks are split until no building lot is wider than this
constexpr float kPavedSurface = 1.0f;   // seed value that tells lit.frag to draw paving slabs

// Pale limestone and render colours, typical of old European city centres.
const glm::vec3 kFacadeColors[] = {
    {0.80f, 0.74f, 0.63f}, {0.86f, 0.82f, 0.74f}, {0.74f, 0.69f, 0.61f},
    {0.79f, 0.71f, 0.58f}, {0.84f, 0.79f, 0.70f}, {0.70f, 0.63f, 0.54f},
};
const glm::vec3 kRoofSlate(0.20f, 0.23f, 0.28f);
const glm::vec3 kChimneyBrick(0.42f, 0.27f, 0.21f);
const glm::vec3 kAsphalt(0.085f, 0.085f, 0.09f);
const glm::vec3 kPaving(0.50f, 0.48f, 0.45f);
const glm::vec3 kPlazaStone(0.60f, 0.56f, 0.50f);
const glm::vec3 kWater(0.03f, 0.06f, 0.08f);

// Random numbers built only on std::mt19937, whose output the C++ standard fixes exactly.
// (The standard distributions may differ between compilers, which would change the city.)
class Random {
public:
    explicit Random(uint32_t seed) : engine(seed) {}

    float Unit() { return static_cast<float>(engine() >> 8) * (1.0f / 16777216.0f); }  // 0 to just below 1
    float Range(float min, float max) { return min + (max - min) * Unit(); }
    int Int(int min, int max) { return min + static_cast<int>(engine() % static_cast<uint32_t>(max - min + 1)); }  // inclusive
    bool Chance(float probability) { return Unit() < probability; }

private:
    std::mt19937 engine;
};

BoxInstance MakeBox(glm::vec3 min, glm::vec3 max, glm::vec3 color, SurfaceMaterial material, float roughness, float seed) {
    BoxInstance box;
    box.center = (min + max) * 0.5f;
    box.material = static_cast<float>(material);
    box.halfSize = (max - min) * 0.5f;
    box.roughness = roughness;
    box.color = color;
    box.seed = seed;
    return box;
}

// A box standing on a ground footprint, spanning heights `bottom` to `top`.
BoxInstance FootprintBox(const Rect& footprint, float bottom, float top, glm::vec3 color,
                         SurfaceMaterial material, float roughness, float seed = 0.0f) {
    return MakeBox({footprint.min.x, bottom, footprint.min.y}, {footprint.max.x, top, footprint.max.y},
                   color, material, roughness, seed);
}

// Street centre lines along one axis, centred on the origin, with randomly sized blocks between them.
std::vector<float> StreetCenterLines(Random& random, int blockCount, float minBlockSize, float maxBlockSize, float streetWidth) {
    std::vector<float> blockSizes(blockCount);
    for (float& size : blockSizes) {
        size = random.Range(minBlockSize, maxBlockSize);
    }
    const float totalWidth = std::accumulate(blockSizes.begin(), blockSizes.end(), 0.0f) + streetWidth * (blockCount + 1);

    std::vector<float> lines(blockCount + 1);
    lines[0] = -totalWidth * 0.5f + streetWidth * 0.5f;
    for (int i = 0; i < blockCount; ++i) {
        lines[i + 1] = lines[i] + streetWidth + blockSizes[i];
    }
    return lines;
}

// Recursively cuts a block into building lots, always across its longer side, until every lot
// is narrow enough to read as a single building.
void SplitIntoLots(const Rect& area, Random& random, std::vector<Rect>& lots) {
    const glm::vec2 size = area.Size();
    const bool cutAcrossX = size.x >= size.y;
    const float longSide = cutAcrossX ? size.x : size.y;
    if (longSide <= kMaxLotSize) {
        lots.push_back(area);
        return;
    }

    const float cutFraction = random.Range(0.35f, 0.65f);
    Rect first = area;
    Rect second = area;
    if (cutAcrossX) {
        const float cut = area.min.x + size.x * cutFraction;
        first.max.x = cut;
        second.min.x = cut;
    } else {
        const float cut = area.min.y + size.y * cutFraction;
        first.max.y = cut;
        second.min.y = cut;
    }
    SplitIntoLots(first, random, lots);
    SplitIntoLots(second, random, lots);
}

// One building: a facade block with a cornice, a slate mansard roof and a few chimneys.
void AddBuilding(CityLayout& city, const Rect& lot, float heightBias, Random& random) {
    int floors = static_cast<int>(std::round(random.Range(3.0f, 6.0f) * heightBias));
    floors = std::clamp(floors, 2, 9);
    if (random.Chance(0.012f)) {
        floors += random.Int(6, 12);  // the odd tower breaks up the skyline
    }

    const float base = kPavementHeight;
    const float top = base + kGroundFloorHeight + static_cast<float>(floors) * kUpperFloorHeight;
    const int colorIndex = random.Int(0, static_cast<int>(std::size(kFacadeColors)) - 1);
    const glm::vec3 color = kFacadeColors[colorIndex] * random.Range(0.94f, 1.06f);
    city.boxes.push_back(FootprintBox(lot, base, top, color, SurfaceMaterial::Facade, 0.85f, random.Unit()));

    // Cornice: a ledge overhanging the facade just under the roof, which casts a crisp shadow line.
    // Its size varies a little per building so neighbouring ledges never share a plane (z-fighting).
    const float overhang = random.Range(0.25f, 0.45f);
    const float corniceTop = top + random.Range(0.25f, 0.40f);
    city.boxes.push_back(FootprintBox(lot.Expanded(overhang), top - 0.15f, corniceTop, color * 1.04f,
                                      SurfaceMaterial::Plain, 0.7f));

    // Mansard roof: a dark slate block set back from the facade.
    const Rect roof = lot.Expanded(-random.Range(0.9f, 1.5f));
    const float roofTop = corniceTop + random.Range(2.0f, 3.4f);
    city.boxes.push_back(FootprintBox(roof, corniceTop, roofTop, kRoofSlate * random.Range(0.85f, 1.15f),
                                      SurfaceMaterial::Plain, 0.45f));

    const int chimneyCount = random.Int(0, 3);
    for (int i = 0; i < chimneyCount; ++i) {
        const glm::vec2 position(random.Range(roof.min.x + 0.5f, roof.max.x - 0.5f),
                                 random.Range(roof.min.y + 0.5f, roof.max.y - 0.5f));
        const Rect chimney{position - glm::vec2(0.35f, 0.5f), position + glm::vec2(0.35f, 0.5f)};
        city.boxes.push_back(FootprintBox(chimney, roofTop - 0.2f, roofTop + random.Range(0.9f, 1.8f),
                                          kChimneyBrick * random.Range(0.85f, 1.15f), SurfaceMaterial::Plain, 0.8f));
    }
}

// An open square: paving, and a fountain with a column in the middle.
void AddPlaza(CityLayout& city, const Rect& area) {
    const float ground = kPavementHeight + 0.05f;
    city.boxes.push_back(FootprintBox(area, 0.0f, ground, kPlazaStone, SurfaceMaterial::Ground, 0.75f, kPavedSurface));

    const glm::vec2 center = area.Center();
    const float basinHalfSize = 4.5f;
    const float wall = 0.45f;
    const float basinTop = ground + 0.8f;
    const Rect basin{center - basinHalfSize, center + basinHalfSize};
    const glm::vec3 stone = kPlazaStone * 1.1f;

    // Four low walls around the water.
    const Rect walls[] = {
        {{basin.min.x, basin.max.y - wall}, {basin.max.x, basin.max.y}},
        {{basin.min.x, basin.min.y}, {basin.max.x, basin.min.y + wall}},
        {{basin.min.x, basin.min.y + wall}, {basin.min.x + wall, basin.max.y - wall}},
        {{basin.max.x - wall, basin.min.y + wall}, {basin.max.x, basin.max.y - wall}},
    };
    for (const Rect& wallFootprint : walls) {
        city.boxes.push_back(FootprintBox(wallFootprint, ground, basinTop, stone, SurfaceMaterial::Plain, 0.6f));
    }
    city.boxes.push_back(FootprintBox(basin.Expanded(-wall), ground, basinTop - 0.2f, kWater, SurfaceMaterial::Plain, 0.05f));

    // Column and capital.
    city.boxes.push_back(FootprintBox({center - 0.7f, center + 0.7f}, ground, ground + 6.0f, stone, SurfaceMaterial::Plain, 0.6f));
    city.boxes.push_back(FootprintBox({center - 1.0f, center + 1.0f}, ground + 6.0f, ground + 6.6f, stone, SurfaceMaterial::Plain, 0.6f));

    city.plazas.push_back(area.Expanded(-1.0f));
}

StreetGraph BuildStreetGraph(const std::vector<float>& linesX, const std::vector<float>& linesZ) {
    StreetGraph graph;
    const uint32_t columns = static_cast<uint32_t>(linesX.size());
    const uint32_t rows = static_cast<uint32_t>(linesZ.size());
    const auto nodeIndex = [columns](uint32_t x, uint32_t z) { return z * columns + x; };

    for (uint32_t z = 0; z < rows; ++z) {
        for (uint32_t x = 0; x < columns; ++x) {
            graph.nodes.emplace_back(linesX[x], linesZ[z]);
        }
    }
    graph.edgesAtNode.resize(graph.nodes.size());

    const auto addEdge = [&graph](uint32_t from, uint32_t to) {
        const uint32_t edgeIndex = static_cast<uint32_t>(graph.edges.size());
        graph.edges.push_back({from, to, glm::distance(graph.nodes[from], graph.nodes[to])});
        graph.edgesAtNode[from].push_back(edgeIndex);
        graph.edgesAtNode[to].push_back(edgeIndex);
    };
    for (uint32_t z = 0; z < rows; ++z) {
        for (uint32_t x = 0; x < columns; ++x) {
            if (x + 1 < columns) {
                addEdge(nodeIndex(x, z), nodeIndex(x + 1, z));
            }
            if (z + 1 < rows) {
                addEdge(nodeIndex(x, z), nodeIndex(x, z + 1));
            }
        }
    }
    return graph;
}
}  // namespace

CityLayout GenerateCity(const CitySettings& settings) {
    Random random(settings.seed);
    CityLayout city;

    // 1. Lay out the street grid. Block sizes vary, so the streets are irregularly spaced.
    const float streetWidth = settings.roadWidth + 2.0f * settings.sidewalkWidth;
    const float halfStreet = streetWidth * 0.5f;
    const std::vector<float> linesX =
        StreetCenterLines(random, settings.blocksX, settings.minBlockSize, settings.maxBlockSize, streetWidth);
    const std::vector<float> linesZ =
        StreetCenterLines(random, settings.blocksZ, settings.minBlockSize, settings.maxBlockSize, streetWidth);

    city.bounds = {{linesX.front() - halfStreet, linesZ.front() - halfStreet},
                   {linesX.back() + halfStreet, linesZ.back() + halfStreet}};
    city.sidewalkCenterOffset = settings.roadWidth * 0.5f + settings.sidewalkWidth * 0.5f;
    city.sidewalkWidth = settings.sidewalkWidth;
    city.walkHeight = kPavementHeight;

    // One large road surface under everything. It reaches far past the city so the ground fades
    // into the haze instead of ending at an edge.
    city.boxes.push_back(FootprintBox(city.bounds.Expanded(1500.0f), -1.0f, 0.0f, kAsphalt, SurfaceMaterial::Ground, 0.9f));

    // 2. Fill every block: either an open square, or pavement with buildings on it.
    const float cityRadius = glm::length(city.bounds.max);
    for (int z = 0; z < settings.blocksZ; ++z) {
        for (int x = 0; x < settings.blocksX; ++x) {
            const Rect block{{linesX[x] + halfStreet, linesZ[z] + halfStreet},
                             {linesX[x + 1] - halfStreet, linesZ[z + 1] - halfStreet}};
            const Rect blockWithPavement = block.Expanded(settings.sidewalkWidth);

            // The central block is always a square, so every city has a main gathering place.
            const bool isCentralSquare = x == settings.blocksX / 2 && z == settings.blocksZ / 2;
            if (isCentralSquare || random.Chance(settings.plazaChance)) {
                AddPlaza(city, blockWithPavement);
                continue;
            }

            city.boxes.push_back(FootprintBox(blockWithPavement, 0.0f, kPavementHeight, kPaving,
                                              SurfaceMaterial::Ground, 0.8f, kPavedSurface));

            // Buildings get taller towards the middle of the city.
            const float centrality = 1.0f - std::min(glm::length(block.Center()) / cityRadius, 1.0f);
            const float heightBias = 0.75f + 0.75f * centrality;

            std::vector<Rect> lots;
            SplitIntoLots(block, random, lots);
            for (const Rect& lot : lots) {
                AddBuilding(city, lot, heightBias, random);
            }
        }
    }

    // 3. The street graph people walk along.
    city.streets = BuildStreetGraph(linesX, linesZ);
    return city;
}
