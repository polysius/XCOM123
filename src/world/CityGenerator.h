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
};

// The street network as a graph: intersections are nodes, street segments between them are edges.
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
    int blocksX = 14;
    int blocksZ = 14;
    float minBlockSize = 42.0f;  // metres
    float maxBlockSize = 78.0f;
    float roadWidth = 8.0f;      // the carriageway between the two pavements
    float sidewalkWidth = 3.5f;
    float plazaChance = 0.08f;   // chance that a block becomes an open square
};

// Everything the generator produces: geometry to draw, and the walkable spaces people use.
struct CityLayout {
    std::vector<BoxInstance> boxes;
    StreetGraph streets;
    std::vector<Rect> plazas;           // open squares people wander in
    Rect bounds;                        // outer edge of the street grid
    float sidewalkCenterOffset = 0.0f;  // distance from a street's centre line to the middle of its pavements
    float sidewalkWidth = 0.0f;
    float walkHeight = 0.0f;            // height of the pavement surface people stand on
};

// Builds a city from `settings`. The same seed always produces the same city.
CityLayout GenerateCity(const CitySettings& settings);
