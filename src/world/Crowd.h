#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "scene/SceneTypes.h"
#include "world/CityGenerator.h"
#include "world/crowd/CrowdSettings.h"
#include "world/crowd/CrowdState.h"
#include "world/crowd/CrowdVisibility.h"
#include "world/crowd/SpatialHash.h"
#include "world/crowd/Walkways.h"

class Frustum;
class JobSystem;

// What the crowd did in the last Update and CollectVisible, for overlays and benchmarks.
struct CrowdStats {
    // How many people simulated at each level of detail.
    uint32_t nearAgents = 0;
    uint32_t midAgents = 0;
    uint32_t farAgents = 0;
    // How many visible people fell in each distance band (see CrowdVisibility.h).
    crowd::CrowdVisibility::BandCounts visibleByBand{};
    // Where the last Update spent its time, in milliseconds.
    double tierMs = 0.0;   // choosing tiers and counting grid cells
    double gridMs = 0.0;   // filling the neighbour grid
    double agentMs = 0.0;  // updating people
};

// A city crowd kept as plain data and updated in parallel on all CPU cores.
//
// People walk the pavements of the street graph alone or in small groups, crossing the
// road at corners, and stop now and then to look around. In squares they stroll between
// spots, rest, or stand talking in circles. They keep their distance from each other and
// let oncoming people pass side by side.
//
// The cost is kept low with simulation level of detail around a focus point, usually the
// camera: near people avoid each other every step, mid distance people refresh their
// avoidance every few steps, and far people only follow their way. All of them keep
// walking at the same speed, so nothing changes visibly when someone changes level.
class Crowd {
public:
    // Runs on the engine's shared job system.
    Crowd();
    explicit Crowd(JobSystem& jobs);

    void Spawn(const CityLayout& city, const CrowdSettings& settings);
    // The point the simulation level of detail is centred on, usually the camera position.
    void SetFocusPoint(glm::vec3 point);
    void Update(float deltaSeconds);
    // Replaces the contents of `visible` with the people inside the camera's view, ordered
    // nearest band first around the focus point.
    void CollectVisible(const Frustum& frustum, std::vector<NpcInstance>& visible) const;
    size_t AgentCount() const { return state.AgentCount(); }

    CrowdStats Stats() const;
    // Read-only access for tools such as the crowd benchmark.
    const crowd::CrowdState& State() const { return state; }
    const crowd::Walkways& WalkableSpace() const { return walkways; }

private:
    void AssignTiersAndCells();
    void BuildNeighbourGrid();
    void StepUnits(float deltaSeconds);

    JobSystem* jobs;
    crowd::Walkways walkways;
    crowd::CrowdState state;
    // Per person, its cell in the neighbour grid, or kNoIndex outside the grid's window.
    std::vector<uint32_t> agentCells;
    crowd::SpatialHash neighbourGrid;
    // Per chunk of people, how many are at each simulation tier.
    std::vector<std::array<uint32_t, 3>> chunkTierCounts;
    mutable crowd::CrowdVisibility visibility;
    glm::vec3 focusPoint{0.0f};
    float simulationTime = 0.0f;
    uint32_t stepIndex = 0;
    double lastTierMs = 0.0;
    double lastGridMs = 0.0;
    double lastAgentMs = 0.0;
};
