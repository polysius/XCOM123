#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "scene/SceneTypes.h"
#include "world/CityGenerator.h"

class Frustum;

struct CrowdSettings {
    uint32_t agentCount = 10000;
    uint32_t seed = 7;
    float plazaShare = 0.3f;  // fraction of people who mill about in squares instead of walking the streets
};

// A city crowd kept as plain data and updated in parallel on all CPU cores.
//
// Two simple behaviours keep it cheap enough for tens of thousands of people:
//  - Street walkers follow the street graph along the pavements and pick a random turn at each
//    intersection. When a turn puts their pavement on the other side, they cross the road.
//  - Plaza wanderers stroll between random spots in a square, pausing now and then.
// People do not avoid each other yet; see the roadmap in README.md.
class Crowd {
public:
    void Spawn(const CityLayout& city, const CrowdSettings& settings);
    void Update(float deltaSeconds);
    // Replaces the contents of `visible` with the people inside the camera's view.
    void CollectVisible(const Frustum& frustum, std::vector<NpcInstance>& visible) const;
    size_t AgentCount() const { return agents.size(); }

private:
    enum class Behaviour : uint8_t { WalkStreets, WanderPlaza };

    struct Agent {
        glm::vec2 position{0.0f};
        glm::vec3 clothingColor{1.0f};
        float heading = 0.0f;
        float speed = 1.3f;  // metres per second
        float heightScale = 1.0f;
        float walkPhase = 0.0f;
        float stride = 0.0f;
        uint32_t random = 1;  // this agent's own random state, so threads never share one
        Behaviour behaviour = Behaviour::WalkStreets;

        // Street walkers
        uint32_t edge = 0;
        float edgeDistance = 0.0f;   // metres travelled along the current edge
        bool towardsEdgeEnd = true;  // walking from edge.from to edge.to, or the other way
        float sideOffset = 0.0f;     // signed distance from the street centre line: which pavement

        // Plaza wanderers
        uint32_t plaza = 0;
        glm::vec2 goal{0.0f};
        float pauseTime = 0.0f;
    };

    void SpawnStreetWalker(Agent& agent) const;
    void SpawnPlazaWanderer(Agent& agent) const;
    void UpdateStreetWalker(Agent& agent, float deltaSeconds) const;
    void UpdatePlazaWanderer(Agent& agent, float deltaSeconds) const;
    static void UpdateAnimation(Agent& agent, glm::vec2 previousPosition, float deltaSeconds);
    glm::vec2 StreetPosition(const Agent& agent) const;
    glm::vec2 PickPlazaGoal(Agent& agent) const;
    uint32_t ChooseNextEdge(uint32_t node, uint32_t currentEdge, uint32_t& random) const;

    std::vector<Agent> agents;
    StreetGraph streets;
    std::vector<Rect> plazas;
    float sidewalkCenterOffset = 0.0f;
    float sidewalkWidth = 0.0f;
    float walkHeight = 0.0f;
};
