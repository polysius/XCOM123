#include "world/Crowd.h"

#include <algorithm>
#include <cmath>
#include <execution>
#include <iterator>
#include <utility>

#include "scene/Frustum.h"

namespace {
constexpr float kTwoPi = 6.28318530718f;
constexpr float kStrideLength = 1.4f;     // metres per full walk cycle (two steps)
constexpr float kCatchUpFactor = 1.8f;    // how much faster than walking pace people move to reach their lane
constexpr float kFountainKeepOut = 5.5f;  // half size of the square kept clear around a plaza's fountain

// Muted, period-style clothing colours.
const glm::vec3 kClothingColors[] = {
    {0.12f, 0.14f, 0.22f}, {0.30f, 0.10f, 0.09f}, {0.20f, 0.22f, 0.13f}, {0.36f, 0.30f, 0.22f},
    {0.55f, 0.50f, 0.42f}, {0.10f, 0.10f, 0.10f}, {0.42f, 0.18f, 0.12f}, {0.62f, 0.60f, 0.56f},
    {0.16f, 0.26f, 0.34f}, {0.48f, 0.38f, 0.20f},
};

// xorshift32: a tiny, fast random generator whose whole state is one integer.
uint32_t NextRandom(uint32_t& state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

float RandomUnit(uint32_t& state) {
    return static_cast<float>(NextRandom(state) >> 8) * (1.0f / 16777216.0f);
}

float RandomRange(uint32_t& state, float min, float max) {
    return min + (max - min) * RandomUnit(state);
}

// Scrambles an integer so that neighbouring agent indices get unrelated random streams.
uint32_t HashInteger(uint32_t value) {
    value ^= value >> 16;
    value *= 0x7feb352dU;
    value ^= value >> 15;
    value *= 0x846ca68bU;
    value ^= value >> 16;
    return value;
}

// Moves `position` towards `target` by at most `maxStep` metres.
void MoveTowards(glm::vec2& position, glm::vec2 target, float maxStep) {
    const glm::vec2 offset = target - position;
    const float distance = glm::length(offset);
    position = (distance <= maxStep) ? target : position + offset * (maxStep / distance);
}

// The shortest signed turn from angle `from` to angle `to`, in -pi..pi.
float AngleDifference(float from, float to) {
    float difference = std::fmod(to - from + kTwoPi * 0.5f, kTwoPi);
    if (difference < 0.0f) {
        difference += kTwoPi;
    }
    return difference - kTwoPi * 0.5f;
}

bool IsNearFountain(glm::vec2 point, const Rect& plaza) {
    const glm::vec2 fromCenter = glm::abs(point - plaza.Center());
    return fromCenter.x < kFountainKeepOut && fromCenter.y < kFountainKeepOut;
}

// True if the straight walk from a to b passes through the square kept clear around the fountain.
bool PathCrossesFountain(glm::vec2 a, glm::vec2 b, const Rect& plaza) {
    // Slab test: clip the segment to the square's X range, then to its Y range.
    const glm::vec2 center = plaza.Center();
    const glm::vec2 direction = b - a;
    float enter = 0.0f;
    float exit = 1.0f;
    for (int axis = 0; axis < 2; ++axis) {
        const float low = center[axis] - kFountainKeepOut - a[axis];
        const float high = center[axis] + kFountainKeepOut - a[axis];
        if (std::abs(direction[axis]) < 1e-6f) {
            if (low > 0.0f || high < 0.0f) {
                return false;  // parallel to this side and outside it
            }
            continue;
        }
        float t0 = low / direction[axis];
        float t1 = high / direction[axis];
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        enter = std::max(enter, t0);
        exit = std::min(exit, t1);
        if (enter > exit) {
            return false;
        }
    }
    return true;
}

glm::vec2 RandomPointInPlaza(const Rect& plaza, uint32_t& random) {
    // Retry points that land on the fountain.
    glm::vec2 point = plaza.min;
    for (int attempt = 0; attempt < 8; ++attempt) {
        point = {RandomRange(random, plaza.min.x, plaza.max.x), RandomRange(random, plaza.min.y, plaza.max.y)};
        if (!IsNearFountain(point, plaza)) {
            break;
        }
    }
    return point;
}
}  // namespace

void Crowd::Spawn(const CityLayout& city, const CrowdSettings& settings) {
    streets = city.streets;
    plazas = city.plazas;
    sidewalkCenterOffset = city.sidewalkCenterOffset;
    sidewalkWidth = city.sidewalkWidth;
    walkHeight = city.walkHeight;
    if (streets.edges.empty()) {
        agents.clear();
        return;
    }

    const float plazaShare = plazas.empty() ? 0.0f : settings.plazaShare;
    const uint32_t colorCount = static_cast<uint32_t>(std::size(kClothingColors));
    agents.assign(settings.agentCount, Agent{});
    for (uint32_t index = 0; index < settings.agentCount; ++index) {
        Agent& agent = agents[index];
        agent.random = HashInteger(index ^ HashInteger(settings.seed)) | 1u;  // xorshift state must never be 0
        agent.speed = RandomRange(agent.random, 1.0f, 1.6f);
        agent.heightScale = RandomRange(agent.random, 0.92f, 1.08f);
        agent.clothingColor = kClothingColors[NextRandom(agent.random) % colorCount] * RandomRange(agent.random, 0.8f, 1.2f);
        agent.walkPhase = RandomRange(agent.random, 0.0f, kTwoPi);

        if (RandomUnit(agent.random) < plazaShare) {
            SpawnPlazaWanderer(agent);
        } else {
            SpawnStreetWalker(agent);
        }
    }
}

void Crowd::SpawnStreetWalker(Agent& agent) const {
    agent.behaviour = Behaviour::WalkStreets;
    agent.edge = NextRandom(agent.random) % static_cast<uint32_t>(streets.edges.size());
    agent.edgeDistance = RandomUnit(agent.random) * streets.edges[agent.edge].length;
    agent.towardsEdgeEnd = RandomUnit(agent.random) < 0.5f;
    // Pick a pavement (left or right of the centre line) and a spot across its width.
    const float side = RandomUnit(agent.random) < 0.5f ? -1.0f : 1.0f;
    agent.sideOffset = side * (sidewalkCenterOffset + RandomRange(agent.random, -0.35f, 0.35f) * sidewalkWidth);
    agent.position = StreetPosition(agent);
}

void Crowd::SpawnPlazaWanderer(Agent& agent) const {
    agent.behaviour = Behaviour::WanderPlaza;
    agent.plaza = NextRandom(agent.random) % static_cast<uint32_t>(plazas.size());
    agent.position = RandomPointInPlaza(plazas[agent.plaza], agent.random);
    agent.goal = PickPlazaGoal(agent);
    agent.pauseTime = RandomRange(agent.random, 0.0f, 4.0f);
}

void Crowd::Update(float deltaSeconds) {
    // Each agent only reads shared data (streets, squares) and writes its own state,
    // so the loop can safely run on all cores at once.
    std::for_each(std::execution::par, agents.begin(), agents.end(), [this, deltaSeconds](Agent& agent) {
        const glm::vec2 previousPosition = agent.position;
        if (agent.behaviour == Behaviour::WalkStreets) {
            UpdateStreetWalker(agent, deltaSeconds);
        } else {
            UpdatePlazaWanderer(agent, deltaSeconds);
        }
        UpdateAnimation(agent, previousPosition, deltaSeconds);
    });
}

void Crowd::UpdateStreetWalker(Agent& agent, float deltaSeconds) const {
    agent.edgeDistance += agent.speed * deltaSeconds;
    while (agent.edgeDistance >= streets.edges[agent.edge].length) {
        // Reached an intersection: carry on along a randomly chosen street.
        const StreetGraph::Edge& finished = streets.edges[agent.edge];
        agent.edgeDistance -= finished.length;
        const uint32_t node = agent.towardsEdgeEnd ? finished.to : finished.from;
        agent.edge = ChooseNextEdge(node, agent.edge, agent.random);
        agent.towardsEdgeEnd = streets.edges[agent.edge].from == node;
    }
    // The lane point moves at walking pace and the person follows it. After a turn the lane can
    // jump to the other side of the road, and catching up with it is the person crossing the street.
    MoveTowards(agent.position, StreetPosition(agent), agent.speed * kCatchUpFactor * deltaSeconds);
}

void Crowd::UpdatePlazaWanderer(Agent& agent, float deltaSeconds) const {
    if (agent.pauseTime > 0.0f) {
        agent.pauseTime -= deltaSeconds;
        return;
    }
    if (glm::length(agent.goal - agent.position) < 0.3f) {
        // Arrived: stand around for a moment, then pick somewhere new.
        agent.pauseTime = RandomRange(agent.random, 0.0f, 6.0f);
        agent.goal = PickPlazaGoal(agent);
        return;
    }
    MoveTowards(agent.position, agent.goal, agent.speed * 0.75f * deltaSeconds);
}

void Crowd::UpdateAnimation(Agent& agent, glm::vec2 previousPosition, float deltaSeconds) {
    // Facing and walk cycle follow how far the person actually moved this frame.
    const glm::vec2 movement = agent.position - previousPosition;
    const float distance = glm::length(movement);
    if (distance > 1e-4f) {
        const float targetHeading = std::atan2(movement.x, movement.y);
        // Turn smoothly rather than snapping to the new direction.
        agent.heading += AngleDifference(agent.heading, targetHeading) * std::min(1.0f, deltaSeconds * 8.0f);
    }
    agent.walkPhase = std::fmod(agent.walkPhase + distance / kStrideLength * kTwoPi, kTwoPi);

    const float speed = distance / std::max(deltaSeconds, 1e-4f);
    const float targetStride = std::min(speed, 1.0f);
    agent.stride += (targetStride - agent.stride) * std::min(1.0f, deltaSeconds * 6.0f);
}

glm::vec2 Crowd::StreetPosition(const Agent& agent) const {
    const StreetGraph::Edge& edge = streets.edges[agent.edge];
    const glm::vec2 start = streets.nodes[agent.towardsEdgeEnd ? edge.from : edge.to];
    const glm::vec2 end = streets.nodes[agent.towardsEdgeEnd ? edge.to : edge.from];
    const glm::vec2 direction = (end - start) / edge.length;
    const glm::vec2 side(-direction.y, direction.x);
    return start + direction * agent.edgeDistance + side * agent.sideOffset;
}

glm::vec2 Crowd::PickPlazaGoal(Agent& agent) const {
    // Prefer a destination that can be reached in a straight line without walking through the fountain.
    const Rect& plaza = plazas[agent.plaza];
    glm::vec2 goal = agent.position;
    for (int attempt = 0; attempt < 8; ++attempt) {
        goal = RandomPointInPlaza(plaza, agent.random);
        if (!PathCrossesFountain(agent.position, goal, plaza)) {
            break;
        }
    }
    return goal;
}

uint32_t Crowd::ChooseNextEdge(uint32_t node, uint32_t currentEdge, uint32_t& random) const {
    const std::vector<uint32_t>& options = streets.edgesAtNode[node];
    if (options.size() == 1) {
        return options[0];  // dead end: turn around
    }
    // Any street except the one we arrived on.
    uint32_t choice = NextRandom(random) % static_cast<uint32_t>(options.size() - 1);
    for (uint32_t edge : options) {
        if (edge == currentEdge) {
            continue;
        }
        if (choice == 0) {
            return edge;
        }
        --choice;
    }
    return options[0];
}

void Crowd::CollectVisible(const Frustum& frustum, std::vector<NpcInstance>& visible) const {
    visible.clear();
    for (const Agent& agent : agents) {
        const glm::vec3 feet(agent.position.x, walkHeight, agent.position.y);
        // A sphere around the whole body, centred at hip height.
        if (!frustum.ContainsSphere(feet + glm::vec3(0.0f, 0.9f, 0.0f), 1.1f)) {
            continue;
        }
        visible.push_back({.position = feet,
                           .heading = agent.heading,
                           .clothingColor = agent.clothingColor,
                           .heightScale = agent.heightScale,
                           .walkPhase = agent.walkPhase,
                           .stride = agent.stride,
                           .unused0 = 0.0f,
                           .unused1 = 0.0f});
    }
}
