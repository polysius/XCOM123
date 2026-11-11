#include "world/crowd/CrowdSpawner.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

#include "world/crowd/AgentUpdate.h"
#include "world/crowd/CrowdMath.h"
#include "world/crowd/CrowdState.h"
#include "world/crowd/StreetRouting.h"
#include "world/crowd/Walkways.h"

namespace crowd {

namespace {

// Squares never get more people than this per square metre of open space, so they stay
// lively without turning into a crush.
constexpr float kMaxPlazaDensity = 0.35f;
constexpr uint32_t kPlacementTries = 40;

// Conversation circles: between kMinCircle and kMaxCircle people, standing on a ring whose
// radius grows with the number of people. The clearance keeps strollers at a polite distance.
constexpr uint32_t kMinCircle = 3;
constexpr uint32_t kMaxCircle = 5;
constexpr float kCircleClearance = 0.45f;

// A circle needs this much room beyond its reserved space, and stays this far from the
// pavements so it never blocks a street.
constexpr float kCircleRoom = 0.6f;
constexpr float kCircleStreetDistance = 0.5f;

// Group followers take these places, x metres to the leader's left and y metres behind.
const glm::vec2 kGroupSlots[] = {{0.62f, 0.1f}, {-0.62f, 0.1f}, {0.0f, 0.95f}};

const glm::vec3 kClothingColors[] = {
    {0.12f, 0.14f, 0.22f}, {0.30f, 0.10f, 0.09f}, {0.20f, 0.22f, 0.13f}, {0.36f, 0.30f, 0.22f},
    {0.55f, 0.50f, 0.42f}, {0.10f, 0.10f, 0.10f}, {0.42f, 0.18f, 0.12f}, {0.62f, 0.60f, 0.56f},
    {0.16f, 0.26f, 0.34f}, {0.48f, 0.38f, 0.20f},
};

float CircleRadius(uint32_t memberCount) {
    return 0.35f + 0.1f * static_cast<float>(memberCount);
}

// A street position: where on which leg someone stands.
struct StreetSpot {
    StreetRoute route;
    float distance = 0.0f;  // along the street from the leg's start node
};

class Spawner {
public:
    Spawner(const CrowdSettings& settings, Walkways& walkways, CrowdState& state)
        : settings(settings), walkways(walkways), state(state), random(SeedFor(0x5EEDu, settings.seed)) {}

    void Run();

private:
    std::vector<uint32_t> SharePeopleAmongPlazas();
    uint32_t SpawnPlazaPeople(uint32_t plaza, uint32_t count);
    bool SpawnChatCircle(uint32_t plaza, uint32_t memberCount);
    bool SpawnWanderer(uint32_t plaza);
    void SpawnStreetPeople(uint32_t count);
    bool SpawnStreetUnit(uint32_t memberCount);
    bool PickStreetSpot(float laneBias, StreetSpot& spot);

    uint32_t NewAgent();
    void FinishLook(uint32_t agent);
    void FaceTowards(uint32_t agent, glm::vec2 point);
    uint32_t PickGroupSize();

    const CrowdSettings& settings;
    Walkways& walkways;
    CrowdState& state;
    uint32_t random;  // for placement; each person also gets a generator of its own
    std::vector<float> cumulativeStreetLength;
};

void Spawner::Run() {
    state.Clear();
    const StreetGraph& streets = walkways.Streets();
    float total = 0.0f;
    for (const StreetGraph::Edge& edge : streets.edges) {
        total += edge.length;
        cumulativeStreetLength.push_back(total);
    }

    uint32_t placed = 0;
    const std::vector<uint32_t> plazaQuotas = SharePeopleAmongPlazas();
    for (uint32_t plaza = 0; plaza < plazaQuotas.size(); ++plaza) {
        placed += SpawnPlazaPeople(plaza, plazaQuotas[plaza]);
    }
    // Whoever did not fit in a square walks the streets.
    SpawnStreetPeople(settings.agentCount - placed);
}

// Splits the square people among the squares by open area, never above the comfortable
// density.
std::vector<uint32_t> Spawner::SharePeopleAmongPlazas() {
    const std::vector<OrientedRect>& plazas = walkways.Plazas();
    std::vector<float> capacity(plazas.size());
    float totalCapacity = 0.0f;
    for (size_t plaza = 0; plaza < plazas.size(); ++plaza) {
        capacity[plaza] = walkways.FreeArea(plazas[plaza]) * kMaxPlazaDensity;
        totalCapacity += capacity[plaza];
    }

    std::vector<uint32_t> quotas(plazas.size(), 0);
    if (totalCapacity < 1.0f) {
        return quotas;
    }
    const float wanted = std::min(static_cast<float>(settings.agentCount) * settings.plazaShare, totalCapacity);
    for (size_t plaza = 0; plaza < plazas.size(); ++plaza) {
        quotas[plaza] = static_cast<uint32_t>(wanted * capacity[plaza] / totalCapacity);
    }
    return quotas;
}

// Spawns up to `count` people in one square and returns how many fitted.
uint32_t Spawner::SpawnPlazaPeople(uint32_t plaza, uint32_t count) {
    uint32_t placed = 0;
    const uint32_t chatters = static_cast<uint32_t>(static_cast<float>(count) * settings.chatShare);
    while (placed + kMinCircle <= chatters) {
        const uint32_t size = std::min(kMinCircle + RandomIndex(random, kMaxCircle - kMinCircle + 1), chatters - placed);
        if (!SpawnChatCircle(plaza, size)) {
            break;  // the square is full of circles; the rest stroll instead
        }
        placed += size;
    }
    while (placed < count && SpawnWanderer(plaza)) {
        ++placed;
    }
    return placed;
}

bool Spawner::SpawnChatCircle(uint32_t plaza, uint32_t memberCount) {
    const OrientedRect& square = walkways.Plazas()[plaza];
    const Rect& area = square.area;  // the square in its own frame
    const float radius = CircleRadius(memberCount);
    const float streetClearance = walkways.OuterEdge() + radius + kCircleStreetDistance;
    for (uint32_t attempt = 0; attempt < kPlacementTries; ++attempt) {
        // The second coordinate is drawn first. The city has always been spawned in this order (it was
        // the order MSVC evaluated the arguments of a vec2 constructor in), and the people's positions
        // depend on it. Written as two statements the order no longer depends on the compiler.
        const float frameY = RandomRange(random, area.min.y + radius, area.max.y - radius);
        const float frameX = RandomRange(random, area.min.x + radius, area.max.x - radius);
        const glm::vec2 center = square.FromFrame({frameX, frameY});
        if (!walkways.IsDiscFree(center, radius + kCircleRoom) ||
            walkways.DistanceToNearestStreet(center) < streetClearance) {
            continue;
        }

        CrowdUnit unit;
        unit.firstAgent = static_cast<uint32_t>(state.AgentCount());
        unit.memberCount = memberCount;
        unit.circleCenter = center;
        unit.talkClockOffset = RandomRange(random, 0.0f, 10.0f);
        for (uint32_t member = 0; member < memberCount; ++member) {
            const uint32_t agent = NewAgent();
            const float angle = unit.talkClockOffset * 2.4f + kTwoPi * static_cast<float>(member) / static_cast<float>(memberCount);
            const glm::vec2 spot = center + radius * glm::vec2(std::cos(angle), std::sin(angle));
            AgentBehaviour& behaviour = state.behaviour[agent];
            behaviour.role = AgentRole::Chatter;
            behaviour.activity = Activity::Chatting;
            behaviour.plaza = plaza;
            behaviour.goal = spot;
            state.motion[agent].position = spot;
            FaceTowards(agent, center);
            FinishLook(agent);
        }
        state.units.push_back(unit);
        walkways.Reserve(center, radius + kCircleClearance);
        return true;
    }
    return false;
}

bool Spawner::SpawnWanderer(uint32_t plaza) {
    glm::vec2 spot;
    if (!PickPlazaSpot(walkways, walkways.Plazas()[plaza], kPlacementTries, random, spot)) {
        return false;
    }
    const uint32_t agent = NewAgent();
    AgentBehaviour& behaviour = state.behaviour[agent];
    behaviour.role = AgentRole::PlazaWanderer;
    behaviour.plaza = plaza;
    // Everyone starts out resting for a while, so the squares do not all set off at once.
    behaviour.activity = Activity::Resting;
    behaviour.activityTimer = RandomRange(behaviour.random, 0.0f, 8.0f);
    behaviour.goal = spot;
    behaviour.preferredSpeed *= 0.8f;  // a stroll, not a walk to work
    state.motion[agent].position = spot;
    state.looks[agent].heading = RandomRange(random, 0.0f, kTwoPi);
    FinishLook(agent);
    state.units.push_back({.firstAgent = agent, .memberCount = 1});
    return true;
}

void Spawner::SpawnStreetPeople(uint32_t count) {
    if (walkways.Streets().edges.empty()) {
        // No streets: everyone strolls in the squares, however crowded.
        for (uint32_t index = 0; index < count && !walkways.Plazas().empty(); ++index) {
            SpawnWanderer(index % static_cast<uint32_t>(walkways.Plazas().size()));
        }
        return;
    }

    uint32_t groupPeopleLeft = static_cast<uint32_t>(static_cast<float>(count) * settings.groupShare);
    uint32_t placed = 0;
    uint32_t failures = 0;
    while (placed < count && failures < kPlacementTries) {
        uint32_t size = 1;
        if (groupPeopleLeft >= 2) {
            size = std::min({PickGroupSize(), groupPeopleLeft, count - placed});
        }
        if (!SpawnStreetUnit(size)) {
            ++failures;
            continue;
        }
        failures = 0;
        placed += size;
        groupPeopleLeft -= size > 1 ? size : 0;
    }
}

uint32_t Spawner::PickGroupSize() {
    const float roll = RandomUnit(random);
    return roll < 0.55f ? 2u : (roll < 0.85f ? 3u : 4u);
}

// Spawns a lone walker (memberCount 1) or a walking group with its leader first.
bool Spawner::SpawnStreetUnit(uint32_t memberCount) {
    const float leaderBias = memberCount > 1 ? RandomRange(random, 0.1f, 0.6f) : RandomRange(random, -0.3f, 0.9f);
    StreetSpot leaderSpot;
    if (!PickStreetSpot(leaderBias, leaderSpot)) {
        return false;
    }

    CrowdUnit unit;
    unit.firstAgent = static_cast<uint32_t>(state.AgentCount());
    unit.memberCount = memberCount;
    float groupPace = 1e9f;
    for (uint32_t member = 0; member < memberCount; ++member) {
        const uint32_t agent = NewAgent();
        AgentBehaviour& behaviour = state.behaviour[agent];
        StreetRoute& route = state.routes[agent];
        route = leaderSpot.route;
        float distance = leaderSpot.distance;
        behaviour.laneBias = leaderBias;
        if (member == 0) {
            behaviour.role = AgentRole::StreetWalker;
            behaviour.activityTimer = NextLookAroundDelay(behaviour.random);
        } else {
            const glm::vec2 slot = kGroupSlots[member - 1];
            behaviour.role = AgentRole::GroupFollower;
            behaviour.goal = slot;
            behaviour.laneBias = leaderBias - slot.x;
            route.lane = LaneOffset(walkways, route.lane, behaviour.laneBias);
            distance = std::max(distance - slot.y, StreetLegStart(walkways, route));
        }
        glm::vec2 position = StreetPoint(walkways, route, distance);
        if (!walkways.CanStandAt(position)) {
            position = state.motion[unit.firstAgent].position;  // the crowd pushes them apart
        }
        state.motion[agent].position = position;
        const glm::vec2 walkingDirection = FrameOf(walkways, route.edge, route.forward).direction;
        state.looks[agent].heading = std::atan2(walkingDirection.x, walkingDirection.y);
        groupPace = std::min(groupPace, behaviour.preferredSpeed);
        FinishLook(agent);
    }
    if (memberCount > 1) {
        // The group walks at the pace of its slowest member, a little slower still so the
        // others can keep their places.
        state.behaviour[unit.firstAgent].preferredSpeed = groupPace * 0.95f;
    }
    state.units.push_back(unit);
    return true;
}

// Picks a random point on a random pavement, more likely on longer streets.
bool Spawner::PickStreetSpot(float laneBias, StreetSpot& spot) {
    const float total = cumulativeStreetLength.back();
    for (uint32_t attempt = 0; attempt < kPlacementTries; ++attempt) {
        const float pick = RandomUnit(random) * total;
        const auto found = std::upper_bound(cumulativeStreetLength.begin(), cumulativeStreetLength.end(), pick);
        StreetRoute route;
        route.edge = static_cast<uint32_t>(std::min<ptrdiff_t>(std::distance(cumulativeStreetLength.begin(), found),
                                                               static_cast<ptrdiff_t>(cumulativeStreetLength.size()) - 1));
        route.forward = RandomUnit(random) < 0.5f;
        route.leg = RouteLeg::Street;
        route.lane = LaneOffset(walkways, RandomUnit(random) < 0.5f ? 1.0f : -1.0f, laneBias);

        const float legStart = StreetLegStart(walkways, route);
        const float legEnd = StreetLegEnd(walkways, route);
        if (legEnd - legStart < 1.0f) {
            continue;  // too short to stand on between its two corners
        }
        const float distance = RandomRange(random, legStart, legEnd);
        if (walkways.CanStandAt(StreetPoint(walkways, route, distance))) {
            spot = {route, distance};
            return true;
        }
    }
    return false;
}

uint32_t Spawner::NewAgent() {
    const uint32_t agent = state.AddAgent();
    AgentBehaviour& behaviour = state.behaviour[agent];
    behaviour.random = SeedFor(agent, settings.seed);
    // Averaging two random numbers makes average walkers common and very slow or fast ones rare.
    const float spread = 0.5f * (RandomUnit(behaviour.random) + RandomUnit(behaviour.random));
    behaviour.preferredSpeed = 0.95f + 0.7f * spread;

    AgentLook& look = state.looks[agent];
    const uint32_t colorCount = static_cast<uint32_t>(std::size(kClothingColors));
    look.clothingColor = kClothingColors[RandomIndex(behaviour.random, colorCount)] * RandomRange(behaviour.random, 0.8f, 1.2f);
    look.heightScale = RandomRange(behaviour.random, kMinHeightScale, kMaxHeightScale);
    look.variation = RandomUnit(behaviour.random);
    look.walkPhase = RandomRange(behaviour.random, 0.0f, kTwoPi);
    look.gesturePhase = RandomUnit(behaviour.random);
    return agent;
}

// Sets up what depends on the person's final speed and size: the length of its stride.
void Spawner::FinishLook(uint32_t agent) {
    AgentBehaviour& behaviour = state.behaviour[agent];
    AgentLook& look = state.looks[agent];
    // Taller and faster people take longer steps.
    const float baseCycle = 1.25f + 0.3f * RandomUnit(behaviour.random);
    behaviour.gaitLength = baseCycle * look.heightScale * (0.8f + 0.2f * behaviour.preferredSpeed / 1.3f);
}

void Spawner::FaceTowards(uint32_t agent, glm::vec2 point) {
    const glm::vec2 direction = point - state.motion[agent].position;
    state.looks[agent].heading = std::atan2(direction.x, direction.y);
}

}  // namespace

void SpawnCrowd(const CrowdSettings& settings, Walkways& walkways, CrowdState& state) {
    Spawner spawner(settings, walkways, state);
    spawner.Run();
}

}  // namespace crowd
