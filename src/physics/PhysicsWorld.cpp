#include "physics/PhysicsWorld.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "physics/Collision.h"

namespace physics {
namespace {

// A grid cell that holds a body must not be one of thousands: bodies spanning more cells than this (a
// ground slab the size of a city, or a huge moving body) are tested against every other body instead.
constexpr int64_t kMaxCellsPerBody = 4096;
// A contact point within this distance of one from the last step is the same point, and starts from its impulses.
constexpr float kWarmStartDistance = 0.06f;
constexpr float kDensityFloor = 1e-3f;
constexpr float kMinSize = 0.005f;  // the smallest size of a shape: 5 mm

uint64_t CellKey(const glm::ivec3& cell) {
    // 21 bits per axis, with an offset so that negative cells pack too. Cells that are very far apart
    // can share a key, which only adds candidates that the bounds test throws out.
    constexpr int kOffset = 1 << 20;
    const auto pack = [](int value) { return static_cast<uint64_t>(value + kOffset) & 0x1FFFFFu; };
    return (pack(cell.x) << 42) | (pack(cell.y) << 21) | pack(cell.z);
}

glm::ivec3 CellOf(glm::vec3 point, float cellSize) {
    return glm::ivec3(glm::floor(point / cellSize));
}

int64_t CellCount(const glm::ivec3& low, const glm::ivec3& high) {
    return static_cast<int64_t>(high.x - low.x + 1) * static_cast<int64_t>(high.y - low.y + 1) *
           static_cast<int64_t>(high.z - low.z + 1);
}

bool BoundsOverlap(glm::vec3 minA, glm::vec3 maxA, glm::vec3 minB, glm::vec3 maxB) {
    return minA.x <= maxB.x && minB.x <= maxA.x && minA.y <= maxB.y && minB.y <= maxA.y && minA.z <= maxB.z &&
           minB.z <= maxA.z;
}

bool IsFinite(glm::vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool IsFinite(const glm::quat& q) {
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

// Bodies whose position has gone wrong are left out of the grids, so that one bad number cannot spread.
bool HasUsableBounds(const Body& body) {
    return IsFinite(body.boundsMin) && IsFinite(body.boundsMax);
}

uint64_t PairKey(uint32_t a, uint32_t b) {
    return (static_cast<uint64_t>(a) << 32) | b;
}

glm::mat3 InverseInertiaInWorld(const Body& body) {
    const glm::mat3 axes = glm::mat3_cast(body.orientation);
    return axes * glm::mat3(glm::vec3(body.inverseInertiaLocal.x, 0.0f, 0.0f), glm::vec3(0.0f, body.inverseInertiaLocal.y, 0.0f),
                            glm::vec3(0.0f, 0.0f, body.inverseInertiaLocal.z)) *
           glm::transpose(axes);
}

void UpdateBounds(Body& body) {
    glm::vec3 extent(body.shape.radius);
    if (body.shape.type == ShapeType::Box) {
        const glm::mat3 axes = glm::mat3_cast(body.orientation);
        const glm::vec3 half = body.shape.halfExtents;
        extent = glm::abs(axes[0]) * half.x + glm::abs(axes[1]) * half.y + glm::abs(axes[2]) * half.z;
    }
    body.boundsMin = body.position - extent;
    body.boundsMax = body.position + extent;
}

// Two unit vectors that are perpendicular to `normal` and to each other.
void TangentsOf(glm::vec3 normal, glm::vec3& first, glm::vec3& second) {
    const glm::vec3 helper = std::abs(normal.x) < 0.57f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    first = glm::normalize(glm::cross(normal, helper));
    second = glm::cross(normal, first);
}

bool IsStill(const Body& body, const PhysicsSettings& settings) {
    return glm::length(body.linearVelocity) < settings.sleepLinearSpeed &&
           glm::length(body.angularVelocity) < settings.sleepAngularSpeed;
}

float FiniteOr(float value, float fallback) {
    return std::isfinite(value) ? value : fallback;
}

glm::vec3 FiniteOr(glm::vec3 value, glm::vec3 fallback) {
    return IsFinite(value) ? value : fallback;
}

}  // namespace

// One contact point as the solver sees it: the lever arms to the two bodies, how hard each of the three
// directions (the normal and two tangents) is to push, and the speed the normal should reach.
struct PhysicsWorld::Constraint {
    uint32_t a = 0;
    uint32_t b = 0;
    glm::vec3 normal{0.0f};
    glm::vec3 tangent[2] = {glm::vec3(0.0f), glm::vec3(0.0f)};
    glm::vec3 leverA{0.0f};
    glm::vec3 leverB{0.0f};
    bool movesA = false;  // false for a body that is static or asleep, which the solver must not touch
    bool movesB = false;
    float inverseMassA = 0.0f;
    float inverseMassB = 0.0f;
    glm::mat3 inverseInertiaA{0.0f};
    glm::mat3 inverseInertiaB{0.0f};
    float normalMass = 0.0f;
    float tangentMass[2] = {0.0f, 0.0f};
    float targetSpeed = 0.0f;  // along the normal, away from the contact
    float friction = 0.0f;
    float rollingResistance = 0.0f;
    ContactPoint* point = nullptr;
};

uint32_t PhysicsWorld::AddBody(const BodyDescription& description) {
    Body body;
    body.shape = description.shape;
    body.shape.radius = std::max(FiniteOr(body.shape.radius, 0.5f), kMinSize);
    body.shape.halfExtents = glm::max(FiniteOr(body.shape.halfExtents, glm::vec3(0.5f)), glm::vec3(kMinSize));
    body.material = description.material;
    body.position = FiniteOr(description.position, glm::vec3(0.0f));
    body.orientation = IsFinite(description.orientation) && glm::length(description.orientation) > 1e-6f
                           ? glm::normalize(description.orientation)
                           : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    body.isStatic = description.isStatic;
    if (!description.isStatic) {
        body.linearVelocity = FiniteOr(description.linearVelocity, glm::vec3(0.0f));
        body.angularVelocity = FiniteOr(description.angularVelocity, glm::vec3(0.0f));
        const float density = std::max(FiniteOr(description.density, 1000.0f), kDensityFloor);
        float mass;
        glm::vec3 inertia;
        if (body.shape.type == ShapeType::Sphere) {
            const float r = body.shape.radius;
            mass = density * (4.0f / 3.0f) * 3.14159265f * r * r * r;
            inertia = glm::vec3(0.4f * mass * r * r);
        } else {
            const glm::vec3 h = body.shape.halfExtents;
            mass = density * 8.0f * h.x * h.y * h.z;
            inertia = glm::vec3(mass / 3.0f * (h.y * h.y + h.z * h.z), mass / 3.0f * (h.x * h.x + h.z * h.z),
                                mass / 3.0f * (h.x * h.x + h.y * h.y));
        }
        body.inverseMass = 1.0f / mass;
        body.inverseInertiaLocal = 1.0f / inertia;
        body.inverseInertiaWorld = InverseInertiaInWorld(body);
    }
    UpdateBounds(body);
    bodies.push_back(body);
    if (body.isStatic) {
        staticGridIsStale = true;
    }
    return static_cast<uint32_t>(bodies.size() - 1);
}

void PhysicsWorld::ApplyImpulse(uint32_t index, glm::vec3 impulse, glm::vec3 worldPoint) {
    Body& body = bodies[index];
    if (body.isStatic) {
        return;
    }
    Wake(index);
    body.linearVelocity += impulse * body.inverseMass;
    body.angularVelocity += InverseInertiaInWorld(body) * glm::cross(worldPoint - body.position, impulse);
}

void PhysicsWorld::SetVelocity(uint32_t index, glm::vec3 linear, glm::vec3 angular) {
    Body& body = bodies[index];
    if (body.isStatic) {
        return;
    }
    Wake(index);
    body.linearVelocity = linear;
    body.angularVelocity = angular;
}

void PhysicsWorld::Wake(uint32_t index) {
    Body& body = bodies[index];
    body.quietSeconds = 0.0f;
    if (!body.isAsleep) {
        return;
    }
    const auto group = sleepingGroups.find(body.sleepGroup);
    if (group == sleepingGroups.end()) {
        body.isAsleep = false;
        body.sleepGroup = kInvalidBody;
        return;
    }
    // A stack wakes as a whole, or the bodies of it left asleep would hang in the air.
    for (uint32_t member : group->second) {
        bodies[member].isAsleep = false;
        bodies[member].quietSeconds = 0.0f;
        bodies[member].sleepGroup = kInvalidBody;
    }
    sleepingGroups.erase(group);
}

void PhysicsWorld::AddForce(uint32_t index, glm::vec3 force, glm::vec3 worldPoint) {
    Body& body = bodies[index];
    if (body.isStatic || body.isAsleep || !IsFinite(force) || !IsFinite(worldPoint)) {
        return;
    }
    body.externalForce += force;
    body.externalTorque += glm::cross(worldPoint - body.position, force);
}

void PhysicsWorld::Disable(uint32_t index) {
    Body& body = bodies[index];
    if (body.isStatic) {
        return;
    }
    Wake(index);
    body.isStatic = true;
    body.inverseMass = 0.0f;
    body.inverseInertiaLocal = glm::vec3(0.0f);
    body.inverseInertiaWorld = glm::mat3(0.0f);
    body.linearVelocity = glm::vec3(0.0f);
    body.angularVelocity = glm::vec3(0.0f);
    staticGridIsStale = true;
}

size_t PhysicsWorld::AwakeBodyCount() const {
    return static_cast<size_t>(std::count_if(bodies.begin(), bodies.end(), [](const Body& b) { return !b.isStatic && !b.isAsleep; }));
}

float PhysicsWorld::KineticEnergy() const {
    float energy = 0.0f;
    for (const Body& body : bodies) {
        if (body.isStatic) {
            continue;
        }
        energy += 0.5f * glm::dot(body.linearVelocity, body.linearVelocity) / body.inverseMass;
        const glm::vec3 spin = glm::transpose(glm::mat3_cast(body.orientation)) * body.angularVelocity;
        for (int i = 0; i < 3; ++i) {
            energy += 0.5f * spin[i] * spin[i] / body.inverseInertiaLocal[i];
        }
    }
    return energy;
}

void PhysicsWorld::Update(float seconds) {
    leftoverSeconds += seconds;
    int steps = 0;
    while (leftoverSeconds >= settings.fixedStep && steps < settings.maxStepsPerUpdate) {
        Step();
        leftoverSeconds -= settings.fixedStep;
        ++steps;
    }
    if (leftoverSeconds >= settings.fixedStep) {
        leftoverSeconds = 0.0f;  // behind for good: drop the time instead of chasing it
    }
}

void PhysicsWorld::Step() {
    RefreshBodies();
    BuildMovingGrid();
    WakeSleepersTouchedByMovingBodies();
    std::vector<std::pair<uint32_t, uint32_t>> pairs;
    FindPairs(pairs);
    BuildManifolds(pairs);
    Solve();
    RemoveOverlaps();
    Integrate();
    UpdateSleep();

    previousManifolds.clear();
    for (const Manifold& manifold : manifolds) {
        previousManifolds[PairKey(manifold.a, manifold.b)] = manifold;
    }
}

// Applies gravity and brings the derived values of every awake body up to date.
void PhysicsWorld::RefreshBodies() {
    for (Body& body : bodies) {
        if (body.isStatic || body.isAsleep) {
            continue;
        }
        body.inverseInertiaWorld = InverseInertiaInWorld(body);
        body.appliedVelocityStep = (settings.gravity + body.externalForce * body.inverseMass) * settings.fixedStep;
        body.linearVelocity += body.appliedVelocityStep;
        body.angularVelocity += body.inverseInertiaWorld * body.externalTorque * settings.fixedStep;
        body.externalForce = glm::vec3(0.0f);
        body.externalTorque = glm::vec3(0.0f);
        UpdateBounds(body);
    }
}

void PhysicsWorld::RebuildStaticGrid() {
    staticGrid.clear();
    hugeStatics.clear();
    for (uint32_t index = 0; index < static_cast<uint32_t>(bodies.size()); ++index) {
        const Body& body = bodies[index];
        if (!body.isStatic) {
            continue;
        }
        const glm::ivec3 low = CellOf(body.boundsMin - kContactMargin, settings.staticCellSize);
        const glm::ivec3 high = CellOf(body.boundsMax + kContactMargin, settings.staticCellSize);
        if (CellCount(low, high) > kMaxCellsPerBody) {
            hugeStatics.push_back(index);
            continue;
        }
        for (int x = low.x; x <= high.x; ++x) {
            for (int y = low.y; y <= high.y; ++y) {
                for (int z = low.z; z <= high.z; ++z) {
                    staticGrid[CellKey({x, y, z})].push_back(index);
                }
            }
        }
    }
    staticGridIsStale = false;
}

// Moving bodies, awake or not, in a grid of their own for this step.
void PhysicsWorld::BuildMovingGrid() {
    movingGrid.clear();
    hugeMoving.clear();
    for (uint32_t index = 0; index < static_cast<uint32_t>(bodies.size()); ++index) {
        const Body& body = bodies[index];
        if (body.isStatic || !HasUsableBounds(body)) {
            continue;
        }
        const glm::ivec3 low = CellOf(body.boundsMin - kContactMargin, settings.dynamicCellSize);
        const glm::ivec3 high = CellOf(body.boundsMax + kContactMargin, settings.dynamicCellSize);
        if (CellCount(low, high) > kMaxCellsPerBody) {
            hugeMoving.push_back(index);
            continue;
        }
        for (int x = low.x; x <= high.x; ++x) {
            for (int y = low.y; y <= high.y; ++y) {
                for (int z = low.z; z <= high.z; ++z) {
                    movingGrid[CellKey({x, y, z})].push_back(index);
                }
            }
        }
    }
}

// Adds to `candidates` the moving bodies that may overlap the box from `low` to `high`.
void PhysicsWorld::CollectMovingCandidates(glm::vec3 low, glm::vec3 high, std::vector<uint32_t>& candidates) const {
    candidates.insert(candidates.end(), hugeMoving.begin(), hugeMoving.end());
    const glm::ivec3 cellLow = CellOf(low, settings.dynamicCellSize);
    const glm::ivec3 cellHigh = CellOf(high, settings.dynamicCellSize);
    if (CellCount(cellLow, cellHigh) > kMaxCellsPerBody) {
        for (uint32_t index = 0; index < static_cast<uint32_t>(bodies.size()); ++index) {
            if (!bodies[index].isStatic && HasUsableBounds(bodies[index])) {
                candidates.push_back(index);
            }
        }
        return;
    }
    for (int x = cellLow.x; x <= cellHigh.x; ++x) {
        for (int y = cellLow.y; y <= cellHigh.y; ++y) {
            for (int z = cellLow.z; z <= cellHigh.z; ++z) {
                const auto cell = movingGrid.find(CellKey({x, y, z}));
                if (cell != movingGrid.end()) {
                    candidates.insert(candidates.end(), cell->second.begin(), cell->second.end());
                }
            }
        }
    }
}

// Something that is moving wakes the group of sleeping bodies it is about to touch, before contacts are
// found, so that the whole group takes part in this step: the body it hits, and the ones it rests on.
// Something that is only settling does not.
void PhysicsWorld::WakeSleepersTouchedByMovingBodies() {
    std::vector<uint32_t> candidates;
    for (uint32_t index = 0; index < static_cast<uint32_t>(bodies.size()); ++index) {
        const Body& body = bodies[index];
        if (body.isStatic || body.isAsleep || !HasUsableBounds(body)) {
            continue;
        }
        // Gravity, and any forces from outside such as the lift of water, have changed the speed of every awake body
        // this step. A body at rest on the ground, or floating, is still, and must not wake the bodies it rests
        // among because of it.
        Body asBeforeForces = body;
        asBeforeForces.linearVelocity -= body.appliedVelocityStep;
        if (IsStill(asBeforeForces, settings)) {
            continue;
        }
        const glm::vec3 reach =
            glm::min(glm::abs(body.linearVelocity) * settings.fixedStep, glm::vec3(settings.maxLookAhead)) + kContactMargin;
        candidates.clear();
        CollectMovingCandidates(body.boundsMin - reach, body.boundsMax + reach, candidates);
        std::sort(candidates.begin(), candidates.end());
        candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
        for (uint32_t other : candidates) {
            const Body& partner = bodies[other];
            if (partner.isAsleep && BoundsOverlap(body.boundsMin - reach, body.boundsMax + reach, partner.boundsMin, partner.boundsMax)) {
                Wake(other);
            }
        }
    }
}

// Lists the pairs of bodies whose bounds overlap and where at least one is awake, each once, with the
// lower index first and in order of index. A body counts as reaching as far as it moves in a step, so
// two bodies that meet head on are found when the gap is the sum of their reaches.
void PhysicsWorld::FindPairs(std::vector<std::pair<uint32_t, uint32_t>>& pairs) {
    if (staticGridIsStale) {
        RebuildStaticGrid();
    }
    const auto reachOf = [this](const Body& body) {
        return glm::min(glm::abs(body.linearVelocity) * settings.fixedStep, glm::vec3(settings.maxLookAhead)) + kContactMargin;
    };
    // The furthest any awake body reaches this step. A partner may come from that far beyond the reach of the
    // body that looks for it, and the search is widened by no more, so a heap at rest costs nothing extra.
    glm::vec3 longestReach(kContactMargin);
    for (const Body& body : bodies) {
        if (!body.isStatic && !body.isAsleep && HasUsableBounds(body)) {
            longestReach = glm::max(longestReach, reachOf(body));
        }
    }
    std::vector<uint32_t> candidates;
    for (uint32_t index = 0; index < static_cast<uint32_t>(bodies.size()); ++index) {
        const Body& body = bodies[index];
        if (body.isStatic || body.isAsleep || !HasUsableBounds(body)) {
            continue;
        }
        // A body that is moving fast looks ahead by the distance it covers in a step, up to a limit.
        const glm::vec3 reach = reachOf(body);
        const glm::vec3 low = body.boundsMin - reach;
        const glm::vec3 high = body.boundsMax + reach;

        candidates.assign(hugeStatics.begin(), hugeStatics.end());
        const glm::ivec3 staticLow = CellOf(low, settings.staticCellSize);
        const glm::ivec3 staticHigh = CellOf(high, settings.staticCellSize);
        if (CellCount(staticLow, staticHigh) > kMaxCellsPerBody) {
            for (uint32_t other = 0; other < static_cast<uint32_t>(bodies.size()); ++other) {
                if (bodies[other].isStatic) {
                    candidates.push_back(other);
                }
            }
        } else {
            for (int x = staticLow.x; x <= staticHigh.x; ++x) {
                for (int y = staticLow.y; y <= staticHigh.y; ++y) {
                    for (int z = staticLow.z; z <= staticHigh.z; ++z) {
                        const auto cell = staticGrid.find(CellKey({x, y, z}));
                        if (cell != staticGrid.end()) {
                            candidates.insert(candidates.end(), cell->second.begin(), cell->second.end());
                        }
                    }
                }
            }
        }
        // A partner that is itself moving fast may come from beyond this body's reach.
        CollectMovingCandidates(low - longestReach, high + longestReach, candidates);

        std::sort(candidates.begin(), candidates.end());
        candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
        for (uint32_t other : candidates) {
            if (other == index) {
                continue;
            }
            const Body& partner = bodies[other];
            const glm::vec3 partnerReach = partner.isStatic ? glm::vec3(kContactMargin) : reachOf(partner);
            if (BoundsOverlap(low, high, partner.boundsMin - partnerReach, partner.boundsMax + partnerReach)) {
                pairs.emplace_back(std::min(index, other), std::max(index, other));
            }
        }
    }
    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
}

void PhysicsWorld::BuildManifolds(const std::vector<std::pair<uint32_t, uint32_t>>& pairs) {
    manifolds.clear();
    for (const auto& [a, b] : pairs) {
        // Bodies that are closing in on each other get contacts before they touch, as far apart as they
        // will have travelled towards each other by the end of the step.
        const float closing = glm::length(bodies[a].linearVelocity - bodies[b].linearVelocity) * settings.fixedStep;
        const float margin = kContactMargin + std::min(closing, settings.maxLookAhead);

        Manifold manifold;
        manifold.a = a;
        manifold.b = b;
        if (!Collide(bodies[a], bodies[b], manifold, margin)) {
            continue;
        }

        // Start each point from the impulses of the nearest point of the last step.
        const auto previous = previousManifolds.find(PairKey(a, b));
        if (previous != previousManifolds.end()) {
            bool used[kMaxContactsPerPair] = {};
            for (int i = 0; i < manifold.count; ++i) {
                int nearest = -1;
                float nearestDistance = kWarmStartDistance;
                for (int j = 0; j < previous->second.count; ++j) {
                    const float distance = glm::distance(manifold.points[i].position, previous->second.points[j].position);
                    if (!used[j] && distance < nearestDistance) {
                        nearestDistance = distance;
                        nearest = j;
                    }
                }
                if (nearest >= 0) {
                    used[nearest] = true;
                    const ContactPoint& old = previous->second.points[nearest];
                    manifold.points[i].normalImpulse = old.normalImpulse;
                    manifold.points[i].tangentImpulse[0] = old.tangentImpulse[0];
                    manifold.points[i].tangentImpulse[1] = old.tangentImpulse[1];
                    manifold.points[i].rollingImpulse = old.rollingImpulse;
                }
            }
        }
        // The bounce of a point comes from the speed of the nearest point of the last step, which may have
        // moved as far as the bodies closed in.
        if (previous != previousManifolds.end()) {
            const float radius = kWarmStartDistance + closing * 1.5f;
            for (int i = 0; i < manifold.count; ++i) {
                float nearestDistance = radius;
                for (int j = 0; j < previous->second.count; ++j) {
                    const float distance = glm::distance(manifold.points[i].position, previous->second.points[j].position);
                    if (distance < nearestDistance) {
                        nearestDistance = distance;
                        manifold.points[i].carriedImpactSpeed = previous->second.points[j].impactSpeed;
                    }
                }
            }
        }
        manifolds.push_back(manifold);
    }
}

void PhysicsWorld::Solve() {
    const float step = settings.fixedStep;
    std::vector<Constraint> constraints;
    constraints.reserve(manifolds.size() * 2);

    for (Manifold& manifold : manifolds) {
        Body& bodyA = bodies[manifold.a];
        Body& bodyB = bodies[manifold.b];
        const float friction = std::sqrt(bodyA.material.friction * bodyB.material.friction);
        const float restitution = std::max(bodyA.material.restitution, bodyB.material.restitution);
        // A ball is slowed in rolling by the full resistance. A box gets a small share of it, enough to calm a
        // tall stack but not to hold a box on an edge it should tip over.
        const bool involvesSphere = bodyA.shape.type == ShapeType::Sphere || bodyB.shape.type == ShapeType::Sphere;
        const float rolling = std::sqrt(bodyA.material.rollingResistance * bodyB.material.rollingResistance) *
                              (involvesSphere ? 1.0f : settings.boxRollingShare);
        const bool movesA = !bodyA.isStatic && !bodyA.isAsleep;
        const bool movesB = !bodyB.isStatic && !bodyB.isAsleep;
        glm::vec3 tangents[2];
        TangentsOf(manifold.normal, tangents[0], tangents[1]);

        for (int i = 0; i < manifold.count; ++i) {
            ContactPoint& point = manifold.points[i];
            Constraint c;
            c.a = manifold.a;
            c.b = manifold.b;
            c.normal = manifold.normal;
            c.tangent[0] = tangents[0];
            c.tangent[1] = tangents[1];
            c.leverA = point.position - bodyA.position;
            c.leverB = point.position - bodyB.position;
            c.movesA = movesA;
            c.movesB = movesB;
            c.inverseMassA = movesA ? bodyA.inverseMass : 0.0f;
            c.inverseMassB = movesB ? bodyB.inverseMass : 0.0f;
            c.inverseInertiaA = movesA ? bodyA.inverseInertiaWorld : glm::mat3(0.0f);
            c.inverseInertiaB = movesB ? bodyB.inverseInertiaWorld : glm::mat3(0.0f);
            c.friction = friction;
            c.rollingResistance = rolling;
            c.point = &point;

            // How much speed one unit of impulse along a direction changes, inverted.
            const auto effectiveMass = [&](glm::vec3 direction) {
                const glm::vec3 turnA = glm::cross(c.inverseInertiaA * glm::cross(c.leverA, direction), c.leverA);
                const glm::vec3 turnB = glm::cross(c.inverseInertiaB * glm::cross(c.leverB, direction), c.leverB);
                const float stiffness = c.inverseMassA + c.inverseMassB + glm::dot(direction, turnA + turnB);
                return stiffness > 0.0f ? 1.0f / stiffness : 0.0f;
            };
            c.normalMass = effectiveMass(c.normal);
            c.tangentMass[0] = effectiveMass(c.tangent[0]);
            c.tangentMass[1] = effectiveMass(c.tangent[1]);

            const glm::vec3 relative = bodyB.linearVelocity + glm::cross(bodyB.angularVelocity, c.leverB) -
                                       bodyA.linearVelocity - glm::cross(bodyA.angularVelocity, c.leverA);
            const float approachSpeed = glm::dot(relative, c.normal);
            // Gravity was added to the speeds at the start of the step. A bounce comes from the speed
            // the bodies were closing at before it, or a ball would bounce a little higher than it fell.
            const float closingFromGravity = glm::dot(settings.gravity * step, c.normal) * ((movesB ? 1.0f : 0.0f) - (movesA ? 1.0f : 0.0f));
            const float impactApproach = approachSpeed - closingFromGravity;
            // The speed the contact should reach along the normal. While the bodies are still apart they may
            // approach only as fast as closes the gap, and the speed they were closing at is remembered. Once
            // they touch, a fast impact bounces: apart at the restitution times the speed they came in at,
            // which is the remembered speed if they were stopped at the surface by the step before. (Overlaps
            // are removed by moving the bodies, after the speeds are solved, so that they gain no speed.)
            float target = 0.0f;
            if (point.depth < -settings.touchTolerance) {
                target = point.depth / step;
                // Remember the fastest approach: a contact that stays apart for several steps has its speed
                // cut at each of them, and the bounce must come from the speed before the first cut. The
                // memory ends as soon as the bodies stop closing in, so that a body hovering a few millimetres
                // above a surface does not bounce from an old impact each time it touches.
                if (impactApproach < -settings.restitutionThreshold) {
                    point.impactSpeed = std::min(point.carriedImpactSpeed, impactApproach);
                } else if (impactApproach <= 0.0f) {
                    point.impactSpeed = point.carriedImpactSpeed;
                }
            } else {
                const float impact = std::min(impactApproach, point.carriedImpactSpeed);
                if (impact < -settings.restitutionThreshold) {
                    target = restitution * -impact;
                }
            }
            c.targetSpeed = target;
            constraints.push_back(c);
        }
    }

    // The impulse a constraint applies to its two bodies, equal and opposite. A body that does not move
    // is not touched at all: even a zero effect would spread a bad number to it.
    const auto apply = [&](const Constraint& c, glm::vec3 impulse) {
        if (c.movesA) {
            Body& bodyA = bodies[c.a];
            bodyA.linearVelocity -= impulse * c.inverseMassA;
            bodyA.angularVelocity -= c.inverseInertiaA * glm::cross(c.leverA, impulse);
        }
        if (c.movesB) {
            Body& bodyB = bodies[c.b];
            bodyB.linearVelocity += impulse * c.inverseMassB;
            bodyB.angularVelocity += c.inverseInertiaB * glm::cross(c.leverB, impulse);
        }
    };
    // A pure turning impulse, equal and opposite on the two bodies.
    const auto applyTurn = [&](const Constraint& c, glm::vec3 turn) {
        if (c.movesA) {
            bodies[c.a].angularVelocity -= c.inverseInertiaA * turn;
        }
        if (c.movesB) {
            bodies[c.b].angularVelocity += c.inverseInertiaB * turn;
        }
    };
    const auto relativeVelocity = [&](const Constraint& c) {
        const Body& bodyA = bodies[c.a];
        const Body& bodyB = bodies[c.b];
        return bodyB.linearVelocity + glm::cross(bodyB.angularVelocity, c.leverB) - bodyA.linearVelocity -
               glm::cross(bodyA.angularVelocity, c.leverA);
    };

    // Start from the impulses of the last step.
    for (const Constraint& c : constraints) {
        apply(c, c.normal * c.point->normalImpulse + c.tangent[0] * c.point->tangentImpulse[0] +
                     c.tangent[1] * c.point->tangentImpulse[1]);
        applyTurn(c, c.point->rollingImpulse);
    }

    for (int iteration = 0; iteration < settings.solverIterations; ++iteration) {
        for (const Constraint& c : constraints) {
            // Friction first: it can use no more than the friction coefficient times the normal impulse.
            const float limit = c.friction * c.point->normalImpulse;
            for (int t = 0; t < 2; ++t) {
                const float change = -glm::dot(relativeVelocity(c), c.tangent[t]) * c.tangentMass[t];
                const float updated = std::clamp(c.point->tangentImpulse[t] + change, -limit, limit);
                apply(c, c.tangent[t] * (updated - c.point->tangentImpulse[t]));
                c.point->tangentImpulse[t] = updated;
            }
            // Rolling and spinning resistance: a turning impulse against the spin of the two bodies
            // relative to each other, about any axis, that the weight pressing them together can
            // supply and no more.
            const glm::vec3 spin = bodies[c.b].angularVelocity - bodies[c.a].angularVelocity;
            const float spinSpeed = glm::length(spin);
            if (spinSpeed > 1e-6f && c.rollingResistance > 0.0f) {
                const glm::vec3 direction = spin / spinSpeed;
                const float stiffness = glm::dot(direction, (c.inverseInertiaA + c.inverseInertiaB) * direction);
                if (stiffness > 0.0f) {
                    glm::vec3 updated = c.point->rollingImpulse - direction * (spinSpeed / stiffness);
                    const float longest = c.rollingResistance * c.point->normalImpulse;
                    const float length = glm::length(updated);
                    if (length > longest) {
                        updated *= longest / length;
                    }
                    applyTurn(c, updated - c.point->rollingImpulse);
                    c.point->rollingImpulse = updated;
                }
            }
            // Then the normal, which can push but never pull.
            const float change = (c.targetSpeed - glm::dot(relativeVelocity(c), c.normal)) * c.normalMass;
            const float updated = std::max(c.point->normalImpulse + change, 0.0f);
            apply(c, c.normal * (updated - c.point->normalImpulse));
            c.point->normalImpulse = updated;
        }
    }
}

// Moves bodies that overlap apart along the contact normals, a share of the overlap at a time. This is
// done to the positions and not to the speeds, so bodies that start inside each other, or are pushed into
// each other, are separated without being flung apart.
void PhysicsWorld::RemoveOverlaps() {
    std::vector<glm::vec3> shift(bodies.size(), glm::vec3(0.0f));
    for (int pass = 0; pass < settings.positionIterations; ++pass) {
        for (const Manifold& manifold : manifolds) {
            const Body& bodyA = bodies[manifold.a];
            const Body& bodyB = bodies[manifold.b];
            const float inverseMassA = (!bodyA.isStatic && !bodyA.isAsleep) ? bodyA.inverseMass : 0.0f;
            const float inverseMassB = (!bodyB.isStatic && !bodyB.isAsleep) ? bodyB.inverseMass : 0.0f;
            const float inverseSum = inverseMassA + inverseMassB;
            if (inverseSum <= 0.0f) {
                continue;
            }
            for (int i = 0; i < manifold.count; ++i) {
                // The overlap that is left after the moves already made.
                const float depth =
                    manifold.points[i].depth - glm::dot(shift[manifold.b] - shift[manifold.a], manifold.normal);
                const float correction =
                    std::min(settings.maxCorrectionPerStep, settings.correctionRate * (depth - settings.penetrationSlop));
                if (correction <= 0.0f) {
                    continue;
                }
                shift[manifold.a] -= manifold.normal * (correction * inverseMassA / inverseSum);
                shift[manifold.b] += manifold.normal * (correction * inverseMassB / inverseSum);
            }
        }
    }
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (!bodies[i].isStatic && !bodies[i].isAsleep) {
            const float length = glm::length(shift[i]);
            const float scale = length > settings.maxCorrectionPerStep ? settings.maxCorrectionPerStep / length : 1.0f;
            bodies[i].position += shift[i] * scale;
        }
    }
}

void PhysicsWorld::Integrate() {
    const float step = settings.fixedStep;
    for (Body& body : bodies) {
        if (body.isStatic || body.isAsleep) {
            continue;
        }
        const glm::vec3 previousPosition = body.position;
        const glm::quat previousOrientation = body.orientation;
        body.angularVelocity *= std::max(0.0f, 1.0f - settings.angularDamping * step);
        body.position += body.linearVelocity * step;
        const glm::quat spin(0.0f, body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z);
        body.orientation = glm::normalize(body.orientation + 0.5f * step * (spin * body.orientation));
        if (!IsFinite(body.position) || !IsFinite(body.orientation)) {
            // Something went wrong with this body: it stops where it was rather than spreading a bad number.
            body.position = previousPosition;
            body.orientation = previousOrientation;
            body.linearVelocity = glm::vec3(0.0f);
            body.angularVelocity = glm::vec3(0.0f);
        }
    }
}

// Puts to sleep the groups of touching bodies that have all been still for long enough.
void PhysicsWorld::UpdateSleep() {
    const uint32_t count = static_cast<uint32_t>(bodies.size());
    for (Body& body : bodies) {
        if (body.isStatic || body.isAsleep) {
            continue;
        }
        body.quietSeconds = IsStill(body, settings) ? body.quietSeconds + settings.fixedStep : 0.0f;
    }

    // Bodies that touch belong to one group; a body that touches only static ones is a group alone.
    std::vector<uint32_t> group(count);
    std::iota(group.begin(), group.end(), 0u);
    const auto find = [&](uint32_t index) {
        while (group[index] != index) {
            group[index] = group[group[index]];
            index = group[index];
        }
        return index;
    };
    for (const Manifold& manifold : manifolds) {
        if (!bodies[manifold.a].isStatic && !bodies[manifold.b].isStatic) {
            group[find(manifold.a)] = find(manifold.b);
        }
    }
    std::vector<float> groupQuiet(count, std::numeric_limits<float>::max());
    for (uint32_t index = 0; index < count; ++index) {
        if (!bodies[index].isStatic && !bodies[index].isAsleep) {
            const uint32_t root = find(index);
            groupQuiet[root] = std::min(groupQuiet[root], bodies[index].quietSeconds);
        }
    }
    std::unordered_map<uint32_t, uint32_t> sleepGroupOfRoot;
    for (uint32_t index = 0; index < count; ++index) {
        Body& body = bodies[index];
        const uint32_t root = find(index);
        if (body.isStatic || body.isAsleep || groupQuiet[root] < settings.sleepSeconds) {
            continue;
        }
        const auto known = sleepGroupOfRoot.find(root);
        const uint32_t sleepGroup = known != sleepGroupOfRoot.end() ? known->second : nextSleepGroup++;
        sleepGroupOfRoot[root] = sleepGroup;
        body.isAsleep = true;
        body.sleepGroup = sleepGroup;
        body.linearVelocity = glm::vec3(0.0f);
        body.angularVelocity = glm::vec3(0.0f);
        sleepingGroups[sleepGroup].push_back(index);
    }
}

}  // namespace physics
