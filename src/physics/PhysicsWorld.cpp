#include "physics/PhysicsWorld.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "physics/Collision.h"

namespace physics {
namespace {

// A grid cell that holds a static body must not be one of thousands: bodies spanning more cells than this
// (a ground slab the size of a city) are tested against every moving body instead.
constexpr int64_t kMaxCellsPerStaticBody = 4096;
// A contact point within this distance of one from the last step is the same point, and starts from its impulses.
constexpr float kWarmStartDistance = 0.06f;
constexpr float kDensityFloor = 1e-3f;

uint64_t CellKey(const glm::ivec3& cell) {
    // 21 bits per axis, with an offset so that negative cells pack too.
    constexpr int kOffset = 1 << 20;
    const auto pack = [](int value) { return static_cast<uint64_t>(value + kOffset) & 0x1FFFFFu; };
    return (pack(cell.x) << 42) | (pack(cell.y) << 21) | pack(cell.z);
}

glm::ivec3 CellOf(glm::vec3 point, float cellSize) {
    return glm::ivec3(glm::floor(point / cellSize));
}

bool BoundsOverlap(glm::vec3 minA, glm::vec3 maxA, glm::vec3 minB, glm::vec3 maxB) {
    return minA.x <= maxB.x && minB.x <= maxA.x && minA.y <= maxB.y && minB.y <= maxA.y && minA.z <= maxB.z &&
           minB.z <= maxA.z;
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

}  // namespace

// One contact point as the solver sees it: the lever arms to the two bodies, how hard each of the three
// directions (the normal and two tangents) is to push, and the speed the normal should reach.
struct PhysicsWorld::Constraint {
    uint32_t a = 0;
    uint32_t b = 0;
    glm::vec3 normal{0.0f};
    glm::vec3 tangent[2];
    glm::vec3 leverA{0.0f};
    glm::vec3 leverB{0.0f};
    float inverseMassA = 0.0f;  // zero for a body that is static or asleep, which the solver must not move
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
    body.material = description.material;
    body.position = description.position;
    body.orientation = glm::normalize(description.orientation);
    body.isStatic = description.isStatic;
    if (!description.isStatic) {
        body.linearVelocity = description.linearVelocity;
        body.angularVelocity = description.angularVelocity;
        const float density = std::max(description.density, kDensityFloor);
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
    bodies[index].isAsleep = false;
    bodies[index].quietSeconds = 0.0f;
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
    if (steps == settings.maxStepsPerUpdate) {
        leftoverSeconds = 0.0f;  // behind for good: drop the time instead of chasing it
    }
}

void PhysicsWorld::Step() {
    RefreshBodies();
    std::vector<std::pair<uint32_t, uint32_t>> pairs;
    FindPairs(pairs);
    BuildManifolds(pairs);
    Solve();
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
        body.linearVelocity += settings.gravity * settings.fixedStep;
        body.inverseInertiaWorld = InverseInertiaInWorld(body);
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
        const int64_t cellCount = static_cast<int64_t>(high.x - low.x + 1) * (high.y - low.y + 1) * (high.z - low.z + 1);
        if (cellCount > kMaxCellsPerStaticBody) {
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

// Lists the pairs of bodies whose bounds overlap and where at least one is awake, each once, with the
// lower index first and in order of index.
void PhysicsWorld::FindPairs(std::vector<std::pair<uint32_t, uint32_t>>& pairs) {
    if (staticGridIsStale) {
        RebuildStaticGrid();
    }
    // Moving bodies, awake or not, in a grid of their own for this step.
    std::unordered_map<uint64_t, std::vector<uint32_t>> movingGrid;
    for (uint32_t index = 0; index < static_cast<uint32_t>(bodies.size()); ++index) {
        const Body& body = bodies[index];
        if (body.isStatic) {
            continue;
        }
        const glm::ivec3 low = CellOf(body.boundsMin - kContactMargin, settings.dynamicCellSize);
        const glm::ivec3 high = CellOf(body.boundsMax + kContactMargin, settings.dynamicCellSize);
        for (int x = low.x; x <= high.x; ++x) {
            for (int y = low.y; y <= high.y; ++y) {
                for (int z = low.z; z <= high.z; ++z) {
                    movingGrid[CellKey({x, y, z})].push_back(index);
                }
            }
        }
    }

    std::vector<uint32_t> candidates;
    for (uint32_t index = 0; index < static_cast<uint32_t>(bodies.size()); ++index) {
        const Body& body = bodies[index];
        if (body.isStatic || body.isAsleep) {
            continue;
        }
        // A body that is moving fast looks ahead by the distance it covers in a step.
        const glm::vec3 reach = glm::abs(body.linearVelocity) * settings.fixedStep + kContactMargin;
        const glm::vec3 low = body.boundsMin - reach;
        const glm::vec3 high = body.boundsMax + reach;

        candidates.assign(hugeStatics.begin(), hugeStatics.end());
        const glm::ivec3 staticLow = CellOf(low, settings.staticCellSize);
        const glm::ivec3 staticHigh = CellOf(high, settings.staticCellSize);
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
        const glm::ivec3 movingLow = CellOf(low, settings.dynamicCellSize);
        const glm::ivec3 movingHigh = CellOf(high, settings.dynamicCellSize);
        for (int x = movingLow.x; x <= movingHigh.x; ++x) {
            for (int y = movingLow.y; y <= movingHigh.y; ++y) {
                for (int z = movingLow.z; z <= movingHigh.z; ++z) {
                    const auto cell = movingGrid.find(CellKey({x, y, z}));
                    if (cell != movingGrid.end()) {
                        candidates.insert(candidates.end(), cell->second.begin(), cell->second.end());
                    }
                }
            }
        }

        std::sort(candidates.begin(), candidates.end());
        candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
        for (uint32_t other : candidates) {
            if (other == index) {
                continue;
            }
            const Body& partner = bodies[other];
            if (BoundsOverlap(low, high, partner.boundsMin - kContactMargin, partner.boundsMax + kContactMargin)) {
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
        Manifold manifold;
        manifold.a = a;
        manifold.b = b;
        if (!Collide(bodies[a], bodies[b], manifold)) {
            continue;
        }
        // Something that is moving wakes whatever it touches; something that is only settling does not.
        for (uint32_t sleeper : {a, b}) {
            const uint32_t other = sleeper == a ? b : a;
            if (bodies[sleeper].isAsleep && !bodies[other].isStatic && !bodies[other].isAsleep &&
                !IsStill(bodies[other], settings)) {
                Wake(sleeper);
            }
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
        const float rolling = std::sqrt(bodyA.material.rollingResistance * bodyB.material.rollingResistance);
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
            // The speed the contact should reach along the normal: apart fast enough to bounce, or to undo
            // an overlap; or, while the bodies are still a little apart, no faster together than closes the gap.
            float target;
            if (point.depth < 0.0f) {
                target = point.depth / step;
            } else {
                target = std::min(settings.correctionRate / step * std::max(point.depth - settings.penetrationSlop, 0.0f),
                                  settings.maxCorrectionSpeed);
                if (approachSpeed < -settings.restitutionThreshold) {
                    target = std::max(target, -restitution * approachSpeed);
                }
            }
            c.targetSpeed = target;
            constraints.push_back(c);
        }
    }

    // The impulse a constraint applies to its two bodies, equal and opposite.
    const auto apply = [&](const Constraint& c, glm::vec3 impulse) {
        Body& bodyA = bodies[c.a];
        Body& bodyB = bodies[c.b];
        bodyA.linearVelocity -= impulse * c.inverseMassA;
        bodyA.angularVelocity -= c.inverseInertiaA * glm::cross(c.leverA, impulse);
        bodyB.linearVelocity += impulse * c.inverseMassB;
        bodyB.angularVelocity += c.inverseInertiaB * glm::cross(c.leverB, impulse);
    };
    // A pure turning impulse, equal and opposite on the two bodies.
    const auto applyTurn = [&](const Constraint& c, glm::vec3 turn) {
        bodies[c.a].angularVelocity -= c.inverseInertiaA * turn;
        bodies[c.b].angularVelocity += c.inverseInertiaB * turn;
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

void PhysicsWorld::Integrate() {
    const float step = settings.fixedStep;
    for (Body& body : bodies) {
        if (body.isStatic || body.isAsleep) {
            continue;
        }
        body.angularVelocity *= std::max(0.0f, 1.0f - settings.angularDamping * step);
        body.position += body.linearVelocity * step;
        const glm::quat spin(0.0f, body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z);
        body.orientation = glm::normalize(body.orientation + 0.5f * step * (spin * body.orientation));
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
    for (uint32_t index = 0; index < count; ++index) {
        Body& body = bodies[index];
        if (!body.isStatic && !body.isAsleep && groupQuiet[find(index)] >= settings.sleepSeconds) {
            body.isAsleep = true;
            body.linearVelocity = glm::vec3(0.0f);
            body.angularVelocity = glm::vec3(0.0f);
        }
    }
}

}  // namespace physics
