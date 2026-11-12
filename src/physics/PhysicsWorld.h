#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "physics/PhysicsTypes.h"

// A deterministic rigid body simulation of spheres and boxes.
//
// The world advances in fixed steps. Each step it applies gravity, finds the pairs of bodies that
// may touch (a hash grid of static bodies and one of the moving ones), builds contacts for them, and
// solves the contacts with sequential impulses: every contact limits how fast two bodies approach
// along its normal and how fast they slide across it, and the solver visits them repeatedly until
// the impulses settle. Impulses of the last step start the next one, which is what lets stacks of
// boxes stand still. Groups of bodies that hardly move fall asleep and cost nothing until something
// moving touches them, and then the whole group wakes together.
//
// Bodies that are about to meet get a speculative contact: the solver lets them approach only as fast as
// closes the gap in one step, so fast bodies cannot pass through thin ones. Overlaps are removed by moving
// the bodies apart, which gives them no speed, so bodies that start inside each other do not fly apart.
//
// The same bodies and the same calls always give the same result: pairs are processed in order of
// their body indices and nothing depends on the order of a hash table.

namespace physics {

struct PhysicsSettings {
    glm::vec3 gravity{0.0f, -9.81f, 0.0f};
    float fixedStep = 1.0f / 120.0f;  // seconds simulated per step
    int maxStepsPerUpdate = 8;        // a long frame is cut short rather than snowballing
    int solverIterations = 12;
    int positionIterations = 4;       // passes of the overlap removal
    float penetrationSlop = 0.005f;   // overlap that is left alone, so resting contacts do not jitter
    float correctionRate = 0.2f;      // the share of an overlap removed per pass
    float maxCorrectionPerStep = 0.05f;  // metres a body is moved to undo an overlap in one step
    float touchTolerance = 0.002f;    // bodies this close count as touching, and bounce if they hit fast
    float restitutionThreshold = 1.0f;   // slower impacts do not bounce
    float angularDamping = 0.1f;      // share of the spin lost per second
    float sleepLinearSpeed = 0.05f;   // metres per second under which a body counts as still
    float sleepAngularSpeed = 0.1f;   // radians per second
    float sleepSeconds = 0.5f;        // how long a whole group must be still before it sleeps
    float maxLookAhead = 2.0f;        // metres: the longest distance ahead a fast body looks for contacts
    float staticCellSize = 8.0f;      // edge of the cells of the grid of static bodies
    float dynamicCellSize = 4.0f;     // and of the grid of moving bodies
};

class PhysicsWorld {
public:
    explicit PhysicsWorld(const PhysicsSettings& worldSettings = {}) : settings(worldSettings) {}

    // Adds a body and returns its index, which stays valid for the life of the world. Sizes that are
    // not positive and numbers that are not finite are replaced by small safe values.
    uint32_t AddBody(const BodyDescription& description);

    const Body& GetBody(uint32_t index) const { return bodies[index]; }
    size_t BodyCount() const { return bodies.size(); }
    const PhysicsSettings& Settings() const { return settings; }

    // Changes the speed of a body by an impulse applied at a world point, and wakes it.
    void ApplyImpulse(uint32_t index, glm::vec3 impulse, glm::vec3 worldPoint);
    void SetVelocity(uint32_t index, glm::vec3 linear, glm::vec3 angular);
    // Wakes a body and the whole group that sleeps with it.
    void Wake(uint32_t index);

    // Simulates `seconds`, in as many fixed steps as fit, carrying the remainder to the next call.
    void Update(float seconds);
    // Simulates exactly one fixed step.
    void Step();

    // Statistics of the last step.
    size_t ContactPairCount() const { return manifolds.size(); }
    size_t AwakeBodyCount() const;
    // The total kinetic energy of the moving bodies in joules, for tests.
    float KineticEnergy() const;

private:
    struct Constraint;  // one contact point as the solver sees it

    void RefreshBodies();
    void BuildMovingGrid();
    void CollectMovingCandidates(glm::vec3 low, glm::vec3 high, std::vector<uint32_t>& candidates) const;
    void WakeSleepersTouchedByMovingBodies();
    void FindPairs(std::vector<std::pair<uint32_t, uint32_t>>& pairs);
    void BuildManifolds(const std::vector<std::pair<uint32_t, uint32_t>>& pairs);
    void Solve();
    void RemoveOverlaps();
    void Integrate();
    void UpdateSleep();
    void RebuildStaticGrid();

    PhysicsSettings settings;
    std::vector<Body> bodies;
    float leftoverSeconds = 0.0f;

    // The grid of static bodies, filled before the first step. Bodies that cover too many cells for the
    // grid are tested against every moving body instead.
    std::unordered_map<uint64_t, std::vector<uint32_t>> staticGrid;
    std::vector<uint32_t> hugeStatics;
    bool staticGridIsStale = true;

    // The grid of moving bodies, awake or asleep, rebuilt every step; and the moving bodies too big for it.
    std::unordered_map<uint64_t, std::vector<uint32_t>> movingGrid;
    std::vector<uint32_t> hugeMoving;

    // Bodies that sleep together, by the group they were put to sleep as.
    std::unordered_map<uint32_t, std::vector<uint32_t>> sleepingGroups;
    uint32_t nextSleepGroup = 0;

    // The contacts found in the last step, in order of body pair, and the same by pair for the next step.
    std::vector<Manifold> manifolds;
    std::unordered_map<uint64_t, Manifold> previousManifolds;
};

}  // namespace physics
