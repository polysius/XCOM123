#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "physics/PhysicsWorld.h"
#include "scene/SceneTypes.h"
#include "world/CityGenerator.h"

// Crates, barrels and balls dropped into the city, falling and tumbling under the physics engine until
// they rest on the streets, the pavements and each other.
//
// The city is the static world: its boxes near the place the props fall become immovable bodies. The
// props themselves are the moving bodies, and every Update turns them into the instances the renderer
// draws, with their full orientation.

namespace physics {

struct PropSettings {
    uint32_t count = 0;
    uint32_t seed = 1;
    // City boxes closer than this to the middle of the drop area (a distance on the ground) are solid
    // for the props. The ground under a prop must be among them, so the area is wider than the drop.
    float colliderRadius = 160.0f;
};

class PropField {
public:
    // Drops the props above the largest square of `city`, or above its centre when it has none.
    void Spawn(const CityLayout& city, const PropSettings& settings);

    // Advances the simulation and refreshes the instances.
    void Update(float seconds);

    size_t Count() const { return boxBodies.size() + sphereBodies.size(); }
    size_t AwakeCount() const { return world.AwakeBodyCount(); }
    std::span<const BodyInstance> Boxes() const { return boxInstances; }
    std::span<const BodyInstance> Spheres() const { return sphereInstances; }
    const PhysicsWorld& World() const { return world; }
    // The middle of the square the props were dropped over, on the ground.
    glm::vec2 DropCentre() const { return dropCentre; }

private:
    void RefreshInstances();

    PhysicsWorld world;
    glm::vec2 dropCentre{0.0f};
    std::vector<uint32_t> boxBodies;     // the world's bodies that are drawn as boxes, with their instances
    std::vector<uint32_t> sphereBodies;
    std::vector<BodyInstance> boxInstances;
    std::vector<BodyInstance> sphereInstances;
};

}  // namespace physics
