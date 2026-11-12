#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "fluid/RiverWater.h"
#include "physics/PhysicsWorld.h"
#include "scene/SceneTypes.h"
#include "world/CityGenerator.h"

// Crates, barrels and balls dropped into the city, falling and tumbling under the physics engine until
// they rest on the streets, the pavements and each other.
//
// The city is the static world: its boxes near the place the props fall become immovable bodies. The
// props themselves are the moving bodies, and every Update turns them into the instances the renderer
// draws, with their full orientation.
//
// Props can share the world with a simulated river (SetWater). The water then lifts a prop by as much as
// the water it pushes aside weighs, so a crate floats and a stone sinks, it drags a prop along with its
// current, and a prop that sinks or rises pushes the water aside or lets it back, which makes the splashes
// and the waves.

namespace physics {

struct PropSettings {
    uint32_t count = 0;
    uint32_t seed = 1;
    // City boxes closer than this to the middle of the drop area (a distance on the ground) are solid
    // for the props. The ground under a prop must be among them, so the area is wider than the drop.
    float colliderRadius = 160.0f;
    // Where to drop the heap instead of over the largest square, for example over the river.
    std::optional<glm::vec2> dropAt;
    // The height of the bed of the river, if the props are dropped over one: a floor is put there, so that what
    // sinks comes to rest on it, since the water of the city has none to stand on.
    std::optional<float> riverBed;
};

class PropField {
public:
    // Drops the props above the largest square of `city`, or above its centre when it has none.
    void Spawn(const CityLayout& city, const PropSettings& settings);

    // The river the props share the world with, or null for none. The river must outlive the field.
    void SetWater(fluid::RiverWater* river) { water = river; }

    // Advances the simulation and refreshes the instances.
    void Update(float seconds);

    size_t Count() const { return boxBodies.size() + sphereBodies.size(); }
    size_t AwakeCount() const { return world.AwakeBodyCount(); }
    std::span<const BodyInstance> Boxes() const { return boxInstances; }
    std::span<const BodyInstance> Spheres() const { return sphereInstances; }
    const PhysicsWorld& World() const { return world; }
    // The middle of the heap the props were dropped in, on the ground. The heap stands a little off the
    // middle of the square.
    glm::vec2 DropCentre() const { return dropCentre; }

private:
    void RefreshInstances();
    // Applies the forces of the water to every prop for one step of `seconds`, and the push of the props to the water.
    void ApplyWater();

    PhysicsWorld world;
    glm::vec2 dropCentre{0.0f};
    float fallLimit = -100.0f;  // props that fall below this height have left the world and are taken out
    fluid::RiverWater* water = nullptr;
    float pendingSeconds = 0.0f;           // time asked for that the stepping with water has not run yet
    std::vector<float> submergedVolume;    // per prop, how much of it was under water at the last step, in cubic metres
    std::vector<uint32_t> boxBodies;     // the world's bodies that are drawn as boxes, with their instances
    std::vector<uint32_t> sphereBodies;
    std::vector<BodyInstance> boxInstances;
    std::vector<BodyInstance> sphereInstances;
};

}  // namespace physics
