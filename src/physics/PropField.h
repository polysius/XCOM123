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

enum class PropKind { Crate, Barrel, Ball };

class PropField {
public:
    // Drops the props above the largest square of `city`, or above its centre when it has none.
    void Spawn(const CityLayout& city, const PropSettings& settings);

    // Props one at a time, for what happens while the program runs. Every prop has an id, from 1, that stays its own for the life of
    // the field: Spawn gives the props it drops ids too, and goes on counting where it was, so that an id that a graph kept from
    // before a new Spawn names no prop and never a different one.
    //
    // The ground of the props is the city near the place Spawn was given, which is where they can fall and rest: HasGroundAt says
    // whether a point is over it. A field that was never spawned has none, and Spawn with a count of 0 makes the ground alone.
    bool HasGroundAt(glm::vec2 point) const;
    // Adds a prop that falls from `position`. `seed` decides its size and how it starts to turn. Returns its id, or 0 when there is
    // no ground under the point.
    uint32_t AddProp(PropKind kind, glm::vec3 position, uint32_t seed);
    // Takes a prop out of the world. Returns false when there is no prop with this id.
    bool RemoveProp(uint32_t id);
    // Changes the speed of a prop by an impulse (newton seconds) at its middle. Returns false when there is no such prop.
    bool ApplyImpulse(uint32_t id, glm::vec3 impulse);

    // The river the props share the world with, or null for none. The river must outlive the field.
    void SetWater(fluid::RiverWater* river) { water = river; }

    // Advances the simulation and refreshes the instances.
    void Update(float seconds);

    size_t Count() const { return boxBodies.size() + sphereBodies.size(); }
    size_t AwakeCount() const { return world.AwakeBodyCount(); }
    std::span<const BodyInstance> Boxes() const { return boxInstances; }
    std::span<const BodyInstance> Spheres() const { return sphereInstances; }
    // The people who may push props this frame, sorted by their index in the crowd. Each one near enough to a prop takes
    // one of a limited number of kinematic bodies and keeps it while they stay in the list; people over the limit have
    // none and walk through props, and PeopleOverflow counts them. Call it before Update, with the people the crowd
    // found near PushablePositions.
    // `seconds` is the time until the next call: the bodies walk to where their people are in that time, and stand
    // still when it is over, however many Updates it takes.
    void SetPeople(std::span<const PersonBody> people, float seconds);
    // Where the props are that a person can push, on the ground. Heavier ones are obstacles, not pushed.
    std::span<const glm::vec2> PushablePositions() const { return pushablePositions; }
    size_t PeopleBodyCount() const;
    // The body in the physics world (see World) that stands for this person, if they have one.
    std::optional<uint32_t> BodyOfPerson(uint32_t agent) const;
    uint32_t PeopleOverflow() const { return peopleOverflow; }
    // The props as obstacles for the crowd, without the ones that people push: the box round each one seen from above, with its lowest and highest height.
    std::span<const MovableObstacle> Obstacles() const { return obstacles; }
    const PhysicsWorld& World() const { return world; }
    // The middle of the heap the props were dropped in, on the ground. The heap stands a little off the
    // middle of the square.
    glm::vec2 DropCentre() const { return dropCentre; }

private:
    void RefreshInstances();
    void StopPeople();
    // Applies the forces of the water to every prop for one step of `seconds`, and the reaction of the props on the water.
    void ApplyWater(float seconds);

    // Where a prop is, by id: in the boxes or in the spheres, and which of them.
    struct PropSlot {
        bool isSphere = false;
        size_t index = 0;
    };
    std::optional<PropSlot> FindProp(uint32_t id) const;

    PhysicsWorld world;
    glm::vec2 groundCentre{0.0f};  // the middle of the city boxes that became solid, and how far they reach; 0 for none
    float groundRadius = 0.0f;
    uint32_t nextPropId = 1;
    std::vector<uint32_t> boxIds;     // per box prop, in step with boxBodies
    std::vector<uint32_t> sphereIds;
    glm::vec2 dropCentre{0.0f};
    float fallLimit = -100.0f;  // props that fall below this height have left the world and are taken out
    fluid::RiverWater* water = nullptr;
    float pendingSeconds = 0.0f;           // time asked for that the stepping with water has not run yet
    std::vector<float> submergedVolume;    // per prop, how much of it was under water at the last step, in cubic metres
    std::vector<uint8_t> restarted;        // per prop: it slept, so its submerged volume is out of date
    std::vector<uint32_t> boxBodies;     // the world's bodies that are drawn as boxes, with their instances
    std::vector<uint32_t> sphereBodies;
    std::vector<BodyInstance> boxInstances;
    std::vector<BodyInstance> sphereInstances;
    std::vector<MovableObstacle> obstacles;  // one for every prop that is too heavy to push, boxes first
    std::vector<glm::vec2> pushablePositions;
    std::vector<uint32_t> personBodies;     // the pool of kinematic bodies, parked when not in use
    std::vector<uint32_t> personAgents;     // per body, the person who has it, or kNoPerson
    float personDriveSecondsLeft = 0.0f;    // how long the bodies of people keep the speed SetPeople gave them
    uint32_t peopleOverflow = 0;
};

}  // namespace physics
