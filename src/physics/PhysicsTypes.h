#pragma once

#include <cstdint>
#include <limits>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Plain data of the rigid body physics. Units are metres, kilograms and seconds, and +Y is up.
// A body's orientation turns local axes into world axes, and the angular velocity is in world axes.

namespace physics {

constexpr uint32_t kInvalidBody = std::numeric_limits<uint32_t>::max();

enum class ShapeType : uint8_t {
    Sphere,
    Box,
};

// A convex shape centred on the body's position, in the body's own axes.
struct Shape {
    ShapeType type = ShapeType::Box;
    float radius = 0.5f;                       // spheres
    glm::vec3 halfExtents{0.5f, 0.5f, 0.5f};   // boxes: half the size along each axis

    static Shape Sphere(float sphereRadius) { return {ShapeType::Sphere, sphereRadius, glm::vec3(sphereRadius)}; }
    static Shape Box(glm::vec3 half) { return {ShapeType::Box, glm::length(half), half}; }
};

// How a surface behaves when it touches another. Two bodies combine their values: friction and rolling
// resistance as the geometric mean, restitution as the larger of the two.
struct Material {
    float friction = 0.6f;
    float restitution = 0.1f;  // 0 = no bounce, 1 = perfectly elastic
    // The turning moment, per newton pressing the surfaces together, that a rolling or spinning body
    // has to overcome: a length in metres. Without it a ball rolls for ever on a flat floor.
    float rollingResistance = 0.05f;
};

struct BodyDescription {
    Shape shape;
    Material material;
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 linearVelocity{0.0f};
    glm::vec3 angularVelocity{0.0f};
    float density = 1000.0f;  // kg per cubic metre, so a one metre cube weighs a tonne
    bool isStatic = false;    // static bodies never move and have infinite mass
};

// Everything the simulation keeps about one body.
struct Body {
    Shape shape;
    Material material;
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 linearVelocity{0.0f};
    glm::vec3 angularVelocity{0.0f};
    float inverseMass = 0.0f;                  // zero for static bodies
    glm::vec3 inverseInertiaLocal{0.0f};       // the diagonal of the inverse inertia tensor in body axes
    glm::mat3 inverseInertiaWorld{0.0f};       // refreshed every step
    glm::vec3 boundsMin{0.0f};                 // world axis aligned box around the shape
    glm::vec3 boundsMax{0.0f};
    float quietSeconds = 0.0f;                 // how long the body has hardly moved
    uint32_t sleepGroup = kInvalidBody;        // while asleep, the group of bodies that sleeps and wakes together
    bool isStatic = false;
    bool isAsleep = false;
};

// One point where two bodies touch. `normalImpulse` and `tangentImpulse` are what the solver applied
// last step, kept to start the next step from.
struct ContactPoint {
    glm::vec3 position{0.0f};  // in the world, between the two surfaces
    float depth = 0.0f;        // how far the shapes overlap; negative while they are a little apart
    float normalImpulse = 0.0f;
    float tangentImpulse[2] = {0.0f, 0.0f};
    glm::vec3 rollingImpulse{0.0f};  // angular impulse that resists rolling
};

constexpr int kMaxContactsPerPair = 4;

// The contacts between two bodies. `normal` is a unit vector from body `a` towards body `b`.
struct Manifold {
    uint32_t a = kInvalidBody;
    uint32_t b = kInvalidBody;
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    int count = 0;
    ContactPoint points[kMaxContactsPerPair];
    // How fast the bodies were closing in, along the normal, while they were still apart (zero if they were
    // not, or closing slowly): when the solver stops them at the surface, they bounce from this speed.
    float impactSpeed = 0.0f;
    float carriedImpactSpeed = 0.0f;  // the same, from the last step, when the contact is first touching
};

}  // namespace physics
