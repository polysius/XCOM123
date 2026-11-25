#pragma once

#include <glm/glm.hpp>

#include "animation/BodyLayer.h"
#include "animation/BodyModel.h"

// Secondary motion: the parts of a body that follow the main motion a moment late, as the head and chest do, and the
// reaction of the body to a push. Both are made of critically damped springs, which return to rest as fast as they can
// without swinging past it, and which are solved exactly, so the result does not depend on the size of the time step.

namespace animation {

// A spring on a vector (for example three small angles) that pulls it to zero, critically damped: the vector never overshoots
// zero when it is released, and it settles in a time of about 5 / omega.
struct SpringState {
    glm::vec3 value{0.0f};
    glm::vec3 velocity{0.0f};
};

// Advances a spring by `dt` seconds under a constant push `force` (an acceleration), with natural frequency `omega` (1 over
// seconds). The spring is pushed to rest at force / omega^2. The solution is exact for the whole step.
void StepCriticallyDamped(SpringState& spring, const glm::vec3& force, float omega, float dt);

// How the head and the chest lag behind the body. The chest tips back as the body speeds up and forwards as it slows down,
// leans out as it turns, and the head follows the chest a little later still.
struct SecondaryMotionSettings {
    float chestFrequency = 9.0f;  // of the spring, 1 over seconds
    float headFrequency = 12.0f;
    float chestGain = 0.9f;       // degrees of lean for each metre a second squared of acceleration
    float headGain = 0.7f;
    float headFollowsChest = 0.5f;  // how much of the lean of the chest the head takes up
    float maxLeanDegrees = 8.0f;
};

class SecondaryMotion {
public:
    explicit SecondaryMotion(const SecondaryMotionSettings& settings = {});

    // Advances by `dt`. `acceleration` is the acceleration of the body in the world, in metres a second squared; `heading` is
    // the direction the body faces (0 is +Z, positive towards +X). Returns the layer to add to the pose. It sets the rotation of
    // the chest (spine 2), the neck and the head, and nothing else.
    BodyLayer Update(float dt, const glm::vec3& acceleration, float heading);

    void Reset();

private:
    SecondaryMotionSettings settings;
    SpringState chest;  // angles in radians: X pitch (forwards positive), Y yaw, Z roll
    SpringState head;
};

// The reaction of a body to a push or a hit: the pelvis is shoved, the trunk bends away and comes back, the head whips.
// A push is described by the change of velocity it gives to the body, which is the impulse divided by the mass, so a heavy
// body moves less than a light one for the same blow.
class HitReaction {
public:
    HitReaction();

    // `deltaVelocity` is in the world, metres a second; `heading` is the direction the body faces.
    void Push(const glm::vec3& deltaVelocity, float heading);

    // Advances by `dt` and returns the layer for this moment. It is added on top of a pose before the legs are solved: the
    // pelvis shifts and the trunk follows, while the feet stay where the gait planted them.
    BodyLayer Update(float dt);

    // The largest of the displacements now, as a fraction of what they were at their peak: 0 once the body has settled.
    bool Settled() const;

private:
    SpringState pelvis;   // metres, in the frame of the body: X left, Y up, Z forwards
    SpringState trunk;    // radians: the bend of the spine, X forwards positive, Z roll
    SpringState head;     // radians
};

}  // namespace animation
