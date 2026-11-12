#pragma once

#include "physics/PhysicsTypes.h"

// Narrow phase: the contacts between two shapes.

namespace physics {

// Shapes that are closer than this count as touching. The solver treats such a contact as a limit
// on how fast the shapes may approach, so a body resting on another keeps its contacts from one
// step to the next instead of losing them each time it settles a hair above the surface.
constexpr float kContactMargin = 0.02f;

// Finds the contacts between bodies `a` and `b`. When the shapes touch or overlap, or are closer
// than `margin`, fills `manifold` (the normal points from `a` to `b`, its contacts carry no
// impulses yet) and returns true. `manifold.a` and `manifold.b` are left for the caller to set.
//
// A margin wider than kContactMargin makes the contact speculative: the shapes are still apart, and the
// solver lets them approach only as fast as closes the gap in one step. The world widens the margin of
// a pair by the distance they close in a step, so that fast bodies cannot pass through each other.
bool Collide(const Body& a, const Body& b, Manifold& manifold, float margin = kContactMargin);

// The axes of a box in the world: the columns of the rotation matrix.
inline glm::mat3 AxesOf(const Body& body) {
    return glm::mat3_cast(body.orientation);
}

}  // namespace physics
