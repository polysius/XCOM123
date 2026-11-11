#pragma once

#include <cstdint>

#include <glm/glm.hpp>

#include "world/crowd/CrowdTypes.h"

// How street walkers find their way: along the pavement of one street, then over the
// crosswalks of an intersection to the pavement of the next street, and so on.
//
// A walker never steps onto a road except on a crosswalk. Turning a corner on the same
// pavement needs no crossing; going anywhere else means crossing one or more of the streets
// that meet at the intersection, one crosswalk at a time.

namespace crowd {

class Walkways;

// A street as walked in one direction.
struct StreetFrame {
    glm::vec2 start{0.0f};      // the node the walk starts from
    glm::vec2 direction{0.0f};  // unit vector towards the end node
    glm::vec2 left{0.0f};       // unit vector to the walker's left
    float length = 0.0f;
    uint32_t startNode = 0;
    uint32_t endNode = 0;
};

StreetFrame FrameOf(const Walkways& walkways, uint32_t edge, bool forward);

// The lane of a person on the pavement on `side` (+1 to the left of the walking direction,
// -1 to the right) who keeps `laneBias` metres to the right of the pavement's middle. The
// result always stays on that pavement with room for the person's body.
float LaneOffset(const Walkways& walkways, float side, float laneBias);

// The point of a street leg `distance` metres from its start node, on the route's lane.
glm::vec2 StreetPoint(const Walkways& walkways, const StreetRoute& route, float distance);

// Where along its street a street leg begins and ends: at the pavement corners of its two
// intersections.
float StreetLegStart(const Walkways& walkways, const StreetRoute& route);
float StreetLegEnd(const Walkways& walkways, const StreetRoute& route);

// True when a walker at `position` has finished its current leg: a street leg at its end
// corner, or a crossing leg at the corner it was walking to.
bool HasFinishedLeg(const Walkways& walkways, const StreetRoute& route, glm::vec2 position);

// Picks where to go at the intersection a street leg ends at: a random next street, and
// the shortest way over the crosswalks to one of its pavements. Sometimes the person picks
// the far pavement instead, so people do not all stick to one side.
RouteDecision ChooseRoute(const Walkways& walkways, const StreetRoute& route, uint32_t& random);

// Starts following `decision`, either on the next street or on its first crosswalk.
void ApplyDecision(const Walkways& walkways, const RouteDecision& decision, float laneBias, StreetRoute& route);

// Moves a finished crossing leg on to its next crosswalk, or onto the next street.
void AdvanceCrossing(const Walkways& walkways, StreetRoute& route);

// The point a walker at `position` should head for to follow its route.
glm::vec2 SteeringTarget(const Walkways& walkways, const StreetRoute& route, glm::vec2 position);

// Keeps a step from `previous` to `next` on the route's walkway: the pavement of a street
// leg, or the crosswalk of a crossing leg. A person who is already outside the walkway (for
// example right after the leg changed) is never pushed further out, only kept from leaving
// it further, so the correction can never make anyone jump.
glm::vec2 ConstrainToRoute(const Walkways& walkways, const StreetRoute& route, glm::vec2 previous, glm::vec2 next);

}  // namespace crowd
