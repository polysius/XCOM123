#include "world/crowd/StreetRouting.h"

#include <algorithm>
#include <cmath>

#include "world/crowd/CrowdMath.h"
#include "world/crowd/Walkways.h"

namespace crowd {

namespace {

// A street leg ends this close to its end corner.
constexpr float kStreetArriveDistance = 0.6f;
// A crossing ends this close to the corner being walked to.
constexpr float kCornerArriveDistance = 0.5f;
// Street walkers steer towards a point this far ahead of them on their lane.
constexpr float kLookahead = 2.0f;
// Chance that a person deliberately takes the other pavement of the next street, when
// reaching it needs at most kMaxDetourCrossings crossings.
constexpr float kOtherSideChance = 0.2f;
constexpr int kMaxDetourCrossings = 2;

uint32_t Wrap(int corner, uint32_t count) {
    const int wrapped = corner % static_cast<int>(count);
    return static_cast<uint32_t>(wrapped < 0 ? wrapped + static_cast<int>(count) : wrapped);
}

float Side(float lane) {
    return lane >= 0.0f ? 1.0f : -1.0f;
}

// The corner a walker on `edge` stands at when it leaves `node`, on the pavement given by
// the sign of its lane. Walking away along arm j, the left pavement leads to corner j and
// the right one to corner j - 1.
uint32_t DepartureCorner(const Walkways& walkways, uint32_t edge, uint32_t node, float lane) {
    const uint32_t arm = walkways.ArmOfEdge(edge, node);
    return lane >= 0.0f ? arm : Wrap(static_cast<int>(arm) - 1, walkways.ArmCount(node));
}

// The corner a walker on `edge` reaches when it arrives at `node`. Walking in along arm a,
// the walker's left is the clockwise side of the arm, so the left pavement ends at corner
// a - 1 and the right one at corner a.
uint32_t ArrivalCorner(const Walkways& walkways, uint32_t edge, uint32_t node, float lane) {
    const uint32_t arm = walkways.ArmOfEdge(edge, node);
    return lane >= 0.0f ? Wrap(static_cast<int>(arm) - 1, walkways.ArmCount(node)) : arm;
}

// The crosswalk a crossing leg is on, as a line from the corner it started at to the corner
// it is walking to, both at the middle of the pavements.
struct Crosswalk {
    glm::vec2 from;
    glm::vec2 to;
};

Crosswalk CurrentCrosswalk(const Walkways& walkways, const StreetRoute& route) {
    const uint32_t armCount = walkways.ArmCount(route.node);
    const uint32_t previousCorner = Wrap(static_cast<int>(route.corner) - route.cornerStep, armCount);
    return {walkways.CornerPoint(route.node, previousCorner, walkways.LaneCenter()),
            walkways.CornerPoint(route.node, route.corner, walkways.LaneCenter())};
}

glm::vec2 ConstrainToStreet(const Walkways& walkways, const StreetRoute& route, glm::vec2 previous, glm::vec2 next) {
    const StreetFrame frame = FrameOf(walkways, route.edge, route.forward);
    const float side = Side(route.lane);
    // Distances from the centre line, counted positive towards this walker's pavement.
    const float previousOffset = side * glm::dot(previous - frame.start, frame.left);
    const float nextOffset = side * glm::dot(next - frame.start, frame.left);
    const float lowest = std::min(walkways.RoadHalfWidth() + kBodyRadius, previousOffset);
    const float highest = std::max(walkways.OuterEdge() - kBodyRadius, previousOffset);
    const float allowed = std::clamp(nextOffset, lowest, highest);
    return next + frame.left * (side * (allowed - nextOffset));
}

glm::vec2 ConstrainToCrosswalk(const Walkways& walkways, const StreetRoute& route, glm::vec2 previous, glm::vec2 next) {
    const Crosswalk crosswalk = CurrentCrosswalk(walkways, route);
    // The crosswalk reaches half a pavement width past both corner points, so it overlaps
    // the pavements it connects.
    const float halfWidth = walkways.SidewalkWidth() * 0.5f;
    const glm::vec2 along = SafeNormalize(crosswalk.to - crosswalk.from);
    const glm::vec2 start = crosswalk.from - along * halfWidth;
    const glm::vec2 end = crosswalk.to + along * halfWidth;

    const auto closestPoint = [&](glm::vec2 point) {
        const glm::vec2 span = end - start;
        const float lengthSquared = std::max(glm::dot(span, span), 1e-12f);
        return start + span * std::clamp(glm::dot(point - start, span) / lengthSquared, 0.0f, 1.0f);
    };
    const float limit = std::max(halfWidth - kBodyRadius, glm::distance(previous, closestPoint(previous)));
    const glm::vec2 closest = closestPoint(next);
    const glm::vec2 offset = next - closest;
    const float distance = glm::length(offset);
    if (distance <= limit) {
        return next;
    }
    return closest + offset * (limit / distance);
}

}  // namespace

StreetFrame FrameOf(const Walkways& walkways, uint32_t edge, bool forward) {
    const StreetGraph& streets = walkways.Streets();
    const StreetGraph::Edge& street = streets.edges[edge];
    StreetFrame frame;
    frame.startNode = forward ? street.from : street.to;
    frame.endNode = forward ? street.to : street.from;
    frame.start = streets.nodes[frame.startNode];
    const glm::vec2 span = streets.nodes[frame.endNode] - frame.start;
    frame.length = glm::length(span);
    frame.direction = SafeNormalize(span);
    frame.left = LeftOf(frame.direction);
    return frame;
}

float LaneOffset(const Walkways& walkways, float side, float laneBias) {
    const float lowest = walkways.RoadHalfWidth() + kBodyRadius;
    const float highest = walkways.OuterEdge() - kBodyRadius;
    // The walker's right is the negative lane direction whichever pavement it is on, so the
    // bias moves the lane by -laneBias, which is -side * laneBias away from the centre line.
    const float distanceFromCentre = walkways.LaneCenter() - Side(side) * laneBias;
    return Side(side) * std::clamp(distanceFromCentre, lowest, highest);
}

glm::vec2 StreetPoint(const Walkways& walkways, const StreetRoute& route, float distance) {
    const StreetFrame frame = FrameOf(walkways, route.edge, route.forward);
    return frame.start + frame.direction * distance + frame.left * route.lane;
}

float StreetLegStart(const Walkways& walkways, const StreetRoute& route) {
    const StreetFrame frame = FrameOf(walkways, route.edge, route.forward);
    const uint32_t corner = DepartureCorner(walkways, route.edge, frame.startNode, route.lane);
    return walkways.CornerSetback(frame.startNode, corner, std::abs(route.lane));
}

float StreetLegEnd(const Walkways& walkways, const StreetRoute& route) {
    const StreetFrame frame = FrameOf(walkways, route.edge, route.forward);
    const uint32_t corner = ArrivalCorner(walkways, route.edge, frame.endNode, route.lane);
    return frame.length - walkways.CornerSetback(frame.endNode, corner, std::abs(route.lane));
}

bool HasFinishedLeg(const Walkways& walkways, const StreetRoute& route, glm::vec2 position) {
    if (route.leg == RouteLeg::Street) {
        const StreetFrame frame = FrameOf(walkways, route.edge, route.forward);
        return glm::dot(position - frame.start, frame.direction) >= StreetLegEnd(walkways, route) - kStreetArriveDistance;
    }

    const glm::vec2 target = walkways.CornerPoint(route.node, route.corner, std::abs(route.lane));
    if (glm::distance(position, target) <= kCornerArriveDistance) {
        return true;
    }
    // Someone pushed aside by the crowd may never get that close, so having walked the
    // whole crosswalk also counts.
    const Crosswalk crosswalk = CurrentCrosswalk(walkways, route);
    const glm::vec2 span = crosswalk.to - crosswalk.from;
    const float length = glm::length(span);
    return length > 1e-3f && glm::dot(position - crosswalk.from, span) / length >= length - kCornerArriveDistance;
}

RouteDecision ChooseRoute(const Walkways& walkways, const StreetRoute& route, uint32_t& random) {
    const StreetGraph& streets = walkways.Streets();
    const StreetFrame frame = FrameOf(walkways, route.edge, route.forward);
    const uint32_t node = frame.endNode;
    const uint32_t armCount = walkways.ArmCount(node);

    RouteDecision decision;
    decision.node = node;
    if (armCount <= 1) {
        // A dead end: turn around and walk back on the same pavement. Its side relative to
        // the new walking direction flips.
        decision.nextEdge = route.edge;
        decision.forward = !route.forward;
        decision.laneSign = -Side(route.lane);
        return decision;
    }

    const uint32_t arrivalArm = walkways.ArmOfEdge(route.edge, node);
    const uint32_t startCorner = ArrivalCorner(walkways, route.edge, node, route.lane);
    // Any street but the one just walked.
    const uint32_t nextArm = Wrap(static_cast<int>(arrivalArm + 1 + RandomIndex(random, armCount - 1)), armCount);

    // The next street can be joined from either of its two corners: corner nextArm leads to
    // its left pavement, corner nextArm - 1 to its right one. For each, count the crosswalks
    // on the shorter way around the intersection.
    struct Option {
        uint32_t corner;
        float laneSign;
        int crossings;
        int8_t step;
    };
    const auto makeOption = [&](uint32_t corner, float laneSign) {
        const int counterClockwise = static_cast<int>(Wrap(static_cast<int>(corner) - static_cast<int>(startCorner), armCount));
        const int clockwise = static_cast<int>(Wrap(static_cast<int>(startCorner) - static_cast<int>(corner), armCount));
        int8_t step = counterClockwise < clockwise ? int8_t{1} : int8_t{-1};
        if (counterClockwise == clockwise) {
            step = RandomUnit(random) < 0.5f ? int8_t{1} : int8_t{-1};
        }
        return Option{corner, laneSign, std::min(counterClockwise, clockwise), step};
    };
    const Option left = makeOption(nextArm, 1.0f);
    const Option right = makeOption(Wrap(static_cast<int>(nextArm) - 1, armCount), -1.0f);

    Option best = left.crossings < right.crossings ? left : right;
    Option other = left.crossings < right.crossings ? right : left;
    if (left.crossings == right.crossings && RandomUnit(random) < 0.5f) {
        std::swap(best, other);
    }
    if (other.crossings <= kMaxDetourCrossings && RandomUnit(random) < kOtherSideChance) {
        best = other;
    }

    decision.nextEdge = walkways.ArmEdge(node, nextArm);
    decision.forward = streets.edges[decision.nextEdge].from == node;
    decision.laneSign = best.laneSign;
    decision.crossings = static_cast<uint8_t>(best.crossings);
    decision.cornerStep = best.step;
    decision.firstCorner = static_cast<uint16_t>(best.crossings > 0 ? Wrap(static_cast<int>(startCorner) + best.step, armCount)
                                                                    : startCorner);
    return decision;
}

void ApplyDecision(const Walkways& walkways, const RouteDecision& decision, float laneBias, StreetRoute& route) {
    route.edge = decision.nextEdge;
    route.forward = decision.forward;
    route.node = decision.node;
    route.lane = LaneOffset(walkways, decision.laneSign, laneBias);
    if (decision.crossings == 0) {
        route.leg = RouteLeg::Street;
        return;
    }
    route.leg = RouteLeg::Crossing;
    route.corner = decision.firstCorner;
    route.cornerStep = decision.cornerStep;
    route.crossingsLeft = static_cast<uint8_t>(decision.crossings - 1);
}

void AdvanceCrossing(const Walkways& walkways, StreetRoute& route) {
    if (route.crossingsLeft == 0) {
        route.leg = RouteLeg::Street;
        return;
    }
    route.corner = static_cast<uint16_t>(Wrap(static_cast<int>(route.corner) + route.cornerStep, walkways.ArmCount(route.node)));
    --route.crossingsLeft;
}

glm::vec2 SteeringTarget(const Walkways& walkways, const StreetRoute& route, glm::vec2 position) {
    if (route.leg == RouteLeg::Crossing) {
        return walkways.CornerPoint(route.node, route.corner, std::abs(route.lane));
    }
    const StreetFrame frame = FrameOf(walkways, route.edge, route.forward);
    const float legStart = StreetLegStart(walkways, route);
    const float legEnd = StreetLegEnd(walkways, route);
    const float along = std::max(glm::dot(position - frame.start, frame.direction), legStart);
    const float targetDistance = std::max(std::min(along + kLookahead, legEnd), legStart);
    return frame.start + frame.direction * targetDistance + frame.left * route.lane;
}

glm::vec2 ConstrainToRoute(const Walkways& walkways, const StreetRoute& route, glm::vec2 previous, glm::vec2 next) {
    if (route.leg == RouteLeg::Street) {
        return ConstrainToStreet(walkways, route, previous, next);
    }
    return ConstrainToCrosswalk(walkways, route, previous, next);
}

}  // namespace crowd
