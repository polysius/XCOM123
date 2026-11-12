#include "world/crowd/AgentUpdate.h"

#include <algorithm>
#include <cmath>

#include "world/crowd/Avoidance.h"
#include "world/crowd/CrowdMath.h"
#include "world/crowd/CrowdState.h"
#include "world/crowd/SpatialHash.h"
#include "world/crowd/StreetRouting.h"
#include "world/crowd/Walkways.h"

namespace crowd {

namespace {

// Movement --------------------------------------------------------------------------------

// How quickly, per second, people change their velocity towards the one they want. Lower
// values look heavier and smoother; higher ones twitchier.
constexpr float kVelocityResponse = 4.0f;
constexpr float kMaxSpeed = 2.2f;
// People hurry a little on crosswalks.
constexpr float kCrossingHurry = 1.1f;

// Simulation level of detail. Far units are updated once every this many steps, a share of
// them each step so that every step costs the same. One update catches up on all the time
// since the last, up to the given limit.
constexpr uint32_t kFarStepInterval = 4;
constexpr float kMaxCatchUpSeconds = 0.25f;

// Avoidance level of detail. Mid tier people refresh their avoidance once every this many
// steps and reuse it in between; far people let theirs fade out at this rate per second,
// so nobody visibly snaps when they change tier.
constexpr uint32_t kMidAvoidanceInterval = 3;
constexpr float kFarAvoidanceFade = 2.0f;
// People standing still give way less: walkers go around them instead.
constexpr float kStandingAvoidanceScale = 0.35f;

// Behaviour -------------------------------------------------------------------------------

// Street walkers only stop to look around this far from either end of a street.
constexpr float kPauseClearance = 3.0f;
// Square strollers stop when this close to the spot they are walking to.
constexpr float kArriveDistance = 0.4f;
// Followers drop out of a group that got further away than this, and how strongly they
// speed up or slow down to keep their place.
constexpr float kGroupLeash = 12.0f;
constexpr float kGroupCatchUpGain = 0.8f;
// Someone walking slower than this share of their preferred speed is making no progress,
// and gives up on the current way after the given number of seconds.
constexpr float kStuckSpeedShare = 0.2f;
constexpr float kWalkerStuckSeconds = 5.0f;
constexpr float kStrollerStuckSeconds = 4.0f;
constexpr uint32_t kGoalTries = 6;
// The chance that a stroller who has finished a pause goes through a portal instead of strolling on.
// Leaving a room is less likely than entering it, so that rooms keep most of the people who start in them.
constexpr float kEnterRoomChance = 0.30f;
constexpr float kLeaveRoomChance = 0.12f;
constexpr float kPortalArriveDistance = 0.5f;

// Animation -------------------------------------------------------------------------------

constexpr float kTurnRate = 7.0f;        // per second, how quickly the body turns to face its way
constexpr float kStrideRate = 5.0f;      // per second, how quickly the walking swing fades in and out
constexpr float kGestureFadeRate = 2.5f;
constexpr float kGestureCycleSeconds = 6.0f;
constexpr float kFullStrideSpeed = 1.3f;

// What a person wants to do this step, before taking anyone else into account.
struct Intent {
    glm::vec2 velocity{0.0f};
    bool standing = false;
    bool facesPoint = false;  // standing people turn to face facePoint
    glm::vec2 facePoint{0.0f};
    GestureKind gesture = GestureKind::None;
    float gestureWeight = 0.0f;
};

Intent StandingIntent(GestureKind gesture, float weight) {
    Intent intent;
    intent.standing = true;
    intent.gesture = gesture;
    intent.gestureWeight = weight;
    return intent;
}

float Fraction(float value) {
    return value - std::floor(value);
}

// Street walkers --------------------------------------------------------------------------

// True when a street leg walker is far enough from both corners to stop without blocking
// a crossing.
bool IsWellInsideStreet(const Walkways& walkways, const StreetRoute& route, glm::vec2 position) {
    if (route.leg != RouteLeg::Street) {
        return false;
    }
    const StreetFrame frame = FrameOf(walkways, route.edge, route.forward);
    const float along = glm::dot(position - frame.start, frame.direction);
    return along > StreetLegStart(walkways, route) + kPauseClearance &&
           along < StreetLegEnd(walkways, route) - kPauseClearance;
}

// Now and then a walker stops for a few seconds to look around.
void UpdateStreetPause(const StepContext& context, const StreetRoute& route, glm::vec2 position,
                       AgentBehaviour& behaviour) {
    behaviour.activityTimer -= context.deltaSeconds;
    if (behaviour.activityTimer > 0.0f) {
        return;
    }
    if (behaviour.activity == Activity::LookingAround) {
        behaviour.activity = Activity::Walking;
        behaviour.activityTimer = NextLookAroundDelay(behaviour.random);
    } else if (IsWellInsideStreet(context.walkways, route, position)) {
        behaviour.activity = Activity::LookingAround;
        behaviour.activityTimer = RandomRange(behaviour.random, 2.0f, 5.0f);
    } else {
        behaviour.activityTimer = 2.0f;  // try again a little further on
    }
}

glm::vec2 RouteDirection(const Walkways& walkways, const StreetRoute& route, glm::vec2 position) {
    return SafeNormalize(SteeringTarget(walkways, route, position) - position);
}

Intent StreetWalkerIntent(const StepContext& context, CrowdState& state, uint32_t agent) {
    AgentBehaviour& behaviour = state.behaviour[agent];
    const StreetRoute& route = state.routes[agent];
    const glm::vec2 position = state.motion[agent].position;
    UpdateStreetPause(context, route, position, behaviour);
    if (behaviour.activity == Activity::LookingAround) {
        return StandingIntent(GestureKind::LookAround, 1.0f);
    }

    const float hurry = route.leg == RouteLeg::Crossing ? kCrossingHurry : 1.0f;
    Intent intent;
    intent.velocity = RouteDirection(context.walkways, route, position) * (behaviour.preferredSpeed * hurry);
    return intent;
}

// Followers walk their own copy of the group's route and adjust their speed to keep their
// place beside or behind the leader. When the leader stops, they stop and chat.
Intent GroupFollowerIntent(const StepContext& context, CrowdState& state, const CrowdUnit& unit, uint32_t agent) {
    AgentBehaviour& behaviour = state.behaviour[agent];
    const glm::vec2 position = state.motion[agent].position;
    const AgentMotion& leader = state.motion[unit.firstAgent];
    if (glm::distance(leader.position, position) > kGroupLeash) {
        // Lost the group, for example after being held up at a crossing: walk on alone.
        behaviour.role = AgentRole::StreetWalker;
        behaviour.activityTimer = NextLookAroundDelay(behaviour.random);
        return StreetWalkerIntent(context, state, agent);
    }

    const glm::vec2 direction = RouteDirection(context.walkways, state.routes[agent], position);
    const float placeError = glm::dot(leader.position - position, direction) - behaviour.goal.y;
    const float leaderSpeed = glm::length(leader.velocity);
    if (leaderSpeed < 0.1f && std::abs(placeError) < 0.4f) {
        Intent intent = StandingIntent(GestureKind::Talk, 0.6f);
        intent.facesPoint = true;
        intent.facePoint = leader.position;
        return intent;
    }
    const float catchUp = std::clamp(placeError * kGroupCatchUpGain, -0.6f, 0.8f);
    const float speed = std::clamp(leaderSpeed + catchUp, 0.0f, behaviour.preferredSpeed * 1.3f);
    Intent intent;
    intent.velocity = direction * speed;
    return intent;
}

// At the end of a street the walker picks its next street. Group members all take the way
// chosen by whichever of them got there first.
RouteDecision DecideAtIntersection(const StepContext& context, CrowdState& state, CrowdUnit& unit, uint32_t agent) {
    const Walkways& walkways = context.walkways;
    AgentBehaviour& behaviour = state.behaviour[agent];
    StreetRoute& route = state.routes[agent];
    const bool walksInGroup =
        unit.memberCount > 1 && (agent == unit.firstAgent || behaviour.role == AgentRole::GroupFollower);
    if (!walksInGroup) {
        return ChooseRoute(walkways, route, behaviour.random);
    }

    const uint32_t arrivalNode = FrameOf(walkways, route.edge, route.forward).endNode;
    if (route.decisionsTaken < unit.decisionCount && unit.groupDecision.node == arrivalNode) {
        route.decisionsTaken = unit.decisionCount;
        return unit.groupDecision;
    }
    const RouteDecision decision = ChooseRoute(walkways, route, behaviour.random);
    unit.groupDecision = decision;
    ++unit.decisionCount;
    route.decisionsTaken = unit.decisionCount;
    return decision;
}

void AdvanceRoute(const StepContext& context, CrowdState& state, CrowdUnit& unit, uint32_t agent) {
    StreetRoute& route = state.routes[agent];
    if (!HasFinishedLeg(context.walkways, route, state.motion[agent].position)) {
        return;
    }
    if (route.leg == RouteLeg::Crossing) {
        AdvanceCrossing(context.walkways, route);
        return;
    }
    const RouteDecision decision = DecideAtIntersection(context, state, unit, agent);
    ApplyDecision(context.walkways, decision, state.behaviour[agent].laneBias, route);
}

// Square people -----------------------------------------------------------------------------

void StartStroll(const StepContext& context, AgentBehaviour& behaviour, glm::vec2 position) {
    const OrientedRect& plaza = context.walkways.Plazas()[behaviour.plaza];
    // Prefer a spot with a clear straight path, but take any spot rather than none.
    for (uint32_t attempt = 0; attempt < kGoalTries; ++attempt) {
        glm::vec2 spot;
        if (!PickPlazaSpot(context.walkways, plaza, 1, behaviour.random, spot)) {
            continue;
        }
        behaviour.goal = spot;
        if (context.walkways.IsPathFree(position, spot)) {
            break;
        }
    }
    behaviour.activity = Activity::Walking;
    behaviour.stuckTimer = 0.0f;
}

// Sends the stroller towards a room or out of one, if its square has a portal and it feels like it.
bool TryStartThroughPortal(const StepContext& context, AgentBehaviour& behaviour) {
    const Walkways& walkways = context.walkways;
    const std::vector<uint32_t>& portals = walkways.PortalsOfPlaza(behaviour.plaza);
    if (portals.empty()) {
        return false;
    }
    const uint32_t portalIndex = portals[HashInteger(behaviour.random) % portals.size()];
    const Walkways::Portal& portal = walkways.Portals()[portalIndex];
    const bool leaving = behaviour.plaza == portal.insidePlaza;
    if (RandomUnit(behaviour.random) >= (leaving ? kLeaveRoomChance : kEnterRoomChance)) {
        return false;
    }
    behaviour.portal = portalIndex;
    behaviour.portalStage = 0;
    behaviour.goal = leaving ? portal.insideSpot : portal.outsideSpot;  // the end of the passage on this side
    behaviour.activity = Activity::Walking;
    behaviour.stuckTimer = 0.0f;
    return true;
}

// Walks to the near end of a portal, then along its passage to the far end, where the person
// joins the square or the room on the other side.
Intent PortalIntent(const StepContext& context, AgentBehaviour& behaviour, glm::vec2 position) {
    const Walkways::Portal& portal = context.walkways.Portals()[behaviour.portal];
    const bool leaving = behaviour.plaza == portal.insidePlaza;
    const glm::vec2 toGoal = behaviour.goal - position;
    const float distance = glm::length(toGoal);
    if (distance < kPortalArriveDistance) {
        if (behaviour.portalStage == 0) {
            behaviour.portalStage = 1;
            behaviour.goal = leaving ? portal.outsideSpot : portal.insideSpot;
        } else if (behaviour.portalStage == 2) {
            // Back where they started: they stroll on in the same square.
            behaviour.portal = kNoIndex;
            behaviour.activity = Activity::Resting;
            behaviour.activityTimer = RandomRange(behaviour.random, 0.5f, 2.0f);
            return StandingIntent(GestureKind::None, 0.0f);
        } else {
            behaviour.plaza = leaving ? portal.outsidePlaza : portal.insidePlaza;
            behaviour.portal = kNoIndex;
            behaviour.activity = Activity::Resting;
            behaviour.activityTimer = RandomRange(behaviour.random, 1.0f, 4.0f);
            return StandingIntent(GestureKind::None, 0.0f);
        }
    }
    Intent intent;
    intent.velocity = SafeNormalize(behaviour.goal - position) * behaviour.preferredSpeed;
    return intent;
}

// Sends someone in a passage back the way they came, or on again if they were already turned back.
void TurnRoundInPassage(const StepContext& context, AgentBehaviour& behaviour) {
    const Walkways::Portal& portal = context.walkways.Portals()[behaviour.portal];
    const bool leaving = behaviour.plaza == portal.insidePlaza;
    const bool headingForOtherSide = behaviour.portalStage == 1;
    // Stage 1 walks to the far end (the other square) and stage 2 to the near end (their own).
    const glm::vec2 farSpot = leaving ? portal.outsideSpot : portal.insideSpot;
    const glm::vec2 nearSpot = leaving ? portal.insideSpot : portal.outsideSpot;
    behaviour.portalStage = headingForOtherSide ? 2 : 1;
    behaviour.goal = headingForOtherSide ? nearSpot : farSpot;
    behaviour.stuckTimer = 0.0f;
}

Intent PlazaWandererIntent(const StepContext& context, CrowdState& state, uint32_t agent) {
    AgentBehaviour& behaviour = state.behaviour[agent];
    const glm::vec2 position = state.motion[agent].position;

    if (behaviour.portal != kNoIndex) {
        return PortalIntent(context, behaviour, position);
    }

    if (behaviour.activity != Activity::Walking) {
        behaviour.activityTimer -= context.deltaSeconds;
        if (behaviour.activityTimer > 0.0f) {
            const bool looking = behaviour.activity == Activity::LookingAround;
            return StandingIntent(looking ? GestureKind::LookAround : GestureKind::None, looking ? 1.0f : 0.0f);
        }
        if (TryStartThroughPortal(context, behaviour)) {
            return PortalIntent(context, behaviour, position);
        }
        StartStroll(context, behaviour, position);
    }

    const glm::vec2 toGoal = behaviour.goal - position;
    const float distance = glm::length(toGoal);
    if (distance < kArriveDistance) {
        const bool looks = RandomUnit(behaviour.random) < 0.5f;
        behaviour.activity = looks ? Activity::LookingAround : Activity::Resting;
        behaviour.activityTimer = looks ? RandomRange(behaviour.random, 2.0f, 5.0f) : RandomRange(behaviour.random, 4.0f, 14.0f);
        return StandingIntent(GestureKind::None, 0.0f);
    }
    // Slow down over the last metre instead of stopping dead.
    const float arrivalEase = std::clamp(distance, 0.3f, 1.0f);
    Intent intent;
    intent.velocity = toGoal / distance * (behaviour.preferredSpeed * arrivalEase);
    return intent;
}

// Conversation circles take turns: one person talks with big gestures while the others
// listen, nodding a little or glancing away now and then.
Intent ChatterIntent(const StepContext& context, const CrowdState& state, const CrowdUnit& unit, uint32_t agent) {
    const AgentBehaviour& behaviour = state.behaviour[agent];
    const glm::vec2 toSpot = behaviour.goal - state.motion[agent].position;
    const float distance = glm::length(toSpot);

    const float turnSeconds = 3.0f + 2.5f * Fraction(unit.talkClockOffset * 0.618f);
    const uint32_t turn = static_cast<uint32_t>(context.timeSeconds / turnSeconds + unit.talkClockOffset);
    const uint32_t speaker = HashInteger(turn + unit.firstAgent * 7919u) % unit.memberCount;
    const uint32_t member = agent - unit.firstAgent;

    Intent intent;
    if (member == speaker) {
        intent = StandingIntent(GestureKind::Talk, 1.0f);
    } else if (HashInteger(turn * 31u + agent) % 8u == 0) {
        intent = StandingIntent(GestureKind::LookAround, 0.5f);
    } else {
        intent = StandingIntent(GestureKind::Talk, 0.25f);
    }
    intent.facesPoint = true;
    intent.facePoint = unit.circleCenter;
    // Someone nudged out of place by passers-by steps back to their spot.
    if (distance > 0.05f) {
        intent.velocity = ClampLength(toSpot * 1.5f, 0.6f);
    }
    return intent;
}

Intent DecideIntent(const StepContext& context, CrowdState& state, const CrowdUnit& unit, uint32_t agent) {
    switch (state.behaviour[agent].role) {
    case AgentRole::StreetWalker:
        return StreetWalkerIntent(context, state, agent);
    case AgentRole::GroupFollower:
        return GroupFollowerIntent(context, state, unit, agent);
    case AgentRole::PlazaWanderer:
        return PlazaWandererIntent(context, state, agent);
    case AgentRole::Chatter:
        return ChatterIntent(context, state, unit, agent);
    }
    return {};
}

// Avoidance level of detail -------------------------------------------------------------------

glm::vec2 AvoidanceSteering(const StepContext& context, CrowdState& state, const CrowdUnit& unit, uint32_t agent,
                            const Intent& intent, glm::vec2& push) {
    AgentBehaviour& behaviour = state.behaviour[agent];
    AvoidanceQuery query;
    query.self = agent;
    query.firstCompanion = unit.firstAgent;
    query.companionCount = unit.memberCount;
    query.position = state.motion[agent].position;
    query.velocity = state.motion[agent].velocity;
    query.forward = SafeNormalize(intent.velocity);

    switch (state.tiers[agent]) {
    case SimTier::Near: {
        query.resolveOverlap = true;
        const AvoidanceResult result = ComputeAvoidance(context.neighbours, query, context.deltaSeconds);
        behaviour.avoidance = result.steering;
        push = result.push;
        break;
    }
    case SimTier::Mid:
        // Spread the refreshes so that each step does a third of the mid tier.
        if ((agent + context.stepIndex) % kMidAvoidanceInterval == 0) {
            behaviour.avoidance = ComputeAvoidance(context.neighbours, query, context.deltaSeconds).steering;
        }
        break;
    case SimTier::Far:
        behaviour.avoidance *= std::max(0.0f, 1.0f - context.deltaSeconds * kFarAvoidanceFade);
        break;
    }
    return intent.standing ? behaviour.avoidance * kStandingAvoidanceScale : behaviour.avoidance;
}

// Staying on walkable space ---------------------------------------------------------------------

// Keeps square people inside their square. Like the route constraint it never pushes
// anyone who is already outside further out.
glm::vec2 KeepInPlaza(const OrientedRect& plaza, glm::vec2 previous, glm::vec2 next) {
    // Clamp in the square's own frame, where its sides are axis-aligned.
    const glm::vec2 previousInFrame = plaza.ToFrame(previous);
    const glm::vec2 low = glm::min(plaza.area.min + kBodyRadius, previousInFrame);
    const glm::vec2 high = glm::max(plaza.area.max - kBodyRadius, previousInFrame);
    return plaza.FromFrame(glm::clamp(plaza.ToFrame(next), low, high));
}

// The step from `previous` towards `next` that keeps the person on its walkway and clear of
// obstacles. When the full step is blocked it tries moving along x or y alone, so people
// slide along walls instead of sticking to them.
glm::vec2 KeepOnWalkway(const StepContext& context, const CrowdState& state, uint32_t agent, glm::vec2 previous,
                        glm::vec2 next) {
    const Walkways& walkways = context.walkways;
    const AgentBehaviour& behaviour = state.behaviour[agent];
    const bool walksStreets = behaviour.role == AgentRole::StreetWalker || behaviour.role == AgentRole::GroupFollower;
    const bool inPassage = !walksStreets && behaviour.portal != kNoIndex && behaviour.portalStage >= 1;
    // Strollers keep out of conversation circles, unless they somehow ended up in one.
    // Someone in a passage ignores them, so a circle beside it cannot stop them there. Crossing the
    // square to the passage they keep out of circles like any other stroller.
    const bool respectReserved = behaviour.role == AgentRole::PlazaWanderer && !inPassage &&
                                 walkways.CanStandAtUnreserved(previous);

    const glm::vec2 candidates[] = {next, {next.x, previous.y}, {previous.x, next.y}};
    for (glm::vec2 candidate : candidates) {
        if (walksStreets) {
            candidate = ConstrainToRoute(walkways, state.routes[agent], previous, candidate);
        } else if (inPassage) {
            candidate = KeepInPlaza(walkways.Portals()[behaviour.portal].passage, previous, candidate);
        } else {
            candidate = KeepInPlaza(walkways.Plazas()[behaviour.plaza], previous, candidate);
        }
        bool free = respectReserved ? walkways.CanStandAtUnreserved(candidate) : walkways.CanStandAt(candidate);
        // Someone a crate has landed beside or on can take any step that is not into a wall or a building, so that they walk
        // out of the crate's reach and are not held in it for as long as it lies there.
        if (!free && walkways.IsNearMovableObstacle(previous)) {
            free = walkways.CanStandAtIgnoringMovables(candidate);
        }
        if (free) {
            return candidate;
        }
    }
    return previous;
}

void Move(const StepContext& context, CrowdState& state, uint32_t agent, const Intent& intent, glm::vec2 steering,
          glm::vec2 push) {
    AgentMotion& motion = state.motion[agent];
    const float deltaSeconds = context.deltaSeconds;
    const glm::vec2 previous = motion.position;

    const glm::vec2 desired = ClampLength(intent.velocity + steering, kMaxSpeed);
    glm::vec2 velocity = motion.velocity + (desired - motion.velocity) * std::min(1.0f, deltaSeconds * kVelocityResponse);
    const glm::vec2 unconstrained = previous + velocity * deltaSeconds + push;
    glm::vec2 next = KeepOnWalkway(context, state, agent, previous, unconstrained);

    // Whatever part of the velocity a kerb or wall stopped is dropped, so people do not keep
    // pressing against it and shoot off when they get past.
    const glm::vec2 correction = next - unconstrained;
    if (glm::dot(correction, correction) > 1e-10f) {
        const glm::vec2 normal = SafeNormalize(correction);
        velocity -= normal * std::min(0.0f, glm::dot(velocity, normal));
    }

    if (!IsFinite(next) || !IsFinite(velocity)) {
        next = previous;
        velocity = glm::vec2(0.0f);
    }
    motion.position = next;
    motion.velocity = velocity;
}

// Getting unstuck ---------------------------------------------------------------------------------

void UpdateStuckTimer(const StepContext& context, CrowdState& state, uint32_t agent, glm::vec2 previous,
                      const Intent& intent) {
    AgentBehaviour& behaviour = state.behaviour[agent];
    if (intent.standing || behaviour.role == AgentRole::Chatter) {
        behaviour.stuckTimer = 0.0f;
        return;
    }
    const float speed = glm::distance(state.motion[agent].position, previous) / context.deltaSeconds;
    if (speed < behaviour.preferredSpeed * kStuckSpeedShare) {
        behaviour.stuckTimer += context.deltaSeconds;
    } else {
        behaviour.stuckTimer = std::max(0.0f, behaviour.stuckTimer - 2.0f * context.deltaSeconds);
    }

    if (behaviour.role == AgentRole::PlazaWanderer) {
        if (behaviour.stuckTimer > kStrollerStuckSeconds) {
            // Someone stuck on the way to a portal gives up and strolls instead. Someone stuck in the
            // passage, where two people walking head on can jam each other, turns round, and turns
            // round again if that is jammed too, so the passage never keeps anyone for good.
            if (behaviour.portal != kNoIndex && behaviour.portalStage == 0) {
                behaviour.portal = kNoIndex;
            }
            if (behaviour.portal == kNoIndex) {
                StartStroll(context, behaviour, state.motion[agent].position);
            } else {
                TurnRoundInPassage(context, behaviour);
            }
        }
        return;
    }
    if (behaviour.stuckTimer > kWalkerStuckSeconds) {
        // Something blocks this line along the pavement: try another one.
        StreetRoute& route = state.routes[agent];
        behaviour.laneBias = RandomRange(behaviour.random, -0.9f, 1.2f);
        if (route.leg == RouteLeg::Street) {
            route.lane = LaneOffset(context.walkways, route.lane, behaviour.laneBias);
        }
        behaviour.stuckTimer = 0.0f;
    }
}

// Animation -----------------------------------------------------------------------------------

void UpdateGesture(AgentLook& look, const Intent& intent, float deltaSeconds) {
    const float fadeStep = deltaSeconds * kGestureFadeRate;
    if (look.gestureKind == intent.gesture) {
        look.gestureWeight = MoveTowards(look.gestureWeight, intent.gestureWeight, fadeStep);
        return;
    }
    // Let the old gesture fade out completely before the new one starts, so it never jumps.
    look.gestureWeight = MoveTowards(look.gestureWeight, 0.0f, fadeStep);
    if (look.gestureWeight <= 0.02f) {
        look.gestureKind = intent.gesture;
    }
}

void Animate(const StepContext& context, CrowdState& state, uint32_t agent, glm::vec2 previous, const Intent& intent) {
    AgentLook& look = state.looks[agent];
    const AgentMotion& motion = state.motion[agent];
    const float deltaSeconds = context.deltaSeconds;
    const float distance = glm::distance(motion.position, previous);
    const float speed = distance / deltaSeconds;

    float targetHeading = look.heading;
    const glm::vec2 toFacePoint = intent.facePoint - motion.position;
    if (intent.facesPoint && speed < 0.3f && glm::dot(toFacePoint, toFacePoint) > 1e-4f) {
        targetHeading = std::atan2(toFacePoint.x, toFacePoint.y);
    } else if (speed > 0.15f) {
        targetHeading = std::atan2(motion.velocity.x, motion.velocity.y);
    }
    look.heading = WrapAngle(look.heading + AngleDifference(targetHeading, look.heading) * std::min(1.0f, deltaSeconds * kTurnRate));

    look.walkPhase = WrapAngle(look.walkPhase + distance / state.behaviour[agent].gaitLength * kTwoPi);
    look.stride = MoveTowards(look.stride, std::clamp(speed / kFullStrideSpeed, 0.0f, 1.15f), deltaSeconds * kStrideRate);
    // Everyone's idle clock runs at a slightly different rate, so nobody moves in step.
    const float gestureRate = (1.0f + 0.3f * look.variation) / kGestureCycleSeconds;
    look.gesturePhase = Fraction(look.gesturePhase + deltaSeconds * gestureRate);
    UpdateGesture(look, intent, deltaSeconds);
}

void UpdateAgent(const StepContext& context, CrowdState& state, CrowdUnit& unit, uint32_t agent) {
    const Intent intent = DecideIntent(context, state, unit, agent);
    glm::vec2 push(0.0f);
    const glm::vec2 steering = AvoidanceSteering(context, state, unit, agent, intent, push);
    const glm::vec2 previous = state.motion[agent].position;
    Move(context, state, agent, intent, steering, push);

    const AgentRole role = state.behaviour[agent].role;
    if (role == AgentRole::StreetWalker || role == AgentRole::GroupFollower) {
        AdvanceRoute(context, state, unit, agent);
    }
    UpdateStuckTimer(context, state, agent, previous, intent);
    Animate(context, state, agent, previous, intent);
}

// Simulation level of detail per unit ----------------------------------------------------------

bool IsWholeUnitFar(const CrowdState& state, const CrowdUnit& unit) {
    for (uint32_t member = 0; member < unit.memberCount; ++member) {
        if (state.tiers[unit.firstAgent + member] != SimTier::Far) {
            return false;
        }
    }
    return true;
}

void SetUnitLag(CrowdState& state, const CrowdUnit& unit, float lagSeconds) {
    for (uint32_t member = 0; member < unit.memberCount; ++member) {
        state.lagSeconds[unit.firstAgent + member] = lagSeconds;
    }
}

}  // namespace

void StepUnit(const StepContext& context, CrowdState& state, uint32_t unitIndex) {
    CrowdUnit& unit = state.units[unitIndex];
    // Members are always updated together, so they all owe the same time.
    const float owedSeconds = state.lagSeconds[unit.firstAgent] + context.deltaSeconds;
    const bool isDue = (unitIndex + context.stepIndex) % kFarStepInterval == 0;
    if (IsWholeUnitFar(state, unit) && !isDue && owedSeconds < kMaxCatchUpSeconds) {
        SetUnitLag(state, unit, owedSeconds);
        return;
    }

    const StepContext unitContext{context.walkways, context.neighbours, std::min(owedSeconds, kMaxCatchUpSeconds),
                                  context.timeSeconds, context.stepIndex};
    for (uint32_t member = 0; member < unit.memberCount; ++member) {
        UpdateAgent(unitContext, state, unit, unit.firstAgent + member);
    }
    SetUnitLag(state, unit, 0.0f);
}

float NextLookAroundDelay(uint32_t& random) {
    return RandomRange(random, 15.0f, 120.0f);
}

bool PickPlazaSpot(const Walkways& walkways, const OrientedRect& plaza, uint32_t tries, uint32_t& random,
                   glm::vec2& spot) {
    const glm::vec2 low = plaza.area.min + kBodyRadius;
    const glm::vec2 high = plaza.area.max - kBodyRadius;
    if (low.x >= high.x || low.y >= high.y) {
        return false;
    }
    // Squares may touch the pavements but people must never wait on a road.
    const float roadClearance = walkways.RoadHalfWidth() + kBodyRadius + 0.3f;
    for (uint32_t attempt = 0; attempt < tries; ++attempt) {
        // The second coordinate is drawn first, as it always was (see SpawnChatCircle).
        const float frameY = RandomRange(random, low.y, high.y);
        const float frameX = RandomRange(random, low.x, high.x);
        const glm::vec2 candidate = plaza.FromFrame({frameX, frameY});
        if (walkways.CanStandAtUnreserved(candidate) && walkways.DistanceToNearestStreet(candidate) >= roadClearance) {
            spot = candidate;
            return true;
        }
    }
    return false;
}

}  // namespace crowd
