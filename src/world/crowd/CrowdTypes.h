#pragma once

#include <cstdint>
#include <limits>

#include <glm/glm.hpp>

// Plain data that describes the people in the crowd. Every person is an index into a few
// parallel arrays (see AgentArrays), split by how the data is used so that each pass over
// the crowd touches as little memory as possible.
//
// Coordinates are 2D on the ground plane: x is world X and y is world Z, as in CityLayout.

namespace crowd {

constexpr uint32_t kNoIndex = std::numeric_limits<uint32_t>::max();

// Seen from above a person is a disc of this radius. Two people closer than twice this
// are touching.
constexpr float kBodyRadius = 0.25f;

// People are between these times the height of the base person mesh.
constexpr float kMinHeightScale = 0.92f;
constexpr float kMaxHeightScale = 1.08f;

// How much simulation a person gets, chosen each step by distance from the focus point.
// Far people are updated less often but drawn extrapolated along their velocity in between,
// and avoidance fades in and out gradually, so a person changing tier never jumps.
enum class SimTier : uint8_t {
    Near,  // every step, with full local avoidance
    Mid,   // every step, with avoidance refreshed every few steps and reused in between
    Far,   // every few steps, following its route without looking at anyone else
};

// What a person is doing right now.
enum class Activity : uint8_t {
    Walking,        // following a street route, or walking to a spot in a square
    LookingAround,  // stopped for a moment to look at the surroundings
    Resting,        // standing still for a moment in a square
    Chatting,       // standing in a conversation circle
};

// The role decides which update rules a person follows for its whole life.
enum class AgentRole : uint8_t {
    StreetWalker,   // walks the street graph on the pavements, crossing at intersections
    GroupFollower,  // walks beside or behind the group leader, the first member of its unit
    PlazaWanderer,  // strolls between random spots in one square
    Chatter,        // stands in a conversation circle in a square
};

// Gesture kinds understood by shaders/npc.vert. The integer part of NpcInstance::unused0.
enum class GestureKind : uint8_t {
    None = 0,
    LookAround = 1,  // head sweeps from side to side
    Talk = 2,        // hand gestures and nods; low weight reads as listening
};

// Position and velocity, the data other people read during avoidance. Avoidance reads the
// copy the spatial hash takes at the start of each step, never these live values, so people
// can be updated in parallel and in any order.
struct AgentMotion {
    glm::vec2 position{0.0f};
    glm::vec2 velocity{0.0f};
};

// A street walker either walks along one street (edge) on a pavement, or crosses the roads
// around one intersection (node) from corner to corner on the crosswalks.
enum class RouteLeg : uint8_t {
    Street,
    Crossing,
};

// Where a street walker is in the street graph and which way it is going.
//
// Corners and arms of a node are numbered as in Walkways: arm i is the i-th street leaving
// the node counter-clockwise, and corner i lies between arm i and arm i + 1.
struct StreetRoute {
    // Street leg: the street being walked. Crossing leg: the street to take afterwards.
    uint32_t edge = 0;
    // Crossing leg: the intersection whose roads are being crossed.
    uint32_t node = 0;
    // Signed distance from the street's centre line, positive to the left of the walking
    // direction. Its sign says which pavement the person walks on.
    float lane = 0.0f;
    // Crossing leg: the corner being walked to.
    uint16_t corner = 0;
    // Group members: how many of the group's route decisions this member has followed.
    uint16_t decisionsTaken = 0;
    // Crossing leg: roads still to cross after reaching the current target corner.
    uint8_t crossingsLeft = 0;
    // Crossing leg: +1 walks counter-clockwise around the node, -1 clockwise.
    int8_t cornerStep = 1;
    // Walking from edge.from towards edge.to (true) or the other way.
    bool forward = true;
    RouteLeg leg = RouteLeg::Street;
};

// A choice made at an intersection: which street comes next and how to get to it. Walking
// groups record their latest choice so that every member takes the same way.
struct RouteDecision {
    uint32_t node = kNoIndex;
    uint32_t nextEdge = 0;
    bool forward = true;
    float laneSign = 1.0f;
    uint16_t firstCorner = 0;
    int8_t cornerStep = 1;
    uint8_t crossings = 0;
};

// Behaviour state: what the person wants and how it moves.
struct AgentBehaviour {
    uint32_t random = 1;           // per-person random number state
    float preferredSpeed = 1.3f;   // walking speed in m/s when nothing is in the way
    float gaitLength = 1.4f;       // metres walked per full walk cycle (two steps)
    // How far this person keeps to the right of its pavement's centre line, in metres. A
    // group follower's value also carries its place beside the leader.
    float laneBias = 0.0f;
    float activityTimer = 0.0f;    // seconds until the current activity ends
    float stuckTimer = 0.0f;       // seconds spent walking without getting anywhere
    // Square people: the spot walked to or stood at. Group followers: their place next to the
    // leader, x metres to the leader's left and y metres behind.
    glm::vec2 goal{0.0f};
    glm::vec2 avoidance{0.0f};     // steering from the last avoidance pass, reused by Mid
    uint32_t plaza = kNoIndex;     // square people: the square they stay in
    // Strollers on their way into or out of a room: the portal they use (kNoIndex when they are
    // not on their way), and whether they are still walking to its near end (0) or along its
    // passage (1).
    uint32_t portal = kNoIndex;
    uint8_t portalStage = 0;
    AgentRole role = AgentRole::StreetWalker;
    Activity activity = Activity::Walking;
};

// What the renderer needs to draw and animate the person.
struct AgentLook {
    glm::vec3 clothingColor{0.5f};
    float heightScale = 1.0f;
    float heading = 0.0f;        // radians, 0 faces +Z
    float walkPhase = 0.0f;      // radians in [0, 2 pi), drives the leg and arm swing
    float stride = 0.0f;         // 0 = standing, 1 = full walking swing
    float gesturePhase = 0.0f;   // cycles in [0, 1), the clock of idle motion and gestures
    float gestureWeight = 0.0f;  // 0..1, how strongly the current gesture plays
    float variation = 0.0f;      // 0..1, per-person variety in gait and posture
    GestureKind gestureKind = GestureKind::None;
};

// A set of people updated together by one thread: a lone walker, a walking group (leader
// first, followers right after it), or a conversation circle. Grouping them lets members
// read and write shared group state without any locking.
struct CrowdUnit {
    uint32_t firstAgent = 0;
    uint32_t memberCount = 1;
    // Conversation circles: the point the members face.
    glm::vec2 circleCenter{0.0f};
    // Conversation circles: shifts whose turn it is to talk, so circles are not in step.
    float talkClockOffset = 0.0f;
    // Walking groups: the latest choice made at an intersection, by whichever member got
    // there first, and how many choices the group has made so far.
    RouteDecision groupDecision;
    uint16_t decisionCount = 0;
};

}  // namespace crowd
