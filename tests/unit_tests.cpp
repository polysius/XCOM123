// Unit tests for the engine's pure logic (no window, no GPU). There is no test framework on
// purpose: each test is a function that calls Check, and main reports every failure it finds.
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/CommandLine.h"
#include "core/FrameStatistics.h"
#include "physics/PropField.h"
#include "world/Crowd.h"
#include "world/TerrainField.h"
#include "world/crowd/Walkways.h"

namespace {
int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

// Parses a command line given as a list of words, without the program name.
Options Parse(std::vector<std::string> words) {
    words.insert(words.begin(), "XCOM123");
    std::vector<char*> argv;
    for (std::string& word : words) {
        argv.push_back(word.data());
    }
    return ParseCommandLine(static_cast<int>(argv.size()), argv.data());
}

bool IsRejected(std::vector<std::string> words) {
    try {
        Parse(std::move(words));
    } catch (const std::invalid_argument&) {
        return true;
    } catch (const std::out_of_range&) {
        return true;
    }
    return false;
}

void CommandLineDefaults() {
    const Options options = Parse({});
    Check(options.npcCount == 10000, "default npc count");
    Check(options.vsync, "vsync is on by default");
    Check(!options.IsBenchmark() && !options.IsScreenshot(), "no mode by default");
}

void CommandLineReadsValues() {
    const Options options = Parse({"--seed", "7", "--npcs", "250", "--sun", "30", "--render-scale", "0.75"});
    Check(options.seed == 7, "seed");
    Check(options.npcCount == 250, "npc count");
    Check(options.sunElevationDegrees == 30.0f, "sun elevation");
    Check(options.renderScale == 0.75f, "render scale");
}

void CommandLineReadsCamera() {
    const Options options = Parse({"--camera", "1,2,3,90,-10"});
    Check(options.camera.has_value(), "camera is set");
    Check(options.camera->position.z == 3.0f && options.camera->yawDegrees == 90.0f, "camera values");
}

void CommandLineRejectsNanAndInfinity() {
    Check(IsRejected({"--time", "nan"}), "--time nan");
    Check(IsRejected({"--sun", "nan"}), "--sun nan");
    Check(IsRejected({"--render-scale", "nan"}), "--render-scale nan");
    Check(IsRejected({"--gpu-budget", "nan"}), "--gpu-budget nan");
    Check(IsRejected({"--benchmark", "10", "--target-fps", "nan"}), "--target-fps nan");
    Check(IsRejected({"--camera", "0,0,inf,0,0"}), "--camera inf");
}

void CommandLineRejectsNegativeUnsigned() {
    Check(IsRejected({"--npcs", "-1"}), "--npcs -1");
    Check(IsRejected({"--seed", "-5"}), "--seed -5");
    Check(IsRejected({"--npcs", "99999999999"}), "--npcs beyond 32 bits");
}

void CommandLineRejectsBadInput() {
    Check(IsRejected({"--npcs"}), "missing value");
    Check(IsRejected({"--frobnicate"}), "unknown option");
    Check(IsRejected({"--benchmark", "0"}), "zero benchmark frames");
    Check(IsRejected({"--render-scale", "2"}), "render scale out of range");
    Check(IsRejected({"--camera", "1,2,3"}), "camera with too few numbers");
}

void CommandLineRejectsConflictingModes() {
    Check(IsRejected({"--benchmark", "10", "--screenshot", "a.png"}), "benchmark with screenshot");
    Check(IsRejected({"--benchmark", "10", "--camera", "0,0,0,0,0"}), "benchmark with camera");
    Check(IsRejected({"--target-fps", "60"}), "target fps without benchmark");
}

void CommandLineCanSwitchTerracesOff() {
    Check(Parse({}).terraces, "terraces are on by default");
    Check(!Parse({"--no-terraces"}).terraces, "and --no-terraces turns them off");
}

void CommandLineReadsSatellites() {
    Check(!Parse({}).satelliteCount.has_value(), "unset by default");
    Check(Parse({"--satellites", "0"}).satelliteCount == 0, "zero is allowed");
    Check(Parse({"--satellites", "4"}).satelliteCount == 4, "four");
    Check(IsRejected({"--satellites", "9"}), "too many satellites");
    Check(IsRejected({"--satellites", "-1"}), "negative satellites");
}

void BenchmarkAndScreenshotTurnVsyncOff() {
    Check(!Parse({"--benchmark", "100"}).vsync, "benchmark disables vsync");
    Check(!Parse({"--screenshot", "a.png"}).vsync, "screenshot disables vsync");
}

void FrameSummaryOfNothingIsEmpty() {
    const FrameTimeSummary summary = SummarizeFrameTimes({}, 16.0);
    Check(summary.frameCount == 0 && summary.worstMs == 0.0, "empty summary");
    Check(Average({}) == 0.0, "average of nothing");
}

void FrameSummaryUsesNearestRank() {
    std::vector<double> frames;
    for (int ms = 1; ms <= 100; ++ms) {
        frames.push_back(ms);
    }
    const FrameTimeSummary summary = SummarizeFrameTimes(frames, 90.0);
    Check(summary.frameCount == 100, "frame count");
    Check(summary.averageMs == 50.5, "average");
    Check(summary.medianMs == 50.0, "median");
    Check(summary.p99Ms == 99.0, "p99");
    Check(summary.p999Ms == 100.0, "p99.9");
    Check(summary.worstMs == 100.0, "worst");
    Check(std::abs(summary.shareOverBudget - 0.10) < 1e-12, "share over budget");
}

void FrameSummaryIgnoresInputOrder() {
    const FrameTimeSummary summary = SummarizeFrameTimes({30.0, 10.0, 20.0}, 25.0);
    Check(summary.medianMs == 20.0 && summary.worstMs == 30.0, "unsorted input");
}

// A city with no streets and one building, 12 m long and 2 m wide, standing at the origin and
// turned by `yaw`. Its long side runs along world X at yaw 0.
crowd::Walkways WalkwaysWithOneBuilding(float yaw) {
    CityLayout city;
    city.bounds = {{-30.0f, -30.0f}, {30.0f, 30.0f}};
    city.boxes.push_back({.center = {0.0f, 1.0f, 0.0f}, .halfSize = {6.0f, 1.0f, 1.0f}, .yaw = yaw});
    crowd::Walkways walkways;
    walkways.Build(city);
    return walkways;
}

void ObstacleStaysAxisAlignedAtYawZero() {
    const crowd::Walkways walkways = WalkwaysWithOneBuilding(0.0f);
    Check(walkways.IsInsideObstacle({5.0f, 0.0f}), "inside near the end of the long side");
    Check(!walkways.IsInsideObstacle({0.0f, 3.0f}), "outside beside the building");
    Check(!walkways.IsInsideObstacle({7.0f, 0.0f}), "outside past the end");
}

void ObstacleFollowsAQuarterTurn() {
    const float quarterTurn = 1.5707964f;
    const crowd::Walkways walkways = WalkwaysWithOneBuilding(quarterTurn);
    Check(walkways.IsInsideObstacle({0.0f, 5.0f}), "the long side now runs along Z");
    Check(walkways.IsInsideObstacle({0.0f, -5.0f}), "and in both directions");
    Check(!walkways.IsInsideObstacle({5.0f, 0.0f}), "world X is now beside the building");
}

void ObstacleFollowsADiagonalTurn() {
    const float eighthTurn = 0.7853982f;
    const crowd::Walkways walkways = WalkwaysWithOneBuilding(eighthTurn);
    // Positive yaw turns +X towards -Z, so the long side runs from (+X, -Z) to (-X, +Z).
    Check(walkways.IsInsideObstacle({3.5f, -3.5f}), "inside along the diagonal");
    Check(walkways.IsInsideObstacle({-3.5f, 3.5f}), "inside along the diagonal, other end");
    Check(!walkways.IsInsideObstacle({3.5f, 3.5f}), "outside across the diagonal");
    Check(!walkways.IsInsideObstacle({5.0f, 0.0f}), "the corner of the enclosing rectangle is empty");
}

// A square, a room 25 m to its east, and a passage between them along Z = 0. Every piece is given
// in the layout the way the generator gives it.
CityLayout CityWithARoom() {
    CityLayout city;
    city.bounds = {{-40.0f, -40.0f}, {40.0f, 40.0f}};
    city.plazas.push_back(Rect{{-30.0f, -10.0f}, {-10.0f, 10.0f}});
    Interior room{LandmarkKind::ParishChurch, OrientedRect(Rect{{10.0f, -6.0f}, {30.0f, 6.0f}}), {10.4f, 0.0f}};
    room.hasPassage = true;
    room.passage = OrientedRect(Rect{{-15.0f, -1.0f}, {12.0f, 1.0f}});
    room.outsideSpot = {-15.0f, 0.0f};
    room.insideSpot = {12.0f, 0.0f};
    city.interiors.push_back(room);
    return city;
}

void ARoomWithAFreePassageIsAPortal() {
    crowd::Walkways walkways;
    walkways.Build(CityWithARoom());
    Check(walkways.Plazas().size() == 2, "the room is a square for the crowd");
    Check(walkways.Portals().size() == 1, "one portal");
    const crowd::Walkways::Portal& portal = walkways.Portals().front();
    Check(portal.outsidePlaza == 0 && portal.insidePlaza == 1, "from the square into the room");
    Check(walkways.PortalsOfPlaza(0).size() == 1 && walkways.PortalsOfPlaza(1).size() == 1, "both ends know it");
    Check(walkways.IsWalkable({0.0f, 0.0f}, 0.0f), "the passage is walkable ground");
    Check(!walkways.IsWalkable({0.0f, 4.0f}, 0.0f), "and the ground beside it is not");
}

void AConversationCannotStandOnAPassage() {
    crowd::Walkways walkways;
    walkways.Build(CityWithARoom());
    Check(!walkways.IsDiscFree({0.0f, 0.0f}, 1.0f), "a disc on the passage is not free");
    Check(!walkways.IsDiscFree({0.0f, 2.0f}, 1.5f), "nor one that reaches into it");
    Check(walkways.IsDiscFree({-20.0f, 6.0f}, 1.0f), "a disc in the square away from the passage is free");
}

void ABlockedPassageIsNotAPortal() {
    CityLayout city = CityWithARoom();
    city.boxes.push_back({.center = {0.0f, 1.0f, 0.0f}, .halfSize = {0.5f, 1.0f, 0.5f}});
    crowd::Walkways walkways;
    walkways.Build(city);
    Check(walkways.Portals().empty(), "a pillar in the passage closes the portal");
    Check(walkways.PortalsOfPlaza(0).empty() && walkways.PortalsOfPlaza(1).empty(), "and neither end lists one");
}

void APassageThatStartsOutsideEverySquareIsNotAPortal() {
    CityLayout city = CityWithARoom();
    city.interiors.front().outsideSpot = {-35.0f, 0.0f};
    crowd::Walkways walkways;
    walkways.Build(city);
    Check(walkways.Portals().empty(), "there is no square to come from");
}

void CommandLineReadsProps() {
    Check(Parse({}).propCount == 0, "no props by default");
    Check(Parse({"--props", "50"}).propCount == 50, "fifty props");
    Check(IsRejected({"--props", "99999"}), "too many props");
    Check(IsRejected({"--props", "-3"}), "negative props");
}

void ABodyIsDrawnWithTheLayoutOfABox() {
    Check(sizeof(BodyInstance) == sizeof(BoxInstance), "body.vert reads the same instance size as box.vert");
}

// A flat ground 120 m across with a square on it, as a city of one block.
CityLayout CityWithASquare() {
    CityLayout city;
    city.bounds = {{-60.0f, -60.0f}, {60.0f, 60.0f}};
    city.boxes.push_back({.center = {0.0f, -0.5f, 0.0f}, .halfSize = {60.0f, 0.5f, 60.0f}});
    city.boxParts.push_back(CityPart::Ground);
    city.plazas.push_back(Rect{{-6.0f, -6.0f}, {6.0f, 6.0f}});     // a small square
    city.plazas.push_back(Rect{{-30.0f, 10.0f}, {30.0f, 50.0f}});  // and a larger one
    return city;
}

bool InstancesAreFinite(std::span<const BodyInstance> instances) {
    for (const BodyInstance& instance : instances) {
        const glm::vec3 c = instance.center;
        const glm::vec4 q = instance.orientation;
        if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z) || !std::isfinite(q.x) || !std::isfinite(q.w)) {
            return false;
        }
    }
    return true;
}

void PropsFallOnTheGroundAndComeToRest() {
    physics::PropField props;
    props.Spawn(CityWithASquare(), {.count = 80, .seed = 5});
    Check(props.Count() == 80, "every prop is made");
    Check(props.Boxes().size() + props.Spheres().size() == 80, "and drawn as a box or a sphere");
    for (int step = 0; step < 20 * 60; ++step) {
        props.Update(1.0f / 60.0f);
    }
    Check(InstancesAreFinite(props.Boxes()) && InstancesAreFinite(props.Spheres()), "no prop has a number that is not a number");
    float lowest = 1e9f;
    for (const BodyInstance& instance : props.Boxes()) {
        lowest = std::min(lowest, instance.center.y);
    }
    for (const BodyInstance& instance : props.Spheres()) {
        lowest = std::min(lowest, instance.center.y);
    }
    Check(lowest > 0.15f, "nothing has sunk into the ground, lowest centre " + std::to_string(lowest));
    Check(props.AwakeCount() < 15, "the heap has come to rest, awake " + std::to_string(props.AwakeCount()));
}

void PropsAreDroppedOverTheLargestSquare() {
    physics::PropField props;
    props.Spawn(CityWithASquare(), {.count = 10, .seed = 1});
    Check(props.DropCentre().y > 10.0f && props.DropCentre().y < 50.0f, "the heap is over the larger square");
    for (const BodyInstance& instance : props.Boxes()) {
        Check(instance.center.z > 10.0f && instance.center.z < 50.0f && std::abs(instance.center.x) < 30.0f, "and every prop starts over it");
    }
}

void PropsAreTheSameEveryTime() {
    physics::PropField first;
    physics::PropField second;
    first.Spawn(CityWithASquare(), {.count = 60, .seed = 9});
    second.Spawn(CityWithASquare(), {.count = 60, .seed = 9});
    for (int step = 0; step < 6 * 60; ++step) {
        first.Update(1.0f / 60.0f);
        second.Update(1.0f / 60.0f);
    }
    bool identical = first.Boxes().size() == second.Boxes().size();
    for (size_t i = 0; identical && i < first.Boxes().size(); ++i) {
        identical = first.Boxes()[i].center == second.Boxes()[i].center && first.Boxes()[i].orientation == second.Boxes()[i].orientation;
    }
    Check(identical, "two heaps with the same seed fall the same way");
}

void APropOnAHeightenedGroundIsDroppedOnItToo() {
    // The props fall on whatever is solid near the square, not only on ground at the level of the city.
    CityLayout city = CityWithASquare();
    city.boxes.push_back({.center = {0.0f, 3.0f, 30.0f}, .halfSize = {30.0f, 3.0f, 20.0f}});  // a platform six metres high under the square
    city.boxParts.push_back(CityPart::Terrace);
    physics::PropField props;
    props.Spawn(city, {.count = 20, .seed = 3});
    for (int step = 0; step < 10 * 60; ++step) {
        props.Update(1.0f / 60.0f);
    }
    float lowest = 1e9f;
    for (const BodyInstance& instance : props.Boxes()) {
        lowest = std::min(lowest, instance.center.y);
    }
    Check(lowest > 6.1f, "props start above a platform higher than the first layer and rest on it, lowest centre " +
                             std::to_string(lowest));
}

void PropsThatFallOutOfTheWorldAreTakenOutOfTheSimulation() {
    CityLayout city;  // no ground at all
    city.bounds = {{-60.0f, -60.0f}, {60.0f, 60.0f}};
    city.plazas.push_back(Rect{{-10.0f, -10.0f}, {10.0f, 10.0f}});
    physics::PropField props;
    props.Spawn(city, {.count = 30, .seed = 2});
    for (int step = 0; step < 10 * 60; ++step) {
        props.Update(1.0f / 60.0f);
    }
    Check(props.AwakeCount() == 0, "props that fell far below the world no longer cost anything, awake " +
                                       std::to_string(props.AwakeCount()));
}

constexpr float kQuarterTurn = 1.5707964f;
constexpr float kEighthTurn = 0.7853982f;

// A 12 m by 2 m rectangle at the origin, turned by `yaw`.
OrientedRect LongRectangle(float yaw, glm::vec2 center = {0.0f, 0.0f}) {
    return {Rect{center - glm::vec2(6.0f, 1.0f), center + glm::vec2(6.0f, 1.0f)}, yaw};
}

void OrientedRectWithoutTurnIsTheRect() {
    const Rect plain{{-6.0f, -1.0f}, {6.0f, 1.0f}};
    const OrientedRect rect = plain;
    Check(rect.Contains({5.9f, 0.9f}) && !rect.Contains({6.1f, 0.0f}), "contains");
    Check(rect.Contains({6.5f, 0.0f}, 1.0f), "tolerance widens it");
    Check(rect.Bounds().min == plain.min && rect.Bounds().max == plain.max, "bounds are the rectangle");
    Check(rect.ToFrame({3.0f, 4.0f}) == glm::vec2(3.0f, 4.0f), "frame is the world");
    Check(rect.Overlaps(OrientedRect(Rect{{5.0f, 0.0f}, {9.0f, 3.0f}})), "overlap");
    Check(!rect.Overlaps(OrientedRect(Rect{{6.0f, -1.0f}, {9.0f, 1.0f}})), "touching edges do not overlap");
}

void OrientedRectFollowsItsTurn() {
    const OrientedRect rect = LongRectangle(kQuarterTurn);
    Check(rect.Contains({0.0f, 5.0f}) && rect.Contains({0.0f, -5.0f}), "long side runs along Z");
    Check(!rect.Contains({5.0f, 0.0f}), "world X is beside it");
    const Rect bounds = rect.Bounds();
    Check(std::abs(bounds.Size().x - 2.0f) < 1e-4f && std::abs(bounds.Size().y - 12.0f) < 1e-4f, "bounds swap sides");
}

void OrientedRectFrameRoundTrips() {
    const OrientedRect rect = LongRectangle(0.6f, {40.0f, -25.0f});
    const glm::vec2 point{51.5f, -22.25f};
    const glm::vec2 back = rect.FromFrame(rect.ToFrame(point));
    Check(glm::distance(back, point) < 1e-3f, "frame round trip");
    // The corners of the frame's rectangle are corners in the world too.
    for (glm::vec2 corner : rect.Corners()) {
        Check(rect.Contains(corner, 1e-3f), "a corner is on the rectangle");
    }
}

void OrientedRectOverlapUsesTheTurnNotTheBounds() {
    const OrientedRect diagonal = LongRectangle(kEighthTurn);
    // Another long rectangle turned the same way, 3 m to the side: a 1 m gap between them even
    // though their bounding boxes overlap a great deal.
    const glm::vec2 side(std::sin(kEighthTurn), std::cos(kEighthTurn));
    const OrientedRect apart = LongRectangle(kEighthTurn, side * 3.0f);
    const OrientedRect close = LongRectangle(kEighthTurn, side * 1.5f);
    Check(diagonal.Bounds().Overlaps(apart.Bounds()), "the bounding boxes do overlap");
    Check(!diagonal.Overlaps(apart), "but the rectangles do not");
    Check(diagonal.Overlaps(close), "overlapping rectangles are found");
    // Two thin rectangles crossing like an X.
    Check(LongRectangle(kEighthTurn).Overlaps(LongRectangle(-kEighthTurn)), "a cross overlaps");
}

void LevelTerrainHasNoLift() {
    const TerrainField level{std::vector<TerrainZone>{}};
    Check(level.IsLevel(), "no zones means level");
    Check(level.LiftAt({123.0f, -456.0f}) == 0.0f, "exactly zero everywhere");
}

void PlateauLiftsOnlyItsOwnArea() {
    const TerrainField terrain({MakePlateau(OrientedRect(Rect{{-10.0f, -10.0f}, {10.0f, 10.0f}}), 4.5f)});
    Check(terrain.LiftAt({0.0f, 0.0f}) == 4.5f && terrain.LiftAt({9.9f, -9.9f}) == 4.5f, "inside");
    Check(terrain.LiftAt({10.1f, 0.0f}) == 0.0f && terrain.LiftAt({0.0f, -10.1f}) == 0.0f, "outside");
}

void TurnedPlateauFollowsItsTurn() {
    // 12 m by 2 m, turned a quarter, so it runs along Z.
    const TerrainField terrain({MakePlateau(LongRectangle(kQuarterTurn), 3.0f)});
    Check(terrain.LiftAt({0.0f, 5.0f}) == 3.0f, "along the long side");
    Check(terrain.LiftAt({5.0f, 0.0f}) == 0.0f, "beside it");
}

void StairClimbsOneRisePerTread() {
    // Ten metres along +X from the ground at lift 1.0, 0.4 m treads and 0.15 m rises.
    const TerrainField terrain({TerrainZone{OrientedRect(Rect{{0.0f, -5.0f}, {10.0f, 5.0f}}), 1.0f, 0.15f, 0.4f}});
    Check(std::abs(terrain.LiftAt({0.1f, 0.0f}) - 1.15f) < 1e-5f, "the first step stands one rise up");
    Check(std::abs(terrain.LiftAt({0.5f, 0.0f}) - 1.30f) < 1e-5f, "the second step");
    Check(std::abs(terrain.LiftAt({9.99f, 0.0f}) - (1.0f + 25.0f * 0.15f)) < 1e-4f, "the last step: 25 treads");
    Check(std::abs(terrain.LiftAt({10.0f, 0.0f}) - (1.0f + 25.0f * 0.15f)) < 1e-4f, "the far edge belongs to the last step");
    Check(terrain.LiftAt({-0.1f, 0.0f}) == 0.0f && terrain.LiftAt({10.1f, 0.0f}) == 0.0f, "level before and after");
}

void StairFarFromTheOriginHasTheRightNumberOfSteps() {
    // 27 steps of 0.4 m. Far from the origin the width of the zone is off by about 1e-4 m, which must not
    // count as a 28th step: the last step stands at the lift of the terrace above it.
    const float rise = 4.5f / 27.0f;
    const TerrainField terrain({TerrainZone{OrientedRect(Rect{{2400.0f, 1500.0f}, {2410.8f, 1507.0f}}), 0.0f, rise, 0.4f}});
    Check(std::abs(terrain.LiftAt({2410.79f, 1503.0f}) - 4.5f) < 1e-3f, "the last step is level with the terrace");
    Check(std::abs(terrain.LiftAt({2410.8f, 1503.0f}) - 4.5f) < 1e-3f, "and so is the far edge");
}

void StairWithoutATreadIsATerrace() {
    const TerrainField terrain({TerrainZone{OrientedRect(Rect{{0.0f, 0.0f}, {10.0f, 10.0f}}), 2.0f, 0.15f, 0.0f}});
    const float lift = terrain.LiftAt({5.0f, 5.0f});
    Check(std::isfinite(lift) && lift == 2.0f, "a zero tread must not give an infinite or undefined lift");
}

void HighestLiftCountsWhereZonesMeet() {
    const TerrainField terrain({MakePlateau(OrientedRect(Rect{{0.0f, 0.0f}, {10.0f, 10.0f}}), 2.0f),
                                MakePlateau(OrientedRect(Rect{{10.0f, 0.0f}, {20.0f, 10.0f}}), 5.0f)});
    Check(terrain.LiftAt({10.0f, 5.0f}) == 5.0f, "on the shared edge");
}

// A city with no streets and one building 2 m tall that stands on the ground it is on, or, with a
// terrace under it, on a terrace 50 m up.
crowd::Walkways WalkwaysWithBuildingOnATerrace(bool terraceRecorded) {
    CityLayout city;
    city.bounds = {{-30.0f, -30.0f}, {30.0f, 30.0f}};
    city.boxes.push_back({.center = {0.0f, 51.0f, 0.0f}, .halfSize = {6.0f, 1.0f, 6.0f}});
    if (terraceRecorded) {
        city.terrain.push_back(MakePlateau(OrientedRect(Rect{{-30.0f, -30.0f}, {30.0f, 30.0f}}), 50.0f));
    }
    crowd::Walkways walkways;
    walkways.Build(city);
    return walkways;
}

void ObstacleOnATerraceIsAnObstacle() {
    // The box spans 50 to 52 m. On the terrace that is the first two metres above the ground, so
    // people walk into it. At the level of the city it is a ceiling far overhead.
    Check(WalkwaysWithBuildingOnATerrace(true).IsInsideObstacle({0.0f, 0.0f}), "a building on a terrace is in the way");
    Check(!WalkwaysWithBuildingOnATerrace(false).IsInsideObstacle({0.0f, 0.0f}), "the same box without a terrace is overhead");
}

// A single street 200 m long on a terrace 100 m above the level of the city, with 300 people on it.
CityLayout StreetOnAHighTerrace() {
    CityLayout city;
    city.bounds = {{-300.0f, -300.0f}, {300.0f, 300.0f}};
    city.roadWidth = 8.0f;
    city.sidewalkWidth = 3.5f;
    city.sidewalkCenterOffset = 5.75f;
    city.walkHeight = 0.15f;
    city.streets.nodes = {{0.0f, 0.0f}, {200.0f, 0.0f}};
    city.streets.edges = {{0, 1, 200.0f}};
    city.streets.edgesAtNode = {{0}, {0}};
    city.terrain.push_back(MakePlateau(OrientedRect(Rect{{-300.0f, -300.0f}, {300.0f, 300.0f}}), 100.0f));
    return city;
}

void CameraOnATerraceSeesThePeopleOnItAsNear() {
    // People simulate in more detail the nearer the camera is, and the gap in height counts. The camera
    // stands on the terrace, so the people on it are beside it. Measured from the level of the city they
    // would look 100 m below, and every one would be simulated as far away.
    const CityLayout city = StreetOnAHighTerrace();
    Crowd crowd;
    crowd.Spawn(city, {.agentCount = 300, .seed = 1});
    crowd.SetFocusPoint({100.0f, 100.0f + city.walkHeight + 1.7f, 0.0f});
    crowd.Update(1.0f / 60.0f);
    Check(crowd.Stats().nearAgents > 0, "people on the camera's own terrace are near it");
}

void WalkHeightFollowsTheTerrain() {
    const crowd::Walkways walkways = WalkwaysWithBuildingOnATerrace(true);
    Check(walkways.WalkHeightAt({0.0f, 0.0f}) == walkways.WalkHeight() + 50.0f, "people stand on the terrace");
    Check(walkways.WalkHeightAt({100.0f, 0.0f}) == walkways.WalkHeight(), "and at the level of the city elsewhere");
}

struct Test {
    const char* name;
    std::function<void()> run;
};
}  // namespace

int main() {
    const std::vector<Test> tests = {
        {"CommandLineDefaults", CommandLineDefaults},
        {"CommandLineReadsValues", CommandLineReadsValues},
        {"CommandLineReadsCamera", CommandLineReadsCamera},
        {"CommandLineRejectsNanAndInfinity", CommandLineRejectsNanAndInfinity},
        {"CommandLineRejectsNegativeUnsigned", CommandLineRejectsNegativeUnsigned},
        {"CommandLineRejectsBadInput", CommandLineRejectsBadInput},
        {"CommandLineRejectsConflictingModes", CommandLineRejectsConflictingModes},
        {"CommandLineCanSwitchTerracesOff", CommandLineCanSwitchTerracesOff},
        {"CommandLineReadsSatellites", CommandLineReadsSatellites},
        {"CommandLineReadsProps", CommandLineReadsProps},
        {"BenchmarkAndScreenshotTurnVsyncOff", BenchmarkAndScreenshotTurnVsyncOff},
        {"FrameSummaryOfNothingIsEmpty", FrameSummaryOfNothingIsEmpty},
        {"FrameSummaryUsesNearestRank", FrameSummaryUsesNearestRank},
        {"FrameSummaryIgnoresInputOrder", FrameSummaryIgnoresInputOrder},
        {"OrientedRectWithoutTurnIsTheRect", OrientedRectWithoutTurnIsTheRect},
        {"OrientedRectFollowsItsTurn", OrientedRectFollowsItsTurn},
        {"OrientedRectFrameRoundTrips", OrientedRectFrameRoundTrips},
        {"OrientedRectOverlapUsesTheTurnNotTheBounds", OrientedRectOverlapUsesTheTurnNotTheBounds},
        {"LevelTerrainHasNoLift", LevelTerrainHasNoLift},
        {"PlateauLiftsOnlyItsOwnArea", PlateauLiftsOnlyItsOwnArea},
        {"TurnedPlateauFollowsItsTurn", TurnedPlateauFollowsItsTurn},
        {"StairClimbsOneRisePerTread", StairClimbsOneRisePerTread},
        {"StairFarFromTheOriginHasTheRightNumberOfSteps", StairFarFromTheOriginHasTheRightNumberOfSteps},
        {"StairWithoutATreadIsATerrace", StairWithoutATreadIsATerrace},
        {"HighestLiftCountsWhereZonesMeet", HighestLiftCountsWhereZonesMeet},
        {"ObstacleOnATerraceIsAnObstacle", ObstacleOnATerraceIsAnObstacle},
        {"CameraOnATerraceSeesThePeopleOnItAsNear", CameraOnATerraceSeesThePeopleOnItAsNear},
        {"WalkHeightFollowsTheTerrain", WalkHeightFollowsTheTerrain},
        {"ObstacleStaysAxisAlignedAtYawZero", ObstacleStaysAxisAlignedAtYawZero},
        {"ObstacleFollowsAQuarterTurn", ObstacleFollowsAQuarterTurn},
        {"ObstacleFollowsADiagonalTurn", ObstacleFollowsADiagonalTurn},
        {"ABodyIsDrawnWithTheLayoutOfABox", ABodyIsDrawnWithTheLayoutOfABox},
        {"PropsFallOnTheGroundAndComeToRest", PropsFallOnTheGroundAndComeToRest},
        {"PropsAreDroppedOverTheLargestSquare", PropsAreDroppedOverTheLargestSquare},
        {"PropsAreTheSameEveryTime", PropsAreTheSameEveryTime},
        {"APropOnAHeightenedGroundIsDroppedOnItToo", APropOnAHeightenedGroundIsDroppedOnItToo},
        {"PropsThatFallOutOfTheWorldAreTakenOutOfTheSimulation", PropsThatFallOutOfTheWorldAreTakenOutOfTheSimulation},
        {"ARoomWithAFreePassageIsAPortal", ARoomWithAFreePassageIsAPortal},
        {"AConversationCannotStandOnAPassage", AConversationCannotStandOnAPassage},
        {"ABlockedPassageIsNotAPortal", ABlockedPassageIsNotAPortal},
        {"APassageThatStartsOutsideEverySquareIsNotAPortal", APassageThatStartsOutsideEverySquareIsNotAPortal},
    };
    for (const Test& test : tests) {
        const int failuresBefore = failureCount;
        try {
            test.run();
        } catch (const std::exception& error) {
            ++failureCount;
            std::cerr << "  threw: " << error.what() << "\n";
        }
        std::cout << (failureCount == failuresBefore ? "PASS  " : "FAIL  ") << test.name << "\n";
    }
    return failureCount == 0 ? 0 : 1;
}
