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

void CommandLineReadsTheRiverSimulation() {
    Check(!Parse({}).riverSimulation && Parse({}).rainPerSecond == 40, "the river is not simulated by default");
    Check(Parse({"--river-sim"}).riverSimulation, "and --river-sim turns it on");
    Check(Parse({"--rain", "300"}).rainPerSecond == 300, "rain");
    Check(IsRejected({"--rain", "99999"}), "too much rain");
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

// A city that is a river 400 m long and 100 m wide, with its water 4.5 m below the street, and no land.
CityLayout CityWithAWideRiver() {
    CityLayout city;
    city.waterLevel = -4.5f;
    city.bounds = {{-200.0f, -200.0f}, {200.0f, 200.0f}};
    city.water.push_back(OrientedRect(Rect{{-200.0f, -50.0f}, {200.0f, 50.0f}}));
    return city;
}

struct PropsInARiver {
    fluid::RiverWater river;
    physics::PropField props;

    explicit PropsInARiver(uint32_t count, uint32_t seed = 1, glm::vec2 dropOffset = glm::vec2(0.0f)) {
        const CityLayout city = CityWithAWideRiver();
        fluid::RiverWaterSettings settings;
        settings.cells = 128;
        settings.spongeWidth = 16.0f;
        river.Build(city, {0.0f, 0.0f}, settings);
        physics::PropSettings dropped{.count = count, .seed = seed};
        dropped.dropAt = river.Centre() + dropOffset;
        dropped.riverBed = river.RestLevel() - settings.depth;
        props.Spawn(city, dropped);
        props.SetWater(&river);
    }

    // One frame of a sixtieth of a second: the props, then the water they have disturbed.
    void Frame() {
        props.Update(1.0f / 60.0f);
        river.Update(1.0f / 60.0f);
    }
};

void WoodFloatsInARiverAndStoneSinksToItsBed() {
    PropsInARiver scene(80);
    float biggestWave = 0.0f;
    for (int frame = 0; frame < 15 * 60; ++frame) {
        scene.Frame();
        for (float offset : scene.river.Field()) {
            if (offset != fluid::kNoWater) {
                biggestWave = std::max(biggestWave, std::abs(offset));
            }
        }
    }
    int floating = 0;
    int sunk = 0;
    int wrong = 0;
    for (const BodyInstance& box : scene.props.Boxes()) {
        const bool stone = box.material == static_cast<float>(SurfaceMaterial::Stone);
        const float surface = scene.river.RestLevel();
        if (stone) {
            // Rests on the bed, 7.5 m down, with its centre half its height above it.
            const bool onTheBed = box.center.y < surface - 6.0f && box.center.y > surface - 7.6f;
            sunk += onTheBed ? 1 : 0;
            wrong += onTheBed ? 0 : 1;
        } else if (box.roughness < 0.6f) {
            continue;  // a barrel: hollow, and floats too, but low in the water; counted with the crates below
        } else {
            const bool atTheSurface = box.center.y > surface - 0.9f && box.center.y < surface + 0.9f;
            floating += atTheSurface ? 1 : 0;
            wrong += atTheSurface ? 0 : 1;
        }
    }
    Check(floating > 15, "crates of wood float at the surface of the river, " + std::to_string(floating));
    Check(sunk >= 5, "and blocks of stone lie on the bed, " + std::to_string(sunk));
    Check(wrong == 0, "and none of them is anywhere else, " + std::to_string(wrong));
    Check(biggestWave > 0.02f, "the props make waves as they fall in, " + std::to_string(biggestWave));
}

void PropsInARiverNeverGoThroughItsBed() {
    PropsInARiver scene(120, 3);
    for (int frame = 0; frame < 12 * 60; ++frame) {
        scene.Frame();
    }
    float lowest = 1e9f;
    for (const BodyInstance& instance : scene.props.Boxes()) {
        lowest = std::min(lowest, instance.center.y);
    }
    for (const BodyInstance& instance : scene.props.Spheres()) {
        lowest = std::min(lowest, instance.center.y);
    }
    Check(lowest > scene.river.RestLevel() - 7.5f - 0.05f, "nothing is below the bed of the river, lowest centre " + std::to_string(lowest));
    Check(InstancesAreFinite(scene.props.Boxes()) && InstancesAreFinite(scene.props.Spheres()), "and no prop has a number that is not a number");
}

void PropsInARiverAreTheSameEveryTime() {
    PropsInARiver first(40, 5);
    PropsInARiver second(40, 5);
    for (int frame = 0; frame < 6 * 60; ++frame) {
        first.Frame();
        second.Frame();
    }
    bool identical = first.props.Boxes().size() == second.props.Boxes().size();
    for (size_t i = 0; identical && i < first.props.Boxes().size(); ++i) {
        identical = first.props.Boxes()[i].center == second.props.Boxes()[i].center;
    }
    for (size_t i = 0; identical && i < first.river.Field().size(); ++i) {
        identical = first.river.Field()[i] == second.river.Field()[i];
    }
    Check(identical, "the props and the water are the same, bit for bit, in two runs");
}

void TheHeapIsDroppedWhereAskedAndOnlyThere() {
    PropsInARiver scene(30);
    for (const BodyInstance& instance : scene.props.Boxes()) {
        Check(glm::distance(glm::vec2(instance.center.x, instance.center.z), scene.river.Centre()) < 12.0f,
              "every prop starts over the middle of the river");
    }
}

// How deep a box sits in the water, as the share of its height that is under the surface.
float DraftOf(const BodyInstance& box, float surface) {
    const glm::mat3 axes = glm::mat3_cast(glm::quat(box.orientation.w, box.orientation.x, box.orientation.y, box.orientation.z));
    const float halfHeight = std::abs(axes[0].y) * box.halfSize.x + std::abs(axes[1].y) * box.halfSize.y + std::abs(axes[2].y) * box.halfSize.z;
    return (surface - (box.center.y - halfHeight)) / (2.0f * halfHeight);
}

void ACrateOfWoodSettlesAtTheDraftItsDensityGivesAndFallsAsleep() {
    PropsInARiver scene(1);  // one crate of wood, at 450 kilograms per cubic metre
    for (int frame = 0; frame < 30 * 60; ++frame) {
        scene.Frame();
    }
    const float draft = DraftOf(scene.props.Boxes().front(), scene.river.RestLevel());
    Check(draft > 0.42f && draft < 0.49f, "a crate of density 450 sinks 45% of its height into the water, " + std::to_string(draft));
    Check(scene.props.AwakeCount() == 0, "and comes to rest and falls asleep, awake " + std::to_string(scene.props.AwakeCount()));
}

void AHeapOfPropsFloatsAndSettlesInsteadOfChurningForEver() {
    PropsInARiver scene(100, 2);
    for (int frame = 0; frame < 50 * 60; ++frame) {
        scene.Frame();
    }
    Check(scene.props.AwakeCount() < 15, "after fifty seconds a heap of a hundred props has nearly all come to rest, awake " +
                                             std::to_string(scene.props.AwakeCount()));
    float biggestWave = 0.0f;
    for (float offset : scene.river.Field()) {
        if (offset != fluid::kNoWater) {
            biggestWave = std::max(biggestWave, std::abs(offset));
        }
    }
    Check(biggestWave < 0.05f, "and the water round it is nearly calm, " + std::to_string(biggestWave));
}

void ACurrentCarriesWhatFloatsWithIt() {
    PropsInARiver scene(1);
    for (int frame = 0; frame < 15 * 60; ++frame) {
        scene.Frame();
    }
    const float start = scene.props.Boxes().front().center.x;
    for (int frame = 0; frame < 12 * 60; ++frame) {
        // A current of a metre a second along the river, kept up round the crate by a push every frame.
        const BodyInstance& crate = scene.props.Boxes().front();
        scene.river.Push({crate.center.x, crate.center.z}, 8.0f, {1.0f, 0.0f});
        scene.Frame();
    }
    const float travelled = scene.props.Boxes().front().center.x - start;
    Check(travelled > 6.0f, "a crate on a current of a metre a second goes along with it, " + std::to_string(travelled) + " m in twelve seconds");
}

void ASleepingCrateIsWokenByTheWaterMovingUnderIt() {
    PropsInARiver scene(1);
    for (int frame = 0; frame < 30 * 60; ++frame) {
        scene.Frame();
    }
    Check(scene.props.AwakeCount() == 0, "the crate is asleep");
    const float start = scene.props.Boxes().front().center.x;
    for (int frame = 0; frame < 10 * 60; ++frame) {
        const BodyInstance& crate = scene.props.Boxes().front();
        scene.river.Push({crate.center.x, crate.center.z}, 8.0f, {1.0f, 0.0f});
        scene.Frame();
    }
    Check(scene.props.Boxes().front().center.x - start > 3.0f, "a current wakes it and takes it along");
}

void WoodPastTheEdgeOfTheSimulatedWindowStillFloats() {
    // The window is 64 m across, and these props are dropped 80 m from its middle, over river that is not simulated.
    PropsInARiver scene(10, 4, {80.0f, 0.0f});
    for (int frame = 0; frame < 20 * 60; ++frame) {
        scene.Frame();
    }
    int floating = 0;
    for (const BodyInstance& box : scene.props.Boxes()) {
        floating += std::abs(box.center.y - scene.river.RestLevel()) < 0.9f ? 1 : 0;
    }
    Check(floating >= 5, "crates over river that is not simulated float on its rest level, " + std::to_string(floating) + " of " +
                             std::to_string(scene.props.Boxes().size()));
}

void BlocksOfStoneAreDroppedOnlyOverARiver() {
    const CityLayout city = CityWithAWideRiver();
    const auto stones = [](const physics::PropField& props) {
        int count = 0;
        for (const BodyInstance& box : props.Boxes()) {
            count += box.material == static_cast<float>(SurfaceMaterial::Stone) ? 1 : 0;
        }
        return count;
    };
    physics::PropField dry;
    dry.Spawn(city, {.count = 80, .seed = 1});
    Check(stones(dry) == 0, "a heap dropped on land has no blocks of stone, which keeps it as it was");
    physics::PropSettings wet{.count = 80, .seed = 1};
    wet.riverBed = -12.0f;
    physics::PropField river;
    river.Spawn(city, wet);
    Check(stones(river) >= 5, "and one dropped over a river has some");
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

// A level city with nothing in it, for the crates that people go round.
crowd::Walkways EmptyWalkways() {
    CityLayout city;
    city.bounds = {{-30.0f, -30.0f}, {30.0f, 30.0f}};
    crowd::Walkways walkways;
    walkways.Build(city);
    return walkways;
}

// A crate 1 m wide standing on the ground at the origin, or, with `bottom` raised, held in the air.
MovableObstacle CrateAtTheOrigin(float bottom) {
    return {.min = {-0.5f, -0.5f}, .max = {0.5f, 0.5f}, .bottom = bottom, .top = bottom + 1.0f};
}

void PeopleKeepABodyRadiusAwayFromACrate() {
    crowd::Walkways walkways = EmptyWalkways();
    const MovableObstacle crate = CrateAtTheOrigin(walkways.WalkHeight());
    walkways.SetMovableObstacles(std::span(&crate, 1));
    Check(!walkways.CanStandAt({0.0f, 0.0f}), "nobody stands inside a crate");
    Check(!walkways.CanStandAt({0.4f, 0.4f}), "nor near its corner");
    Check(walkways.CanStandAt({2.0f, 0.0f}), "but a metre and a half from its side they can");
    Check(walkways.CanStandAt({5.0f, 5.0f}), "and anywhere else");
}

void ACrateThatIsRemovedOpensThePlaceAgain() {
    crowd::Walkways walkways = EmptyWalkways();
    const MovableObstacle crate = CrateAtTheOrigin(walkways.WalkHeight());
    walkways.SetMovableObstacles(std::span(&crate, 1));
    walkways.SetMovableObstacles({});
    Check(walkways.CanStandAt({0.0f, 0.0f}), "the place is open when the crate is gone");
    Check(!walkways.CanStepOutOfMovableObstacle({0.0f, 0.0f}, {0.0f, 0.0f}), "and nobody needs to step out of it");
}

void ACrateThatMovedClosesItsNewPlaceAndOpensTheOld() {
    crowd::Walkways walkways = EmptyWalkways();
    MovableObstacle crate = CrateAtTheOrigin(walkways.WalkHeight());
    walkways.SetMovableObstacles(std::span(&crate, 1));
    crate.min += glm::vec2(10.0f, 0.0f);
    crate.max += glm::vec2(10.0f, 0.0f);
    walkways.SetMovableObstacles(std::span(&crate, 1));
    Check(walkways.CanStandAt({0.0f, 0.0f}), "the old place is open");
    Check(!walkways.CanStandAt({10.0f, 0.0f}), "the new place is closed");
}

void ACrateInTheAirOrFarBelowIsNoObstacle() {
    crowd::Walkways walkways = EmptyWalkways();
    const float ground = walkways.WalkHeight();
    const MovableObstacle overhead = CrateAtTheOrigin(ground + 2.5f);  // people walk under it
    walkways.SetMovableObstacles(std::span(&overhead, 1));
    Check(walkways.CanStandAt({0.0f, 0.0f}), "people walk under a crate held high");
    const MovableObstacle sunk = CrateAtTheOrigin(ground - 3.0f);  // it ends well below their feet
    walkways.SetMovableObstacles(std::span(&sunk, 1));
    Check(walkways.CanStandAt({0.0f, 0.0f}), "and over one that is down in the river bed");
}

void ANonFiniteCrateIsIgnored() {
    crowd::Walkways walkways = EmptyWalkways();
    const float notANumber = std::numeric_limits<float>::quiet_NaN();
    const MovableObstacle broken = {.min = {notANumber, 0.0f}, .max = {1.0f, 1.0f}, .bottom = 0.0f, .top = 1.0f};
    walkways.SetMovableObstacles(std::span(&broken, 1));
    Check(walkways.CanStandAt({0.0f, 0.0f}), "a crate with no position closes nothing");
}

// A crate 1 m wide whose middle is `shift` metres east of the origin. The grid of the walkways has cells of half a metre,
// so where the margin of a crate falls between their centres depends on where the crate stands.
MovableObstacle CrateShiftedEast(float ground, float shift) {
    MovableObstacle crate = CrateAtTheOrigin(ground);
    crate.min.x += shift;
    crate.max.x += shift;
    return crate;
}

void ACrateHasAMarginOfOneBodyRadius() {
    crowd::Walkways walkways = EmptyWalkways();
    bool marginIsClosed = false;
    for (float shift = 0.0f; shift < 0.5f; shift += 0.1f) {
        const MovableObstacle crate = CrateShiftedEast(walkways.WalkHeight(), shift);
        walkways.SetMovableObstacles(std::span(&crate, 1));
        for (float x = crate.max.x; x < crate.max.x + 0.3f; x += 0.05f) {
            marginIsClosed = marginIsClosed || !walkways.CanStandAt({x, 0.0f});
        }
        Check(walkways.CanStandAt({crate.max.x + 1.0f, 0.0f}), "and not much more than it");
    }
    Check(marginIsClosed, "the margin round the crate is closed to people");
    const MovableObstacle crate = CrateAtTheOrigin(walkways.WalkHeight());
    walkways.SetMovableObstacles(std::span(&crate, 1));
    Check(!walkways.CanStandAtUnreserved({0.0f, 0.0f}), "a crate closes places for those who keep out of conversations too");
}

void SomeoneInACrateMayWalkOutButNotDeeperAndNotThroughWalls() {
    CityLayout city;
    city.bounds = {{-30.0f, -30.0f}, {30.0f, 30.0f}};
    city.boxes.push_back({.center = {0.0f, 1.0f, 4.0f}, .halfSize = {6.0f, 1.0f, 1.0f}});  // a wall of a building to the north
    crowd::Walkways walkways;
    walkways.Build(city);
    const glm::vec2 farOutside(-5.0f, 0.0f);
    bool foundMarginPlace = false;
    for (float shift = 0.0f; shift < 0.5f; shift += 0.1f) {
        const MovableObstacle crate = CrateShiftedEast(walkways.WalkHeight(), shift);
        walkways.SetMovableObstacles(std::span(&crate, 1));
        const glm::vec2 middle = 0.5f * (crate.min + crate.max);
        Check(walkways.CanStepOutOfMovableObstacle(middle, middle + glm::vec2(0.0f, 0.1f)), "from inside a crate one may step on inside it");
        Check(!walkways.CanStepOutOfMovableObstacle(farOutside, middle), "from outside one may not walk into it");
        Check(!walkways.CanStepOutOfMovableObstacle(middle, {middle.x, 4.0f}), "and nobody may step into a wall of a building");
        for (float x = crate.max.x; x < crate.max.x + 0.3f; x += 0.05f) {
            const glm::vec2 place(x, 0.0f);
            // In the margin: closed to standing, yet a step in from there would be deeper.
            foundMarginPlace = foundMarginPlace || (!walkways.CanStandAt(place) && walkways.CanStepOutOfMovableObstacle(place, place) &&
                                                    !walkways.CanStepOutOfMovableObstacle(place, middle));
        }
    }
    Check(foundMarginPlace, "from the margin the middle of the crate is a step too deep");
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
        {"CommandLineReadsTheRiverSimulation", CommandLineReadsTheRiverSimulation},
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
        {"PeopleKeepABodyRadiusAwayFromACrate", PeopleKeepABodyRadiusAwayFromACrate},
        {"ACrateThatIsRemovedOpensThePlaceAgain", ACrateThatIsRemovedOpensThePlaceAgain},
        {"ACrateThatMovedClosesItsNewPlaceAndOpensTheOld", ACrateThatMovedClosesItsNewPlaceAndOpensTheOld},
        {"ACrateInTheAirOrFarBelowIsNoObstacle", ACrateInTheAirOrFarBelowIsNoObstacle},
        {"ANonFiniteCrateIsIgnored", ANonFiniteCrateIsIgnored},
        {"ACrateHasAMarginOfOneBodyRadius", ACrateHasAMarginOfOneBodyRadius},
        {"SomeoneInACrateMayWalkOutButNotDeeperAndNotThroughWalls", SomeoneInACrateMayWalkOutButNotDeeperAndNotThroughWalls},
        {"ObstacleStaysAxisAlignedAtYawZero", ObstacleStaysAxisAlignedAtYawZero},
        {"ObstacleFollowsAQuarterTurn", ObstacleFollowsAQuarterTurn},
        {"ObstacleFollowsADiagonalTurn", ObstacleFollowsADiagonalTurn},
        {"ABodyIsDrawnWithTheLayoutOfABox", ABodyIsDrawnWithTheLayoutOfABox},
        {"PropsFallOnTheGroundAndComeToRest", PropsFallOnTheGroundAndComeToRest},
        {"PropsAreDroppedOverTheLargestSquare", PropsAreDroppedOverTheLargestSquare},
        {"PropsAreTheSameEveryTime", PropsAreTheSameEveryTime},
        {"APropOnAHeightenedGroundIsDroppedOnItToo", APropOnAHeightenedGroundIsDroppedOnItToo},
        {"PropsThatFallOutOfTheWorldAreTakenOutOfTheSimulation", PropsThatFallOutOfTheWorldAreTakenOutOfTheSimulation},
        {"WoodFloatsInARiverAndStoneSinksToItsBed", WoodFloatsInARiverAndStoneSinksToItsBed},
        {"PropsInARiverNeverGoThroughItsBed", PropsInARiverNeverGoThroughItsBed},
        {"PropsInARiverAreTheSameEveryTime", PropsInARiverAreTheSameEveryTime},
        {"TheHeapIsDroppedWhereAskedAndOnlyThere", TheHeapIsDroppedWhereAskedAndOnlyThere},
        {"ACrateOfWoodSettlesAtTheDraftItsDensityGivesAndFallsAsleep", ACrateOfWoodSettlesAtTheDraftItsDensityGivesAndFallsAsleep},
        {"AHeapOfPropsFloatsAndSettlesInsteadOfChurningForEver", AHeapOfPropsFloatsAndSettlesInsteadOfChurningForEver},
        {"ACurrentCarriesWhatFloatsWithIt", ACurrentCarriesWhatFloatsWithIt},
        {"ASleepingCrateIsWokenByTheWaterMovingUnderIt", ASleepingCrateIsWokenByTheWaterMovingUnderIt},
        {"WoodPastTheEdgeOfTheSimulatedWindowStillFloats", WoodPastTheEdgeOfTheSimulatedWindowStillFloats},
        {"BlocksOfStoneAreDroppedOnlyOverARiver", BlocksOfStoneAreDroppedOnlyOverARiver},
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
