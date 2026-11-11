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
        {"CommandLineReadsSatellites", CommandLineReadsSatellites},
        {"BenchmarkAndScreenshotTurnVsyncOff", BenchmarkAndScreenshotTurnVsyncOff},
        {"FrameSummaryOfNothingIsEmpty", FrameSummaryOfNothingIsEmpty},
        {"FrameSummaryUsesNearestRank", FrameSummaryUsesNearestRank},
        {"FrameSummaryIgnoresInputOrder", FrameSummaryIgnoresInputOrder},
        {"OrientedRectWithoutTurnIsTheRect", OrientedRectWithoutTurnIsTheRect},
        {"OrientedRectFollowsItsTurn", OrientedRectFollowsItsTurn},
        {"OrientedRectFrameRoundTrips", OrientedRectFrameRoundTrips},
        {"OrientedRectOverlapUsesTheTurnNotTheBounds", OrientedRectOverlapUsesTheTurnNotTheBounds},
        {"ObstacleStaysAxisAlignedAtYawZero", ObstacleStaysAxisAlignedAtYawZero},
        {"ObstacleFollowsAQuarterTurn", ObstacleFollowsAQuarterTurn},
        {"ObstacleFollowsADiagonalTurn", ObstacleFollowsADiagonalTurn},
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
