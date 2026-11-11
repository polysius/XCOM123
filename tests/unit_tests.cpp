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
        {"BenchmarkAndScreenshotTurnVsyncOff", BenchmarkAndScreenshotTurnVsyncOff},
        {"FrameSummaryOfNothingIsEmpty", FrameSummaryOfNothingIsEmpty},
        {"FrameSummaryUsesNearestRank", FrameSummaryUsesNearestRank},
        {"FrameSummaryIgnoresInputOrder", FrameSummaryIgnoresInputOrder},
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
