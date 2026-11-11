#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <glm/glm.hpp>

// A camera placement given on the command line, in degrees as a person would type it.
struct CameraPlacement {
    glm::vec3 position{0.0f};
    float yawDegrees = 0.0f;    // 0 looks along +Z, 90 along +X
    float pitchDegrees = 0.0f;  // positive looks up
};

// Everything the command line can set. Unset optional values mean "use the mode's default".
struct Options {
    uint32_t seed = 1789;
    uint32_t npcCount = 10000;
    bool vsync = true;
    bool showHelp = false;

    // Benchmark: fly the scripted camera path for this many frames, report, then quit.
    int benchmarkFrames = 0;
    std::optional<double> targetFps;  // the benchmark fails when p99 frame time misses this rate

    // Screenshot: render this many frames, save the last one as a PNG, then quit.
    std::string screenshotPath;
    int screenshotFrames = 60;

    std::optional<CameraPlacement> camera;
    std::optional<float> worldTimeSeconds;
    std::optional<float> sunElevationDegrees;

    // Resolution: a fixed render scale turns the dynamic controller off.
    std::optional<float> renderScale;
    bool dynamicResolution = true;
    float gpuBudgetMs = 14.0f;

    bool IsBenchmark() const { return benchmarkFrames > 0; }
    bool IsScreenshot() const { return !screenshotPath.empty(); }
};

// Throws std::invalid_argument (or std::out_of_range) on input it cannot use.
Options ParseCommandLine(int argc, char** argv);
void PrintUsage();
