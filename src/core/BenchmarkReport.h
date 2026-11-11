#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "render/IRenderModule.h"

// What the frame loop measured for one benchmark frame.
struct BenchmarkSample {
    double cpuFrameMs = 0.0;  // wall-clock time from the start of the previous frame to this one
    double crowdMs = 0.0;     // crowd simulation and culling
    size_t peopleOnScreen = 0;
    size_t shotIndex = 0;     // which shot of the camera path the frame belongs to
};

// Collects benchmark frames and prints the final report.
class BenchmarkRecorder {
public:
    explicit BenchmarkRecorder(std::vector<std::string> shotNames);

    void Record(const BenchmarkSample& sample, const RenderStats& stats);

    // Prints the report. Returns false only when a target frame rate is given and the 99th
    // percentile frame time misses it.
    bool PrintReport(size_t totalPeople, std::optional<double> targetFps) const;

private:
    void PrintGpuTimes() const;
    void PrintShots() const;

    std::vector<std::string> shotNames;
    std::vector<double> cpuFrameMs;
    std::vector<double> crowdMs;
    std::vector<double> gpuFrameMs;
    std::vector<double> renderScales;
    std::vector<std::string> passNames;
    std::vector<double> passMsTotals;
    std::vector<std::vector<double>> cpuMsByShot;
    std::vector<std::vector<double>> gpuMsByShot;
    double peopleOnScreenTotal = 0.0;
};
