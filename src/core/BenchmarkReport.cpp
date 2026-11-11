#include "core/BenchmarkReport.h"

#include <algorithm>
#include <cstdio>
#include <utility>

#include "core/FrameStatistics.h"

namespace {
// The frame time that holds 60 frames per second.
constexpr double k60FpsFrameMs = 1000.0 / 60.0;
}  // namespace

BenchmarkRecorder::BenchmarkRecorder(std::vector<std::string> names)
    : shotNames(std::move(names)), cpuMsByShot(shotNames.size()), gpuMsByShot(shotNames.size()) {}

void BenchmarkRecorder::Record(const BenchmarkSample& sample, const RenderStats& stats) {
    cpuFrameMs.push_back(sample.cpuFrameMs);
    crowdMs.push_back(sample.crowdMs);
    renderScales.push_back(stats.renderScale);
    peopleOnScreenTotal += static_cast<double>(sample.peopleOnScreen);
    const size_t shot = std::min(sample.shotIndex, shotNames.size() - 1);
    cpuMsByShot[shot].push_back(sample.cpuFrameMs);

    // GPU times arrive a frame or two late; the first frames have none yet.
    if (!stats.gpuTimingAvailable || stats.gpuMeasuredFrames == 0) {
        return;
    }
    gpuFrameMs.push_back(stats.gpuFrameMs);
    gpuMsByShot[shot].push_back(stats.gpuFrameMs);
    if (passNames.empty()) {
        for (const GpuPassTiming& pass : stats.passes) {
            passNames.emplace_back(pass.name);
        }
        passMsTotals.assign(passNames.size(), 0.0);
    }
    for (size_t index = 0; index < stats.passes.size() && index < passMsTotals.size(); ++index) {
        passMsTotals[index] += stats.passes[index].latestMs;
    }
}

bool BenchmarkRecorder::PrintReport(size_t totalPeople, std::optional<double> targetFps) const {
    if (cpuFrameMs.empty()) {
        std::printf("Benchmark: no frames were measured\n");
        return !targetFps.has_value();
    }
    const FrameTimeSummary cpu = SummarizeFrameTimes(cpuFrameMs, k60FpsFrameMs);
    const double averageOnScreen = peopleOnScreenTotal / static_cast<double>(cpuFrameMs.size());

    std::printf("Benchmark: %zu frames, %zu people (%.0f on screen on average)\n", cpu.frameCount, totalPeople,
                averageOnScreen);
    std::printf("  CPU frame time   average %.2f ms (%.0f fps), median %.2f, p99 %.2f, p99.9 %.2f, worst %.2f ms\n",
                cpu.averageMs, 1000.0 / cpu.averageMs, cpu.medianMs, cpu.p99Ms, cpu.p999Ms, cpu.worstMs);
    std::printf("  over 16.67 ms    %.2f%% of frames\n", cpu.shareOverBudget * 100.0);
    PrintGpuTimes();
    std::printf("  crowd            %.3f ms average for simulation and culling\n", Average(crowdMs));
    // Many changes would mean the dynamic resolution controller is oscillating.
    size_t scaleChanges = 0;
    for (size_t index = 1; index < renderScales.size(); ++index) {
        scaleChanges += renderScales[index] != renderScales[index - 1] ? 1 : 0;
    }
    std::printf("  render scale     %.2f average, %.2f lowest, changed %zu times\n", Average(renderScales),
                *std::min_element(renderScales.begin(), renderScales.end()), scaleChanges);
    PrintShots();

    if (!targetFps) {
        return true;
    }
    const double targetMs = 1000.0 / *targetFps;
    const bool passed = cpu.p99Ms <= targetMs;
    std::printf("  target %.0f fps   p99 %.2f ms against %.2f ms: %s\n", *targetFps, cpu.p99Ms, targetMs,
                passed ? "PASS" : "FAIL");
    return passed;
}

void BenchmarkRecorder::PrintGpuTimes() const {
    if (gpuFrameMs.empty()) {
        std::printf("  GPU frame time   not available on this GPU\n");
        return;
    }
    const FrameTimeSummary gpu = SummarizeFrameTimes(gpuFrameMs, k60FpsFrameMs);
    std::printf("  GPU frame time   average %.2f ms, median %.2f, p99 %.2f, worst %.2f ms\n", gpu.averageMs,
                gpu.medianMs, gpu.p99Ms, gpu.worstMs);
    std::printf("  GPU passes      ");
    const double samples = static_cast<double>(gpuFrameMs.size());
    for (size_t index = 0; index < passNames.size(); ++index) {
        std::printf(" %s %.2f ms%s", passNames[index].c_str(), passMsTotals[index] / samples,
                    index + 1 < passNames.size() ? "," : "\n");
    }
}

void BenchmarkRecorder::PrintShots() const {
    std::printf("  per shot         CPU average / p99, GPU average\n");
    for (size_t shot = 0; shot < shotNames.size(); ++shot) {
        if (cpuMsByShot[shot].empty()) {
            continue;
        }
        const FrameTimeSummary cpu = SummarizeFrameTimes(cpuMsByShot[shot], k60FpsFrameMs);
        std::printf("    %-16s %6.2f / %6.2f ms, GPU %6.2f ms\n", shotNames[shot].c_str(), cpu.averageMs, cpu.p99Ms,
                    Average(gpuMsByShot[shot]));
    }
}
