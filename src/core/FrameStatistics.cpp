#include "core/FrameStatistics.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace {
// Nearest-rank percentile of sorted values: the smallest value that at least `fraction` of all
// values are less than or equal to. It always returns a frame time that really happened.
double Percentile(const std::vector<double>& sorted, double fraction) {
    const auto rank = static_cast<size_t>(std::ceil(fraction * static_cast<double>(sorted.size())));
    return sorted[std::clamp<size_t>(rank, 1, sorted.size()) - 1];
}
}  // namespace

double Average(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

FrameTimeSummary SummarizeFrameTimes(std::vector<double> frameMs, double budgetMs) {
    FrameTimeSummary summary;
    if (frameMs.empty()) {
        return summary;
    }
    std::sort(frameMs.begin(), frameMs.end());
    const auto overBudget = std::count_if(frameMs.begin(), frameMs.end(), [budgetMs](double ms) { return ms > budgetMs; });

    summary.frameCount = frameMs.size();
    summary.averageMs = Average(frameMs);
    summary.medianMs = Percentile(frameMs, 0.5);
    summary.p99Ms = Percentile(frameMs, 0.99);
    summary.p999Ms = Percentile(frameMs, 0.999);
    summary.worstMs = frameMs.back();
    summary.shareOverBudget = static_cast<double>(overBudget) / static_cast<double>(frameMs.size());
    return summary;
}
