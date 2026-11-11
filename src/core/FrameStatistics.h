#pragma once

#include <cstddef>
#include <vector>

// Summary of a list of frame times, in milliseconds.
struct FrameTimeSummary {
    size_t frameCount = 0;
    double averageMs = 0.0;
    double medianMs = 0.0;
    double p99Ms = 0.0;   // 99% of frames were at least this fast
    double p999Ms = 0.0;  // 99.9% of frames were at least this fast
    double worstMs = 0.0;
    double shareOverBudget = 0.0;  // fraction of frames slower than the budget, 0 to 1
};

FrameTimeSummary SummarizeFrameTimes(std::vector<double> frameMs, double budgetMs);

double Average(const std::vector<double>& values);
