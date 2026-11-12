#include "fluid/ShallowWater.h"

#include <algorithm>
#include <cmath>

namespace fluid {
namespace {

// More small steps than this in one call would mean something is wrong: the water is deeper or faster
// than any river, or the frame is far too long. The call then simulates less time than asked for.
constexpr int kMaxSmallSteps = 64;
// Time that a call could not run is kept for the next calls, but no more than this much: a game that is
// seconds behind is better off skipping ahead than running the water at full speed for a long time.
constexpr float kMaxPendingSeconds = 2.0f;
// A source reaches at least this many cells, so that a point anywhere in a cell is within reach of the
// centre of one: the furthest a point can be from the nearest centre is 0.71 of a cell.
constexpr float kMinReachInCells = 0.75f;

}  // namespace

void ShallowWater::Initialise(const WaterSettings& waterSettings) {
    settings = waterSettings;
    settings.width = std::max(settings.width, 2);
    settings.height = std::max(settings.height, 2);
    const size_t cells = static_cast<size_t>(settings.width) * static_cast<size_t>(settings.height);
    bed.assign(cells, 0.0f);
    depth.assign(cells, 0.0f);
    solid.assign(cells, 0);
    outflowScale.assign(cells, 1.0f);
    flowU.assign(static_cast<size_t>(settings.width + 1) * static_cast<size_t>(settings.height), 0.0f);
    flowV.assign(static_cast<size_t>(settings.width) * static_cast<size_t>(settings.height + 1), 0.0f);
    scratchU = flowU;
    scratchV = flowV;
}

void ShallowWater::SetBed(int i, int j, float bedHeight) {
    bed[Index(i, j)] = bedHeight;
}

void ShallowWater::SetSolid(int i, int j, bool isSolid) {
    solid[Index(i, j)] = isSolid ? 1 : 0;
    if (isSolid) {
        depth[Index(i, j)] = 0.0f;
        flowU[SideU(i, j)] = 0.0f;
        flowU[SideU(i + 1, j)] = 0.0f;
        flowV[SideV(i, j)] = 0.0f;
        flowV[SideV(i, j + 1)] = 0.0f;
    }
}

void ShallowWater::SetDepth(int i, int j, float waterDepth) {
    if (solid[Index(i, j)] == 0) {
        depth[Index(i, j)] = std::max(waterDepth, 0.0f);
    }
}

void ShallowWater::FillToLevel(float level) {
    for (size_t cell = 0; cell < depth.size(); ++cell) {
        depth[cell] = solid[cell] == 0 ? std::max(level - bed[cell], 0.0f) : 0.0f;
    }
}

float ShallowWater::AddVolume(glm::vec2 point, float radius, float volume) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(radius) || !std::isfinite(volume)) {
        return 0.0f;
    }
    const float cellArea = settings.cellSize * settings.cellSize;
    const glm::vec2 inGrid = (point - settings.origin) / settings.cellSize;
    const float reach = std::max(radius / settings.cellSize, kMinReachInCells);
    const int i0 = std::max(static_cast<int>(std::floor(inGrid.x - reach)), 0);
    const int i1 = std::min(static_cast<int>(std::ceil(inGrid.x + reach)), settings.width - 1);
    const int j0 = std::max(static_cast<int>(std::floor(inGrid.y - reach)), 0);
    const int j1 = std::min(static_cast<int>(std::ceil(inGrid.y + reach)), settings.height - 1);

    // Each cell takes a share of the volume that falls off smoothly from the centre.
    struct Share {
        size_t cell;
        float weight;
    };
    std::vector<Share> shares;
    float totalWeight = 0.0f;
    for (int j = j0; j <= j1; ++j) {
        for (int i = i0; i <= i1; ++i) {
            if (solid[Index(i, j)] != 0) {
                continue;
            }
            const float distance = glm::length(glm::vec2(static_cast<float>(i) + 0.5f, static_cast<float>(j) + 0.5f) - inGrid) / reach;
            if (distance < 1.0f) {
                const float weight = 1.0f - distance * distance;
                shares.push_back({Index(i, j), weight});
                totalWeight += weight;
            }
        }
    }
    if (shares.empty() || totalWeight <= 0.0f) {
        // Nothing within reach but a wall, or a point at the very edge of the grid: the nearest cell that is
        // not a wall takes it all, if it is inside the grid.
        if (inGrid.x < 0.0f || inGrid.y < 0.0f || inGrid.x >= static_cast<float>(settings.width) ||
            inGrid.y >= static_cast<float>(settings.height)) {
            return 0.0f;
        }
        float nearest = 1e9f;
        size_t nearestCell = 0;
        for (int j = j0; j <= j1; ++j) {
            for (int i = i0; i <= i1; ++i) {
                const float distance = glm::length(glm::vec2(static_cast<float>(i) + 0.5f, static_cast<float>(j) + 0.5f) - inGrid);
                if (solid[Index(i, j)] == 0 && distance < nearest) {
                    nearest = distance;
                    nearestCell = Index(i, j);
                }
            }
        }
        if (nearest > 1e8f) {
            return 0.0f;
        }
        shares.push_back({nearestCell, 1.0f});
        totalWeight = 1.0f;
    }
    float done = 0.0f;
    for (const Share& share : shares) {
        const float wanted = volume * share.weight / totalWeight;
        if (wanted >= 0.0f) {
            depth[share.cell] += wanted / cellArea;
            done += wanted;
        } else {
            const float taken = std::min(-wanted, depth[share.cell] * cellArea);
            depth[share.cell] -= taken / cellArea;
            done -= taken;
        }
    }
    return done;
}

void ShallowWater::AddVelocity(glm::vec2 point, float radius, glm::vec2 velocity) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(radius) || !std::isfinite(velocity.x) ||
        !std::isfinite(velocity.y)) {
        return;
    }
    const glm::vec2 inGrid = (point - settings.origin) / settings.cellSize;
    const float reach = std::max(radius / settings.cellSize, kMinReachInCells);
    const auto weightAt = [&](float x, float z) {
        const float distance = glm::length(glm::vec2(x, z) - inGrid) / reach;
        return distance < 1.0f ? 1.0f - distance * distance : 0.0f;
    };
    // Only the sides within reach of the point are looked at.
    const int i0 = std::max(static_cast<int>(std::floor(inGrid.x - reach)), 0);
    const int i1 = std::min(static_cast<int>(std::ceil(inGrid.x + reach)), settings.width);
    const int j0 = std::max(static_cast<int>(std::floor(inGrid.y - reach)), 0);
    const int j1 = std::min(static_cast<int>(std::ceil(inGrid.y + reach)), settings.height);
    for (int j = j0; j < j1; ++j) {
        for (int i = std::max(i0, 1); i < std::min(i1 + 1, settings.width); ++i) {
            const float weight = weightAt(static_cast<float>(i), static_cast<float>(j) + 0.5f);
            if (weight > 0.0f && !solid[Index(i - 1, j)] && !solid[Index(i, j)] && (IsWet(i - 1, j) || IsWet(i, j))) {
                flowU[SideU(i, j)] += weight * (velocity.x - flowU[SideU(i, j)]);
            }
        }
    }
    for (int j = std::max(j0, 1); j < std::min(j1 + 1, settings.height); ++j) {
        for (int i = i0; i < i1; ++i) {
            const float weight = weightAt(static_cast<float>(i) + 0.5f, static_cast<float>(j));
            if (weight > 0.0f && !solid[Index(i, j - 1)] && !solid[Index(i, j)] && (IsWet(i, j - 1) || IsWet(i, j))) {
                flowV[SideV(i, j)] += weight * (velocity.y - flowV[SideV(i, j)]);
            }
        }
    }
}

double ShallowWater::TotalVolume() const {
    double total = 0.0;
    for (float d : depth) {
        total += d;
    }
    return total * static_cast<double>(settings.cellSize) * static_cast<double>(settings.cellSize);
}

float ShallowWater::FastestWave() const {
    float deepest = 0.0f;
    for (size_t cell = 0; cell < depth.size(); ++cell) {
        deepest = std::max(deepest, depth[cell]);
    }
    float fastestFlow = 0.0f;
    for (float u : flowU) {
        fastestFlow = std::max(fastestFlow, std::abs(u));
    }
    for (float v : flowV) {
        fastestFlow = std::max(fastestFlow, std::abs(v));
    }
    return std::sqrt(settings.gravity * deepest) + fastestFlow;
}

void ShallowWater::Step(float seconds) {
    if (!(seconds > 0.0f) || !std::isfinite(seconds)) {
        return;  // nothing, a negative time, or a number that is not one
    }
    pendingSeconds = std::min(pendingSeconds + seconds, kMaxPendingSeconds);
    const float longestStep = settings.courant * settings.cellSize / std::max(FastestWave(), 1e-3f);
    // Counted in floating point first: a very long time would overflow an int.
    const int steps = static_cast<int>(std::clamp(std::ceil(pendingSeconds / longestStep), 1.0f, static_cast<float>(kMaxSmallSteps)));
    const float step = std::min(pendingSeconds / static_cast<float>(steps), longestStep);
    for (int i = 0; i < steps; ++i) {
        SmallStep(step);
    }
    pendingSeconds = std::max(pendingSeconds - step * static_cast<float>(steps), 0.0f);
    if (pendingSeconds < 1e-6f) {
        pendingSeconds = 0.0f;
    }
}

void ShallowWater::SmallStep(float dt) {
    AccelerateByTheSurfaceSlope(dt);
    if (settings.advect) {
        Advect(dt);
    }
    MoveWater(dt);
}

// The water flows down the slope of its surface, and loses a little speed to the bed.
void ShallowWater::AccelerateByTheSurfaceSlope(float dt) {
    const float keep = std::exp(-settings.drag * dt);
    const float push = settings.gravity * dt / settings.cellSize;
    // The flow between two cells along one axis, given the cell on each side. Returns the new flow.
    const auto accelerate = [&](float flow, size_t first, size_t second) {
        if (solid[first] != 0 || solid[second] != 0) {
            return 0.0f;
        }
        const bool firstWet = depth[first] > settings.minDepth;
        const bool secondWet = depth[second] > settings.minDepth;
        if (!firstWet && !secondWet) {
            return 0.0f;
        }
        const float firstSurface = bed[first] + depth[first];
        const float secondSurface = bed[second] + depth[second];
        // Water does not flow uphill out of a dry cell: nothing there pushes it.
        if ((!firstWet && bed[first] >= secondSurface) || (!secondWet && bed[second] >= firstSurface)) {
            return 0.0f;
        }
        return (flow - push * (secondSurface - firstSurface)) * keep;
    };
    for (int j = 0; j < settings.height; ++j) {
        for (int i = 1; i < settings.width; ++i) {
            flowU[SideU(i, j)] = accelerate(flowU[SideU(i, j)], Index(i - 1, j), Index(i, j));
        }
    }
    for (int j = 1; j < settings.height; ++j) {
        for (int i = 0; i < settings.width; ++i) {
            flowV[SideV(i, j)] = accelerate(flowV[SideV(i, j)], Index(i, j - 1), Index(i, j));
        }
    }
}

// The flow along X at a point given in grid units, blended between the sides around it.
float ShallowWater::SampleU(float x, float z) const {
    if (!std::isfinite(x) || !std::isfinite(z)) {
        return 0.0f;
    }
    const float fx = std::clamp(x, 0.0f, static_cast<float>(settings.width));
    const float fz = std::clamp(z - 0.5f, 0.0f, static_cast<float>(settings.height - 1));
    const int i = std::min(static_cast<int>(fx), settings.width - 1);
    const int j = std::min(static_cast<int>(fz), settings.height - 2);
    const float tx = fx - static_cast<float>(i);
    const float tz = fz - static_cast<float>(j);
    const float a = flowU[SideU(i, j)] * (1.0f - tx) + flowU[SideU(i + 1, j)] * tx;
    const float b = flowU[SideU(i, j + 1)] * (1.0f - tx) + flowU[SideU(i + 1, j + 1)] * tx;
    return a * (1.0f - tz) + b * tz;
}

float ShallowWater::SampleV(float x, float z) const {
    if (!std::isfinite(x) || !std::isfinite(z)) {
        return 0.0f;
    }
    const float fx = std::clamp(x - 0.5f, 0.0f, static_cast<float>(settings.width - 1));
    const float fz = std::clamp(z, 0.0f, static_cast<float>(settings.height));
    const int i = std::min(static_cast<int>(fx), settings.width - 2);
    const int j = std::min(static_cast<int>(fz), settings.height - 1);
    const float tx = fx - static_cast<float>(i);
    const float tz = fz - static_cast<float>(j);
    const float a = flowV[SideV(i, j)] * (1.0f - tx) + flowV[SideV(i + 1, j)] * tx;
    const float b = flowV[SideV(i, j + 1)] * (1.0f - tx) + flowV[SideV(i + 1, j + 1)] * tx;
    return a * (1.0f - tz) + b * tz;
}

// The flow carries its own speed along: the speed at a side now is the speed that was at the place the water
// at that side came from, a step ago.
void ShallowWater::Advect(float dt) {
    const float cellsPerMetre = 1.0f / settings.cellSize;
    for (int j = 0; j < settings.height; ++j) {
        for (int i = 1; i < settings.width; ++i) {
            const size_t side = SideU(i, j);
            if (flowU[side] == 0.0f) {
                scratchU[side] = 0.0f;
                continue;
            }
            const float x = static_cast<float>(i);
            const float z = static_cast<float>(j) + 0.5f;
            const float u = flowU[side];
            const float v = SampleV(x, z);
            scratchU[side] = SampleU(x - u * dt * cellsPerMetre, z - v * dt * cellsPerMetre);
        }
    }
    for (int j = 1; j < settings.height; ++j) {
        for (int i = 0; i < settings.width; ++i) {
            const size_t side = SideV(i, j);
            if (flowV[side] == 0.0f) {
                scratchV[side] = 0.0f;
                continue;
            }
            const float x = static_cast<float>(i) + 0.5f;
            const float z = static_cast<float>(j);
            const float u = SampleU(x, z);
            const float v = flowV[side];
            scratchV[side] = SampleV(x - u * dt * cellsPerMetre, z - v * dt * cellsPerMetre);
        }
    }
    // The new speeds replace the old ones. A side that was still was skipped above, so advection never
    // starts a flow where the slope did not.
    for (int j = 0; j < settings.height; ++j) {
        for (int i = 1; i < settings.width; ++i) {
            flowU[SideU(i, j)] = scratchU[SideU(i, j)];
        }
    }
    for (int j = 1; j < settings.height; ++j) {
        for (int i = 0; i < settings.width; ++i) {
            flowV[SideV(i, j)] = scratchV[SideV(i, j)];
        }
    }
}

// Moves the water from cell to cell through the sides. The depth that flows is that of the water on the
// upstream side, above the higher of the two beds, and no cell gives away more water than it holds, so the
// depth never goes below zero and the volume is exactly what it was. What flows through every side is worked
// out first, from the depths as they are at the start, and only then is any depth changed: otherwise the
// result would depend on the order the sides are visited in, and the water would lean towards one direction.
void ShallowWater::MoveWater(float dt) {
    const float perCell = dt / settings.cellSize;  // flow speed to depth moved
    // The depth that moves through a side, positive from `first` to `second`.
    const auto moved = [&](float flow, size_t first, size_t second) {
        if (flow == 0.0f) {
            return 0.0f;
        }
        const size_t upstream = flow > 0.0f ? first : second;
        const float level = bed[upstream] + depth[upstream];
        const float faceDepth = std::max(level - std::max(bed[first], bed[second]), 0.0f);
        return faceDepth * flow * perCell;
    };

    // What flows through every side, and what each cell would give away in all.
    std::fill(outflowScale.begin(), outflowScale.end(), 0.0f);
    for (int j = 0; j < settings.height; ++j) {
        for (int i = 1; i < settings.width; ++i) {
            const size_t first = Index(i - 1, j);
            const size_t second = Index(i, j);
            const float amount = moved(flowU[SideU(i, j)], first, second);
            scratchU[SideU(i, j)] = amount;
            outflowScale[amount > 0.0f ? first : second] += std::abs(amount);
        }
    }
    for (int j = 1; j < settings.height; ++j) {
        for (int i = 0; i < settings.width; ++i) {
            const size_t first = Index(i, j - 1);
            const size_t second = Index(i, j);
            const float amount = moved(flowV[SideV(i, j)], first, second);
            scratchV[SideV(i, j)] = amount;
            outflowScale[amount > 0.0f ? first : second] += std::abs(amount);
        }
    }
    // How much of that each cell can really give.
    for (size_t cell = 0; cell < outflowScale.size(); ++cell) {
        const float giveAway = outflowScale[cell];
        outflowScale[cell] = giveAway > depth[cell] ? depth[cell] / giveAway : 1.0f;
    }

    // Then the water moves, every side by what was worked out for it.
    for (int j = 0; j < settings.height; ++j) {
        for (int i = 1; i < settings.width; ++i) {
            const size_t first = Index(i - 1, j);
            const size_t second = Index(i, j);
            float amount = scratchU[SideU(i, j)];
            amount *= outflowScale[amount > 0.0f ? first : second];
            depth[first] -= amount;
            depth[second] += amount;
        }
    }
    for (int j = 1; j < settings.height; ++j) {
        for (int i = 0; i < settings.width; ++i) {
            const size_t first = Index(i, j - 1);
            const size_t second = Index(i, j);
            float amount = scratchV[SideV(i, j)];
            amount *= outflowScale[amount > 0.0f ? first : second];
            depth[first] -= amount;
            depth[second] += amount;
        }
    }
    for (float& d : depth) {
        d = std::max(d, 0.0f);  // only rounding can make it negative
    }
}

glm::vec2 ShallowWater::VelocityAt(glm::vec2 point) const {
    const glm::vec2 inGrid = (point - settings.origin) / settings.cellSize;
    return {SampleU(inGrid.x, inGrid.y), SampleV(inGrid.x, inGrid.y)};
}

float ShallowWater::SurfaceHeightAt(glm::vec2 point, float fallback) const {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return fallback;
    }
    const glm::vec2 inGrid = (point - settings.origin) / settings.cellSize - 0.5f;
    if (inGrid.x < -0.5f || inGrid.y < -0.5f || inGrid.x > static_cast<float>(settings.width) - 0.5f ||
        inGrid.y > static_cast<float>(settings.height) - 0.5f) {
        return fallback;
    }
    const int i0 = std::clamp(static_cast<int>(std::floor(inGrid.x)), 0, settings.width - 2);
    const int j0 = std::clamp(static_cast<int>(std::floor(inGrid.y)), 0, settings.height - 2);
    const float tx = std::clamp(inGrid.x - static_cast<float>(i0), 0.0f, 1.0f);
    const float tz = std::clamp(inGrid.y - static_cast<float>(j0), 0.0f, 1.0f);
    float sum = 0.0f;
    float weights = 0.0f;
    for (int dj = 0; dj < 2; ++dj) {
        for (int di = 0; di < 2; ++di) {
            if (!IsWet(i0 + di, j0 + dj)) {
                continue;
            }
            const float weight = (di == 0 ? 1.0f - tx : tx) * (dj == 0 ? 1.0f - tz : tz);
            sum += weight * SurfaceHeight(i0 + di, j0 + dj);
            weights += weight;
        }
    }
    return weights > 1e-4f ? sum / weights : fallback;
}

bool ShallowWater::IsWetAt(glm::vec2 point) const {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return false;
    }
    const glm::vec2 inGrid = (point - settings.origin) / settings.cellSize;
    const int i = static_cast<int>(std::floor(inGrid.x));
    const int j = static_cast<int>(std::floor(inGrid.y));
    return i >= 0 && j >= 0 && i < settings.width && j < settings.height && IsWet(i, j);
}

}  // namespace fluid
