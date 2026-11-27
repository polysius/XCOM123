#include "world/stream/WorldSkeleton.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "world/CityBuild.h"
#include "world/CityPlan.h"

namespace world {
namespace {

// Salts keep each decision on its own random stream, so changing one does not reshuffle the others.
constexpr uint32_t kLineSalt = 0x4C494E45u;
constexpr uint32_t kSiteSalt = 0x53495445u;
constexpr uint32_t kDistrictSalt = 0x44495354u;
constexpr uint32_t kRiverPhaseSalt = 0x52495650u;
constexpr uint32_t kRiverWaveSalt = 0x52495657u;
constexpr uint32_t kSplitSalt = 0x53504C54u;
constexpr uint32_t kLandmarkSalt = 0x4C4E444Du;

// Districts are the cells of a Voronoi diagram whose sites are one per square of this size, jittered.
constexpr float kDistrictLattice = 768.0f;

constexpr float kTwoPi = 6.2831853f;
constexpr float kMaxRiverAmplitude = 14.0f;
// Room kept between the water, its banks and the arterials on each side of the river's row.
constexpr float kRiverSideMargin = 4.0f;

// The chance that a cell that could hold a landmark does.
constexpr float kLandmarkChance = 0.10f;
// Landmarks need a block of at least this size (the ones the fixed city gives its smallest landmarks).
constexpr float kMinLandmarkCell = 100.0f;

// Weights in the order of the District enum: old quarter, grand, market, residential, parkland.
constexpr float kDistrictWeights[] = {0.22f, 0.20f, 0.20f, 0.28f, 0.10f};

float CellWidth(const Rect& area) {
    return area.max.x - area.min.x;
}

float CellDepth(const Rect& area) {
    return area.max.y - area.min.y;
}

District PickDistrict(float roll) {
    float total = 0.0f;
    for (size_t i = 0; i < std::size(kDistrictWeights); ++i) {
        total += kDistrictWeights[i];
        if (roll < total) {
            return static_cast<District>(i);
        }
    }
    return District::Residential;
}

glm::vec2 DistrictSite(uint32_t seed, int latticeX, int latticeZ) {
    const float jitterX = Hash01(seed, kSiteSalt, latticeX, latticeZ);
    const float jitterZ = Hash01(seed, kSiteSalt + 1u, latticeX, latticeZ);
    return {(static_cast<float>(latticeX) + 0.15f + 0.7f * jitterX) * kDistrictLattice,
            (static_cast<float>(latticeZ) + 0.15f + 0.7f * jitterZ) * kDistrictLattice};
}

// Splits [low, high] into `count` parts with the inner cuts nudged by the hash, and returns the cuts.
std::vector<float> InnerCuts(uint32_t seed, uint32_t salt, int column, int row, float low, float high, int count) {
    std::vector<float> cuts;
    for (int k = 1; k < count; ++k) {
        const float nudge = (Hash01(seed, salt + static_cast<uint32_t>(k), column, row) - 0.5f) * 0.3f;
        cuts.push_back(low + (high - low) * (static_cast<float>(k) + nudge) / static_cast<float>(count));
    }
    return cuts;
}

// How many blocks fit along a cell side, given the district's preferred block size.
int BlocksAlong(float length, const city::DistrictStyle& style, float roll) {
    const float preferred = style.minBlock + (style.maxBlock - style.minBlock) * roll;
    return std::max(1, static_cast<int>(std::lround(length / preferred)));
}

// The rectangles of the grid made by the cuts inside `area`.
std::vector<Rect> BlocksBetween(const Rect& area, const std::vector<float>& xSplits, const std::vector<float>& zSplits) {
    std::vector<float> xs{area.min.x};
    xs.insert(xs.end(), xSplits.begin(), xSplits.end());
    xs.push_back(area.max.x);
    std::vector<float> zs{area.min.y};
    zs.insert(zs.end(), zSplits.begin(), zSplits.end());
    zs.push_back(area.max.y);

    std::vector<Rect> blocks;
    for (size_t z = 0; z + 1 < zs.size(); ++z) {
        for (size_t x = 0; x + 1 < xs.size(); ++x) {
            blocks.push_back({{xs[x], zs[z]}, {xs[x + 1], zs[z + 1]}});
        }
    }
    return blocks;
}

std::optional<LandmarkKind> ChooseLandmark(uint32_t seed, const CellPlan& plan) {
    const bool wholeCell = plan.blocks.size() == 1;
    const bool bigEnough = std::min(CellWidth(plan.area), CellDepth(plan.area)) >= kMinLandmarkCell;
    if (plan.river || !wholeCell || !bigEnough || plan.district == District::Parkland) {
        return std::nullopt;
    }
    if (Hash01(seed, kLandmarkSalt, plan.column, plan.row) >= kLandmarkChance) {
        return std::nullopt;
    }
    return Hash01(seed, kLandmarkSalt + 1u, plan.column, plan.row) < 0.5f ? LandmarkKind::ClockTower
                                                                          : LandmarkKind::ParishChurch;
}

}  // namespace

int FloorDiv(int numerator, int denominator) {
    const int quotient = numerator / denominator;
    return (numerator % denominator != 0 && (numerator < 0) != (denominator < 0)) ? quotient - 1 : quotient;
}

uint32_t HashBits(uint32_t seed, uint32_t salt, int a, int b) {
    return city::MixSeed(city::MixSeed(city::MixSeed(seed, salt), static_cast<uint32_t>(a)), static_cast<uint32_t>(b));
}

float Hash01(uint32_t seed, uint32_t salt, int a, int b) {
    return static_cast<float>(HashBits(seed, salt, a, b) >> 8) * (1.0f / 16777216.0f);
}

float GridLine(uint32_t seed, Axis axis, int index) {
    const float jitter = (Hash01(seed, kLineSalt + static_cast<uint32_t>(axis), index, 0) - 0.5f) * 2.0f * kLineJitter;
    return static_cast<float>(index) * kCellPitch + jitter;
}

Rect CellArea(uint32_t seed, int column, int row) {
    return {{GridLine(seed, Axis::X, column), GridLine(seed, Axis::Z, row)},
            {GridLine(seed, Axis::X, column + 1), GridLine(seed, Axis::Z, row + 1)}};
}

District DistrictAt(uint32_t seed, glm::vec2 point) {
    const int latticeX = static_cast<int>(std::floor(point.x / kDistrictLattice));
    const int latticeZ = static_cast<int>(std::floor(point.y / kDistrictLattice));
    float nearest = INFINITY;
    int nearestX = latticeX;
    int nearestZ = latticeZ;
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dx = -1; dx <= 1; ++dx) {
            const float distance = glm::distance(point, DistrictSite(seed, latticeX + dx, latticeZ + dz));
            if (distance < nearest) {
                nearest = distance;
                nearestX = latticeX + dx;
                nearestZ = latticeZ + dz;
            }
        }
    }
    return PickDistrict(Hash01(seed, kDistrictSalt, nearestX, nearestZ));
}

bool IsRiverRow(uint32_t seed, int row) {
    const int phase = static_cast<int>(HashBits(seed, kRiverPhaseSalt, 0, 0) % kRiverPeriodRows);
    const int offset = row - phase;
    return ((offset % kRiverPeriodRows) + kRiverPeriodRows) % kRiverPeriodRows == 0;
}

float RiverCentreZ(uint32_t seed, int row, float x) {
    const float low = GridLine(seed, Axis::Z, row);
    const float high = GridLine(seed, Axis::Z, row + 1);
    const float room = (high - low) * 0.5f - kStreetHalfWidth - kRiverHalfWidth - kBankThickness - kRiverSideMargin;
    const float amplitude = std::clamp(room, 0.0f, kMaxRiverAmplitude);
    const float phaseLong = Hash01(seed, kRiverWaveSalt, row, 0) * kTwoPi;
    const float phaseShort = Hash01(seed, kRiverWaveSalt, row, 1) * kTwoPi;
    const float wave = 0.65f * std::sin(x * (kTwoPi / 420.0f) + phaseLong) + 0.35f * std::sin(x * (kTwoPi / 170.0f) + phaseShort);
    return (low + high) * 0.5f + amplitude * wave;
}

CellPlan PlanCell(uint32_t seed, int column, int row) {
    CellPlan plan;
    plan.column = column;
    plan.row = row;
    plan.area = CellArea(seed, column, row);
    plan.district = DistrictAt(seed, plan.area.Center());
    plan.river = IsRiverRow(seed, row);
    if (plan.river) {
        return plan;  // the river fills the cell, so there are no local streets or blocks
    }

    const city::DistrictStyle& style = city::StyleOf(plan.district);
    const int columns = BlocksAlong(CellWidth(plan.area), style, Hash01(seed, kSplitSalt, column, row));
    const int rows = BlocksAlong(CellDepth(plan.area), style, Hash01(seed, kSplitSalt + 100u, column, row));
    plan.xSplits = InnerCuts(seed, kSplitSalt + 200u, column, row, plan.area.min.x, plan.area.max.x, columns);
    plan.zSplits = InnerCuts(seed, kSplitSalt + 300u, column, row, plan.area.min.y, plan.area.max.y, rows);
    plan.blocks = BlocksBetween(plan.area, plan.xSplits, plan.zSplits);
    plan.landmark = ChooseLandmark(seed, plan);
    return plan;
}

}  // namespace world
