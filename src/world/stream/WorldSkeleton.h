#pragma once

// The skeleton of the infinite streamed world: the street grid, the districts and the rivers, all as pure functions of a
// seed and a position. Nothing here is stored or generated in order, so any chunk can be built alone and neighbouring
// chunks agree on every street and every river bank without ever talking to each other.
//
// The world is a lattice of arterial streets. The line with index i running along Z (a constant X) lies at
// i * kCellPitch plus a hashed jitter, and the same holds for lines along X. The rectangle between two neighbouring lines
// of each axis is a cell. A cell is split into blocks by local streets that stay inside it and end in T junctions on the
// arterials, so no street ever has to continue into a cell that another chunk builds.

#include <cstdint>
#include <optional>
#include <vector>

#include <glm/glm.hpp>

#include "world/CityGenerator.h"

namespace world {

// A chunk is two cells wide in each direction. 256 m is a trade between three costs. Smaller chunks (128 m) would put
// 4000+ chunk draws and queue entries in a 700 m load radius and make every chunk too small to hold a landmark; larger
// ones (512 m) take 4 times the generation time per chunk, so one late chunk costs a visible hole, and make a unit of
// upload and culling coarse. At 256 m a chunk is a few thousand boxes, which generates in well under a frame budget on
// a worker, and about 40 of them fill the view.
inline constexpr float kChunkSize = 256.0f;
inline constexpr int kCellsPerChunk = 2;
inline constexpr float kCellPitch = kChunkSize / static_cast<float>(kCellsPerChunk);
// How far an arterial may move from its lattice position. Cells stay between 104 and 152 m wide.
inline constexpr float kLineJitter = 12.0f;

// The same street widths as the fixed city, so the crowd and the materials behave the same.
inline constexpr float kRoadHalfWidth = 4.0f;
inline constexpr float kSidewalkWidth = 3.5f;
inline constexpr float kStreetHalfWidth = kRoadHalfWidth + kSidewalkWidth;

// A river runs along X through one row of cells in every kRiverPeriodRows rows, between the two streets that bound the row.
inline constexpr int kRiverPeriodRows = 12;
inline constexpr float kRiverHalfWidth = 20.0f;
inline constexpr float kBankThickness = 5.0f;

// Which family of arterials: lines with a constant X (running along Z), or a constant Z (running along X).
enum class Axis : uint8_t { X, Z };

// Division rounding towards minus infinity, for chunk and cell indices that go negative.
int FloorDiv(int numerator, int denominator);

// A number in [0, 1) that depends only on the seed, a salt and two integers.
float Hash01(uint32_t seed, uint32_t salt, int a, int b);
uint32_t HashBits(uint32_t seed, uint32_t salt, int a, int b);

// The position of arterial `index` of one axis.
float GridLine(uint32_t seed, Axis axis, int index);
// The rectangle between the centre lines of the arterials around a cell.
Rect CellArea(uint32_t seed, int column, int row);

District DistrictAt(uint32_t seed, glm::vec2 point);

bool IsRiverRow(uint32_t seed, int row);
// Z of the centre line of the river of `row` at `x`. The river meanders inside the row's band.
float RiverCentreZ(uint32_t seed, int row, float x);

// What a cell is made of, decided from its index alone.
struct CellPlan {
    int column = 0;
    int row = 0;
    Rect area;                    // between the centre lines of the arterials around it
    District district = District::Residential;
    bool river = false;           // the river of the row runs through the cell; it holds no blocks
    std::vector<float> xSplits;   // local streets inside the cell, as X of lines running along Z, ascending
    std::vector<float> zSplits;   // and as Z of lines running along X
    std::vector<Rect> blocks;     // between the centre lines of the streets around each block
    std::optional<LandmarkKind> landmark;  // a landmark stands on blocks[0], the whole cell
};

CellPlan PlanCell(uint32_t seed, int column, int row);

}  // namespace world
