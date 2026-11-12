#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

// A simulation of water that is shallow compared with the waves on it: rivers, canals, ponds and the
// splashes of things dropped in them.
//
// The water lives on a grid of square cells seen from above. Each cell holds how deep the water is
// over the bed, and each side between two cells holds how fast the water flows through it (the
// staggered, or MAC, grid). A step does what the shallow water equations say:
//   1. the slope of the water surface accelerates the water down the slope, so a bump spreads out as
//      waves that travel at sqrt(gravity * depth);
//   2. the water carries its own speed along (advection), so currents bend round obstacles;
//   3. the water that flows through the sides moves from cell to cell, so that the total volume is
//      kept exactly and no cell gives away more water than it has.
// Steps are cut into smaller ones as the speed of the waves needs, so that any frame time is stable.
//
// Walls are solid cells that no water enters, a bed can slope, and cells with no water are dry, which
// water floods over when it reaches them. The same calls always give the same result.

namespace fluid {

struct WaterSettings {
    int width = 128;               // cells along X
    int height = 128;              // cells along Z
    float cellSize = 0.5f;         // metres
    glm::vec2 origin{0.0f};        // world X and Z of the corner of cell (0, 0)
    float gravity = 9.81f;
    float drag = 0.02f;            // share of the flow speed lost per second to friction with the bed
    float minDepth = 1e-3f;        // water shallower than this counts as a dry cell
    float courant = 0.45f;         // how far, in cells, the fastest wave may travel in one small step
    bool advect = true;            // whether the flow carries its own speed along
};

class ShallowWater {
public:
    void Initialise(const WaterSettings& waterSettings);

    // The bed and the walls --------------------------------------------------------------------

    // Raises or lowers the bed of a cell. The depth of water there is kept.
    void SetBed(int i, int j, float bedHeight);
    // Makes a cell a wall, which no water enters, or an ordinary cell again. A cell that becomes a wall loses
    // its water.
    void SetSolid(int i, int j, bool solid);
    // Puts water of the given depth in a cell, or none.
    void SetDepth(int i, int j, float waterDepth);
    // Fills every cell that is not a wall with water up to the surface height `level`; cells whose bed is
    // higher stay dry.
    void FillToLevel(float level);

    // Sources ----------------------------------------------------------------------------------

    // Adds `volume` cubic metres of water spread over the cells within `radius` of a world point, more at
    // its centre. The same volume goes in whatever the cell size, so a splash is as big as the thing that
    // made it. Negative volume takes water out. Returns the volume actually added, which is less when
    // the cells have no water to give.
    float AddVolume(glm::vec2 point, float radius, float volume);
    // Pushes the water within `radius` of a world point, more at its centre, so that it flows with `velocity`
    // (metres per second along X and Z), as the hull of a boat or a leg in the river does. A point or a
    // velocity that is not a finite number is ignored.
    void AddVelocity(glm::vec2 point, float radius, glm::vec2 velocity);

    // Time -------------------------------------------------------------------------------------

    // Advances the water by `seconds`, in as many small steps as its waves need. A call never runs more than
    // 64 small steps; the time it could not fit is kept and run by the next calls, up to two seconds of it, so
    // that a slow frame does not make the water run slow. Seconds that are not a finite number above zero are
    // ignored.
    void Step(float seconds);
    // The seconds asked for that have not been simulated yet.
    float PendingSeconds() const { return pendingSeconds; }

    // What the water looks like ----------------------------------------------------------------

    const WaterSettings& Settings() const { return settings; }
    int Width() const { return settings.width; }
    int Height() const { return settings.height; }
    bool IsSolid(int i, int j) const { return solid[Index(i, j)] != 0; }
    float Bed(int i, int j) const { return bed[Index(i, j)]; }
    float Depth(int i, int j) const { return depth[Index(i, j)]; }
    // The height of the water surface of a cell: its bed plus the water over it.
    float SurfaceHeight(int i, int j) const { return bed[Index(i, j)] + depth[Index(i, j)]; }
    // The cubic metres of water in the whole grid.
    double TotalVolume() const;
    // The speed of the water at a world point, found between the sides around it.
    glm::vec2 VelocityAt(glm::vec2 point) const;
    // The height of the water surface at a world point, blended between the wet cells around it. A point over
    // dry cells, or outside the grid, gets `fallback`.
    float SurfaceHeightAt(glm::vec2 point, float fallback) const;
    // True if the world point is over a cell with water in it.
    bool IsWetAt(glm::vec2 point) const;
    // The fastest the waves can travel now, in metres per second: what sets the size of the small steps.
    float FastestWave() const;

private:
    size_t Index(int i, int j) const { return static_cast<size_t>(j) * static_cast<size_t>(settings.width) + static_cast<size_t>(i); }
    // The side between cell (i - 1, j) and cell (i, j), and between (i, j - 1) and (i, j).
    size_t SideU(int i, int j) const { return static_cast<size_t>(j) * static_cast<size_t>(settings.width + 1) + static_cast<size_t>(i); }
    size_t SideV(int i, int j) const { return static_cast<size_t>(j) * static_cast<size_t>(settings.width) + static_cast<size_t>(i); }
    bool IsWet(int i, int j) const { return solid[Index(i, j)] == 0 && depth[Index(i, j)] > settings.minDepth; }

    void SmallStep(float dt);
    void AccelerateByTheSurfaceSlope(float dt);
    void Advect(float dt);
    void MoveWater(float dt);
    float SampleU(float x, float z) const;  // the flow along X at a point in grid units
    float SampleV(float x, float z) const;

    WaterSettings settings;
    std::vector<float> bed;
    std::vector<float> depth;
    std::vector<uint8_t> solid;
    std::vector<float> flowU;  // (width + 1) * height: through the sides that face along X
    std::vector<float> flowV;  // width * (height + 1): through the sides that face along Z
    std::vector<float> scratchU;
    std::vector<float> scratchV;
    std::vector<float> outflowScale;  // per cell, how much of its outgoing flux it can really give
    float pendingSeconds = 0.0f;
};

}  // namespace fluid
