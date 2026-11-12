#pragma once

#include <cstdint>
#include <random>
#include <span>
#include <vector>

#include "fluid/ShallowWater.h"
#include "world/CityGenerator.h"

// The water of the city's river or canal near one place, simulated with the shallow water solver.
//
// The simulation covers a square window of the city. The cells of the window that lie in the water of the
// city (the water rectangles of the layout) are water, and every other cell is a wall: the quays, the
// levees and the land. The edges of the window are a sponge, so that waves leave it where a wall would turn
// them back. Rain, splashes and the push of things in the water disturb it, and every Update turns what
// the surface looks like into numbers the renderer draws.

namespace fluid {

struct RiverWaterSettings {
    int cells = 256;               // cells along each side of the window, between 2 and 2048
    float cellSize = 0.5f;         // metres
    float depth = 7.5f;            // metres of water over the bed of the river
    // The sponge along each edge of the window: wide and strong enough that about 3% of a wave comes back.
    float spongeWidth = 24.0f;     // cells
    float spongeRate = 10.0f;      // how fast it soaks up waves at the very edge, per second
    float rainPerSecond = 0.0f;    // drops that fall on the window each second
    float dropVolume = 0.03f;      // cubic metres of water in a drop
    float dropRadius = 0.4f;       // metres
    uint32_t seed = 1;
};

// The value in the field of a cell that has no water, which the shader reads as "nothing to draw here".
inline constexpr float kNoWater = -1000.0f;

class RiverWater {
public:
    // Builds the window around the middle of the water of `city` that is nearest to `near`. Returns false when the city
    // has no water, and then nothing else may be used.
    bool Build(const CityLayout& city, glm::vec2 near, const RiverWaterSettings& waterSettings);

    // Runs the simulation for `seconds` and refreshes the field.
    void Update(float seconds);

    // Adds or takes water at a world point, as a thing falling in or lifted out would.
    void Splash(glm::vec2 point, float radius, float volume);
    // Pushes the water at a world point towards `velocity`.
    void Push(glm::vec2 point, float radius, glm::vec2 velocity);

    bool IsBuilt() const { return built; }
    // The middle of the window, in world X and Z.
    glm::vec2 Centre() const { return Origin() + 0.5f * static_cast<float>(Width()) * CellSize(); }
    const ShallowWater& Water() const { return water; }
    // How far the surface of every cell is above the level of the still river, row by row from the minimum Z; kNoWater
    // where a cell has no water.
    std::span<const float> Field() const { return field; }
    glm::vec2 Origin() const { return water.Settings().origin; }
    float CellSize() const { return water.Settings().cellSize; }
    int Width() const { return water.Width(); }
    int Height() const { return water.Height(); }
    // The height of the still surface of the river.
    float RestLevel() const { return restLevel; }
    // How many cells of the window hold water.
    int WaterCellCount() const { return waterCells; }

private:
    void RefreshField();
    void DropRain(float seconds);

    ShallowWater water;
    RiverWaterSettings settings;
    std::vector<float> field;
    std::mt19937 random;
    float restLevel = 0.0f;
    float rainCarry = 0.0f;  // part of a drop left over from the last Update
    int waterCells = 0;
    bool built = false;
};

}  // namespace fluid
