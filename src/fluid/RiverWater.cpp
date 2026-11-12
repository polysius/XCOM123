#include "fluid/RiverWater.h"

#include <algorithm>
#include <cmath>

namespace fluid {
namespace {

constexpr int kMaxCells = 2048;
// The most drops that fall in one Update. It is a second of rain at the heaviest rate the command line allows.
constexpr float kMaxDropsPerUpdate = 5000.0f;

// The point of `rect` that is nearest to `point`.
glm::vec2 NearestPointOf(const OrientedRect& rect, glm::vec2 point) {
    const glm::vec2 local = rect.ToFrame(point);
    return rect.FromFrame(glm::clamp(local, rect.area.min, rect.area.max));
}

}  // namespace

bool RiverWater::Build(const CityLayout& city, glm::vec2 near, const RiverWaterSettings& waterSettings) {
    built = false;
    field.clear();
    waterCells = 0;
    settings = waterSettings;
    settings.cells = std::clamp(settings.cells, 2, kMaxCells);
    if (!(settings.cellSize > 1e-3f)) {
        settings.cellSize = RiverWaterSettings{}.cellSize;
    }
    random.seed(settings.seed);
    rainCarry = 0.0f;

    // The window is centred on the water nearest to the given place. A rectangle with no area is no water.
    bool found = false;
    glm::vec2 centre(0.0f);
    for (const OrientedRect& rect : city.water) {
        if (rect.IsEmpty()) {
            continue;
        }
        const glm::vec2 candidate = NearestPointOf(rect, near);
        if (!found || glm::distance(candidate, near) < glm::distance(centre, near)) {
            centre = candidate;
            found = true;
        }
    }
    if (!found) {
        return false;
    }

    WaterSettings grid;
    grid.width = settings.cells;
    grid.height = settings.cells;
    grid.cellSize = settings.cellSize;
    grid.origin = centre - 0.5f * static_cast<float>(settings.cells) * settings.cellSize;
    water.Initialise(grid);

    restLevel = city.waterLevel;
    waterCells = 0;
    // Only the rectangles that reach the window are looked at for each cell.
    const Rect window{grid.origin, grid.origin + static_cast<float>(settings.cells) * settings.cellSize};
    std::vector<const OrientedRect*> nearby;
    for (const OrientedRect& rect : city.water) {
        const Rect bounds = rect.Bounds();
        if (!rect.IsEmpty() && bounds.min.x < window.max.x && bounds.max.x > window.min.x && bounds.min.y < window.max.y && bounds.max.y > window.min.y) {
            nearby.push_back(&rect);
        }
    }
    for (int j = 0; j < settings.cells; ++j) {
        for (int i = 0; i < settings.cells; ++i) {
            const glm::vec2 cellCentre = grid.origin + (glm::vec2(static_cast<float>(i), static_cast<float>(j)) + 0.5f) * settings.cellSize;
            bool inWater = false;
            for (const OrientedRect* rect : nearby) {
                if (rect->Contains(cellCentre)) {
                    inWater = true;
                    break;
                }
            }
            if (!inWater) {
                water.SetSolid(i, j, true);
                continue;
            }
            water.SetBed(i, j, restLevel - settings.depth);
            ++waterCells;
        }
    }
    water.FillToLevel(restLevel);

    // The band of sponge along each edge, strongest at the very edge.
    for (int j = 0; j < settings.cells; ++j) {
        for (int i = 0; i < settings.cells; ++i) {
            const int fromEdge = std::min({i, j, settings.cells - 1 - i, settings.cells - 1 - j});
            if (static_cast<float>(fromEdge) < settings.spongeWidth) {
                const float share = 1.0f - static_cast<float>(fromEdge) / settings.spongeWidth;
                water.SetSponge(i, j, settings.spongeRate * share * share);
            }
        }
    }
    field.assign(static_cast<size_t>(water.Width()) * static_cast<size_t>(water.Height()), kNoWater);
    RefreshField();
    built = true;
    return true;
}

void RiverWater::Update(float seconds) {
    if (!built || !(seconds > 0.0f) || !std::isfinite(seconds)) {
        return;
    }
    DropRain(seconds);
    water.Step(seconds);
    RefreshField();
}

void RiverWater::Splash(glm::vec2 point, float radius, float volume) {
    if (built) {
        water.AddVolume(point, radius, volume);
    }
}

void RiverWater::Push(glm::vec2 point, float radius, glm::vec2 velocity) {
    if (built) {
        water.AddVelocity(point, radius, velocity);
    }
}

// Rain falls at random places of the window, but not in the sponge along its edge, where it would make ripples
// that the sponge only half soaks up, and a drop that lands on a wall is lost.
void RiverWater::DropRain(float seconds) {
    if (!(settings.rainPerSecond > 0.0f)) {
        return;
    }
    // A long frame does not make a downpour: no more drops fall in one update than a second of the heaviest rain.
    rainCarry = std::min(rainCarry + settings.rainPerSecond * seconds, kMaxDropsPerUpdate);
    const auto unit = [this]() { return static_cast<float>(random() >> 8) * (1.0f / 16777216.0f); };
    const float margin = std::min(settings.spongeWidth, 0.5f * static_cast<float>(water.Width()) - 1.0f) * settings.cellSize;
    const float extent = static_cast<float>(water.Width()) * settings.cellSize - 2.0f * std::max(margin, 0.0f);
    while (rainCarry >= 1.0f) {
        rainCarry -= 1.0f;
        const glm::vec2 point = Origin() + std::max(margin, 0.0f) + glm::vec2(unit(), unit()) * extent;
        water.AddVolume(point, settings.dropRadius, settings.dropVolume);
    }
}

void RiverWater::RefreshField() {
    size_t cell = 0;
    for (int j = 0; j < water.Height(); ++j) {
        for (int i = 0; i < water.Width(); ++i, ++cell) {
            const bool hasWater = !water.IsSolid(i, j) && water.Depth(i, j) > water.Settings().minDepth;
            field[cell] = hasWater ? water.SurfaceHeight(i, j) - restLevel : kNoWater;
        }
    }
}

}  // namespace fluid
