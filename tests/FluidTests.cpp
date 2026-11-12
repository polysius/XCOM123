// Tests of the shallow water simulation (no window, no GPU). Like the other test programs there is no test
// framework: each test is a function that calls Check, and main reports every failure it finds. Each test
// sets up a small pool or channel and checks what physics says must happen: still water stays still, waves
// travel at the speed of sqrt(g d), walls turn them back, volume is never lost, water flows down slopes and
// floods dry ground, and the same pool always behaves the same way.
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "fluid/ShallowWater.h"

namespace {

using fluid::ShallowWater;
using fluid::WaterSettings;

int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

bool Near(double value, double expected, double tolerance) {
    return std::abs(value - expected) <= tolerance;
}

// A pool of `width` by `height` cells with a wall all round it, filled to `level` over a flat bed at zero.
ShallowWater MakePool(int width, int height, float level, float cellSize = 0.5f, bool advect = true) {
    WaterSettings settings;
    settings.width = width;
    settings.height = height;
    settings.cellSize = cellSize;
    settings.advect = advect;
    ShallowWater water;
    water.Initialise(settings);
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            if (i == 0 || j == 0 || i == width - 1 || j == height - 1) {
                water.SetSolid(i, j, true);
            }
        }
    }
    water.FillToLevel(level);
    return water;
}

void Run(ShallowWater& water, float seconds, float frame = 1.0f / 60.0f) {
    for (float t = 0.0f; t < seconds; t += frame) {
        water.Step(frame);
    }
}

float HighestSurface(const ShallowWater& water) {
    float highest = -1e9f;
    for (int j = 0; j < water.Height(); ++j) {
        for (int i = 0; i < water.Width(); ++i) {
            if (!water.IsSolid(i, j) && water.Depth(i, j) > 0.0f) {
                highest = std::max(highest, water.SurfaceHeight(i, j));
            }
        }
    }
    return highest;
}

float LowestDepth(const ShallowWater& water) {
    float lowest = 1e9f;
    for (int j = 0; j < water.Height(); ++j) {
        for (int i = 0; i < water.Width(); ++i) {
            lowest = std::min(lowest, water.Depth(i, j));
        }
    }
    return lowest;
}

// Still water ---------------------------------------------------------------------------------

void StillWaterStaysStill() {
    ShallowWater water = MakePool(40, 40, 2.0f);
    Run(water, 20.0f);
    float spread = 0.0f;
    for (int j = 1; j < 39; ++j) {
        for (int i = 1; i < 39; ++i) {
            spread = std::max(spread, std::abs(water.SurfaceHeight(i, j) - 2.0f));
        }
    }
    Check(spread < 1e-5f, "a pool that nothing disturbs stays flat, spread " + std::to_string(spread));
    Check(glm::length(water.VelocityAt({10.0f, 10.0f})) < 1e-5f, "and the water in it does not move");
}

void StillWaterOverAnUnevenBedStaysStill() {
    ShallowWater water = MakePool(40, 40, 3.0f);
    for (int j = 1; j < 39; ++j) {
        for (int i = 1; i < 39; ++i) {
            water.SetBed(i, j, 0.4f * std::sin(0.5f * static_cast<float>(i)) * std::cos(0.4f * static_cast<float>(j)));
        }
    }
    water.FillToLevel(3.0f);
    Run(water, 20.0f);
    float spread = 0.0f;
    for (int j = 1; j < 39; ++j) {
        for (int i = 1; i < 39; ++i) {
            spread = std::max(spread, std::abs(water.SurfaceHeight(i, j) - 3.0f));
        }
    }
    Check(spread < 1e-4f, "water over a bumpy bed keeps a level surface, spread " + std::to_string(spread));
}

// Volume and waves ----------------------------------------------------------------------------

void VolumeIsKeptWhileWavesRunAboutThePool() {
    ShallowWater water = MakePool(60, 60, 2.0f);
    const double before = water.TotalVolume();
    water.AddVolume({15.0f, 15.0f}, 2.0f, 5.0f);
    const double start = water.TotalVolume();
    Check(Near(start - before, 5.0, 1e-3), "a splash adds the volume it was asked to");
    Run(water, 30.0f);
    Check(Near(water.TotalVolume(), start, start * 1e-6), "no water is made or lost in thirty seconds of waves, change " +
                                                              std::to_string(water.TotalVolume() - start));
    Check(LowestDepth(water) >= 0.0f, "and no cell goes below zero depth");
}

// A long narrow channel along X, closed at both ends.
ShallowWater MakeChannel(float level) {
    return MakePool(400, 8, level, 0.5f);
}

void WavesTravelAtTheSpeedOfShallowWater() {
    ShallowWater water = MakeChannel(2.0f);
    // A small splash, a few centimetres high: a big one runs faster than sqrt(g d), as big waves do.
    water.AddVolume({20.0f, 2.0f}, 2.0f, 0.8f);
    const float level = 2.0f;
    Run(water, 10.0f);
    // The front is the furthest point to the right where the surface is clearly above the level.
    float front = 0.0f;
    for (int i = 1; i < 399; ++i) {
        if (water.SurfaceHeight(i, 4) - level > 1e-4f) {
            front = std::max(front, (static_cast<float>(i) + 0.5f) * 0.5f);
        }
    }
    const float speed = std::sqrt(9.81f * 2.0f);
    const float travelled = front - 20.0f;
    Check(travelled > 0.85f * speed * 10.0f && travelled < 1.15f * speed * 10.0f,
          "the front of a wave in two metres of water has gone about " + std::to_string(speed * 10.0f) + " m in ten seconds, went " +
              std::to_string(travelled));
}

void AWaveIsTurnedBackByTheWall() {
    ShallowWater water = MakeChannel(2.0f);
    water.AddVolume({180.0f, 2.0f}, 2.0f, 6.0f);
    float direct = 0.0f;
    float reflected = 0.0f;
    for (float t = 0.0f; t < 18.0f; t += 1.0f / 60.0f) {
        water.Step(1.0f / 60.0f);
        const float rise = water.SurfaceHeightAt({160.0f, 2.0f}, 2.0f) - 2.0f;
        // The wave going left passes x = 160 after about 20 / 4.4 = 4.5 s. The half that went right comes
        // back off the wall at 200 m and passes it after about 60 / 4.4 = 13.5 s.
        if (t > 3.0f && t < 6.5f) {
            direct = std::max(direct, rise);
        }
        if (t > 11.5f && t < 16.0f) {
            reflected = std::max(reflected, rise);
        }
    }
    Check(direct > 1e-3f, "the wave that goes straight at the point arrives");
    Check(reflected > 0.3f * direct, "and the wave that went the other way comes back off the wall, " + std::to_string(reflected) +
                                         " against " + std::to_string(direct));
}

void ASplashSpreadsTheSameInEveryDirection() {
    ShallowWater water = MakePool(61, 61, 2.0f);
    water.AddVolume({15.25f, 15.25f}, 1.5f, 2.0f);  // the middle of the middle cell
    Run(water, 5.0f);
    float asymmetry = 0.0f;
    for (int j = 1; j < 60; ++j) {
        for (int i = 1; i < 60; ++i) {
            asymmetry = std::max(asymmetry, std::abs(water.SurfaceHeight(i, j) - water.SurfaceHeight(j, i)));
            asymmetry = std::max(asymmetry, std::abs(water.SurfaceHeight(i, j) - water.SurfaceHeight(60 - i, j)));
        }
    }
    Check(asymmetry < 1e-4f, "a splash in the middle of a square pool stays symmetric, asymmetry " + std::to_string(asymmetry));
}

void BigTimeStepsAreStable() {
    ShallowWater water = MakePool(60, 60, 5.0f);
    water.AddVolume({15.0f, 15.0f}, 2.0f, 40.0f);
    const double start = water.TotalVolume();
    Run(water, 20.0f, 1.0f / 30.0f);
    Check(HighestSurface(water) < 6.0f && HighestSurface(water) > 4.0f, "five metres of water with thirtieth of a second frames does not blow up");
    Check(Near(water.TotalVolume(), start, start * 1e-6), "and keeps its volume");
    Run(water, 5.0f, 0.25f);
    Check(std::isfinite(HighestSurface(water)) && HighestSurface(water) < 7.0f, "even frames of a quarter of a second are survived");
}

// Beds, dry ground and walls --------------------------------------------------------------------

void WaterRunsDownASlope() {
    WaterSettings settings;
    settings.width = 60;
    settings.height = 8;
    ShallowWater water;
    water.Initialise(settings);
    for (int j = 0; j < 8; ++j) {
        for (int i = 0; i < 60; ++i) {
            water.SetBed(i, j, 3.0f - 0.1f * static_cast<float>(i));
            if (j == 0 || j == 7 || i == 59) {
                water.SetSolid(i, j, true);
            }
        }
    }
    for (int j = 1; j < 7; ++j) {
        for (int i = 0; i < 10; ++i) {
            water.SetDepth(i, j, 0.5f);  // a pond at the top of the slope
        }
    }
    const double volume = water.TotalVolume();
    Run(water, 30.0f);
    double lowHalf = 0.0;
    for (int j = 1; j < 7; ++j) {
        for (int i = 30; i < 59; ++i) {
            lowHalf += water.Depth(i, j);
        }
    }
    Check(lowHalf * 0.25 > 0.5 * volume * 0.5, "most of the water has run down into the lower half of the slope");
    Check(Near(water.TotalVolume(), volume, volume * 1e-6), "without losing any");
    Check(LowestDepth(water) >= 0.0f, "or going below zero depth");
}

void AWaveBreaksOverDryGroundAndFloodsIt() {
    WaterSettings settings;
    settings.width = 100;
    settings.height = 8;
    ShallowWater water;
    water.Initialise(settings);
    for (int j = 0; j < 8; ++j) {
        for (int i = 0; i < 100; ++i) {
            if (j == 0 || j == 7 || i == 0 || i == 99) {
                water.SetSolid(i, j, true);
            }
        }
    }
    for (int j = 1; j < 7; ++j) {
        for (int i = 1; i < 30; ++i) {
            water.SetDepth(i, j, 2.0f);  // a dam of water at the left; the rest is dry
        }
    }
    const double volume = water.TotalVolume();
    Run(water, 12.0f);
    Check(water.IsWetAt({40.0f, 2.0f}), "the water has flooded the dry ground 40 m from the dam");
    Check(Near(water.TotalVolume(), volume, volume * 1e-6), "and no water was made or lost on the way");
    Check(LowestDepth(water) >= 0.0f, "nor did a cell go below zero depth");
}

void NoWaterEntersAWall() {
    ShallowWater water = MakeChannel(2.0f);
    for (int j = 0; j < 6; ++j) {
        water.SetSolid(200, j, true);  // a wall across the channel with a gap at the end
    }
    water.AddVolume({20.0f, 2.0f}, 2.0f, 10.0f);
    Run(water, 20.0f);
    bool dry = true;
    for (int j = 0; j < 6; ++j) {
        dry = dry && water.Depth(200, j) == 0.0f;
    }
    Check(dry, "the cells of a wall stay dry");
    // The only way past the wall is the gap at the far end of it, and some water gets through.
    Check(water.SurfaceHeight(220, 4) - 2.0f > 1e-4f, "water gets round the wall through the gap");
}

// Sources and queries -------------------------------------------------------------------------

void ASourceAddsAndTakesTheVolumeAskedFor() {
    ShallowWater water = MakePool(40, 40, 1.0f);
    const double before = water.TotalVolume();
    const float added = water.AddVolume({10.0f, 10.0f}, 3.0f, 4.0f);
    Check(Near(added, 4.0, 1e-3) && Near(water.TotalVolume() - before, 4.0, 1e-3), "four cubic metres go in");
    const float taken = water.AddVolume({10.0f, 10.0f}, 3.0f, -2.5f);
    Check(Near(taken, -2.5, 1e-3) && Near(water.TotalVolume() - before, 1.5, 1e-3), "and two and a half come out");
    const float tooMuch = water.AddVolume({10.0f, 10.0f}, 0.5f, -1000.0f);
    Check(tooMuch > -1000.0f && LowestDepth(water) >= 0.0f, "no more comes out than the cells hold");
}

void ThePushOfAHullStartsACurrent() {
    ShallowWater water = MakePool(60, 60, 2.0f);
    water.AddVelocity({15.0f, 15.0f}, 2.0f, {3.0f, 0.0f});
    const glm::vec2 velocity = water.VelocityAt({15.0f, 15.0f});
    Check(velocity.x > 2.0f && velocity.x <= 3.0f && std::abs(velocity.y) < 0.01f, "the water at the push moves with it");
    const glm::vec2 far = water.VelocityAt({25.0f, 25.0f});
    Check(glm::length(far) < 1e-4f, "and the water far from it does not");
}

void TheSurfaceHeightIsReadBetweenTheCells() {
    ShallowWater water = MakePool(40, 40, 2.0f);
    Check(Near(water.SurfaceHeightAt({10.0f, 10.0f}, -1.0f), 2.0, 1e-4), "a calm pool is two metres up everywhere");
    Check(water.SurfaceHeightAt({-50.0f, 10.0f}, -1.0f) == -1.0f, "a point outside the grid gets the fallback");
    Check(water.IsWetAt({10.0f, 10.0f}) && !water.IsWetAt({0.1f, 0.1f}), "the middle is wet and a wall is not");
}

void TheSameWaterAlwaysBehavesTheSameWay() {
    ShallowWater first = MakePool(50, 50, 2.0f);
    ShallowWater second = MakePool(50, 50, 2.0f);
    for (ShallowWater* water : {&first, &second}) {
        water->AddVolume({12.0f, 9.0f}, 2.0f, 7.0f);
        water->AddVelocity({8.0f, 14.0f}, 1.5f, {2.0f, -1.0f});
        Run(*water, 8.0f);
    }
    bool identical = true;
    for (int j = 0; j < 50; ++j) {
        for (int i = 0; i < 50; ++i) {
            identical = identical && first.Depth(i, j) == second.Depth(i, j);
        }
    }
    Check(identical, "two pools given the same splash are identical to the last bit");
}

// What the review of the solver found -------------------------------------------------------------

// A pool of 100 cells with a wall all round it, a tall dam of water at one end and a thin sheet over the rest.
ShallowWater MakeDamBreak() {
    WaterSettings settings;
    settings.width = 100;
    settings.height = 8;
    settings.cellSize = 0.5f;
    ShallowWater water;
    water.Initialise(settings);
    for (int j = 0; j < 8; ++j) {
        for (int i = 0; i < 100; ++i) {
            if (j == 0 || j == 7 || i == 0 || i == 99) {
                water.SetSolid(i, j, true);
            } else {
                water.SetDepth(i, j, i < 20 ? 5.0f : 0.1f);
            }
        }
    }
    return water;
}

void ADamBreakNeverPilesTheWaterHigherThanTheDam() {
    ShallowWater water = MakeDamBreak();
    float highest = 0.0f;
    for (float t = 0.0f; t < 15.0f; t += 1.0f / 60.0f) {
        water.Step(1.0f / 60.0f);
        highest = std::max(highest, HighestSurface(water));
    }
    Check(highest < 5.0f + 1e-3f, "the surface never rises above the dam it came from, highest " + std::to_string(highest));
}

void APartlyDryLakeOnABumpySlopeStaysAtRest() {
    WaterSettings settings;
    settings.width = 60;
    settings.height = 12;
    ShallowWater water;
    water.Initialise(settings);
    for (int j = 0; j < 12; ++j) {
        for (int i = 0; i < 60; ++i) {
            if (j == 0 || j == 11 || i == 0 || i == 59) {
                water.SetSolid(i, j, true);
            } else {
                water.SetBed(i, j, 0.12f * static_cast<float>(i) + 0.15f * std::sin(0.9f * static_cast<float>(i + j)));
            }
        }
    }
    water.FillToLevel(2.5f);  // the lake reaches up the slope and the rest of it is dry
    const double volume = water.TotalVolume();
    float fastest = 0.0f;
    for (float t = 0.0f; t < 20.0f; t += 1.0f / 60.0f) {
        water.Step(1.0f / 60.0f);
        for (int j = 1; j < 11; ++j) {
            for (int i = 1; i < 59; ++i) {
                fastest = std::max(fastest, glm::length(water.VelocityAt({(static_cast<float>(i) + 0.5f) * 0.5f, (static_cast<float>(j) + 0.5f) * 0.5f})));
            }
        }
    }
    Check(fastest < 1e-3f, "a lake at rest on a slope, with its edge on dry bumps, does not start to flow, fastest " + std::to_string(fastest));
    Check(Near(water.TotalVolume(), volume, volume * 1e-6), "and keeps its volume");
}

void TheFastestWaveCountsTheCurrentToo() {
    ShallowWater water = MakePool(40, 40, 0.5f);
    const float still = water.FastestWave();
    water.AddVelocity({10.0f, 10.0f}, 2.0f, {10.0f, 0.0f});
    Check(water.FastestWave() > still + 5.0f, "a current of ten metres a second makes the water faster than its waves alone");
}

void NumbersThatAreNotNumbersDoNotHarmTheWater() {
    ShallowWater water = MakePool(40, 40, 2.0f);
    const double volume = water.TotalVolume();
    const float nan = std::nanf("");
    const float infinity = std::numeric_limits<float>::infinity();
    water.AddVelocity({10.0f, 10.0f}, 2.0f, {nan, 0.0f});
    water.AddVelocity({nan, 10.0f}, 2.0f, {1.0f, 0.0f});
    Check(water.AddVolume({10.0f, 10.0f}, 2.0f, nan) == 0.0f, "a volume that is not a number adds nothing");
    Check(water.AddVolume({infinity, 10.0f}, 2.0f, 1.0f) == 0.0f, "nor does a point at infinity");
    water.Step(nan);
    water.Step(infinity);
    water.Step(-1.0f);
    Run(water, 2.0f);
    Check(Near(water.TotalVolume(), volume, volume * 1e-6), "and the pool is as it was");
    Check(std::isfinite(HighestSurface(water)) && Near(HighestSurface(water), 2.0, 1e-3), "and still flat");
    Check(!water.IsWetAt({nan, 1.0f}) && water.SurfaceHeightAt({nan, nan}, -3.0f) == -3.0f, "questions about such points get plain answers");
}

void ALongFrameIsNotLostButRunOverTheNextOnes() {
    // Deep water at a small cell size needs many small steps for even a short time.
    WaterSettings settings;
    settings.width = 400;
    settings.height = 8;
    settings.cellSize = 0.1f;
    const auto make = [&settings]() {
        ShallowWater water;
        water.Initialise(settings);
        for (int j = 0; j < 8; ++j) {
            for (int i = 0; i < 400; ++i) {
                if (j == 0 || j == 7 || i == 0 || i == 399) {
                    water.SetSolid(i, j, true);
                } else {
                    water.SetDepth(i, j, 30.0f);
                }
            }
        }
        water.AddVolume({4.0f, 0.4f}, 0.3f, 0.05f);
        return water;
    };
    const auto front = [](const ShallowWater& water) {
        float furthest = 0.0f;
        for (int i = 1; i < 399; ++i) {
            if (water.SurfaceHeight(i, 4) - 30.0f > 1e-5f) {
                furthest = static_cast<float>(i) * 0.1f;
            }
        }
        return furthest;
    };
    ShallowWater slow = make();
    slow.Step(1.0f);  // one long frame
    Check(slow.PendingSeconds() > 0.3f, "a frame that needs more small steps than one call may run leaves the rest pending");
    int calls = 0;
    while (slow.PendingSeconds() > 0.0f && calls < 40) {
        slow.Step(1e-4f);
        ++calls;
    }
    Check(slow.PendingSeconds() == 0.0f, "and the next calls run it all");
    ShallowWater reference = make();
    for (int i = 0; i < 60; ++i) {
        reference.Step(1.0f / 60.0f);
    }
    const float slowFront = front(slow);
    const float referenceFront = front(reference);
    Check(std::abs(slowFront - referenceFront) < 0.15f * referenceFront,
          "after all of it the wave has gone as far as with sixty frames, " + std::to_string(slowFront) + " against " + std::to_string(referenceFront));
}

void AVeryLongFrameIsCappedAndNotRunBackwards() {
    ShallowWater water = MakePool(40, 40, 2.0f);
    water.AddVolume({10.0f, 10.0f}, 2.0f, 4.0f);
    water.Step(1e9f);
    Check(water.PendingSeconds() <= 2.0f, "no more than two seconds are kept for later");
    ShallowWater other = MakePool(40, 40, 2.0f);
    other.AddVolume({10.0f, 10.0f}, 2.0f, 4.0f);
    other.Step(1e6f);
    bool same = true;
    for (int j = 0; j < 40; ++j) {
        for (int i = 0; i < 40; ++i) {
            same = same && water.Depth(i, j) == other.Depth(i, j);
        }
    }
    Check(same, "a frame of a million seconds and one of a billion seconds are both cut to the same thing");
}

void ASmallSplashAddsItsVolumeWhereverItFalls() {
    ShallowWater water = MakePool(40, 40, 1.0f);
    uint32_t state = 4242u;
    const auto next = [&state]() {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) / 16777216.0f;
    };
    int missed = 0;
    for (int i = 0; i < 500; ++i) {
        const glm::vec2 point(2.0f + 16.0f * next(), 2.0f + 16.0f * next());
        const float added = water.AddVolume(point, 0.0f, 0.01f);
        missed += std::abs(added - 0.01f) > 1e-5f ? 1 : 0;
    }
    // Including the points on the edge and the corner of a cell.
    for (const glm::vec2& point : {glm::vec2(5.5f, 5.5f), glm::vec2(5.0f, 5.0f), glm::vec2(5.25f, 5.5f)}) {
        missed += std::abs(water.AddVolume(point, 0.0f, 0.01f) - 0.01f) > 1e-5f ? 1 : 0;
    }
    Check(missed == 0, "a drop falling anywhere in the pool adds all its water, missed " + std::to_string(missed));
}

void APushOnlyTouchesTheSidesNearIt() {
    ShallowWater water = MakePool(200, 200, 1.0f);
    water.AddVelocity({50.0f, 50.0f}, 1.0f, {2.0f, 2.0f});
    Check(glm::length(water.VelocityAt({50.0f, 50.0f})) > 1.0f, "the water at the point moves");
    Check(glm::length(water.VelocityAt({80.0f, 80.0f})) == 0.0f && glm::length(water.VelocityAt({20.0f, 80.0f})) == 0.0f,
          "and the water far from it does not");
}

}  // namespace

int main() {
    struct Test {
        const char* name;
        std::function<void()> run;
    };
    const std::vector<Test> tests = {
        {"StillWaterStaysStill", StillWaterStaysStill},
        {"StillWaterOverAnUnevenBedStaysStill", StillWaterOverAnUnevenBedStaysStill},
        {"VolumeIsKeptWhileWavesRunAboutThePool", VolumeIsKeptWhileWavesRunAboutThePool},
        {"WavesTravelAtTheSpeedOfShallowWater", WavesTravelAtTheSpeedOfShallowWater},
        {"AWaveIsTurnedBackByTheWall", AWaveIsTurnedBackByTheWall},
        {"ASplashSpreadsTheSameInEveryDirection", ASplashSpreadsTheSameInEveryDirection},
        {"BigTimeStepsAreStable", BigTimeStepsAreStable},
        {"WaterRunsDownASlope", WaterRunsDownASlope},
        {"AWaveBreaksOverDryGroundAndFloodsIt", AWaveBreaksOverDryGroundAndFloodsIt},
        {"NoWaterEntersAWall", NoWaterEntersAWall},
        {"ASourceAddsAndTakesTheVolumeAskedFor", ASourceAddsAndTakesTheVolumeAskedFor},
        {"ThePushOfAHullStartsACurrent", ThePushOfAHullStartsACurrent},
        {"TheSurfaceHeightIsReadBetweenTheCells", TheSurfaceHeightIsReadBetweenTheCells},
        {"TheSameWaterAlwaysBehavesTheSameWay", TheSameWaterAlwaysBehavesTheSameWay},
        {"ADamBreakNeverPilesTheWaterHigherThanTheDam", ADamBreakNeverPilesTheWaterHigherThanTheDam},
        {"APartlyDryLakeOnABumpySlopeStaysAtRest", APartlyDryLakeOnABumpySlopeStaysAtRest},
        {"TheFastestWaveCountsTheCurrentToo", TheFastestWaveCountsTheCurrentToo},
        {"NumbersThatAreNotNumbersDoNotHarmTheWater", NumbersThatAreNotNumbersDoNotHarmTheWater},
        {"ALongFrameIsNotLostButRunOverTheNextOnes", ALongFrameIsNotLostButRunOverTheNextOnes},
        {"AVeryLongFrameIsCappedAndNotRunBackwards", AVeryLongFrameIsCappedAndNotRunBackwards},
        {"ASmallSplashAddsItsVolumeWhereverItFalls", ASmallSplashAddsItsVolumeWhereverItFalls},
        {"APushOnlyTouchesTheSidesNearIt", APushOnlyTouchesTheSidesNearIt},
    };
    for (const Test& test : tests) {
        const int failuresBefore = failureCount;
        try {
            test.run();
        } catch (const std::exception& error) {
            ++failureCount;
            std::cerr << "  threw: " << error.what() << "\n";
        }
        std::cout << (failureCount == failuresBefore ? "PASS  " : "FAIL  ") << test.name << "\n";
    }
    std::cout << (failureCount == 0 ? "all fluid tests passed\n" : "fluid tests failed\n");
    return failureCount == 0 ? 0 : 1;
}
