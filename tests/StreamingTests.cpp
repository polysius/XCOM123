// Tests of the streamed world, with no window and no GPU: the chunk generator (a chunk is a pure function of the seed and
// its address, neighbours agree on shared streets and water, the budget and the time per chunk) and, through
// RunStreamingManagerTests, the streaming manager. Like the other test programs there is no framework: each test calls
// Check, and main reports every failure it finds.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

#include "render/ChunkCulling.h"
#include "render/RangeAllocator.h"
#include "render/RetirementQueue.h"
#include "scene/Frustum.h"
#include "world/stream/ChunkGenerator.h"
#include "world/stream/WorldSkeleton.h"

int RunStreamingManagerTests();

namespace {

using streaming::ChunkCoord;
using Clock = std::chrono::steady_clock;

int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

std::string Describe(ChunkCoord coord) {
    return "chunk (" + std::to_string(coord.x) + ", " + std::to_string(coord.z) + ")";
}

constexpr uint32_t kSeed = 1789;
constexpr int kBoxBudgetPerChunk = 16000;
constexpr double kChunkTimeBudgetMs = 30.0;

// FNV-1a over a byte range: a cheap fingerprint to compare two chunks without writing a comparison for every field.
uint64_t Fingerprint(uint64_t hash, const void* bytes, size_t count) {
    const auto* data = static_cast<const unsigned char*>(bytes);
    for (size_t i = 0; i < count; ++i) {
        hash = (hash ^ data[i]) * 1099511628211ull;
    }
    return hash;
}

uint64_t FingerprintOf(const world::ChunkData& chunk) {
    uint64_t hash = 14695981039346656037ull;
    hash = Fingerprint(hash, chunk.layout.boxes.data(), chunk.layout.boxes.size() * sizeof(BoxInstance));
    hash = Fingerprint(hash, chunk.layout.boxParts.data(), chunk.layout.boxParts.size() * sizeof(CityPart));
    hash = Fingerprint(hash, chunk.layout.streets.nodes.data(), chunk.layout.streets.nodes.size() * sizeof(glm::vec2));
    hash = Fingerprint(hash, chunk.layout.streetLights.data(), chunk.layout.streetLights.size() * sizeof(glm::vec3));
    return hash;
}

world::ChunkSettings Settings() {
    world::ChunkSettings settings;
    settings.seed = kSeed;
    return settings;
}

// Whether a ground point lies inside the footprint of a box.
bool Covers(const BoxInstance& box, glm::vec2 point) {
    const glm::vec2 offset = point - glm::vec2(box.center.x, box.center.z);
    const float cosine = std::cos(box.yaw);
    const float sine = std::sin(box.yaw);
    const float alongX = offset.x * cosine - offset.y * sine;
    const float alongZ = offset.x * sine + offset.y * cosine;
    return std::abs(alongX) <= box.halfSize.x && std::abs(alongZ) <= box.halfSize.z;
}

bool HasPart(const world::ChunkData& chunk, CityPart part, glm::vec2 point) {
    for (size_t i = 0; i < chunk.layout.boxes.size(); ++i) {
        if (chunk.layout.boxParts[i] == part && Covers(chunk.layout.boxes[i], point)) {
            return true;
        }
    }
    return false;
}

// A chunk coordinate whose cells include a river row, so that tests of water have water to look at.
ChunkCoord RiverChunk() {
    for (int z = -10; z <= 10; ++z) {
        if (world::IsRiverRow(kSeed, z * world::kCellsPerChunk) || world::IsRiverRow(kSeed, z * world::kCellsPerChunk + 1)) {
            return {0, z};
        }
    }
    return {0, 0};
}

// --- The skeleton ----------------------------------------------------------------------------------------------

void TestSkeletonIsPure() {
    std::cout << "skeleton\n";
    Check(world::FloorDiv(-1, 2) == -1 && world::FloorDiv(-2, 2) == -1 && world::FloorDiv(-3, 2) == -2 && world::FloorDiv(3, 2) == 1,
          "FloorDiv rounds towards minus infinity");
    for (int i = -50; i < 50; ++i) {
        const float a = world::GridLine(kSeed, world::Axis::X, i);
        const float b = world::GridLine(kSeed, world::Axis::X, i + 1);
        Check(a == world::GridLine(kSeed, world::Axis::X, i), "a grid line is the same on every call");
        const float gap = b - a;
        Check(gap >= world::kCellPitch - 2.0f * world::kLineJitter && gap <= world::kCellPitch + 2.0f * world::kLineJitter,
              "neighbouring arterials are one cell pitch apart, give or take the jitter");
    }
    const glm::vec2 point(1234.5f, -987.6f);
    Check(world::DistrictAt(kSeed, point) == world::DistrictAt(kSeed, point), "a district is a function of position");
    // Every kind of district should turn up somewhere in a few square kilometres.
    bool seen[static_cast<size_t>(District::Count)] = {};
    for (int z = -20; z < 20; ++z) {
        for (int x = -20; x < 20; ++x) {
            seen[static_cast<size_t>(world::DistrictAt(kSeed, {x * 200.0f, z * 200.0f}))] = true;
        }
    }
    for (size_t kind = 0; kind < static_cast<size_t>(District::Count); ++kind) {
        Check(seen[kind], "every district kind occurs");
    }
    int riverRows = 0;
    for (int row = -60; row < 60; ++row) {
        riverRows += world::IsRiverRow(kSeed, row) ? 1 : 0;
    }
    Check(riverRows == 10, "one row in twelve holds a river");
}

void TestRiverStaysInItsBand() {
    std::cout << "river band\n";
    for (int row = -60; row < 60; ++row) {
        if (!world::IsRiverRow(kSeed, row)) {
            continue;
        }
        const float low = world::GridLine(kSeed, world::Axis::Z, row);
        const float high = world::GridLine(kSeed, world::Axis::Z, row + 1);
        const float reach = world::kRiverHalfWidth + world::kBankThickness + world::kStreetHalfWidth;
        for (float x = -1000.0f; x < 1000.0f; x += 7.0f) {
            const float centre = world::RiverCentreZ(kSeed, row, x);
            Check(centre - reach > low && centre + reach < high, "the river and its banks keep clear of the arterials of its row");
        }
    }
}

// --- Chunks --------------------------------------------------------------------------------------------------

void TestChunkIsRepeatable() {
    std::cout << "a chunk generated twice\n";
    for (const ChunkCoord coord : {ChunkCoord{0, 0}, ChunkCoord{-3, 5}, RiverChunk(), ChunkCoord{40, -40}}) {
        const auto first = world::GenerateChunk(Settings(), coord);
        const auto second = world::GenerateChunk(Settings(), coord);
        Check(!first->layout.boxes.empty(), Describe(coord) + " has boxes");
        Check(FingerprintOf(*first) == FingerprintOf(*second), Describe(coord) + " is the same both times");
    }
    world::ChunkSettings other = Settings();
    other.seed = kSeed + 1;
    Check(FingerprintOf(*world::GenerateChunk(Settings(), {2, 2})) != FingerprintOf(*world::GenerateChunk(other, {2, 2})),
          "another seed gives another chunk");
}

// Nodes of a chunk's street graph that lie on a vertical (constant X) or horizontal (constant Z) line.
std::vector<float> NodesOnLine(const world::ChunkData& chunk, bool constantX, float value) {
    std::vector<float> along;
    for (const glm::vec2 node : chunk.layout.streets.nodes) {
        if ((constantX ? node.x : node.y) == value) {
            along.push_back(constantX ? node.y : node.x);
        }
    }
    std::sort(along.begin(), along.end());
    return along;
}

void TestNeighboursShareStreets() {
    std::cout << "neighbouring chunks share the streets on their border\n";
    const std::pair<ChunkCoord, ChunkCoord> pairs[] = {
        {{0, 0}, {1, 0}}, {{-1, 3}, {0, 3}}, {{7, -2}, {7, -1}}, {{-5, -5}, {-5, -4}}};
    for (const auto& [west, east] : pairs) {
        const bool alongZ = west.x != east.x;  // the shared border runs along Z when the chunks are side by side
        const auto a = world::GenerateChunk(Settings(), west);
        const auto b = world::GenerateChunk(Settings(), east);
        const int borderIndex = alongZ ? east.x * world::kCellsPerChunk : east.z * world::kCellsPerChunk;
        const float border = world::GridLine(kSeed, alongZ ? world::Axis::X : world::Axis::Z, borderIndex);
        const std::vector<float> fromA = NodesOnLine(*a, alongZ, border);
        const std::vector<float> fromB = NodesOnLine(*b, alongZ, border);
        // Both chunks span the same cells along the border, so both have a junction where each of the arterials that
        // cross the border meets it, at exactly the same coordinate.
        const int firstCross = (alongZ ? west.z : west.x) * world::kCellsPerChunk;
        for (int cross = firstCross; cross <= firstCross + world::kCellsPerChunk; ++cross) {
            const float line = world::GridLine(kSeed, alongZ ? world::Axis::Z : world::Axis::X, cross);
            Check(std::find(fromA.begin(), fromA.end(), line) != fromA.end(), Describe(west) + " has a junction on the shared arterial");
            Check(std::find(fromB.begin(), fromB.end(), line) != fromB.end(), Describe(east) + " has a junction on the shared arterial");
        }
    }
}

void TestNeighboursShareWater() {
    std::cout << "water continues across chunk borders\n";
    const ChunkCoord west = RiverChunk();
    const ChunkCoord east{west.x + 1, west.z};
    const auto a = world::GenerateChunk(Settings(), west);
    const auto b = world::GenerateChunk(Settings(), east);
    const int borderColumn = east.x * world::kCellsPerChunk;
    const float borderX = world::GridLine(kSeed, world::Axis::X, borderColumn);
    int riverRowsChecked = 0;
    for (int row = west.z * world::kCellsPerChunk; row < (west.z + 1) * world::kCellsPerChunk; ++row) {
        if (!world::IsRiverRow(kSeed, row)) {
            continue;
        }
        ++riverRowsChecked;
        const float centre = world::RiverCentreZ(kSeed, row, borderX);
        for (float offset = -world::kRiverHalfWidth + 1.0f; offset <= world::kRiverHalfWidth - 1.0f; offset += 2.0f) {
            for (const float dx : {-0.5f, 0.0f, 0.5f}) {
                const glm::vec2 point(borderX + dx, centre + offset);
                const bool inWest = HasPart(*a, CityPart::Water, point);
                const bool inEast = HasPart(*b, CityPart::Water, point);
                // Off the centre line the end of a river box is slanted by the slope of the river, so near the banks the
                // two chunks each cover only their own side; together they must leave no gap.
                const bool onCentreLine = std::abs(offset) < 1.5f;
                Check(onCentreLine ? (inWest && inEast) : (inWest || inEast),
                      "water crosses the border without a gap (offset " + std::to_string(offset) + ", dx " + std::to_string(dx) + ")");
            }
        }
        // The banks too, a few metres out from the water.
        const glm::vec2 bank(borderX, centre + world::kRiverHalfWidth + 2.0f);
        Check(HasPart(*a, CityPart::Embankment, bank) && HasPart(*b, CityPart::Embankment, bank), "the bank crosses the border");
    }
    Check(riverRowsChecked > 0, "the chunk chosen for the test holds a river");
}

// The top of the highest box over a ground point, or -1000 when nothing covers it.
float SurfaceHeightAt(const world::ChunkData& chunk, glm::vec2 point) {
    float height = -1000.0f;
    for (const BoxInstance& box : chunk.layout.boxes) {
        if (Covers(box, point)) {
            height = std::max(height, box.center.y + box.halfSize.y);
        }
    }
    return height;
}

void TestNoHolesInTheGround() {
    std::cout << "no holes in the ground\n";
    for (const ChunkCoord coord : {ChunkCoord{0, 0}, ChunkCoord{-4, 2}, RiverChunk(), ChunkCoord{9, 9}}) {
        const auto chunk = world::GenerateChunk(Settings(), coord);
        // The cells of a chunk reach a little past or stop a little short of its nominal square, by the jitter of the arterials.
        // The asphalt of the north and east arterials is laid by the chunks beyond them, so stop at the kerb.
        Rect rect = chunk->layout.bounds;
        rect.max -= world::kRoadHalfWidth;
        int holes = 0;
        for (float z = rect.min.y + 3.0f; z < rect.max.y - 3.0f; z += 3.1f) {
            for (float x = rect.min.x + 3.0f; x < rect.max.x - 3.0f; x += 3.1f) {
                // Land and water both count: the ground is solid or it is the bed under a river, never a view of the sky.
                if (SurfaceHeightAt(*chunk, {x, z}) < -5.0f) {
                    if (holes++ < 3) {
                        std::printf("    hole at %.1f, %.1f\n", x, z);
                    }
                }
            }
        }
        Check(holes == 0, Describe(coord) + " has no uncovered ground (" + std::to_string(holes) + " holes)");
    }
}

// --- Budgets and timing ----------------------------------------------------------------------------------------

void TestBudgetsAndTime() {
    std::cout << "box budget and time per chunk\n";
    size_t most = 0;
    size_t total = 0;
    double slowestMs = 0.0;
    double totalMs = 0.0;
    int count = 0;
    for (int z = -5; z < 5; ++z) {
        for (int x = -5; x < 5; ++x) {
            const Clock::time_point start = Clock::now();
            const auto chunk = world::GenerateChunk(Settings(), {x, z});
            const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
            most = std::max(most, chunk->layout.boxes.size());
            total += chunk->layout.boxes.size();
            slowestMs = std::max(slowestMs, ms);
            totalMs += ms;
            ++count;
        }
    }
    std::printf("    %d chunks: boxes average %zu, most %zu; time average %.2f ms, slowest %.2f ms (no crowd)\n", count,
                total / static_cast<size_t>(count), most, totalMs / count, slowestMs);
    Check(most <= static_cast<size_t>(kBoxBudgetPerChunk), "no chunk exceeds the box budget of " + std::to_string(kBoxBudgetPerChunk));
    Check(totalMs / count < kChunkTimeBudgetMs, "a chunk takes less than " + std::to_string(kChunkTimeBudgetMs) + " ms on average");

    world::ChunkSettings withPeople = Settings();
    withPeople.agentCount = 280;
    double crowdTotalMs = 0.0;
    double crowdSlowestMs = 0.0;
    for (int i = 0; i < 12; ++i) {
        const Clock::time_point start = Clock::now();
        const auto chunk = world::GenerateChunk(withPeople, {i - 6, 3});
        const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        crowdTotalMs += ms;
        crowdSlowestMs = std::max(crowdSlowestMs, ms);
        Check(chunk->crowd && chunk->crowd->AgentCount() == 280, "the crowd of a chunk has the people asked for");
    }
    std::printf("    with a crowd of 280: time average %.2f ms, slowest %.2f ms\n", crowdTotalMs / 12, crowdSlowestMs);
}

// --- Order and threads ---------------------------------------------------------------------------------------

std::vector<uint64_t> FingerprintsOf(const std::vector<ChunkCoord>& order, unsigned threads) {
    std::vector<uint64_t> fingerprints(order.size());
    std::vector<std::thread> workers;
    for (unsigned t = 0; t < threads; ++t) {
        workers.emplace_back([&, t] {
            for (size_t i = t; i < order.size(); i += threads) {
                fingerprints[i] = FingerprintOf(*world::GenerateChunk(Settings(), order[i]));
            }
        });
    }
    for (std::thread& worker : workers) {
        worker.join();
    }
    return fingerprints;
}

void TestOrderAndThreadsDoNotMatter() {
    std::cout << "generation order and thread count\n";
    std::vector<ChunkCoord> forward;
    for (int z = -3; z <= 3; ++z) {
        for (int x = -3; x <= 3; ++x) {
            forward.push_back({x, z});
        }
    }
    std::vector<ChunkCoord> backward(forward.rbegin(), forward.rend());
    const std::vector<uint64_t> single = FingerprintsOf(forward, 1);
    const std::vector<uint64_t> reversed = FingerprintsOf(backward, 1);
    const std::vector<uint64_t> threaded = FingerprintsOf(forward, 6);
    for (size_t i = 0; i < forward.size(); ++i) {
        Check(single[i] == reversed[reversed.size() - 1 - i], Describe(forward[i]) + " is the same when generated in the reverse order");
        Check(single[i] == threaded[i], Describe(forward[i]) + " is the same when generated on six threads");
    }
}

// --- The renderer's CPU side: allocator, deferred release, culling -------------------------------------------------

void TestRangeAllocator() {
    std::cout << "range allocator\n";
    RangeAllocator allocator(100);
    const auto a = allocator.Allocate(30);
    const auto b = allocator.Allocate(20);
    const auto c = allocator.Allocate(10);
    Check(a && b && c && *a == 0 && *b == 30 && *c == 50, "ranges are handed out one after the other");
    Check(!allocator.Allocate(0).has_value(), "zero units cannot be allocated");
    Check(!allocator.Allocate(41).has_value(), "a request longer than the free range fails");

    // Best fit: the smaller hole is used so that the large one stays whole.
    allocator.Free(*a, 30);
    allocator.Free(*c, 10);  // holes of 30 at 0 and 50 (10 + the 40 behind it)
    const auto fit = allocator.Allocate(8);
    Check(fit && *fit == 0, "the smallest hole that fits is used");
    allocator.Free(*fit, 8);
    allocator.Free(*b, 20);
    Check(allocator.FreeRangeCount() == 1 && allocator.LargestFree() == 100 && allocator.FreeTotal() == 100,
          "freed neighbours merge back into one range");

    // Random use against a model that marks every unit.
    constexpr uint32_t kCapacity = 4000;
    RangeAllocator pool(kCapacity);
    std::vector<int> owner(kCapacity, -1);
    struct Live {
        uint32_t start;
        uint32_t count;
    };
    std::vector<Live> live;
    uint32_t state = 12345;
    auto next = [&state] {
        state = state * 1664525u + 1013904223u;
        return state >> 8;
    };
    bool overlap = false;
    for (int step = 0; step < 20000; ++step) {
        if (live.empty() || next() % 3 != 0) {
            const uint32_t count = 1 + next() % 120;
            if (const auto start = pool.Allocate(count)) {
                for (uint32_t i = *start; i < *start + count; ++i) {
                    overlap = overlap || owner[i] != -1;
                    owner[i] = step;
                }
                live.push_back({*start, count});
            }
        } else {
            const size_t pick = next() % live.size();
            for (uint32_t i = live[pick].start; i < live[pick].start + live[pick].count; ++i) {
                owner[i] = -1;
            }
            pool.Free(live[pick].start, live[pick].count);
            live.erase(live.begin() + static_cast<std::ptrdiff_t>(pick));
        }
    }
    uint32_t used = 0;
    for (const Live& range : live) {
        used += range.count;
    }
    Check(!overlap, "no two live ranges ever overlap");
    Check(pool.UsedTotal() == used, "the allocator counts the units in use as the model does");
    for (const Live& range : live) {
        pool.Free(range.start, range.count);
    }
    Check(pool.FreeRangeCount() == 1 && pool.LargestFree() == kCapacity, "everything freed leaves one whole range");
}

void TestRetirementWaitsForFramesInFlight() {
    std::cout << "deferred release\n";
    // Two frames in flight: when frame M starts, frames up to M - 2 are complete. A range removed while frame N is prepared
    // may be read by frame N - 1, so it must stay until frame N + 1 starts.
    RetirementQueue<int> queue;
    constexpr uint64_t kRemovedAt = 10;
    queue.Defer(7, kRemovedAt);
    uint64_t releasedAtFrame = 0;
    for (uint64_t frame = kRemovedAt; frame < kRemovedAt + 5 && releasedAtFrame == 0; ++frame) {
        const int64_t completed = static_cast<int64_t>(frame) - 2;
        if (!queue.Collect(completed).empty()) {
            releasedAtFrame = frame;
        }
    }
    Check(releasedAtFrame == kRemovedAt + 1, "a range removed during frame N is released when frame N + 1 starts");
    Check(queue.WaitingCount() == 0, "a released range leaves the queue");
    queue.Defer(1, 0);
    Check(queue.Collect(-1).size() == 1, "a range removed before the first frame needs no wait");
}

void TestChunkCulling() {
    std::cout << "chunk culling\n";
    const glm::mat4 projection = glm::perspective(glm::radians(60.0f), 16.0f / 9.0f, 0.1f, 1000.0f);
    const glm::mat4 view = glm::lookAt(glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const Frustum camera(projection * view);
    auto box = [](glm::vec3 centre, float half) { return std::make_pair(centre - half, centre + half); };
    auto seen = [&](glm::vec3 centre, float half) {
        const auto [low, high] = box(centre, half);
        return camera.ContainsBox(low, high);
    };
    Check(seen({0.0f, 0.0f, -50.0f}, 5.0f), "a box in front is seen");
    Check(!seen({0.0f, 0.0f, 50.0f}, 5.0f), "a box behind the camera is culled");
    Check(!seen({500.0f, 0.0f, -50.0f}, 5.0f), "a box far to the side is culled");
    Check(seen({60.0f, 0.0f, -50.0f}, 40.0f), "a big box that straddles the edge of the view is kept");
    Check(!seen({0.0f, 0.0f, -2000.0f}, 5.0f), "a box beyond the far plane is culled");
    Check(seen({0.0f, 0.0f, 0.0f}, 3.0f), "a box around the camera is kept");

    // The sun's view is an orthographic box; the same test serves.
    const glm::mat4 sunProjection = glm::orthoRH_ZO(-100.0f, 100.0f, -100.0f, 100.0f, 0.0f, 500.0f);
    const Frustum sun(sunProjection * view);
    Check(sun.ContainsBox({-5.0f, -5.0f, -100.0f}, {5.0f, 5.0f, -90.0f}), "a box inside the sun's view is kept");
    Check(!sun.ContainsBox({300.0f, -5.0f, -100.0f}, {310.0f, 5.0f, -90.0f}), "a box outside the sun's view is culled");

    std::vector<InstanceRange> ranges{{1, 0, 3}, {0, 10, 5}, {0, 0, 10}, {0, 40, 2}};
    MergeAdjacentRanges(ranges);
    Check(ranges.size() == 3 && ranges[0].page == 0 && ranges[0].first == 0 && ranges[0].count == 15 && ranges[1].first == 40 &&
              ranges[2].page == 1,
          "ranges that touch are one draw");
}

}  // namespace

int main() {
    TestRangeAllocator();
    TestRetirementWaitsForFramesInFlight();
    TestChunkCulling();
    TestSkeletonIsPure();
    TestRiverStaysInItsBand();
    TestChunkIsRepeatable();
    TestNeighboursShareStreets();
    TestNeighboursShareWater();
    TestNoHolesInTheGround();
    TestBudgetsAndTime();
    TestOrderAndThreadsDoNotMatter();

    std::cout << "streaming manager\n";
    failureCount += RunStreamingManagerTests();

    if (failureCount != 0) {
        std::cerr << failureCount << " check(s) failed\n";
        return 1;
    }
    std::cout << "All streaming tests passed\n";
    return 0;
}
