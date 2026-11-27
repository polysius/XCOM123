// Tests of the chunk streaming manager (no window, no GPU). A fake generator makes payloads that only have a size, and a fake
// clock lets the tests decide what time it is. Like the other test programs there is no test framework: each test is a
// function that calls Check, and RunStreamingManagerTests reports every failure it finds and returns how many there were.
//
// Most tests run the manager in synchronous mode (no worker threads), where the outcome of every frame is exact. The tests that
// are about threads (budgeted delivery, leaks, failures, the stress test) use workers and call WaitUntilIdle to make the
// timing repeatable.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "streaming/StreamingManager.h"

namespace {

using namespace streaming;

int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

// A payload that only has a size. It counts the payloads alive, so that tests can see that nothing leaks.
class FakePayload : public ChunkPayload {
public:
    FakePayload(size_t residentBytes, size_t uploadBytes) : residentBytes_(residentBytes), uploadBytes_(uploadBytes) {
        ++liveCount;
    }
    ~FakePayload() override { --liveCount; }

    size_t ResidentBytes() const override { return residentBytes_; }
    size_t UploadBytes() const override { return uploadBytes_; }

    static std::atomic<int> liveCount;

private:
    size_t residentBytes_;
    size_t uploadBytes_;
};

std::atomic<int> FakePayload::liveCount{0};

GeneratorFunction SizedGenerator(size_t residentBytes, size_t uploadBytes) {
    return [=](ChunkCoord) { return std::make_shared<FakePayload>(residentBytes, uploadBytes); };
}

GeneratorFunction SmallGenerator() {
    return SizedGenerator(1000, 1000);
}

// Remembers the order in which chunks were asked for. Only meaningful in synchronous mode, where there is one caller.
struct GenerationLog {
    std::mutex mutex;
    std::vector<ChunkCoord> order;
};

GeneratorFunction LoggingGenerator(GenerationLog& log) {
    return [&log](ChunkCoord coord) {
        std::lock_guard<std::mutex> lock(log.mutex);
        log.order.push_back(coord);
        return std::make_shared<FakePayload>(1000, 1000);
    };
}

// Settings with room for everything, so that a test only meets the limit it is about. A player at (128, 128) wants the 3x3
// chunks around chunk (0, 0).
StreamingSettings SyncSettings() {
    StreamingSettings settings;
    settings.chunkSize = 256.0f;
    settings.loadRadius = 300.0f;
    settings.unloadRadius = 400.0f;
    settings.prefetchRadius = 0.0f;
    settings.teleportDistance = 5000.0f;
    settings.maxResidentChunks = 1000;
    settings.memoryBudgetBytes = size_t(1) << 40;
    settings.uploadBytesPerFrame = size_t(1) << 40;
    settings.uploadChunksPerFrame = 1000;
    settings.workerThreads = 0;
    return settings;
}

const glm::vec2 kCentreOfOrigin(128.0f, 128.0f);
const glm::vec2 kStanding(0.0f, 0.0f);

bool Contains(const std::vector<ChunkCoord>& coords, ChunkCoord coord) {
    return std::find(coords.begin(), coords.end(), coord) != coords.end();
}

size_t IndexOf(const std::vector<ChunkCoord>& coords, ChunkCoord coord) {
    return static_cast<size_t>(std::find(coords.begin(), coords.end(), coord) - coords.begin());
}

std::vector<ChunkCoord> DeliveredCoords(const StreamingFrame& frame) {
    std::vector<ChunkCoord> coords;
    for (const ResidentChunk& chunk : frame.delivered) {
        coords.push_back(chunk.coord);
    }
    return coords;
}

// ---------------------------------------------------------------------------------------------------------------------

void TestOrdering() {
    {
        GenerationLog log;
        StreamingSettings settings = SyncSettings();
        settings.uploadChunksPerFrame = 1;
        StreamingManager manager(settings, LoggingGenerator(log));
        for (int frame = 0; frame < 12; ++frame) {
            manager.Update(kCentreOfOrigin, kStanding);
        }
        Check(log.order.size() == 9, "standing player: the 3x3 chunks around the player are generated");
        if (log.order.size() == 9) {
            Check(log.order[0] == ChunkCoord({0, 0}), "standing player: the chunk under the player comes first");
            for (size_t i = 1; i <= 4; ++i) {
                const ChunkCoord c = log.order[i];
                Check(std::abs(c.x) + std::abs(c.z) == 1, "standing player: the four edge neighbours come before the corners");
            }
            for (size_t i = 5; i <= 8; ++i) {
                const ChunkCoord c = log.order[i];
                Check(std::abs(c.x) + std::abs(c.z) == 2, "standing player: the four corners come last");
            }
        }
    }
    {
        GenerationLog log;
        StreamingSettings settings = SyncSettings();
        settings.uploadChunksPerFrame = 1;
        StreamingManager manager(settings, LoggingGenerator(log));
        for (int frame = 0; frame < 12; ++frame) {
            manager.Update(kCentreOfOrigin, glm::vec2(10.0f, 0.0f));
        }
        Check(log.order.size() == 9, "moving player: the 3x3 chunks around the player are generated");
        if (log.order.size() == 9) {
            Check(log.order[0] == ChunkCoord({0, 0}), "moving player: the chunk under the player comes first");
            Check(log.order[1] == ChunkCoord({1, 0}), "moving player: the chunk straight ahead comes next");
            Check(IndexOf(log.order, {1, 0}) < IndexOf(log.order, {-1, 0}),
                  "moving player: a chunk ahead comes before an equally distant chunk behind");
            Check(IndexOf(log.order, {1, 1}) < IndexOf(log.order, {-1, 1}),
                  "moving player: a corner ahead comes before the equally distant corner behind");
        }
    }
}

void TestCancelsStaleRequests() {
    StreamingSettings settings = SyncSettings();
    settings.prefetchRadius = 300.0f;
    settings.uploadChunksPerFrame = 1;
    StreamingManager manager(settings, SmallGenerator());

    // Walking east, the chunks at x = 2 are prefetched. Only one chunk per frame is made, so most requests are still queued
    // when the player turns around.
    manager.Update(kCentreOfOrigin, glm::vec2(100.0f, 0.0f));
    Check(manager.Stats().cancelled == 0, "nothing is cancelled while the wanted set stays the same");
    manager.Update(kCentreOfOrigin, glm::vec2(-100.0f, 0.0f));
    Check(manager.Stats().cancelled >= 3, "after turning around the three queued chunks to the east are cancelled");

    for (int frame = 0; frame < 40; ++frame) {
        manager.Update(kCentreOfOrigin, glm::vec2(-100.0f, 0.0f));
    }
    Check(!manager.IsResident({2, 0}), "a chunk that was cancelled is not generated");
    Check(manager.IsResident({-2, 0}), "the chunks the player now walks towards are loaded");
}

void TestHysteresis() {
    StreamingSettings settings = SyncSettings();
    settings.loadRadius = 700.0f;
    settings.unloadRadius = 800.0f;
    StreamingManager manager(settings, SmallGenerator());

    // Chunk (5, 0) starts at x = 1280, so the player at x is 1280 - x away from it.
    const auto at = [](float x) { return glm::vec2(x, 128.0f); };
    manager.Update(at(570.0f), kStanding);  // 710 away: between the radii
    Check(!manager.IsResident({5, 0}), "a chunk between the load and unload radius is not loaded");
    manager.Update(at(590.0f), kStanding);  // 690 away
    Check(manager.IsResident({5, 0}), "a chunk inside the load radius is loaded");
    manager.Update(at(570.0f), kStanding);
    Check(manager.IsResident({5, 0}), "a chunk between the radii that is resident stays");

    // Walk back and forth over the border. After the first round trip nothing may be loaded or unloaded again.
    const auto walkRoundTrip = [&] {
        for (int x = 570; x <= 590; ++x) {
            manager.Update(at(static_cast<float>(x)), kStanding);
        }
        for (int x = 590; x >= 570; --x) {
            manager.Update(at(static_cast<float>(x)), kStanding);
        }
    };
    walkRoundTrip();
    const StreamingStats before = manager.Stats();
    for (int round = 0; round < 20; ++round) {
        walkRoundTrip();
    }
    const StreamingStats after = manager.Stats();
    Check(after.requested == before.requested, "walking over the border requests nothing new");
    Check(after.delivered == before.delivered, "walking over the border delivers nothing new");
    Check(after.unloaded == before.unloaded, "walking over the border unloads nothing");
    Check(after.cancelled == before.cancelled, "walking over the border cancels nothing");
}

// Updates until all the chunks of the 3x3 neighbourhood are generated (the workers are idle), then keeps updating and records
// what each Update delivered. Returns the number of chunks delivered by each Update, starting with the first one.
std::vector<size_t> DeliveriesPerFrame(StreamingManager& manager, size_t expectedTotal) {
    std::vector<size_t> perFrame;
    size_t total = 0;
    for (int frame = 0; frame < 40 && total < expectedTotal; ++frame) {
        const StreamingFrame result = manager.Update(kCentreOfOrigin, kStanding);
        manager.WaitUntilIdle();
        perFrame.push_back(result.delivered.size());
        total += result.delivered.size();
    }
    return perFrame;
}

size_t MaxOf(const std::vector<size_t>& values) {
    return values.empty() ? 0 : *std::max_element(values.begin(), values.end());
}

void TestBudgetedDelivery() {
    {
        StreamingSettings settings = SyncSettings();
        settings.workerThreads = 2;
        settings.uploadBytesPerFrame = 2500;  // room for two chunks of 1000 upload bytes
        StreamingManager manager(settings, SmallGenerator());
        const std::vector<size_t> perFrame = DeliveriesPerFrame(manager, 9);
        Check(MaxOf(perFrame) == 2, "byte budget of 2500 lets two chunks of 1000 bytes through per frame");
        Check(manager.Stats().residentChunks == 9, "byte budget: everything is delivered in the end");
        Check(manager.Stats().deliveredBytesMaxFrame <= 2500, "byte budget: no frame uploads more than the budget");
    }
    {
        StreamingSettings settings = SyncSettings();
        settings.workerThreads = 2;
        settings.uploadChunksPerFrame = 3;
        StreamingManager manager(settings, SmallGenerator());
        const std::vector<size_t> perFrame = DeliveriesPerFrame(manager, 9);
        Check(MaxOf(perFrame) == 3, "count budget of 3 lets three chunks through per frame");
        Check(manager.Stats().residentChunks == 9, "count budget: everything is delivered in the end");
    }
    {
        StreamingSettings settings = SyncSettings();
        settings.workerThreads = 2;
        settings.uploadBytesPerFrame = 100;  // less than one chunk
        StreamingManager manager(settings, SizedGenerator(1000, 10000));
        const std::vector<size_t> perFrame = DeliveriesPerFrame(manager, 9);
        Check(manager.Stats().residentChunks == 9, "a chunk bigger than the byte budget is not starved");
        Check(MaxOf(perFrame) == 1, "a chunk bigger than the byte budget goes alone");
        size_t deliveringFrames = 0;
        for (size_t count : perFrame) {
            deliveringFrames += count > 0 ? 1 : 0;
        }
        Check(deliveringFrames == 9, "every frame with ready chunks delivers one, even over the byte budget");
    }
    {
        // The clock moves one second per reading. The Update reads it once to start, so after one delivery it is one second
        // into the frame, and after the second it is two: the 1.5 second limit stops delivery at two chunks.
        double fakeTime = 0.0;
        StreamingSettings settings = SyncSettings();
        settings.workerThreads = 2;
        settings.uploadSecondsPerFrame = 1.5;
        StreamingManager manager(settings, SmallGenerator(), [&fakeTime] { return fakeTime += 1.0; });
        const std::vector<size_t> perFrame = DeliveriesPerFrame(manager, 9);
        Check(MaxOf(perFrame) == 2, "time budget stops delivery once the frame has used its time");
        Check(manager.Stats().residentChunks == 9, "time budget: everything is delivered in the end");
    }
}

void TestTeleport() {
    StreamingSettings settings = SyncSettings();
    settings.uploadChunksPerFrame = 1;
    settings.teleportBoostFactor = 1;  // this test is about the order and the budget, without the boost after a jump
    StreamingManager manager(settings, SmallGenerator());

    manager.Update(kCentreOfOrigin, kStanding);
    Check(manager.Stats().queuedChunks == 8, "setup: eight chunks wait in the queue");

    const glm::vec2 farAway(100128.0f, 128.0f);
    const StreamingFrame frame = manager.Update(farAway, glm::vec2(100.0f, 0.0f));
    const StreamingStats stats = manager.Stats();
    Check(stats.teleports == 1, "a jump over the teleport distance is counted");
    Check(stats.cancelled == 8, "a teleport drops the whole queue");
    Check(Contains(frame.unloaded, {0, 0}), "the chunk left behind by a teleport is unloaded");
    Check(frame.delivered.size() == 1 && frame.delivered[0].coord == ChunkCoord({391, 0}),
          "after a teleport the chunk under the player is loaded first");
    Check(!manager.IsResident({0, 0}), "no old chunk survives a teleport far away");

    for (int i = 0; i < 12; ++i) {
        manager.Update(farAway, kStanding);
    }
    bool neighbourhoodLoaded = true;
    for (int x = 390; x <= 392; ++x) {
        for (int z = -1; z <= 1; ++z) {
            neighbourhoodLoaded = neighbourhoodLoaded && manager.IsResident({x, z});
        }
    }
    Check(neighbourhoodLoaded, "the neighbourhood of the new place is loaded");
    Check(manager.Stats().teleports == 1, "standing still is not a teleport");
}

// A fixed path: east at 150 metres per second with a wide sideways swing, so that chunks are loaded ahead and unloaded behind
// and the prefetch point swings from side to side. Returns the position and velocity of a frame at 60 frames per second.
std::pair<glm::vec2, glm::vec2> ScriptedPath(int frame) {
    const float t = static_cast<float>(frame) / 60.0f;
    const glm::vec2 position(150.0f * t, 800.0f * std::sin(0.5f * t));
    const glm::vec2 velocity(150.0f, 400.0f * std::cos(0.5f * t));
    return {position, velocity};
}

struct PathResult {
    std::vector<ChunkCoord> finalResident;
    std::vector<std::vector<ChunkCoord>> deliveredPerFrame;
    StreamingStats stats;
};

PathResult RunScriptedPath(unsigned workerThreads) {
    StreamingSettings settings = SyncSettings();
    settings.loadRadius = 700.0f;
    settings.unloadRadius = 800.0f;
    settings.prefetchRadius = 450.0f;
    settings.teleportDistance = 1200.0f;
    settings.uploadChunksPerFrame = 256;
    settings.workerThreads = workerThreads;
    StreamingManager manager(settings, SmallGenerator());

    PathResult result;
    const auto step = [&](glm::vec2 position, glm::vec2 velocity) {
        result.deliveredPerFrame.push_back(DeliveredCoords(manager.Update(position, velocity)));
        manager.WaitUntilIdle();
    };
    for (int frame = 0; frame < 600; ++frame) {
        const auto [position, velocity] = ScriptedPath(frame);
        step(position, velocity);
    }
    // Standing still at the end lets a threaded run catch up with a synchronous one.
    const glm::vec2 endPosition = ScriptedPath(599).first;
    for (int frame = 0; frame < 40; ++frame) {
        step(endPosition, kStanding);
    }
    result.finalResident = manager.ResidentCoords();
    result.stats = manager.Stats();
    return result;
}

void TestDeterminism() {
    const PathResult first = RunScriptedPath(0);
    const PathResult second = RunScriptedPath(0);
    const PathResult threaded = RunScriptedPath(3);

    Check(first.stats.unloaded > 0 && first.stats.delivered > 30, "the scripted path loads and unloads chunks");
    Check(first.finalResident == second.finalResident, "synchronous runs end with the same resident set");
    Check(first.deliveredPerFrame == second.deliveredPerFrame, "synchronous runs deliver the same chunks in every frame");
    Check(first.finalResident == threaded.finalResident, "a threaded run ends with the same resident set as a synchronous one");
    Check(!first.finalResident.empty(), "the resident set at the end of the path is not empty");
    Check(first.deliveredPerFrame[0].size() > 1, "the first frame loads the whole neighbourhood");
}

void TestMemoryCap() {
    {
        StreamingSettings settings = SyncSettings();
        settings.maxResidentChunks = 5;
        StreamingManager manager(settings, SmallGenerator());
        for (int frame = 0; frame < 10; ++frame) {
            manager.Update(kCentreOfOrigin, kStanding);
            Check(manager.Stats().residentChunks <= 5, "resident chunks never exceed the cap");
        }
        const std::vector<ChunkCoord> resident = manager.ResidentCoords();
        const std::vector<ChunkCoord> nearest = {{-1, 0}, {0, -1}, {0, 0}, {0, 1}, {1, 0}};
        Check(resident == nearest, "under a cap the nearest chunks are the ones that stay");
        Check(manager.Stats().readyChunks == 4, "chunks that do not fit wait in the ready queue");
    }
    {
        StreamingSettings settings = SyncSettings();
        settings.memoryBudgetBytes = 3500;  // three chunks of 1000
        StreamingManager manager(settings, SmallGenerator());
        for (int frame = 0; frame < 10; ++frame) {
            manager.Update(kCentreOfOrigin, kStanding);
            Check(manager.Stats().residentBytes <= 3500, "resident bytes never exceed the memory budget");
        }
        Check(manager.Stats().residentChunks == 3, "the memory budget holds three chunks of 1000 bytes");
        Check(manager.IsResident({0, 0}), "under a memory budget the chunk under the player stays");
        Check(manager.Stats().residentBytesPeak <= 3500, "the peak of resident bytes respects the budget");
    }
    {
        // The player walks 512 metres east. The old chunks are beyond the load radius but not beyond the (huge) unload radius,
        // so only the cap pushes them out, farthest first, to make room for the chunks the player now needs.
        StreamingSettings settings = SyncSettings();
        settings.maxResidentChunks = 9;
        settings.unloadRadius = 5000.0f;
        StreamingManager manager(settings, SmallGenerator());
        manager.Update(kCentreOfOrigin, kStanding);
        Check(manager.Stats().residentChunks == 9, "setup: nine chunks are resident");
        for (int frame = 0; frame < 5; ++frame) {
            manager.Update(glm::vec2(640.0f, 128.0f), kStanding);
            Check(manager.Stats().residentChunks <= 9, "the cap holds while chunks are swapped");
        }
        std::vector<ChunkCoord> wanted;
        for (int x = 1; x <= 3; ++x) {
            for (int z = -1; z <= 1; ++z) {
                wanted.push_back({x, z});
            }
        }
        Check(manager.ResidentCoords() == wanted, "chunks beyond the load radius make room for chunks within it");
    }
}

// Payloads alive must be explained by what the manager holds: resident, ready, in flight, or on the way to be destroyed.
void CheckLiveCountExplained(const StreamingManager& manager, const std::string& when) {
    const StreamingStats stats = manager.Stats();
    const int bound = static_cast<int>(stats.residentChunks + stats.readyChunks + stats.inFlightChunks +
                                       stats.pendingDestroyChunks);
    Check(FakePayload::liveCount.load() <= bound, "live payloads are all accounted for " + when);
}

void TestNoLeak() {
    Check(FakePayload::liveCount.load() == 0, "setup: no payload alive before the test");
    std::set<ChunkCoord> mirror;  // what a caller would know from the frames
    {
        StreamingSettings settings = SyncSettings();
        settings.loadRadius = 700.0f;
        settings.unloadRadius = 800.0f;
        settings.uploadChunksPerFrame = 3;
        settings.workerThreads = 2;
        StreamingManager manager(settings, SmallGenerator());

        for (int frame = 0; frame < 900; ++frame) {
            const StreamingFrame result = manager.Update(glm::vec2(3.3f * static_cast<float>(frame), 0.0f), glm::vec2(200.0f, 0.0f));
            for (ChunkCoord coord : result.unloaded) {
                mirror.erase(coord);
            }
            for (const ResidentChunk& chunk : result.delivered) {
                mirror.insert(chunk.coord);
            }
            manager.WaitUntilIdle();
            // Read the statistics first: from then on payloads can only be destroyed, never made, because nothing is queued.
            CheckLiveCountExplained(manager, "while streaming");
        }
        const std::vector<ChunkCoord> resident = manager.ResidentCoords();
        Check(std::set<ChunkCoord>(resident.begin(), resident.end()) == mirror,
              "the frames tell the caller exactly which chunks are resident");

        const StreamingStats stats = manager.Stats();
        Check(stats.delivered == stats.unloaded + stats.residentChunks, "every delivered chunk is resident or reported unloaded");
        Check(stats.unloaded > 50, "the walk unloads the chunks it leaves behind");
    }
    Check(FakePayload::liveCount.load() == 0, "no payload is alive after the manager is destroyed");
}

void TestLatencyPercentiles() {
    double fakeTime = 0.0;
    StreamingSettings settings = SyncSettings();
    settings.uploadChunksPerFrame = 1;
    StreamingManager manager(settings, SmallGenerator(), [&fakeTime] { return fakeTime; });

    // Nine chunks are requested at time 0 and delivered one per frame, a tenth of a second apart.
    for (int frame = 0; frame < 9; ++frame) {
        manager.Update(kCentreOfOrigin, kStanding);
        fakeTime += 0.1;
    }
    const StreamingStats stats = manager.Stats();
    Check(stats.latencies.size() == 9, "one latency sample per delivered chunk");
    Check(std::abs(stats.LatencyPercentile(0.0) - 0.0) < 1e-9, "the shortest latency is zero");
    Check(std::abs(stats.LatencyPercentile(0.5) - 0.4) < 1e-9, "the median latency is 0.4 seconds");
    Check(std::abs(stats.LatencyPercentile(1.0) - 0.8) < 1e-9, "the longest latency is 0.8 seconds");
    Check(StreamingStats().LatencyPercentile(0.5) == 0.0, "the percentile of no samples is zero");
}

void TestFailingGenerators() {
    // Chunks with odd x return null, chunks with even x and odd z throw. Of the 3x3 around the origin only (0, 0) works.
    const GeneratorFunction flaky = [](ChunkCoord coord) -> std::shared_ptr<const ChunkPayload> {
        if (coord.x % 2 != 0) {
            return nullptr;
        }
        if (coord.z % 2 != 0) {
            throw std::runtime_error("generator failure");
        }
        return std::make_shared<FakePayload>(1000, 1000);
    };

    for (unsigned workers : {0u, 2u}) {
        StreamingSettings settings = SyncSettings();
        settings.workerThreads = workers;
        settings.uploadChunksPerFrame = 2;
        {
            StreamingManager manager(settings, flaky);
            for (int frame = 0; frame < 30; ++frame) {
                manager.Update(kCentreOfOrigin, kStanding);
                manager.WaitUntilIdle();
            }
            const StreamingStats stats = manager.Stats();
            const std::string mode = workers == 0 ? "synchronous: " : "threaded: ";
            Check(stats.failed == 8, mode + "each failing chunk is tried once and not retried every frame");
            Check(stats.generated == 1, mode + "the working chunk is generated");
            Check(manager.ResidentCoords() == std::vector<ChunkCoord>({{0, 0}}), mode + "only the working chunk is resident");
            Check(stats.queuedChunks == 0, mode + "failed chunks leave nothing in the queue");
        }
        Check(FakePayload::liveCount.load() == 0, "failing generators leak nothing");
    }
}

// A generator that takes a random moment and now and then fails, to shake out races.
GeneratorFunction JitteryGenerator() {
    return [](ChunkCoord coord) -> std::shared_ptr<const ChunkPayload> {
        thread_local std::mt19937 random(static_cast<uint32_t>(std::hash<std::thread::id>()(std::this_thread::get_id())));
        const unsigned spins = random() % 40;
        for (unsigned i = 0; i < spins; ++i) {
            std::this_thread::yield();
        }
        const unsigned roll = random() % 100;
        if (roll == 0) {
            return nullptr;
        }
        if (roll == 1) {
            throw std::runtime_error("generator failure");
        }
        return std::make_shared<FakePayload>(1000 + static_cast<size_t>(coord.x & 7) * 100, 500);
    };
}

void RunStressSeed(uint32_t seed) {
    constexpr size_t kCap = 40;
    StreamingSettings settings = SyncSettings();
    settings.loadRadius = 300.0f;
    settings.unloadRadius = 400.0f;
    settings.prefetchRadius = 200.0f;
    settings.lookAheadSeconds = 1.0f;
    settings.teleportDistance = 1000.0f;
    settings.maxResidentChunks = kCap;
    settings.uploadChunksPerFrame = 4;
    settings.workerThreads = 4;

    std::mt19937 random(seed);
    std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
    glm::vec2 position(0.0f);
    glm::vec2 velocity(30.0f, 0.0f);
    std::set<ChunkCoord> mirror;
    const int failuresBefore = failureCount;

    {
        StreamingManager manager(settings, JitteryGenerator());
        for (int frame = 0; frame < 20000; ++frame) {
            velocity += glm::vec2(unit(random), unit(random)) * 4.0f;
            if (glm::length(velocity) > 80.0f) {
                velocity = glm::normalize(velocity) * 80.0f;
            }
            position += velocity / 60.0f;
            if (random() % 1500 == 0) {
                position += glm::vec2(unit(random), unit(random)) * 6000.0f;
            }

            const StreamingFrame result = manager.Update(position, velocity);

            std::set<ChunkCoord> delivered;
            for (const ResidentChunk& chunk : result.delivered) {
                Check(delivered.insert(chunk.coord).second, "a chunk is delivered once per frame");
                Check(mirror.count(chunk.coord) == 0, "a chunk that is delivered was not resident already");
                Check(chunk.payload != nullptr, "a delivered chunk has a payload");
            }
            for (ChunkCoord coord : result.unloaded) {
                Check(delivered.count(coord) == 0, "a chunk is not delivered and unloaded in the same frame");
                Check(mirror.erase(coord) == 1, "an unloaded chunk was resident");
            }
            mirror.insert(delivered.begin(), delivered.end());

            const std::vector<ChunkCoord> resident = manager.ResidentCoords();
            Check(resident.size() <= kCap, "resident chunks never exceed the cap");
            Check(resident == std::vector<ChunkCoord>(mirror.begin(), mirror.end()),
                  "the frames always add up to the resident set");
            if (failureCount > failuresBefore) {
                return;  // one broken invariant would repeat 20000 times
            }
        }
        manager.WaitUntilIdle();
    }
    Check(FakePayload::liveCount.load() == 0, "no payload is alive after the stress run");
}

void TestThreadSafetyStress() {
    const int failuresBefore = failureCount;
    for (uint32_t seed = 1; seed <= 10; ++seed) {
        RunStressSeed(seed);
        if (failureCount > failuresBefore) {
            return;
        }
    }
}

// Distance from a point to the nearest point of a chunk of 256 m.
float DistanceTo(ChunkCoord coord, glm::vec2 point) {
    const glm::vec2 low(static_cast<float>(coord.x) * 256.0f, static_cast<float>(coord.z) * 256.0f);
    return glm::length(point - glm::clamp(point, low, low + glm::vec2(256.0f)));
}

// A full memory cap makes room by unloading chunks, but never one that is within the load radius, even when a nearer chunk is
// waiting for the room: the chunks around the player must not be traded away for each other.
void TestMakeRoomNeverEvictsInsideLoadRadius() {
    StreamingSettings settings = SyncSettings();
    settings.maxResidentChunks = 5;
    StreamingManager manager(settings, SmallGenerator());

    manager.Update(kCentreOfOrigin, kStanding);
    Check(manager.Stats().residentChunks == 5, "setup: the cap of five chunks is full");

    // Move so that some resident chunks are behind (beyond the load radius) and some are still inside it but now farther than
    // chunks that are waiting for room.
    const glm::vec2 moved(378.0f, 128.0f);
    bool evictedInside = false;
    for (int frame = 0; frame < 5; ++frame) {
        const StreamingFrame result = manager.Update(moved, kStanding);
        for (const ChunkCoord& coord : result.unloaded) {
            evictedInside = evictedInside || DistanceTo(coord, moved) <= settings.loadRadius;
        }
        Check(manager.Stats().residentChunks <= 5, "the cap is kept");
    }
    Check(!evictedInside, "a chunk within the load radius is never unloaded to make room");
    Check(manager.IsResident({0, -1}) && manager.IsResident({0, 1}), "the chunks inside the radius stay, though nearer chunks wait");
}

// After a teleport everything around the new place is missing at once. The first frames deliver and generate more per frame,
// nearest first, so the gap closes in a few frames instead of a few seconds.
int FramesToFillAfterTeleport(uint32_t boostFactor) {
    StreamingSettings settings = SyncSettings();
    settings.loadRadius = 700.0f;
    settings.unloadRadius = 800.0f;
    settings.uploadChunksPerFrame = 2;
    settings.teleportBoostFactor = boostFactor;
    settings.teleportDistance = 1200.0f;
    StreamingManager manager(settings, SmallGenerator());
    for (int frame = 0; frame < 40; ++frame) {
        manager.Update({128.0f, 128.0f}, kStanding);
    }
    const glm::vec2 landing(20128.0f, 128.0f);
    // Everything within 0.75 of the load radius has to be there.
    std::vector<ChunkCoord> needed;
    for (int z = -4; z <= 4; ++z) {
        for (int x = 78; x <= 86; ++x) {
            if (DistanceTo({x, z}, landing) <= 0.75f * settings.loadRadius) {
                needed.push_back({x, z});
            }
        }
    }
    for (int frame = 1; frame <= 60; ++frame) {
        manager.Update(landing, kStanding);
        const bool all = std::all_of(needed.begin(), needed.end(), [&](ChunkCoord coord) { return manager.IsResident(coord); });
        if (all) {
            return frame;
        }
    }
    return 61;
}

void TestTeleportFillsQuickly() {
    const int boosted = FramesToFillAfterTeleport(8);
    const int plain = FramesToFillAfterTeleport(1);
    std::cout << "    after a teleport the " << "nearby chunks are all there after " << boosted << " frames with the boost, " << plain
              << " without\n";
    Check(boosted <= 4, "with the boost the chunks near the landing place are all resident within four frames");
    Check(plain > boosted * 2, "without the boost the same takes much longer, so the boost is what closes the gap");
}

void TestLatencySamplesAreBounded() {
    StreamingSettings settings = SyncSettings();
    settings.latencySamples = 16;
    settings.uploadChunksPerFrame = 1;
    double fakeTime = 0.0;
    StreamingManager manager(settings, SmallGenerator(), [&fakeTime] { return fakeTime; });
    for (int frame = 0; frame < 200; ++frame) {
        // The player walks on, so that new chunks keep arriving.
        manager.Update({128.0f + 40.0f * static_cast<float>(frame), 128.0f}, {40.0f, 0.0f});
        fakeTime += 0.1;
    }
    const StreamingStats stats = manager.Stats();
    Check(stats.delivered > 16, "setup: more chunks were delivered than the ring holds");
    Check(stats.latencies.size() == 16, "only the latest latency samples are kept");
}

}  // namespace

int RunStreamingManagerTests() {
    failureCount = 0;
    const std::vector<std::pair<std::string, void (*)()>> tests = {
        {"ordering", TestOrdering},
        {"cancels stale requests", TestCancelsStaleRequests},
        {"hysteresis", TestHysteresis},
        {"budgeted delivery", TestBudgetedDelivery},
        {"teleport", TestTeleport},
        {"determinism", TestDeterminism},
        {"memory cap", TestMemoryCap},
        {"no leak", TestNoLeak},
        {"latency percentiles", TestLatencyPercentiles},
        {"failing generators", TestFailingGenerators},
        {"make room keeps the load radius", TestMakeRoomNeverEvictsInsideLoadRadius},
        {"teleport fills quickly", TestTeleportFillsQuickly},
        {"latency samples are bounded", TestLatencySamplesAreBounded},
        {"thread safety stress", TestThreadSafetyStress},
    };
    for (const auto& [name, run] : tests) {
        const int failuresBefore = failureCount;
        const auto start = std::chrono::steady_clock::now();
        run();
        const auto milliseconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        std::cout << "streaming " << name << ": " << (failureCount == failuresBefore ? "ok" : "FAILED") << " (" << milliseconds
                  << " ms)\n";
    }
    return failureCount;
}
