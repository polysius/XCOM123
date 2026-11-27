#include "streaming/StreamingManager.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <utility>

namespace streaming {

namespace {

// How strongly chunks in the direction of travel are preferred. A chunk straight ahead looks 40 percent closer than it is.
constexpr double kAheadBias = 0.4;
// Below this speed (metres per second) the player has no direction.
constexpr float kMinDirectionSpeed = 0.01f;
// The chunk under the player beats every other chunk, whatever the direction of travel.
constexpr double kPlayerChunkPriority = -1.0;

ChunkCoord ChunkAt(glm::vec2 point, float chunkSize) {
    return {static_cast<int32_t>(std::floor(point.x / chunkSize)), static_cast<int32_t>(std::floor(point.y / chunkSize))};
}

glm::vec2 ChunkCentre(ChunkCoord coord, float chunkSize) {
    return {(static_cast<float>(coord.x) + 0.5f) * chunkSize, (static_cast<float>(coord.z) + 0.5f) * chunkSize};
}

// Distance from a point to the nearest point of the chunk; zero inside it.
float DistanceToChunk(ChunkCoord coord, glm::vec2 point, float chunkSize) {
    const glm::vec2 low(static_cast<float>(coord.x) * chunkSize, static_cast<float>(coord.z) * chunkSize);
    const glm::vec2 high = low + glm::vec2(chunkSize);
    const glm::vec2 nearest = glm::clamp(point, low, high);
    return glm::length(point - nearest);
}

void AddChunksWithin(glm::vec2 point, float radius, float chunkSize,
                     std::unordered_set<ChunkCoord, ChunkCoordHash>& coords) {
    const ChunkCoord first = ChunkAt(point - glm::vec2(radius), chunkSize);
    const ChunkCoord last = ChunkAt(point + glm::vec2(radius), chunkSize);
    for (int32_t x = first.x; x <= last.x; ++x) {
        for (int32_t z = first.z; z <= last.z; ++z) {
            if (DistanceToChunk({x, z}, point, chunkSize) <= radius) {
                coords.insert({x, z});
            }
        }
    }
}

StreamingSettings Sanitized(StreamingSettings settings) {
    settings.chunkSize = std::max(settings.chunkSize, 1.0f);
    settings.unloadRadius = std::max(settings.unloadRadius, settings.loadRadius);
    return settings;
}

}  // namespace

double SteadyClockSeconds() {
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

double StreamingStats::LatencyPercentile(double fraction) const {
    if (latencies.empty()) {
        return 0.0;
    }
    std::vector<double> sorted = latencies;
    std::sort(sorted.begin(), sorted.end());
    const double clamped = std::clamp(fraction, 0.0, 1.0);
    const size_t rank = static_cast<size_t>(std::llround(clamped * static_cast<double>(sorted.size() - 1)));
    return sorted[rank];
}

StreamingManager::StreamingManager(StreamingSettings settings, GeneratorFunction generator, ClockFunction clock)
    : settings_(Sanitized(settings)), generator_(std::move(generator)), clock_(std::move(clock)) {
    StartWorkers();
}

StreamingManager::~StreamingManager() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    workAvailable_.notify_all();
    for (std::thread& worker : workers_) {
        worker.join();
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// What is wanted
// ---------------------------------------------------------------------------------------------------------------------

bool StreamingManager::IsWanted(const Interest& interest, ChunkCoord coord) const {
    return DistanceToChunk(coord, interest.position, settings_.chunkSize) <= settings_.loadRadius ||
           DistanceToChunk(coord, interest.predicted, settings_.chunkSize) <= settings_.prefetchRadius;
}

// Everything wanted is kept, because unloadRadius is never below loadRadius.
bool StreamingManager::IsKept(const Interest& interest, ChunkCoord coord) const {
    return DistanceToChunk(coord, interest.position, settings_.chunkSize) <= settings_.unloadRadius ||
           DistanceToChunk(coord, interest.predicted, settings_.chunkSize) <= settings_.prefetchRadius;
}

// Distance to the centre of the chunk, made smaller for chunks in the direction of travel so that they are generated first
// when distances are similar.
double StreamingManager::Priority(const Interest& interest, ChunkCoord coord) const {
    if (ChunkAt(interest.position, settings_.chunkSize) == coord) {
        return kPlayerChunkPriority;
    }
    const glm::vec2 toChunk = ChunkCentre(coord, settings_.chunkSize) - interest.position;
    const float distance = glm::length(toChunk);
    const float speed = glm::length(interest.velocity);
    if (speed < kMinDirectionSpeed || distance <= 0.0f) {
        return distance;
    }
    const double cosine = glm::dot(interest.velocity / speed, toChunk / distance);
    return distance * (1.0 - kAheadBias * std::max(0.0, cosine));
}

std::vector<ChunkCoord> StreamingManager::WantedCoords(const Interest& interest) const {
    std::unordered_set<ChunkCoord, ChunkCoordHash> unique;
    AddChunksWithin(interest.position, settings_.loadRadius, settings_.chunkSize, unique);
    AddChunksWithin(interest.predicted, settings_.prefetchRadius, settings_.chunkSize, unique);
    std::vector<ChunkCoord> coords(unique.begin(), unique.end());
    std::sort(coords.begin(), coords.end());
    return coords;
}

// ---------------------------------------------------------------------------------------------------------------------
// Workers
// ---------------------------------------------------------------------------------------------------------------------

void StreamingManager::StartWorkers() {
    for (unsigned i = 0; i < settings_.workerThreads; ++i) {
        workers_.emplace_back([this] { WorkerLoop(); });
    }
}

void StreamingManager::WorkerLoop() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        workAvailable_.wait(lock, [this] { return stopping_ || !queue_.empty() || !graveyard_.empty(); });
        if (stopping_) {
            return;
        }
        if (!graveyard_.empty()) {
            DestroyGraveyard(lock);
        } else {
            GenerateNext(lock);
        }
    }
}

void StreamingManager::GenerateNext(std::unique_lock<std::mutex>& lock) {
    auto best = queue_.begin();
    for (auto it = queue_.begin(); it != queue_.end(); ++it) {
        const bool moreUrgent = it->second.priority < best->second.priority;
        const bool tie = it->second.priority == best->second.priority;
        if (moreUrgent || (tie && it->first < best->first)) {
            best = it;
        }
    }
    const ChunkCoord coord = best->first;
    const double requestTime = best->second.requestTime;
    queue_.erase(best);
    inFlight_.emplace(coord, requestTime);

    lock.unlock();
    std::shared_ptr<const ChunkPayload> payload = RunGenerator(coord);
    lock.lock();

    // The result and the end of the flight change together, so no observer sees a payload that is neither in flight nor ready.
    inFlight_.erase(coord);
    if (payload) {
        ++stats_.generated;
        ready_[coord] = ReadyChunk{std::move(payload), requestTime};
    } else {
        ++stats_.failed;
        failed_.insert(coord);
    }
    if (queue_.empty() && inFlight_.empty()) {
        idle_.notify_all();
    }
}

std::shared_ptr<const ChunkPayload> StreamingManager::RunGenerator(ChunkCoord coord) const {
    try {
        return generator_(coord);
    } catch (...) {
        // A throwing generator must not take a worker thread, and with it the process, down.
        return nullptr;
    }
}

void StreamingManager::DestroyGraveyard(std::unique_lock<std::mutex>& lock) {
    std::vector<std::shared_ptr<const ChunkPayload>> dead;
    dead.swap(graveyard_);
    destroyingChunks_ += dead.size();
    const size_t count = dead.size();

    lock.unlock();
    dead.clear();
    lock.lock();

    destroyingChunks_ -= count;
}

void StreamingManager::WaitUntilIdle() {
    if (workers_.empty()) {
        return;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    idle_.wait(lock, [this] { return queue_.empty() && inFlight_.empty(); });
}

// ---------------------------------------------------------------------------------------------------------------------
// Update
// ---------------------------------------------------------------------------------------------------------------------

StreamingFrame StreamingManager::Update(glm::vec2 position, glm::vec2 velocity) {
    const double now = clock_();
    Interest interest;
    interest.position = position;
    interest.velocity = velocity;
    interest.predicted = position + velocity * settings_.lookAheadSeconds;

    StreamingFrame frame;
    std::unique_lock<std::mutex> lock(mutex_);
    const bool synchronous = workers_.empty();
    if (synchronous) {
        DestroyGraveyard(lock);
    }

    const bool teleported =
        hasPreviousPosition_ && glm::length(position - previousPosition_) > settings_.teleportDistance;
    if (teleported) {
        DropTeleportedWork();
    }
    hasPreviousPosition_ = true;
    previousPosition_ = position;

    ReplanQueue(interest, now);
    DiscardUnwantedResults(interest);
    UnloadLeftBehind(interest, frame);

    if (synchronous) {
        for (uint32_t i = 0; i < settings_.uploadChunksPerFrame && !queue_.empty(); ++i) {
            GenerateNext(lock);
        }
    }
    DeliverReady(interest, now, frame);

    lock.unlock();
    workAvailable_.notify_all();
    return frame;
}

// After a jump nothing that was planned for the old place matters. Chunks being generated right now finish and are judged
// by the normal rules when they arrive.
void StreamingManager::DropTeleportedWork() {
    stats_.cancelled += queue_.size() + ready_.size();
    queue_.clear();
    for (auto& entry : ready_) {
        Bury(std::move(entry.second.payload));
    }
    ready_.clear();
    ++stats_.teleports;
}

bool StreamingManager::AlreadyHandled(ChunkCoord coord) const {
    return resident_.count(coord) > 0 || queue_.count(coord) > 0 || inFlight_.count(coord) > 0 ||
           ready_.count(coord) > 0 || failed_.count(coord) > 0;
}

// Brings the queue in line with what is wanted now: forgets requests nobody wants any more, re-scores the rest (the player
// has moved) and adds new ones. A request keeps the time it was first queued.
void StreamingManager::ReplanQueue(const Interest& interest, double now) {
    for (auto it = queue_.begin(); it != queue_.end();) {
        if (IsWanted(interest, it->first)) {
            it->second.priority = Priority(interest, it->first);
            ++it;
        } else {
            ++stats_.cancelled;
            it = queue_.erase(it);
        }
    }

    // A failed chunk stays failed while the player is near it, so that it is not retried every frame.
    for (auto it = failed_.begin(); it != failed_.end();) {
        it = IsWanted(interest, *it) ? std::next(it) : failed_.erase(it);
    }

    for (const ChunkCoord& coord : WantedCoords(interest)) {
        if (!AlreadyHandled(coord)) {
            queue_[coord] = QueuedRequest{Priority(interest, coord), now};
            ++stats_.requested;
        }
    }
}

void StreamingManager::DiscardUnwantedResults(const Interest& interest) {
    for (auto it = ready_.begin(); it != ready_.end();) {
        if (IsKept(interest, it->first)) {
            ++it;
        } else {
            ++stats_.cancelled;
            Bury(std::move(it->second.payload));
            it = ready_.erase(it);
        }
    }
}

void StreamingManager::UnloadLeftBehind(const Interest& interest, StreamingFrame& frame) {
    for (auto it = resident_.begin(); it != resident_.end();) {
        auto next = std::next(it);
        if (!IsKept(interest, it->first)) {
            Unload(it, frame);
        }
        it = next;
    }
}

void StreamingManager::DeliverReady(const Interest& interest, double now, StreamingFrame& frame) {
    std::vector<std::pair<float, ChunkCoord>> order;
    for (const auto& entry : ready_) {
        order.emplace_back(DistanceToChunk(entry.first, interest.position, settings_.chunkSize), entry.first);
    }
    std::sort(order.begin(), order.end());

    size_t uploadedBytes = 0;
    for (const auto& candidate : order) {
        const ChunkCoord coord = candidate.second;
        auto readyEntry = ready_.find(coord);
        const size_t uploadBytes = readyEntry->second.payload->UploadBytes();

        // The first chunk of a frame is exempt from the budget, so that a chunk bigger than the budget still gets through.
        if (!frame.delivered.empty()) {
            const bool tooMany = frame.delivered.size() >= settings_.uploadChunksPerFrame;
            const bool tooManyBytes = uploadedBytes + uploadBytes > settings_.uploadBytesPerFrame;
            const bool tooSlow = settings_.uploadSecondsPerFrame > 0.0 &&
                                 clock_() - now >= settings_.uploadSecondsPerFrame;
            if (tooMany || tooManyBytes || tooSlow) {
                break;
            }
        }
        const size_t residentBytes = readyEntry->second.payload->ResidentBytes();
        if (!MakeRoom(interest, coord, residentBytes, frame)) {
            break;
        }

        ReadyChunk ready = std::move(readyEntry->second);
        ready_.erase(readyEntry);
        RecordDelivery(coord, std::move(ready), now, frame);
        uploadedBytes += uploadBytes;
    }

    stats_.deliveredBytesLastFrame = uploadedBytes;
    stats_.deliveredBytesMaxFrame = std::max(stats_.deliveredBytesMaxFrame, uploadedBytes);
}

// Makes space for one more resident chunk by unloading chunks that lie beyond the load radius, farthest first. A chunk within
// the load radius is never evicted, and neither is one nearer than the incoming chunk (that would only swap two chunks back
// and forth). Nothing is unloaded unless that is enough: returns false and changes nothing if the chunk cannot fit.
bool StreamingManager::MakeRoom(const Interest& interest, ChunkCoord incoming, size_t incomingBytes,
                                StreamingFrame& frame) {
    size_t count = resident_.size();
    size_t bytes = residentBytes_;
    const auto fits = [&] {
        return count + 1 <= settings_.maxResidentChunks && bytes + incomingBytes <= settings_.memoryBudgetBytes;
    };
    if (fits()) {
        return true;
    }

    const float incomingDistance = DistanceToChunk(incoming, interest.position, settings_.chunkSize);
    std::vector<std::pair<float, ChunkCoord>> candidates;
    for (const auto& entry : resident_) {
        const float distance = DistanceToChunk(entry.first, interest.position, settings_.chunkSize);
        if (distance > settings_.loadRadius && distance > incomingDistance) {
            candidates.emplace_back(distance, entry.first);
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
        return a.first != b.first ? a.first > b.first : a.second < b.second;
    });

    size_t victimCount = 0;
    while (!fits() && victimCount < candidates.size()) {
        --count;
        bytes -= resident_.at(candidates[victimCount].second).bytes;
        ++victimCount;
    }
    if (!fits()) {
        return false;
    }
    for (size_t i = 0; i < victimCount; ++i) {
        Unload(resident_.find(candidates[i].second), frame);
    }
    return true;
}

void StreamingManager::Unload(std::map<ChunkCoord, ResidentEntry>::iterator entry, StreamingFrame& frame) {
    frame.unloaded.push_back(entry->first);
    residentBytes_ -= entry->second.bytes;
    Bury(std::move(entry->second.payload));
    resident_.erase(entry);
    ++stats_.unloaded;
}

// The payload may be big, so it is destroyed by a worker (or by the next Update in synchronous mode), not on this frame.
void StreamingManager::Bury(std::shared_ptr<const ChunkPayload> payload) {
    graveyard_.push_back(std::move(payload));
}

void StreamingManager::RecordDelivery(ChunkCoord coord, ReadyChunk ready, double now, StreamingFrame& frame) {
    ResidentEntry entry;
    entry.payload = ready.payload;
    entry.bytes = ready.payload->ResidentBytes();
    residentBytes_ += entry.bytes;
    resident_.emplace(coord, std::move(entry));

    frame.delivered.push_back({coord, std::move(ready.payload)});
    stats_.latencies.push_back(std::max(0.0, now - ready.requestTime));
    ++stats_.delivered;
    stats_.residentChunksPeak = std::max(stats_.residentChunksPeak, resident_.size());
    stats_.residentBytesPeak = std::max(stats_.residentBytesPeak, residentBytes_);
}

// ---------------------------------------------------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------------------------------------------------

std::vector<ChunkCoord> StreamingManager::ResidentCoords() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<ChunkCoord> coords;
    coords.reserve(resident_.size());
    for (const auto& entry : resident_) {
        coords.push_back(entry.first);
    }
    return coords;
}

bool StreamingManager::IsResident(ChunkCoord coord) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return resident_.count(coord) > 0;
}

StreamingStats StreamingManager::Stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    StreamingStats stats = stats_;
    stats.residentChunks = resident_.size();
    stats.residentBytes = residentBytes_;
    stats.queuedChunks = queue_.size();
    stats.inFlightChunks = inFlight_.size();
    stats.readyChunks = ready_.size();
    stats.pendingDestroyChunks = graveyard_.size() + destroyingChunks_;
    return stats;
}

}  // namespace streaming
