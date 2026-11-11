#include "core/JobSystem.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <utility>

namespace {

// An idle worker keeps polling the queue this long before it goes to sleep. Work arrives in
// bursts (several ParallelFor calls back to back within a frame), and a polling worker picks
// up the next one at once, while waking a sleeping thread takes tens of microseconds. The
// window is short so idle workers do not burn a laptop's power budget between frames.
constexpr std::chrono::microseconds kIdleSpinTime(100);

// ParallelFor aims for this many chunks per thread, so a thread that gets delayed (by the OS
// or by other programs) holds up only a small slice of the work.
constexpr size_t kChunksPerThread = 4;

// Shared by the threads that run one ParallelFor. Each thread claims chunks from nextChunk
// until none are left. Helper jobs keep this alive through a shared_ptr, because one may only
// start after the ParallelFor call has already returned; it then finds no chunk to claim and
// never touches body, which belongs to the caller.
struct ParallelForWork {
    ParallelForWork(size_t count, size_t chunkSize, const JobSystem::RangeFunction& body)
        : count(count), chunkSize(chunkSize), chunkCount((count + chunkSize - 1) / chunkSize), body(&body) {}

    void RunChunks() {
        for (;;) {
            const size_t chunk = nextChunk.fetch_add(1, std::memory_order_relaxed);
            if (chunk >= chunkCount) {
                return;
            }
            const size_t begin = chunk * chunkSize;
            (*body)(begin, std::min(begin + chunkSize, count));
            if (finishedChunks.fetch_add(1, std::memory_order_acq_rel) + 1 == chunkCount) {
                finishedChunks.notify_all();
            }
        }
    }

    void WaitUntilFinished() {
        size_t finished = finishedChunks.load(std::memory_order_acquire);
        while (finished != chunkCount) {
            finishedChunks.wait(finished, std::memory_order_acquire);
            finished = finishedChunks.load(std::memory_order_acquire);
        }
    }

    const size_t count;
    const size_t chunkSize;
    const size_t chunkCount;
    const JobSystem::RangeFunction* body;
    std::atomic<size_t> nextChunk{0};
    std::atomic<size_t> finishedChunks{0};
};

size_t ChooseChunkSize(size_t count, size_t minChunkSize, unsigned threadCount) {
    const size_t targetChunks = static_cast<size_t>(threadCount) * kChunksPerThread;
    const size_t balancedSize = (count + targetChunks - 1) / targetChunks;
    return std::max<size_t>({balancedSize, minChunkSize, 1});
}

}  // namespace

unsigned JobSystem::DefaultWorkerCount() {
    const unsigned hardwareThreads = std::thread::hardware_concurrency();
    return hardwareThreads > 1 ? hardwareThreads - 1 : 0;
}

JobSystem& JobSystem::Shared() {
    static JobSystem shared;
    return shared;
}

JobSystem::JobSystem(unsigned workerCount) {
    workers.reserve(workerCount);
    for (unsigned i = 0; i < workerCount; ++i) {
        workers.emplace_back([this] { WorkerLoop(); });
    }
}

JobSystem::~JobSystem() {
    {
        std::lock_guard lock(queueMutex);
        stopping = true;
    }
    jobAvailable.notify_all();
    // Workers finish every job still queued before they exit, so nothing submitted is lost.
    for (std::thread& worker : workers) {
        worker.join();
    }
}

void JobSystem::ParallelFor(size_t count, size_t minChunkSize, const RangeFunction& body) {
    if (count == 0) {
        return;
    }
    const size_t chunkSize = ChooseChunkSize(count, minChunkSize, ThreadCount());
    const size_t chunkCount = (count + chunkSize - 1) / chunkSize;
    if (chunkCount == 1 || workers.empty()) {
        body(0, count);
        return;
    }

    auto work = std::make_shared<ParallelForWork>(count, chunkSize, body);
    const size_t helperCount = std::min<size_t>(workers.size(), chunkCount - 1);
    std::vector<std::function<void()>> helpers;
    helpers.reserve(helperCount);
    for (size_t i = 0; i < helperCount; ++i) {
        helpers.emplace_back([work] { work->RunChunks(); });
    }
    PushJobs(std::move(helpers));

    work->RunChunks();
    work->WaitUntilFinished();
}

void JobSystem::Submit(JobGroup& group, std::function<void()> job) {
    group.pendingJobs.fetch_add(1, std::memory_order_relaxed);
    std::vector<std::function<void()>> jobs;
    jobs.emplace_back([&group, job = std::move(job)] {
        job();
        if (group.pendingJobs.fetch_sub(1, std::memory_order_acq_rel) == 1) {
            group.pendingJobs.notify_all();
        }
    });
    if (workers.empty()) {
        // Without workers nobody else would ever run it, so run it right here.
        jobs.front()();
        return;
    }
    PushJobs(std::move(jobs));
}

void JobSystem::Wait(JobGroup& group) {
    std::function<void()> job;
    while (!group.IsDone()) {
        if (TryTakeJob(job)) {
            job();
            continue;
        }
        const uint32_t pending = group.pendingJobs.load(std::memory_order_acquire);
        if (pending != 0) {
            group.pendingJobs.wait(pending, std::memory_order_acquire);
        }
    }
}

void JobSystem::WorkerLoop() {
    std::function<void()> job;
    while (WaitForJob(job)) {
        job();
    }
}

// Waits until a job is available and takes it. Returns false when the pool is shutting down
// and the queue is empty.
bool JobSystem::WaitForJob(std::function<void()>& job) {
    const auto spinEnd = std::chrono::steady_clock::now() + kIdleSpinTime;
    while (std::chrono::steady_clock::now() < spinEnd) {
        if (queuedJobCount.load(std::memory_order_relaxed) != 0 && TryTakeJob(job)) {
            return true;
        }
        std::this_thread::yield();
    }

    std::unique_lock lock(queueMutex);
    jobAvailable.wait(lock, [this] { return stopping || !queue.empty(); });
    if (queue.empty()) {
        return false;
    }
    job = std::move(queue.front());
    queue.pop_front();
    queuedJobCount.store(queue.size(), std::memory_order_relaxed);
    return true;
}

bool JobSystem::TryTakeJob(std::function<void()>& job) {
    std::lock_guard lock(queueMutex);
    if (queue.empty()) {
        return false;
    }
    job = std::move(queue.front());
    queue.pop_front();
    queuedJobCount.store(queue.size(), std::memory_order_relaxed);
    return true;
}

void JobSystem::PushJobs(std::vector<std::function<void()>>&& jobs) {
    {
        std::lock_guard lock(queueMutex);
        for (std::function<void()>& job : jobs) {
            queue.push_back(std::move(job));
        }
        queuedJobCount.store(queue.size(), std::memory_order_relaxed);
    }
    if (jobs.size() == 1) {
        jobAvailable.notify_one();
    } else {
        jobAvailable.notify_all();
    }
}
