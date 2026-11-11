#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

// Tracks a batch of jobs handed to JobSystem::Submit so the caller can wait for all of them.
class JobGroup {
public:
    bool IsDone() const { return pendingJobs.load(std::memory_order_acquire) == 0; }

private:
    friend class JobSystem;
    std::atomic<uint32_t> pendingJobs{0};
};

// A fixed pool of worker threads, sized to the CPU, that runs short jobs.
//
// The main use is ParallelFor, which splits an index range into chunks and runs them on the
// workers and the calling thread together. The calling thread always takes part, so a
// ParallelFor issued from inside a job cannot deadlock, and a pool without workers simply
// runs everything on the caller.
//
// Jobs must not throw: an exception escaping a job ends the program.
class JobSystem {
public:
    // Processes one chunk of a ParallelFor: the half-open index range [begin, end).
    using RangeFunction = std::function<void(size_t begin, size_t end)>;

    // One worker per hardware thread, minus one for the thread that issues the work.
    static unsigned DefaultWorkerCount();

    // A process-wide pool that systems share, created on first use and shut down at exit.
    static JobSystem& Shared();

    explicit JobSystem(unsigned workerCount = DefaultWorkerCount());
    ~JobSystem();

    JobSystem(const JobSystem&) = delete;
    JobSystem& operator=(const JobSystem&) = delete;

    unsigned WorkerCount() const { return static_cast<unsigned>(workers.size()); }

    // Threads that run ParallelFor chunks: the workers plus the calling thread.
    unsigned ThreadCount() const { return WorkerCount() + 1; }

    // Runs body over [0, count) split into chunks of at least minChunkSize indices and returns
    // once every chunk has finished. Chunks run in any order and on any thread, so body must
    // only write data that belongs to its own range.
    void ParallelFor(size_t count, size_t minChunkSize, const RangeFunction& body);

    // Queues one job to run on a worker. Wait(group) returns once all of the group's jobs ran.
    void Submit(JobGroup& group, std::function<void()> job);

    // Blocks until every job submitted with this group has finished, running queued jobs on
    // this thread in the meantime so the wait does not waste a core.
    void Wait(JobGroup& group);

private:
    void WorkerLoop();
    bool WaitForJob(std::function<void()>& job);
    bool TryTakeJob(std::function<void()>& job);
    void PushJobs(std::vector<std::function<void()>>&& jobs);

    std::vector<std::thread> workers;
    std::mutex queueMutex;
    std::condition_variable jobAvailable;
    std::deque<std::function<void()>> queue;
    // Mirrors queue.size() so idle workers can poll without taking the lock.
    std::atomic<size_t> queuedJobCount{0};
    bool stopping = false;
};
