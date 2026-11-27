#pragma once

#include <chrono>
#include <exception>
#include <functional>
#include <optional>
#include <string>
#include <utility>

#include "core/JobSystem.h"

// One piece of work that runs on the engine's job system while the frame goes on, and is collected once it is done. The panels use it
// for generators that take longer than a frame: they start it, show how long it has run, and poll for the result.
//
// Only one run at a time. The task waits for its job in the destructor, so what the job uses must outlive the task.

namespace panel_model {

template <typename Result>
struct TaskOutcome {
    std::optional<Result> value;  // empty when the work failed
    std::string error;            // why it failed, or empty
    double milliseconds = 0.0;    // how long the work took
};

template <typename Result>
class BackgroundTask {
public:
    BackgroundTask() = default;
    BackgroundTask(const BackgroundTask&) = delete;
    BackgroundTask& operator=(const BackgroundTask&) = delete;
    ~BackgroundTask() { WaitForJob(); }

    // Starts the work on the shared job system. Returns false, and does nothing, while the last run has not been collected.
    bool Start(std::function<Result()> work) {
        if (running) {
            return false;
        }
        outcome = TaskOutcome<Result>{};
        running = true;
        startedAt = Clock::now();
        // Jobs must not throw, so whatever the work throws is kept as the error.
        JobSystem::Shared().Submit(group, [this, work = std::move(work)]() {
            const Clock::time_point begin = Clock::now();
            try {
                outcome.value = work();
            } catch (const std::exception& error) {
                outcome.error = error.what();
            } catch (...) {
                outcome.error = "The work failed.";
            }
            outcome.milliseconds = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
        });
        return true;
    }

    // Whether the work has been started and not yet collected (it may have finished).
    bool IsRunning() const { return running; }

    // Seconds since the work started while it runs, and 0 otherwise.
    double SecondsRunning() const {
        return running ? std::chrono::duration<double>(Clock::now() - startedAt).count() : 0.0;
    }

    // The outcome, once the work has finished; empty while it is still running or when nothing was started. Gives each run's outcome once.
    std::optional<TaskOutcome<Result>> Poll() {
        if (!running || !group.IsDone()) {
            return std::nullopt;
        }
        running = false;
        return std::move(outcome);
    }

private:
    using Clock = std::chrono::steady_clock;

    void WaitForJob() {
        if (running) {
            JobSystem::Shared().Wait(group);
            running = false;
        }
    }

    JobGroup group;
    TaskOutcome<Result> outcome;
    bool running = false;
    Clock::time_point startedAt;
};

}  // namespace panel_model
