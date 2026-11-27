#pragma once

#include <cstdint>
#include <utility>
#include <vector>

// Things that the CPU has finished with but that frames already submitted to the GPU may still read, such as the range of a
// chunk that was unloaded. They wait here until the last frame that could use them has completed.
//
// A thing deferred while frame N is being prepared may be used by frame N - 1 (still in flight) but not by frame N, so it
// can go once frame N - 1 has completed. The renderer knows that frame M - 2 has completed when it has waited for the
// fence of frame M's slot, with two frames in flight, and passes that to Collect.
template <typename T>
class RetirementQueue {
public:
    // `frame` is the number of the frame being prepared, which will not use the thing.
    void Defer(T thing, uint64_t frame) { waiting.push_back({std::move(thing), frame}); }

    // Removes and returns the things no frame in flight can still use, given the newest frame known to be complete.
    // `completedFrame` is signed so that "no frame has completed yet" can be -1.
    std::vector<T> Collect(int64_t completedFrame) {
        std::vector<T> retired;
        std::vector<Entry> stillWaiting;
        for (Entry& entry : waiting) {
            const bool lastUserCompleted = completedFrame >= static_cast<int64_t>(entry.frame) - 1;
            if (lastUserCompleted) {
                retired.push_back(std::move(entry.thing));
            } else {
                stillWaiting.push_back(std::move(entry));
            }
        }
        waiting = std::move(stillWaiting);
        return retired;
    }

    size_t WaitingCount() const { return waiting.size(); }

private:
    struct Entry {
        T thing;
        uint64_t frame;
    };
    std::vector<Entry> waiting;
};
