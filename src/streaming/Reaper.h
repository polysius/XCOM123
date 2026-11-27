#pragma once

#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace streaming {

// Frees things on a thread of its own. The frame loop hands over the last reference it holds to something big (a chunk with
// its boxes and its crowd) and carries on; the memory is released here, so freeing never costs a frame time. If another
// thread still holds a reference, that thread may be the one that frees it, which is why every holder of a streamed chunk
// either lives on a worker or retires its reference through a Reaper.
class Reaper {
public:
    Reaper();
    // Frees everything still waiting, then stops.
    ~Reaper();

    Reaper(const Reaper&) = delete;
    Reaper& operator=(const Reaper&) = delete;

    // Takes the reference and returns at once; it never frees anything itself.
    void Retire(std::shared_ptr<const void> thing);

    // How many things are waiting or being freed right now.
    size_t Pending() const;

private:
    void Run();

    mutable std::mutex mutex;
    std::condition_variable wake;
    std::vector<std::shared_ptr<const void>> waiting;
    size_t beingFreed = 0;
    bool stopping = false;
    std::thread thread;
};

}  // namespace streaming
