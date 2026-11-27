#include "streaming/Reaper.h"

#include <utility>

namespace streaming {

Reaper::Reaper() : thread([this] { Run(); }) {}

Reaper::~Reaper() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        stopping = true;
    }
    wake.notify_one();
    thread.join();
}

void Reaper::Retire(std::shared_ptr<const void> thing) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        waiting.push_back(std::move(thing));
    }
    wake.notify_one();
}

size_t Reaper::Pending() const {
    std::lock_guard<std::mutex> lock(mutex);
    return waiting.size() + beingFreed;
}

void Reaper::Run() {
    std::unique_lock<std::mutex> lock(mutex);
    while (true) {
        wake.wait(lock, [this] { return stopping || !waiting.empty(); });
        std::vector<std::shared_ptr<const void>> doomed;
        doomed.swap(waiting);
        beingFreed = doomed.size();
        lock.unlock();
        doomed.clear();  // the destructors run here, away from the lock and from the frame
        lock.lock();
        beingFreed = 0;
        if (stopping && waiting.empty()) {
            return;
        }
    }
}

}  // namespace streaming
