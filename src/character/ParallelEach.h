#pragma once

#include <cstddef>

#include "core/JobSystem.h"

namespace character {

// Runs body(index) for every index in [0, count), on the job system when there is one. The body must only write data
// that belongs to its own index.
template <typename Body>
void ParallelEach(JobSystem* jobs, int count, const Body& body) {
    const auto run = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            body(static_cast<int>(i));
        }
    };
    if (jobs != nullptr) {
        jobs->ParallelFor(static_cast<size_t>(count), 2, run);
    } else {
        run(0, static_cast<size_t>(count));
    }
}

}  // namespace character
