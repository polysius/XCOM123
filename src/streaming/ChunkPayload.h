#pragma once

#include <cstddef>

namespace streaming {

// What a generator makes for one chunk, as far as the streaming manager is concerned: something with a size. The manager
// never looks inside; the code that consumes finished chunks knows the real type and casts to it.
class ChunkPayload {
public:
    virtual ~ChunkPayload() = default;

    // Bytes the payload keeps in memory while the chunk is resident. Counted against the memory budget.
    virtual size_t ResidentBytes() const = 0;
    // Bytes that go to the GPU when the chunk is delivered. Counted against the per-frame upload budget.
    virtual size_t UploadBytes() const = 0;
};

}  // namespace streaming
