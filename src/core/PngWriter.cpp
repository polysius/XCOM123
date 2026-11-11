#include "core/PngWriter.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <vector>

namespace {
constexpr uint32_t kRgbaBytesPerPixel = 4;
constexpr uint32_t kRgbBytesPerPixel = 3;
// A deflate stored block holds at most 65535 bytes.
constexpr size_t kMaxStoredBlockBytes = 65535;

void AppendBigEndian32(std::vector<uint8_t>& out, uint32_t value) {
    out.push_back(static_cast<uint8_t>(value >> 24));
    out.push_back(static_cast<uint8_t>(value >> 16));
    out.push_back(static_cast<uint8_t>(value >> 8));
    out.push_back(static_cast<uint8_t>(value));
}

// The CRC-32 that PNG uses on every chunk (polynomial 0xEDB88320, as in zip and gzip).
uint32_t Crc32(std::span<const uint8_t> bytes) {
    static const std::array<uint32_t, 256> table = [] {
        std::array<uint32_t, 256> entries{};
        for (uint32_t index = 0; index < 256; ++index) {
            uint32_t value = index;
            for (int bit = 0; bit < 8; ++bit) {
                value = (value & 1u) ? 0xEDB88320u ^ (value >> 1) : value >> 1;
            }
            entries[index] = value;
        }
        return entries;
    }();

    uint32_t crc = 0xFFFFFFFFu;
    for (const uint8_t byte : bytes) {
        crc = table[(crc ^ byte) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

// The Adler-32 checksum that ends a zlib stream.
uint32_t Adler32(std::span<const uint8_t> bytes) {
    constexpr uint32_t kModulus = 65521;
    uint32_t low = 1;
    uint32_t high = 0;
    for (const uint8_t byte : bytes) {
        low = (low + byte) % kModulus;
        high = (high + low) % kModulus;
    }
    return (high << 16) | low;
}

// A PNG chunk: length, four-letter type, data, and a CRC over the type and the data.
void AppendChunk(std::vector<uint8_t>& file, const char (&type)[5], std::span<const uint8_t> data) {
    AppendBigEndian32(file, static_cast<uint32_t>(data.size()));
    const size_t typeStart = file.size();
    file.insert(file.end(), type, type + 4);
    file.insert(file.end(), data.begin(), data.end());
    const std::span<const uint8_t> typeAndData(file.data() + typeStart, 4 + data.size());
    AppendBigEndian32(file, Crc32(typeAndData));
}

// The image rows as PNG expects them before compression: each row starts with a filter-type
// byte (0, no filter), followed by RGB pixels.
std::vector<uint8_t> BuildScanlines(uint32_t width, uint32_t height, std::span<const uint8_t> rgba) {
    const size_t rowBytes = 1 + static_cast<size_t>(width) * kRgbBytesPerPixel;
    std::vector<uint8_t> scanlines;
    scanlines.reserve(rowBytes * height);
    for (uint32_t y = 0; y < height; ++y) {
        scanlines.push_back(0);
        const uint8_t* row = rgba.data() + static_cast<size_t>(y) * width * kRgbaBytesPerPixel;
        for (uint32_t x = 0; x < width; ++x) {
            const uint8_t* pixel = row + static_cast<size_t>(x) * kRgbaBytesPerPixel;
            scanlines.insert(scanlines.end(), pixel, pixel + kRgbBytesPerPixel);
        }
    }
    return scanlines;
}

// Wraps raw bytes in a zlib stream made of uncompressed deflate blocks.
std::vector<uint8_t> ZlibStore(std::span<const uint8_t> raw) {
    std::vector<uint8_t> stream;
    stream.reserve(raw.size() + raw.size() / kMaxStoredBlockBytes * 5 + 16);
    // zlib header: deflate with a 32 KB window, no preset dictionary, "fastest" level. The two
    // bytes read as a big-endian number divisible by 31, as the format requires.
    stream.push_back(0x78);
    stream.push_back(0x01);

    size_t offset = 0;
    do {
        const size_t blockBytes = std::min(kMaxStoredBlockBytes, raw.size() - offset);
        const bool isLastBlock = offset + blockBytes == raw.size();
        // Block header: the final-block bit, then block type 00 (stored), padded to a byte.
        stream.push_back(isLastBlock ? 1 : 0);
        // Block length and its one's complement, both little-endian.
        const auto length = static_cast<uint16_t>(blockBytes);
        const auto inverted = static_cast<uint16_t>(~length);
        stream.push_back(static_cast<uint8_t>(length));
        stream.push_back(static_cast<uint8_t>(length >> 8));
        stream.push_back(static_cast<uint8_t>(inverted));
        stream.push_back(static_cast<uint8_t>(inverted >> 8));
        stream.insert(stream.end(), raw.begin() + static_cast<std::ptrdiff_t>(offset),
                      raw.begin() + static_cast<std::ptrdiff_t>(offset + blockBytes));
        offset += blockBytes;
    } while (offset < raw.size());

    AppendBigEndian32(stream, Adler32(raw));
    return stream;
}
}  // namespace

bool WritePng(const std::string& path, uint32_t width, uint32_t height, std::span<const uint8_t> rgba) {
    const size_t expectedBytes = static_cast<size_t>(width) * height * kRgbaBytesPerPixel;
    if (width == 0 || height == 0 || rgba.size() < expectedBytes) {
        std::cerr << "PNG: image data does not match " << width << "x" << height << "\n";
        return false;
    }

    std::vector<uint8_t> file = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};

    // Header: size, 8 bits per channel, colour type 2 (RGB), default compression and filtering,
    // no interlacing.
    std::vector<uint8_t> header;
    AppendBigEndian32(header, width);
    AppendBigEndian32(header, height);
    header.insert(header.end(), {8, 2, 0, 0, 0});
    AppendChunk(file, "IHDR", header);

    AppendChunk(file, "IDAT", ZlibStore(BuildScanlines(width, height, rgba)));
    AppendChunk(file, "IEND", {});

    std::ofstream out(path, std::ios::binary);
    if (!out) {
        std::cerr << "PNG: cannot open " << path << " for writing\n";
        return false;
    }
    out.write(reinterpret_cast<const char*>(file.data()), static_cast<std::streamsize>(file.size()));
    return static_cast<bool>(out);
}
