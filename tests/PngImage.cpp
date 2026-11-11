#include "PngImage.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>

namespace {

uint32_t Crc32(const uint8_t* data, size_t size, uint32_t crc = 0) {
    crc = ~crc;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

uint32_t Adler32(const std::vector<uint8_t>& data) {
    uint32_t a = 1;
    uint32_t b = 0;
    for (uint8_t byte : data) {
        a = (a + byte) % 65521u;
        b = (b + a) % 65521u;
    }
    return (b << 16) | a;
}

// Writes a deflate stream one bit at a time, least significant bit first as the format requires.
class BitWriter {
public:
    void Bits(uint32_t value, int count) {
        for (int i = 0; i < count; ++i) {
            Bit((value >> i) & 1u);
        }
    }

    // Huffman codes are stored most significant bit first.
    void Code(uint32_t code, int length) {
        for (int i = length - 1; i >= 0; --i) {
            Bit((code >> i) & 1u);
        }
    }

    std::vector<uint8_t> Finish() {
        if (bitCount > 0) {
            bytes.push_back(current);
        }
        return std::move(bytes);
    }

private:
    void Bit(uint32_t bit) {
        current = static_cast<uint8_t>(current | (bit << bitCount));
        if (++bitCount == 8) {
            bytes.push_back(current);
            current = 0;
            bitCount = 0;
        }
    }

    std::vector<uint8_t> bytes;
    uint8_t current = 0;
    int bitCount = 0;
};

// Writes a literal or length symbol with the fixed Huffman code table of deflate.
void FixedSymbol(BitWriter& writer, int symbol) {
    if (symbol < 144) {
        writer.Code(0x30u + static_cast<uint32_t>(symbol), 8);
    } else if (symbol < 256) {
        writer.Code(0x190u + static_cast<uint32_t>(symbol - 144), 9);
    } else if (symbol < 280) {
        writer.Code(static_cast<uint32_t>(symbol - 256), 7);
    } else {
        writer.Code(0xC0u + static_cast<uint32_t>(symbol - 280), 8);
    }
}

// Writes a match of `length` bytes copied from one byte back (a run of the previous byte).
void RunOfPreviousByte(BitWriter& writer, int length) {
    static const int kBase[] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31,
                                35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
    static const int kExtraBits[] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2,
                                     3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
    int code = 28;
    while (kBase[code] > length) {
        --code;
    }
    FixedSymbol(writer, 257 + code);
    writer.Bits(static_cast<uint32_t>(length - kBase[code]), kExtraBits[code]);
    writer.Code(0, 5);  // distance code 0 means a distance of one byte
}

// Compresses with one fixed-Huffman block that encodes runs of equal bytes as matches. Maps are
// mostly flat colour, so after the Sub filter this already shrinks them a lot.
std::vector<uint8_t> Deflate(const std::vector<uint8_t>& data) {
    BitWriter writer;
    writer.Bits(1, 1);  // the final block
    writer.Bits(1, 2);  // compressed with the fixed codes
    size_t i = 0;
    while (i < data.size()) {
        FixedSymbol(writer, data[i]);
        size_t run = 0;
        while (i + 1 + run < data.size() && data[i + 1 + run] == data[i] && run < 258) {
            ++run;
        }
        if (run >= 3) {
            RunOfPreviousByte(writer, static_cast<int>(run));
            i += 1 + run;
        } else {
            ++i;
        }
    }
    FixedSymbol(writer, 256);  // end of block
    return writer.Finish();
}

void AppendBigEndian(std::vector<uint8_t>& out, uint32_t value) {
    out.push_back(static_cast<uint8_t>(value >> 24));
    out.push_back(static_cast<uint8_t>(value >> 16));
    out.push_back(static_cast<uint8_t>(value >> 8));
    out.push_back(static_cast<uint8_t>(value));
}

void AppendChunk(std::vector<uint8_t>& out, const char* type, const std::vector<uint8_t>& data) {
    AppendBigEndian(out, static_cast<uint32_t>(data.size()));
    const size_t typeStart = out.size();
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), data.begin(), data.end());
    AppendBigEndian(out, Crc32(out.data() + typeStart, out.size() - typeStart));
}

}  // namespace

RgbImage::RgbImage(int width, int height, uint8_t red, uint8_t green, uint8_t blue)
    : width(width), height(height), pixels(static_cast<size_t>(width) * static_cast<size_t>(height) * 3) {
    for (size_t i = 0; i < pixels.size(); i += 3) {
        pixels[i] = red;
        pixels[i + 1] = green;
        pixels[i + 2] = blue;
    }
}

void RgbImage::Set(int x, int y, uint8_t red, uint8_t green, uint8_t blue) {
    if (x < 0 || y < 0 || x >= width || y >= height) {
        return;
    }
    const size_t index = (static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 3;
    pixels[index] = red;
    pixels[index + 1] = green;
    pixels[index + 2] = blue;
}

void RgbImage::FillRect(int x0, int y0, int x1, int y1, uint8_t red, uint8_t green, uint8_t blue) {
    for (int y = std::max(0, y0); y <= std::min(height - 1, y1); ++y) {
        for (int x = std::max(0, x0); x <= std::min(width - 1, x1); ++x) {
            Set(x, y, red, green, blue);
        }
    }
}

void RgbImage::DrawLine(int x0, int y0, int x1, int y1, uint8_t red, uint8_t green, uint8_t blue) {
    // Bresenham's line algorithm.
    const int dx = std::abs(x1 - x0);
    const int dy = -std::abs(y1 - y0);
    const int stepX = x0 < x1 ? 1 : -1;
    const int stepY = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (true) {
        Set(x0, y0, red, green, blue);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int doubled = 2 * error;
        if (doubled >= dy) {
            error += dy;
            x0 += stepX;
        }
        if (doubled <= dx) {
            error += dx;
            y0 += stepY;
        }
    }
}

bool RgbImage::SavePng(const std::string& path) const {
    // Each row starts with filter type 1 (Sub): bytes store the difference to the pixel on the left.
    const size_t rowBytes = static_cast<size_t>(width) * 3;
    std::vector<uint8_t> filtered;
    filtered.reserve((rowBytes + 1) * static_cast<size_t>(height));
    for (int y = 0; y < height; ++y) {
        const uint8_t* row = pixels.data() + static_cast<size_t>(y) * rowBytes;
        filtered.push_back(1);
        for (size_t i = 0; i < rowBytes; ++i) {
            const uint8_t left = i >= 3 ? row[i - 3] : 0;
            filtered.push_back(static_cast<uint8_t>(row[i] - left));
        }
    }

    std::vector<uint8_t> zlib = {0x78, 0x01};
    const std::vector<uint8_t> compressed = Deflate(filtered);
    zlib.insert(zlib.end(), compressed.begin(), compressed.end());
    AppendBigEndian(zlib, Adler32(filtered));

    std::vector<uint8_t> header;
    AppendBigEndian(header, static_cast<uint32_t>(width));
    AppendBigEndian(header, static_cast<uint32_t>(height));
    header.insert(header.end(), {8, 2, 0, 0, 0});  // 8 bits per channel, RGB, no interlacing

    std::vector<uint8_t> file = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    AppendChunk(file, "IHDR", header);
    AppendChunk(file, "IDAT", zlib);
    AppendChunk(file, "IEND", {});

    std::ofstream stream(path, std::ios::binary);
    stream.write(reinterpret_cast<const char*>(file.data()), static_cast<std::streamsize>(file.size()));
    return static_cast<bool>(stream);
}
