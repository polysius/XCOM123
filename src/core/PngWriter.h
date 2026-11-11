#pragma once

#include <cstdint>
#include <span>
#include <string>

// Writes an 8-bit RGBA image (rows top to bottom, 4 bytes per pixel) as a PNG file.
//
// The image data is stored without compression (deflate "stored" blocks), which keeps the
// writer small and dependency-free. Files are larger than usual but open in any image viewer.
// The alpha channel is dropped: screenshots are opaque.
bool WritePng(const std::string& path, uint32_t width, uint32_t height, std::span<const uint8_t> rgba);
