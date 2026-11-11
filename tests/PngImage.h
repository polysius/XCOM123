#pragma once

#include <cstdint>
#include <string>
#include <vector>

// A small RGB image that can be saved as a PNG file. It exists so the city tests can draw a map
// without pulling in an image library.
class RgbImage {
public:
    RgbImage(int width, int height, uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0);

    int Width() const { return width; }
    int Height() const { return height; }

    // Pixels outside the image are ignored, so callers can draw shapes that cross the border.
    void Set(int x, int y, uint8_t red, uint8_t green, uint8_t blue);
    void FillRect(int x0, int y0, int x1, int y1, uint8_t red, uint8_t green, uint8_t blue);
    void DrawLine(int x0, int y0, int x1, int y1, uint8_t red, uint8_t green, uint8_t blue);

    // Writes the image as an 8-bit RGB PNG. Returns false if the file cannot be written.
    bool SavePng(const std::string& path) const;

private:
    int width;
    int height;
    std::vector<uint8_t> pixels;  // rows top to bottom, three bytes per pixel
};
