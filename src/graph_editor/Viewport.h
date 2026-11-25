#pragma once

#include <glm/glm.hpp>

// Which part of the canvas the editor window shows, and how large. Screen points are measured from the top-left corner of the
// editor window, in pixels; canvas points are where nodes are placed.
//
// Every conversion goes through `origin` (the canvas point at the top-left corner of the window) and `zoom` (pixels per
// canvas unit), so panning and zooming cannot drift apart from what hit tests see.

namespace graph_editor {

constexpr float kMinZoom = 0.2f;
constexpr float kMaxZoom = 2.5f;

struct Rect {
    glm::vec2 min{0.0f};
    glm::vec2 max{0.0f};

    glm::vec2 Size() const { return max - min; }
    bool Contains(glm::vec2 point) const { return point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y; }
    bool Overlaps(const Rect& other) const { return min.x <= other.max.x && max.x >= other.min.x && min.y <= other.max.y && max.y >= other.min.y; }
};

// A rectangle from any two opposite corners.
Rect RectFromCorners(glm::vec2 a, glm::vec2 b);

class Viewport {
public:
    glm::vec2 Origin() const { return origin; }
    float Zoom() const { return zoom; }

    glm::vec2 ToCanvas(glm::vec2 screen) const { return screen / zoom + origin; }
    glm::vec2 ToScreen(glm::vec2 canvas) const { return (canvas - origin) * zoom; }

    // Drags the canvas along with the pointer: a pointer that moved right by 10 pixels shows content that was 10 pixels left.
    void PanByScreen(glm::vec2 screenDelta) { origin -= screenDelta / zoom; }

    // Zooms in by a factor above 1 or out by one below it, keeping the canvas point under `screenPoint` where it is on screen,
    // so that the wheel zooms toward the pointer. The zoom stays within kMinZoom and kMaxZoom.
    void ZoomAt(glm::vec2 screenPoint, float factor);

    // Moves the view so that a canvas point is in the middle of a window of `screenSize`, at the same zoom.
    void CenterOn(glm::vec2 canvasPoint, glm::vec2 screenSize) { origin = canvasPoint - screenSize * 0.5f / zoom; }

    // Shows `area` of the canvas, as large as fits in a window of `screenSize` with a margin around it, and centred.
    void Frame(const Rect& area, glm::vec2 screenSize, float screenMargin = 40.0f);

private:
    glm::vec2 origin{0.0f};
    float zoom = 1.0f;
};

}  // namespace graph_editor
