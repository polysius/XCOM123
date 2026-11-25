#include "graph_editor/Viewport.h"

#include <algorithm>

namespace graph_editor {

Rect RectFromCorners(glm::vec2 a, glm::vec2 b) {
    return {glm::min(a, b), glm::max(a, b)};
}

void Viewport::ZoomAt(glm::vec2 screenPoint, float factor) {
    const glm::vec2 canvasUnderPointer = ToCanvas(screenPoint);
    zoom = std::clamp(zoom * factor, kMinZoom, kMaxZoom);
    origin = canvasUnderPointer - screenPoint / zoom;
}

void Viewport::Frame(const Rect& area, glm::vec2 screenSize, float screenMargin) {
    const glm::vec2 room = glm::max(screenSize - 2.0f * glm::vec2(screenMargin), glm::vec2(1.0f));
    const glm::vec2 size = glm::max(area.Size(), glm::vec2(1.0f));
    zoom = std::clamp(std::min(room.x / size.x, room.y / size.y), kMinZoom, 1.0f);
    const glm::vec2 centre = (area.min + area.max) * 0.5f;
    origin = centre - screenSize * 0.5f / zoom;
}

}  // namespace graph_editor
