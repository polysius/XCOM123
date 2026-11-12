#include "world/CityInteriors.h"

#include <algorithm>
#include <cmath>

namespace city {
namespace {

constexpr float kFloorBottom = 0.1f;       // the floor slab starts below the pavement so no gap shows at the doorway
constexpr float kCeilingThickness = 0.35f;
constexpr float kPewRowPitch = 1.7f;       // distance between the backs of two pews
constexpr float kPewSeatDepth = 0.45f;
constexpr float kPewBackDepth = 0.1f;
constexpr float kPewSeatHeight = 0.45f;
constexpr float kPewBackHeight = 1.0f;
constexpr float kCandleHalfSize = 0.06f;

void Block(BoxWriter& boxes, const LocalFrame& frame, float u0, float u1, float v0, float v1, float bottom, float top,
           const Look& look) {
    boxes.Footprint(frame.Area(u0, u1, v0, v1), bottom, top, look);
}

// One end wall of a room, from `v0` to `v1` between the side walls, with an optional doorway in the middle.
void BuildEndWall(BoxWriter& boxes, const LocalFrame& frame, float u0, float u1, float v0, float v1, float height,
                  const std::optional<Doorway>& door, const Look& wall) {
    if (!door) {
        Block(boxes, frame, u0, u1, v0, v1, 0.0f, height, wall);
        return;
    }
    const float halfWidth = std::min(door->halfWidth, (v1 - v0) * 0.5f - 0.5f);
    Block(boxes, frame, u0, u1, v0, -halfWidth, 0.0f, height, wall);
    Block(boxes, frame, u0, u1, halfWidth, v1, 0.0f, height, wall);
    Block(boxes, frame, u0, u1, -halfWidth, halfWidth, door->height, height, wall);
}

// The slab that carries the floor through the thickness of an end wall's doorway.
void BuildThreshold(BoxWriter& boxes, const LocalFrame& frame, float u0, float u1, const Doorway& door, const Look& floor) {
    Block(boxes, frame, u0, u1, -door.halfWidth, door.halfWidth, kFloorBottom, kPlazaHeight, floor);
}

}  // namespace

Rect RoomFloor(const LocalFrame& frame, const RoomSpec& room) {
    const float t = room.wallThickness;
    return frame.Area(room.uFront + t, room.uBack - t, room.vLeft + t, room.vRight - t);
}

void BuildRoomWalls(BoxWriter& boxes, const LocalFrame& frame, const RoomSpec& room, const Look& wall) {
    const float t = room.wallThickness;
    // The side walls run the whole length, and the end walls fit between them so that no two
    // boxes share a face.
    Block(boxes, frame, room.uFront, room.uBack, room.vLeft, room.vLeft + t, 0.0f, room.height, wall);
    Block(boxes, frame, room.uFront, room.uBack, room.vRight - t, room.vRight, 0.0f, room.height, wall);
    BuildEndWall(boxes, frame, room.uFront, room.uFront + t, room.vLeft + t, room.vRight - t, room.height, room.frontDoor, wall);
    BuildEndWall(boxes, frame, room.uBack - t, room.uBack, room.vLeft + t, room.vRight - t, room.height, room.backDoor, wall);
}

void BuildRoomFloor(BoxWriter& boxes, const LocalFrame& frame, const RoomSpec& room, const Look& floor) {
    const float t = room.wallThickness;
    Block(boxes, frame, room.uFront + t, room.uBack - t, room.vLeft + t, room.vRight - t, kFloorBottom, kPlazaHeight, floor);
    if (room.frontDoor) {
        BuildThreshold(boxes, frame, room.uFront, room.uFront + t, *room.frontDoor, floor);
    }
    if (room.backDoor) {
        BuildThreshold(boxes, frame, room.uBack - t, room.uBack, *room.backDoor, floor);
    }
}

void BuildRoomCeiling(BoxWriter& boxes, const LocalFrame& frame, const RoomSpec& room, const Look& ceiling) {
    const float t = room.wallThickness;
    Block(boxes, frame, room.uFront + t, room.uBack - t, room.vLeft + t, room.vRight - t, room.height - kCeilingThickness,
          room.height, ceiling);
}

void AddPews(BoxWriter& boxes, const LocalFrame& frame, const RoomSpec& room, const PewLayout& layout, const Look& wood) {
    const float halfWidth = (room.vRight - room.vLeft) * 0.5f - room.wallThickness;
    const float innerEdge = layout.centreAisle * 0.5f;
    const float outerEdge = halfWidth - layout.sideAisle;
    if (outerEdge <= innerEdge) {
        return;
    }
    int rowsInBlock = 0;
    for (float u = layout.uFirst; u + kPewSeatDepth + kPewBackDepth <= layout.uLast; u += kPewRowPitch) {
        if (rowsInBlock == layout.rowsPerBlock) {
            rowsInBlock = 0;  // a walkway across the rows
            continue;
        }
        ++rowsInBlock;
        for (float side : {-1.0f, 1.0f}) {
            const float v0 = side * innerEdge;
            const float v1 = side * outerEdge;
            // The back faces the pew behind it, and the seat faces the altar.
            Block(boxes, frame, u, u + kPewBackDepth, v0, v1, kFloorBottom, kPewBackHeight, wood);
            Block(boxes, frame, u + kPewBackDepth, u + kPewBackDepth + kPewSeatDepth, v0, v1, kFloorBottom, kPewSeatHeight, wood);
        }
    }
}

void AddAltar(BoxWriter& boxes, const LocalFrame& frame, float u, const Look& stone) {
    constexpr float kHalfDepth = 0.55f;
    constexpr float kHalfLength = 1.3f;
    constexpr float kHeight = 1.1f;
    Block(boxes, frame, u - kHalfDepth, u + kHalfDepth, -kHalfLength, kHalfLength, kFloorBottom, kHeight, stone);
    const Look candle{{1.0f, 0.82f, 0.55f}, SurfaceMaterial::Lamp, 0.3f, 0.0f};
    for (float side : {-1.0f, 1.0f}) {
        const glm::vec2 at = frame.Point(u, side * (kHalfLength - 0.35f));
        boxes.Footprint({at - glm::vec2(kCandleHalfSize), at + glm::vec2(kCandleHalfSize)}, kHeight, kHeight + 0.3f, candle);
    }
}

}  // namespace city
