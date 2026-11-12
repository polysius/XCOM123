#pragma once

// A small kit for the rooms inside landmarks: hollow walls with doorways, a stone floor, a ceiling,
// and furniture laid out on a grid of tiles. Internal to src/world.
//
// Rooms are described in the frame of the landmark (see CityFrame.h): u runs from the front of the
// landmark to its back and v across it. A room is a rectangle in those coordinates with a wall of
// the same thickness all round, and each of its two end walls may have a doorway in the middle.

#include <optional>

#include "world/CityFrame.h"

namespace city {

struct Doorway {
    float halfWidth;
    float height;
};

struct RoomSpec {
    float uFront;
    float uBack;
    float vLeft;   // the smaller v
    float vRight;
    float height;  // of the walls, from the ground
    float wallThickness = 0.8f;
    std::optional<Doorway> frontDoor;  // in the wall at uFront
    std::optional<Doorway> backDoor;   // in the wall at uBack
};

// The floor of the room inside its walls, in the world.
Rect RoomFloor(const LocalFrame& frame, const RoomSpec& room);

// The four walls of the room with their doorways, from the ground up to `room.height`.
void BuildRoomWalls(BoxWriter& boxes, const LocalFrame& frame, const RoomSpec& room, const Look& wall);

// A stone floor covering the room inside its walls, and a ceiling slab under the top of the walls.
void BuildRoomFloor(BoxWriter& boxes, const LocalFrame& frame, const RoomSpec& room, const Look& floor);
void BuildRoomCeiling(BoxWriter& boxes, const LocalFrame& frame, const RoomSpec& room, const Look& ceiling);

// Rows of pews on both sides of a central aisle. The pews fill the floor from `uFirst` to `uLast`,
// leave `centreAisle` metres clear down the middle and `sideAisle` metres along each wall. A
// walkway across the rows is left every `rowsPerBlock` rows.
struct PewLayout {
    float uFirst;
    float uLast;
    float centreAisle;
    float sideAisle;
    int rowsPerBlock;
};
void AddPews(BoxWriter& boxes, const LocalFrame& frame, const RoomSpec& room, const PewLayout& layout, const Look& wood);

// A stone altar table with two candles, standing at (u, 0) and facing the front of the landmark.
void AddAltar(BoxWriter& boxes, const LocalFrame& frame, float u, const Look& stone);

}  // namespace city
