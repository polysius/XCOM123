#pragma once

// The height of the ground people walk on, as a function of where they stand.
//
// Most of the city is level: its walking surface is CityLayout::walkHeight everywhere, and a point
// outside every terrain zone has a lift of exactly zero. A terrace stands higher, by a lift that
// is added to walkHeight, and a stair climbs to one.
//
// Every point has one height: no level of the city lies under another, so a function of x and z
// is enough to place a person, and nothing walks beneath a terrace.

#include <vector>

#include <glm/glm.hpp>

#include "world/CityGenerator.h"

class TerrainField {
public:
    TerrainField() = default;
    explicit TerrainField(const std::vector<TerrainZone>& zones) { Build(zones); }

    void Build(const std::vector<TerrainZone>& zones);

    // True when the whole city is level, and LiftAt is zero everywhere.
    bool IsLevel() const { return entries.empty(); }

    // How far above CityLayout::walkHeight people stand at `point`. Where zones meet, the highest
    // lift counts.
    float LiftAt(glm::vec2 point) const;

private:
    struct Entry {
        Rect bounds;       // the zone's axis-aligned bounds, to reject most points cheaply
        glm::vec2 center;  // the middle of the zone
        float cosine;      // of the zone's yaw
        float sine;
        Rect area;         // the zone in its own frame, where it is axis-aligned
        float startLift;
        float rise;
        float tread;
    };
    std::vector<Entry> entries;
};

// A flat terrace: the walking surface of `area` stands `lift` metres higher.
inline TerrainZone MakePlateau(const OrientedRect& area, float lift) {
    return {area, lift, 0.0f, 1.0f};
}
