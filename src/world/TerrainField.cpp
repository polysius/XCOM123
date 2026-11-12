#include "world/TerrainField.h"

#include <cmath>

void TerrainField::Build(const std::vector<TerrainZone>& zones) {
    entries.clear();
    entries.reserve(zones.size());
    for (const TerrainZone& zone : zones) {
        // A stair needs a tread to climb one step per. Without one the zone is a terrace at its start
        // lift, which is better than a height that is not a number.
        const bool isStair = zone.rise > 0.0f && zone.tread > 0.0f;
        entries.push_back({zone.area.Bounds(), zone.area.Center(), std::cos(zone.area.yaw), std::sin(zone.area.yaw),
                           zone.area.area, zone.startLift, isStair ? zone.rise : 0.0f, isStair ? zone.tread : 1.0f});
    }
}

float TerrainField::LiftAt(glm::vec2 point) const {
    float lift = 0.0f;
    for (const Entry& entry : entries) {
        if (point.x < entry.bounds.min.x || point.x > entry.bounds.max.x || point.y < entry.bounds.min.y ||
            point.y > entry.bounds.max.y) {
            continue;
        }
        // The point in the zone's own frame (the inverse turn of OrientedRect::ToFrame).
        const glm::vec2 offset = point - entry.center;
        const glm::vec2 local = entry.center + glm::vec2(offset.x * entry.cosine - offset.y * entry.sine,
                                                         offset.x * entry.sine + offset.y * entry.cosine);
        if (local.x < entry.area.min.x || local.x > entry.area.max.x || local.y < entry.area.min.y ||
            local.y > entry.area.max.y) {
            continue;
        }
        // On a stair the surface is the top of the step under the point: step k, counted from the foot
        // of the first riser, stands (k + 1) rises above the ground the stair starts from.
        float here = entry.startLift;
        if (entry.rise > 0.0f) {
            // A point on the far edge belongs to the last step, not to one beyond it.
            const float lastStep = std::ceil((entry.area.max.x - entry.area.min.x) / entry.tread - 1e-4f) - 1.0f;
            const float step = std::min(std::floor((local.x - entry.area.min.x) / entry.tread), lastStep);
            here += (step + 1.0f) * entry.rise;
        }
        lift = std::max(lift, here);
    }
    return lift;
}
