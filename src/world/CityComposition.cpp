#include "world/CityComposition.h"

#include <cmath>

namespace city {

namespace {

// Appends every element of `source` to `target`, each one moved by `place`.
template <typename T, typename Place>
void AppendEach(std::vector<T>& target, const std::vector<T>& source, Place&& place) {
    target.reserve(target.size() + source.size());
    for (const T& element : source) {
        target.push_back(place(element));
    }
}

glm::vec3 PlaceOnGround(const Placement& placement, glm::vec3 position) {
    const glm::vec2 moved = placement.Apply({position.x, position.z});
    return {moved.x, position.y + placement.lift, moved.y};
}

}  // namespace

glm::vec2 Placement::Turn(glm::vec2 direction) const {
    if (yaw == 0.0f) {
        return direction;
    }
    const float cosine = std::cos(yaw);
    const float sine = std::sin(yaw);
    return {direction.x * cosine + direction.y * sine, -direction.x * sine + direction.y * cosine};
}

OrientedRect Placement::Apply(const OrientedRect& rect) const {
    // The rectangle keeps its size; its centre moves and its own turn adds to the placement's.
    const glm::vec2 halfSize = rect.Size() * 0.5f;
    const glm::vec2 center = Apply(rect.Center());
    return {Rect{center - halfSize, center + halfSize}, rect.yaw + yaw};
}

void AppendPart(CityLayout& city, const CityLayout& part, const Placement& placement) {
    AppendEach(city.boxes, part.boxes, [&](BoxInstance box) {
        box.center = PlaceOnGround(placement, box.center);
        box.yaw += placement.yaw;
        return box;
    });
    city.boxParts.insert(city.boxParts.end(), part.boxParts.begin(), part.boxParts.end());

    // The street graph keeps its shape; its indices move past the ones already in the city.
    const uint32_t nodeOffset = static_cast<uint32_t>(city.streets.nodes.size());
    const uint32_t edgeOffset = static_cast<uint32_t>(city.streets.edges.size());
    AppendEach(city.streets.nodes, part.streets.nodes, [&](glm::vec2 node) { return placement.Apply(node); });
    AppendEach(city.streets.edges, part.streets.edges, [&](StreetGraph::Edge edge) {
        edge.from += nodeOffset;
        edge.to += nodeOffset;
        return edge;
    });
    AppendEach(city.streets.edgesAtNode, part.streets.edgesAtNode, [&](std::vector<uint32_t> edges) {
        for (uint32_t& edge : edges) {
            edge += edgeOffset;
        }
        return edges;
    });

    AppendEach(city.plazas, part.plazas, [&](const OrientedRect& rect) { return placement.Apply(rect); });
    AppendEach(city.parks, part.parks, [&](const OrientedRect& rect) { return placement.Apply(rect); });
    AppendEach(city.water, part.water, [&](const OrientedRect& rect) { return placement.Apply(rect); });
    AppendEach(city.bridges, part.bridges, [&](Bridge bridge) {
        bridge.edge += edgeOffset;
        bridge.deck = placement.Apply(bridge.deck);
        return bridge;
    });
    AppendEach(city.blocks, part.blocks, [&](CityBlock block) {
        block.land = placement.Apply(block.land);
        return block;
    });
    AppendEach(city.landmarks, part.landmarks, [&](Landmark landmark) {
        landmark.footprint = placement.Apply(landmark.footprint);
        landmark.entrance = placement.Apply(landmark.entrance);
        return landmark;
    });
    AppendEach(city.interiors, part.interiors, [&](Interior interior) {
        interior.floor = placement.Apply(interior.floor);
        interior.door = placement.Apply(interior.door);
        return interior;
    });
    AppendEach(city.streetLights, part.streetLights, [&](glm::vec3 light) { return PlaceOnGround(placement, light); });
    AppendEach(city.trees, part.trees, [&](Tree tree) {
        tree.base = PlaceOnGround(placement, tree.base);
        return tree;
    });
    AppendEach(city.pointsOfInterest, part.pointsOfInterest, [&](PointOfInterest point) {
        point.position = PlaceOnGround(placement, point.position);
        point.facing = placement.Turn(point.facing);
        return point;
    });

    // The part's own terrain stands that much higher still.
    AppendEach(city.terrain, part.terrain, [&](TerrainZone zone) {
        zone.area = placement.Apply(zone.area);
        zone.startLift += placement.lift;
        return zone;
    });

    // The bounds hold the part's outer edge, which is a turned rectangle.
    const OrientedRect partEdge = placement.Apply(OrientedRect(part.bounds));
    const Rect partBounds = partEdge.Bounds();
    city.bounds.min = glm::min(city.bounds.min, partBounds.min);
    city.bounds.max = glm::max(city.bounds.max, partBounds.max);
}

}  // namespace city
