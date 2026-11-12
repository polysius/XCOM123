#include "core/BenchmarkCameraPath.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <glm/gtc/matrix_transform.hpp>

#include "scene/Frustum.h"
#include "world/CityGenerator.h"
#include "world/Crowd.h"

namespace {
constexpr float kEyeHeight = 1.7f;
// Grid points closer than this in one coordinate count as lying on the same straight street.
constexpr float kSameLineTolerance = 0.5f;
// A street shorter than this is not worth a shot of its own; the path falls back to a line
// through the middle of the city.
constexpr float kMinimumStreetLength = 100.0f;

glm::vec3 OnGround(glm::vec2 point, float height) {
    return {point.x, height, point.y};
}

// A straight line along the city's streets at ground level, in the X-Z plane.
struct StreetLine {
    glm::vec2 start{0.0f};
    glm::vec2 end{0.0f};
};

// The longest run of street grid points that share one coordinate with the point nearest the
// city centre. alongX picks a street running in X (constant Z), otherwise one running in Z.
StreetLine FindCentralStreet(const CityLayout& city, bool alongX) {
    const glm::vec2 centre = city.coreBounds.Center();
    const StreetLine fallback = alongX ? StreetLine{{city.coreBounds.min.x, centre.y}, {city.coreBounds.max.x, centre.y}}
                                       : StreetLine{{centre.x, city.coreBounds.min.y}, {centre.x, city.coreBounds.max.y}};
    const std::vector<glm::vec2>& nodes = city.streets.nodes;
    if (nodes.empty()) {
        return fallback;
    }

    const auto nearestToCentre = std::min_element(nodes.begin(), nodes.end(), [centre](glm::vec2 a, glm::vec2 b) {
        return glm::distance(a, centre) < glm::distance(b, centre);
    });
    // The coordinate that stays fixed along the street, and the one that runs along it.
    const auto fixedOf = [alongX](glm::vec2 point) { return alongX ? point.y : point.x; };
    const auto runOf = [alongX](glm::vec2 point) { return alongX ? point.x : point.y; };
    const float line = fixedOf(*nearestToCentre);

    StreetLine street{*nearestToCentre, *nearestToCentre};
    for (const glm::vec2 node : nodes) {
        if (std::abs(fixedOf(node) - line) > kSameLineTolerance) {
            continue;
        }
        if (runOf(node) < runOf(street.start)) {
            street.start = node;
        }
        if (runOf(node) > runOf(street.end)) {
            street.end = node;
        }
    }
    return glm::distance(street.start, street.end) >= kMinimumStreetLength ? street : fallback;
}

// The square with the most people per square metre, counted with a top-down box over each one.
// Returns nullptr when the city has no squares.
const OrientedRect* FindBusiestPlaza(const CityLayout& city, const Crowd& crowd) {
    const OrientedRect* busiest = nullptr;
    float bestDensity = -1.0f;
    std::vector<NpcInstance> inside;
    for (const OrientedRect& plaza : city.plazas) {
        if (!city.coreBounds.Contains(plaza.Center())) {
            continue;  // the path stays in the core, so that it is the same however many satellites there are
        }
        const glm::vec2 centre = plaza.Center();
        const glm::vec2 halfSize = plaza.Size() * 0.5f;
        // Looking straight down with the square's own +Z as "up" on screen, so the box's sides line up
        // with the square's. That direction is +Z itself unless the square is turned.
        const glm::vec3 up(std::sin(plaza.yaw), 0.0f, std::cos(plaza.yaw));
        const glm::mat4 view = glm::lookAt(OnGround(centre, 500.0f), OnGround(centre, 0.0f), up);
        const glm::mat4 projection = glm::orthoRH_ZO(-halfSize.x, halfSize.x, -halfSize.y, halfSize.y, 1.0f, 1000.0f);
        crowd.CollectVisible(Frustum(projection * view), inside);

        const float area = std::max(plaza.Size().x * plaza.Size().y, 1.0f);
        const float density = static_cast<float>(inside.size()) / area;
        if (density > bestDensity) {
            bestDensity = density;
            busiest = &plaza;
        }
    }
    return busiest;
}

BenchmarkCameraPath::Shot AerialOrbit(glm::vec2 centre, float extent) {
    // A quarter circle high above the city, always looking at its centre.
    BenchmarkCameraPath::Shot shot{.name = "aerial orbit", .share = 0.20f};
    const float radius = extent * 0.55f;
    const float height = extent * 0.45f;
    constexpr int kSteps = 6;
    for (int step = 0; step <= kSteps; ++step) {
        const float angle = glm::radians(200.0f + 90.0f * static_cast<float>(step) / kSteps);
        const glm::vec2 point = centre + radius * glm::vec2(std::sin(angle), std::cos(angle));
        shot.poses.push_back({OnGround(point, height), OnGround(centre, 0.0f)});
    }
    return shot;
}

BenchmarkCameraPath::Shot LowCrossing(const CityLayout& city, float extent) {
    // Low over one corner, drifting towards the centre while looking at the far corner: the
    // view that shows the most buildings and people at once.
    const glm::vec2 nearCorner = city.coreBounds.min;
    const glm::vec2 farCorner = city.coreBounds.max;
    const glm::vec2 centre = city.coreBounds.Center();
    return {.name = "low crossing",
            .share = 0.10f,
            .poses = {{OnGround(nearCorner, extent * 0.10f), OnGround(farCorner, 0.0f)},
                      {OnGround(glm::mix(nearCorner, centre, 0.6f), extent * 0.06f), OnGround(farCorner, 0.0f)}}};
}

BenchmarkCameraPath::Shot StreetWalk(const std::string& name, float share, const StreetLine& street, float eyeHeight) {
    // At eye height in the middle of the road, looking all the way down it, moving forwards.
    const glm::vec2 direction = glm::normalize(street.end - street.start);
    const glm::vec2 from = glm::mix(street.start, street.end, 0.05f);
    const glm::vec2 to = glm::mix(street.start, street.end, 0.35f);
    const glm::vec2 lookAhead = direction * 400.0f;
    return {.name = name,
            .share = share,
            .poses = {{OnGround(from, eyeHeight), OnGround(from + lookAhead, eyeHeight)},
                      {OnGround(to, eyeHeight), OnGround(to + lookAhead, eyeHeight)}}};
}

BenchmarkCameraPath::Shot PlazaWalk(const OrientedRect& plaza, float eyeHeight) {
    // A slow walk from one edge of the square towards its middle, looking across it.
    const glm::vec2 centre = plaza.Center();
    const glm::vec2 diagonal = plaza.Size() * 0.5f;
    const glm::vec2 from = plaza.FromFrame(centre - diagonal * 0.9f);
    const glm::vec2 to = plaza.FromFrame(centre - diagonal * 0.1f);
    const glm::vec2 farSide = plaza.FromFrame(centre + diagonal * 2.0f);
    return {.name = "busiest square",
            .share = 0.25f,
            .poses = {{OnGround(from, eyeHeight), OnGround(farSide, eyeHeight)},
                      {OnGround(to, eyeHeight), OnGround(farSide, eyeHeight)}}};
}

BenchmarkCameraPath::Shot RiseOut(glm::vec2 start, glm::vec2 cityCentre, float eyeHeight, float extent) {
    // From the crowd up to a high view of the whole city.
    return {.name = "rise out",
            .share = 0.10f,
            .poses = {{OnGround(start, eyeHeight + 3.0f), OnGround(cityCentre, 0.0f)},
                      {OnGround(start, extent * 0.35f), OnGround(cityCentre, 0.0f)}}};
}
}  // namespace

BenchmarkCameraPath::BenchmarkCameraPath(const CityLayout& city, const Crowd& crowd) {
    const glm::vec2 centre = city.coreBounds.Center();
    const float extent = std::max(city.coreBounds.Size().x, city.coreBounds.Size().y);
    // The path stays in the core of the city, which is level (only satellite districts are raised).
    const float eyeHeight = city.walkHeight + kEyeHeight;

    // Without any square, the "square" shots use the city centre instead.
    const OrientedRect* busiest = FindBusiestPlaza(city, crowd);
    const OrientedRect plaza = busiest != nullptr ? *busiest : OrientedRect(Rect{centre - glm::vec2(20.0f), centre + glm::vec2(20.0f)});

    shots.push_back(AerialOrbit(centre, extent));
    shots.push_back(LowCrossing(city, extent));
    shots.push_back(StreetWalk("street along X", 0.20f, FindCentralStreet(city, true), eyeHeight));
    shots.push_back(StreetWalk("street along Z", 0.15f, FindCentralStreet(city, false), eyeHeight));
    shots.push_back(PlazaWalk(plaza, eyeHeight));
    shots.push_back(RiseOut(plaza.Center(), centre, eyeHeight, extent));
}

size_t BenchmarkCameraPath::ShotIndexAt(float progress) const {
    float shotEnd = 0.0f;
    for (size_t index = 0; index < shots.size(); ++index) {
        shotEnd += shots[index].share;
        if (progress < shotEnd) {
            return index;
        }
    }
    return shots.size() - 1;
}

BenchmarkCameraPath::Pose BenchmarkCameraPath::PoseAt(float progress) const {
    progress = std::clamp(progress, 0.0f, 1.0f);
    const size_t index = ShotIndexAt(progress);
    float shotStart = 0.0f;
    for (size_t earlier = 0; earlier < index; ++earlier) {
        shotStart += shots[earlier].share;
    }
    const Shot& shot = shots[index];
    const float withinShot = std::clamp((progress - shotStart) / shot.share, 0.0f, 1.0f);

    // Pick the pair of poses around this point of the shot and blend between them.
    const float scaled = withinShot * static_cast<float>(shot.poses.size() - 1);
    const size_t from = std::min(static_cast<size_t>(scaled), shot.poses.size() - 1);
    const size_t to = std::min(from + 1, shot.poses.size() - 1);
    const float blend = scaled - static_cast<float>(from);
    return {glm::mix(shot.poses[from].position, shot.poses[to].position, blend),
            glm::mix(shot.poses[from].target, shot.poses[to].target, blend)};
}
