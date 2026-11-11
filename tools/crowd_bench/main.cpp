// crowd_bench: times the crowd simulation without a window or a GPU.
//
// For each crowd size it builds the default city, spawns the crowd, then steps it at a fixed
// 60 Hz while a street level camera moves through the city, timing Crowd::Update and
// Crowd::CollectVisible separately. Afterwards it checks the crowd's state independently of
// the crowd code: no NaNs, nobody inside a building or on the road away from a crossing,
// and how many people near the camera overlap.
//
// It only uses the public Crowd API that the engine uses, so the same file also builds
// against older versions of the crowd for comparisons.
//
// Usage: crowd_bench [--npcs 50000,100000,200000] [--steps 300] [--seed 1789] [--satellites 0]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "scene/Frustum.h"
#include "scene/SceneTypes.h"
#include "world/CityGenerator.h"
#include "world/Crowd.h"

namespace {

using Clock = std::chrono::steady_clock;

constexpr float kStepSeconds = 1.0f / 60.0f;
constexpr uint32_t kWarmupSteps = 30;

// The camera, matching FlyCamera: 60 degree field of view, 16:9, reversed depth.
constexpr float kFieldOfView = glm::radians(60.0f);
constexpr float kAspectRatio = 16.0f / 9.0f;
constexpr float kNearPlane = 0.2f;
constexpr float kFarPlane = 4000.0f;
constexpr float kEyeHeight = 1.7f;
constexpr float kCameraSpeed = 12.0f;  // metres per second, a fast walk through the crowd

// Sanity check settings.
constexpr float kNearRadius = 35.0f;        // people this close to the camera are checked for overlaps
constexpr float kOverlapDistance = 0.3f;    // people closer than this count as overlapping
constexpr float kWalkableTolerance = 0.1f;  // slack, in metres, for the walkable space test

struct BenchOptions {
    std::vector<uint32_t> agentCounts{50000, 100000, 200000};
    uint32_t steps = 300;
    uint32_t seed = 1789;
    int satellites = -1;  // satellite districts around the city; negative keeps the generator's default
};

std::vector<uint32_t> ParseCountList(const std::string& text) {
    std::vector<uint32_t> counts;
    std::stringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ',')) {
        counts.push_back(static_cast<uint32_t>(std::stoul(item)));
    }
    return counts;
}

BenchOptions ParseOptions(int argc, char** argv) {
    BenchOptions options;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string name = argv[i];
        const std::string value = argv[i + 1];
        if (name == "--npcs") {
            options.agentCounts = ParseCountList(value);
        } else if (name == "--steps") {
            options.steps = static_cast<uint32_t>(std::stoul(value));
        } else if (name == "--seed") {
            options.seed = static_cast<uint32_t>(std::stoul(value));
        } else if (name == "--satellites") {
            options.satellites = std::stoi(value);
        } else {
            std::cerr << "Unknown option " << name << " (ignored)\n";
        }
    }
    return options;
}

double MillisecondsBetween(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

struct TimingSummary {
    double average = 0.0;
    double p99 = 0.0;
    double worst = 0.0;
};

TimingSummary Summarize(std::vector<double> samples) {
    TimingSummary summary;
    if (samples.empty()) {
        return summary;
    }
    std::sort(samples.begin(), samples.end());
    for (double sample : samples) {
        summary.average += sample;
    }
    summary.average /= static_cast<double>(samples.size());
    const size_t p99Index = std::min(samples.size() - 1, samples.size() * 99 / 100);
    summary.p99 = samples[p99Index];
    summary.worst = samples.back();
    return summary;
}

// The camera path ---------------------------------------------------------------------------

// The area the street network covers. Computed from the graph so that the benchmark relies
// only on the documented street graph.
Rect StreetBounds(const CityLayout& city) {
    Rect bounds{glm::vec2(1e30f), glm::vec2(-1e30f)};
    for (glm::vec2 node : city.streets.nodes) {
        bounds.min = glm::min(bounds.min, node);
        bounds.max = glm::max(bounds.max, node);
    }
    return bounds;
}

struct CameraPose {
    glm::vec3 position;
    glm::vec3 forward;
};

// A loop around the city centre at eye height, looking along the way and a little down.
// It passes through blocks as well as streets; the crowd does not care.
CameraPose CameraOnPath(const CityLayout& city, const Rect& bounds, float seconds) {
    const glm::vec2 center = bounds.Center();
    const glm::vec2 halfSize = bounds.Size() * 0.5f;
    const float radius = 0.3f * std::min(halfSize.x, halfSize.y);
    const float angle = seconds * kCameraSpeed / radius;
    const glm::vec2 ground = center + radius * glm::vec2(std::cos(angle), std::sin(angle));
    const glm::vec2 tangent(-std::sin(angle), std::cos(angle));

    CameraPose pose;
    pose.position = glm::vec3(ground.x, city.walkHeight + kEyeHeight, ground.y);
    pose.forward = glm::normalize(glm::vec3(tangent.x, -0.1f, tangent.y));
    return pose;
}

glm::mat4 ViewProjectionOf(const CameraPose& pose) {
    const glm::mat4 view = glm::lookAt(pose.position, pose.position + pose.forward, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 projection = glm::perspectiveRH_ZO(kFieldOfView, kAspectRatio, kFarPlane, kNearPlane);
    projection[1][1] *= -1.0f;
    return projection * view;
}

// A view from high above that contains the whole city, to read back everyone's position.
glm::mat4 WholeCityViewProjection(const Rect& bounds) {
    const glm::vec2 center = bounds.Center();
    const glm::vec2 halfSize = bounds.Size() * 0.5f + 200.0f;
    const glm::vec3 above(center.x, 1000.0f, center.y);
    const glm::mat4 view = glm::lookAt(above, glm::vec3(center.x, 0.0f, center.y), glm::vec3(0.0f, 0.0f, -1.0f));
    const glm::mat4 projection = glm::orthoRH_ZO(-halfSize.x, halfSize.x, -halfSize.y, halfSize.y, 1.0f, 2000.0f);
    return projection * view;
}

// Optional crowd features -------------------------------------------------------------------

// Older crowds have no focus point; the benchmark then runs them without one.
template <typename CrowdType>
void SetFocusIfSupported(CrowdType& crowd, glm::vec3 point) {
    if constexpr (requires { crowd.SetFocusPoint(point); }) {
        crowd.SetFocusPoint(point);
    }
}

// Sums the time the crowd reports for each phase of its update, when it reports any.
struct PhaseTimes {
    double tiers = 0.0;
    double grid = 0.0;
    double agents = 0.0;
};

template <typename CrowdType>
void AddPhaseTimesIfSupported(const CrowdType& crowd, PhaseTimes& sum) {
    if constexpr (requires { crowd.Stats().agentMs; }) {
        const auto stats = crowd.Stats();
        sum.tiers += stats.tierMs;
        sum.grid += stats.gridMs;
        sum.agents += stats.agentMs;
    }
}

template <typename CrowdType>
void PrintLevelOfDetailIfSupported(const CrowdType& crowd) {
    if constexpr (requires { crowd.Stats(); }) {
        const auto stats = crowd.Stats();
        std::cout << "  simulation tiers (last step): near " << stats.nearAgents << ", mid " << stats.midAgents
                  << ", far " << stats.farAgents << "\n  visible by distance band:";
        for (uint32_t count : stats.visibleByBand) {
            std::cout << " " << count;
        }
        std::cout << "\n";
    }
}

// The independent sanity check ------------------------------------------------------------------

// Answers where a point is, straight from the city layout rather than from the crowd's own
// walkway data, so that a bug in the crowd cannot hide itself.
class WalkableSpaceCheck {
public:
    enum class Place { Walkable, InsideObstacle, OnRoad, OutsideWalkways };

    explicit WalkableSpaceCheck(const CityLayout& city) : plazas(city.plazas) {
        roadHalfWidth = city.sidewalkCenterOffset - city.sidewalkWidth * 0.5f;
        outerEdge = city.sidewalkCenterOffset + city.sidewalkWidth * 0.5f;
        // The road near a street's ends is the intersection and its crosswalks. This assumes
        // streets meet at roughly right angles, as in the current generator.
        crossingZone = outerEdge + 1.5f;

        for (const StreetGraph::Edge& edge : city.streets.edges) {
            const glm::vec2 start = city.streets.nodes[edge.from];
            const glm::vec2 delta = city.streets.nodes[edge.to] - start;
            const float length = glm::length(delta);
            if (length > 1e-3f) {
                streets.push_back({start, delta / length, length});
            }
        }
        AddObstacles(city);
    }

    Place Classify(glm::vec2 point) const {
        if (IsInsideObstacle(point)) {
            return Place::InsideObstacle;
        }
        for (const OrientedRect& plaza : plazas) {
            if (plaza.Contains(point, kWalkableTolerance)) {
                return Place::Walkable;
            }
        }
        bool onStreetArea = false;
        for (const Street& street : streets) {
            const glm::vec2 offset = point - street.start;
            const float along = glm::dot(offset, street.direction);
            const float across = std::abs(offset.x * street.direction.y - offset.y * street.direction.x);
            if (along < -outerEdge || along > street.length + outerEdge || across > outerEdge + kWalkableTolerance) {
                continue;
            }
            const bool awayFromEnds = along > crossingZone && along < street.length - crossingZone;
            if (awayFromEnds && across < roadHalfWidth - kWalkableTolerance) {
                return Place::OnRoad;
            }
            onStreetArea = true;
        }
        return onStreetArea ? Place::Walkable : Place::OutsideWalkways;
    }

private:
    struct Street {
        glm::vec2 start;
        glm::vec2 direction;
        float length;
    };

    static constexpr float kCellSize = 16.0f;

    // Boxes a person could walk into: they reach from below head height to above the ankles.
    // Footprints shrink by a few centimetres so that people brushing a wall do not count.
    void AddObstacles(const CityLayout& city) {
        for (const BoxInstance& box : city.boxes) {
            const float bottom = box.center.y - box.halfSize.y;
            const float top = box.center.y + box.halfSize.y;
            if (bottom > city.walkHeight + 1.6f || top < city.walkHeight + 0.25f) {
                continue;
            }
            const glm::vec2 center(box.center.x, box.center.z);
            const glm::vec2 halfSize = glm::vec2(box.halfSize.x, box.halfSize.z) - 0.05f;
            if (halfSize.x > 0.0f && halfSize.y > 0.0f) {
                obstacles.push_back({Rect{center - halfSize, center + halfSize}, box.yaw});
            }
        }
        // Bucket the obstacles into a coarse grid so each test looks at only a few.
        const Rect bounds = StreetBounds(city);
        gridOrigin = bounds.min - 50.0f;
        const glm::vec2 size = bounds.Size() + 100.0f;
        gridColumns = static_cast<int>(std::ceil(size.x / kCellSize));
        gridRows = static_cast<int>(std::ceil(size.y / kCellSize));
        cells.assign(static_cast<size_t>(gridColumns) * static_cast<size_t>(gridRows), {});
        for (uint32_t index = 0; index < obstacles.size(); ++index) {
            const Rect obstacleBounds = obstacles[index].Bounds();
            const glm::ivec2 low = CellOf(obstacleBounds.min);
            const glm::ivec2 high = CellOf(obstacleBounds.max);
            for (int y = low.y; y <= high.y; ++y) {
                for (int x = low.x; x <= high.x; ++x) {
                    cells[static_cast<size_t>(y) * static_cast<size_t>(gridColumns) + static_cast<size_t>(x)].push_back(index);
                }
            }
        }
    }

    glm::ivec2 CellOf(glm::vec2 point) const {
        const glm::vec2 cell = glm::floor((point - gridOrigin) / kCellSize);
        return glm::clamp(glm::ivec2(cell), glm::ivec2(0), glm::ivec2(gridColumns - 1, gridRows - 1));
    }

    bool IsInsideObstacle(glm::vec2 point) const {
        const glm::ivec2 cell = CellOf(point);
        for (uint32_t index : cells[static_cast<size_t>(cell.y) * static_cast<size_t>(gridColumns) + static_cast<size_t>(cell.x)]) {
            if (obstacles[index].Contains(point)) {
                return true;
            }
        }
        return false;
    }

    std::vector<OrientedRect> plazas;
    std::vector<Street> streets;
    std::vector<OrientedRect> obstacles;
    std::vector<std::vector<uint32_t>> cells;
    glm::vec2 gridOrigin{0.0f};
    int gridColumns = 0;
    int gridRows = 0;
    float roadHalfWidth = 0.0f;
    float outerEdge = 0.0f;
    float crossingZone = 0.0f;
};

struct SanityReport {
    size_t readBack = 0;
    size_t nonFinite = 0;
    size_t insideObstacle = 0;
    size_t onRoad = 0;
    size_t outsideWalkways = 0;
    size_t nearAgents = 0;
    size_t overlappingPairs = 0;
};

bool IsFinite(const NpcInstance& npc) {
    return std::isfinite(npc.position.x) && std::isfinite(npc.position.y) && std::isfinite(npc.position.z) &&
           std::isfinite(npc.heading) && std::isfinite(npc.walkPhase) && std::isfinite(npc.stride);
}

size_t CountOverlappingPairs(std::vector<glm::vec2> points) {
    // Sorting by x lets each person compare only with those within the overlap distance in x.
    std::sort(points.begin(), points.end(), [](glm::vec2 a, glm::vec2 b) { return a.x < b.x; });
    size_t pairs = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size() && points[j].x - points[i].x < kOverlapDistance; ++j) {
            if (glm::distance(points[i], points[j]) < kOverlapDistance) {
                ++pairs;
            }
        }
    }
    return pairs;
}

SanityReport CheckCrowd(const Crowd& crowd, const CityLayout& city, glm::vec3 camera) {
    std::vector<NpcInstance> everyone;
    crowd.CollectVisible(Frustum(WholeCityViewProjection(StreetBounds(city))), everyone);

    SanityReport report;
    report.readBack = everyone.size();
    const WalkableSpaceCheck walkable(city);
    std::vector<glm::vec2> nearPoints;
    for (const NpcInstance& npc : everyone) {
        if (!IsFinite(npc)) {
            ++report.nonFinite;
            continue;
        }
        const glm::vec2 point(npc.position.x, npc.position.z);
        switch (walkable.Classify(point)) {
        case WalkableSpaceCheck::Place::Walkable:
            break;
        case WalkableSpaceCheck::Place::InsideObstacle:
            ++report.insideObstacle;
            break;
        case WalkableSpaceCheck::Place::OnRoad:
            ++report.onRoad;
            break;
        case WalkableSpaceCheck::Place::OutsideWalkways:
            ++report.outsideWalkways;
            break;
        }
        if (glm::distance(point, glm::vec2(camera.x, camera.z)) < kNearRadius) {
            nearPoints.push_back(point);
        }
    }
    report.nearAgents = nearPoints.size();
    report.overlappingPairs = CountOverlappingPairs(std::move(nearPoints));
    return report;
}

// One benchmark run --------------------------------------------------------------------------------

void PrintTiming(const char* label, const TimingSummary& timing) {
    std::cout << std::fixed << std::setprecision(3) << "  " << std::left << std::setw(8) << label << std::right
              << " avg " << timing.average << " ms   p99 " << timing.p99 << " ms   max " << timing.worst << " ms\n";
}

void RunBenchmark(const CityLayout& city, uint32_t agentCount, const BenchOptions& options) {
    Crowd crowd;
    const Clock::time_point spawnStart = Clock::now();
    crowd.Spawn(city, {.agentCount = agentCount, .seed = options.seed * 7919u + 1u});
    const double spawnMs = MillisecondsBetween(spawnStart, Clock::now());

    std::vector<NpcInstance> visible;
    std::vector<double> updateMs;
    std::vector<double> cullMs;
    std::vector<double> totalMs;
    double visibleSum = 0.0;
    PhaseTimes phaseSum;
    const Rect bounds = StreetBounds(city);
    CameraPose camera{};
    for (uint32_t step = 0; step < kWarmupSteps + options.steps; ++step) {
        camera = CameraOnPath(city, bounds, static_cast<float>(step) * kStepSeconds);
        const Clock::time_point start = Clock::now();
        SetFocusIfSupported(crowd, camera.position);
        crowd.Update(kStepSeconds);
        const Clock::time_point updated = Clock::now();
        crowd.CollectVisible(Frustum(ViewProjectionOf(camera)), visible);
        const Clock::time_point culled = Clock::now();

        if (step >= kWarmupSteps) {
            updateMs.push_back(MillisecondsBetween(start, updated));
            cullMs.push_back(MillisecondsBetween(updated, culled));
            totalMs.push_back(MillisecondsBetween(start, culled));
            visibleSum += static_cast<double>(visible.size());
            AddPhaseTimesIfSupported(crowd, phaseSum);
        }
    }

    std::cout << "\n" << agentCount << " people requested, " << crowd.AgentCount() << " spawned in " << std::fixed
              << std::setprecision(1) << spawnMs << " ms; " << options.steps << " steps of " << kStepSeconds * 1000.0f
              << " ms after " << kWarmupSteps << " warm-up steps\n";
    PrintTiming("update", Summarize(updateMs));
    PrintTiming("cull", Summarize(cullMs));
    PrintTiming("total", Summarize(totalMs));
    const double stepCount = static_cast<double>(options.steps);
    if (phaseSum.agents > 0.0) {
        std::cout << std::setprecision(3) << "  update phases avg: tiers " << phaseSum.tiers / stepCount << " ms, grid "
                  << phaseSum.grid / stepCount << " ms, people " << phaseSum.agents / stepCount << " ms\n";
    }
    std::cout << "  visible per step: " << std::setprecision(0) << visibleSum / stepCount << "\n"
              << std::setprecision(1);
    PrintLevelOfDetailIfSupported(crowd);

    const SanityReport sanity = CheckCrowd(crowd, city, camera.position);
    std::cout << "  sanity: read back " << sanity.readBack << ", non-finite " << sanity.nonFinite
              << ", inside obstacles " << sanity.insideObstacle << ", on road away from crossings " << sanity.onRoad
              << ", outside walkways " << sanity.outsideWalkways << "\n"
              << "          " << sanity.nearAgents << " people within " << kNearRadius << " m of the camera, "
              << sanity.overlappingPairs << " pairs closer than " << kOverlapDistance << " m\n";
}

}  // namespace

int main(int argc, char** argv) {
    const BenchOptions options = ParseOptions(argc, argv);

    CitySettings citySettings;
    citySettings.seed = options.seed;
    if (options.satellites >= 0) {
        citySettings.satelliteCount = options.satellites;
    }
    const CityLayout city = GenerateCity(citySettings);
    std::cout << "City (seed " << options.seed << "): " << city.boxes.size() << " boxes, " << city.streets.edges.size()
              << " street segments, " << city.plazas.size() << " squares\n";

    for (uint32_t agentCount : options.agentCounts) {
        RunBenchmark(city, agentCount, options);
    }
    return 0;
}
