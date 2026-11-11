#pragma once

#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>

struct CityLayout;
class Crowd;

// A fixed camera flight through the city for benchmarks.
//
// It is built only from the city layout and where the crowd stands at spawn time, so the same
// seed always flies the same path. The path is a list of shots that together cover the kinds of
// views that are expensive: high aerial views of the whole city, a low view across it, views
// along the longest streets at eye height, and a slow walk through the busiest square.
class BenchmarkCameraPath {
public:
    struct Pose {
        glm::vec3 position{0.0f};
        glm::vec3 target{0.0f};
    };

    struct Shot {
        std::string name;
        float share = 0.0f;       // fraction of the benchmark spent on this shot
        std::vector<Pose> poses;  // visited at even intervals, with straight lines between them
    };

    BenchmarkCameraPath(const CityLayout& city, const Crowd& crowd);

    // Where the camera is `progress` (0 to 1) of the way through the whole benchmark.
    Pose PoseAt(float progress) const;
    // Index of the shot that covers `progress`.
    size_t ShotIndexAt(float progress) const;
    std::span<const Shot> Shots() const { return shots; }

private:
    std::vector<Shot> shots;
};
