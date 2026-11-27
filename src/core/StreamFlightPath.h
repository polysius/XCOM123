#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <glm/glm.hpp>

// The route of the streaming benchmark: a long, winding flight through the streamed world, the same every run. It cruises at
// walking-pace-of-a-fast-flyer speed, with two bursts at the speed of a fast jet pack, at changing heights down to roof level, so
// that the streaming is asked for every kind of load: chunks coming up slowly in front, and chunks that must be there a moment
// after the camera has left the last ones behind.
//
// The path is stepped one frame at a time with a fixed time step, so a run covers the same ground however long its frames take.
class StreamFlightPath {
public:
    struct Pose {
        glm::vec3 position{0.0f};
        glm::vec3 target{0.0f};
        float speed = 0.0f;  // metres per second
    };

    // The speeds of the two kinds of flying, metres per second.
    static constexpr float kCruiseSpeed = 30.0f;
    static constexpr float kBurstSpeed = 100.0f;
    // The last leg is a stress test far faster than anyone flies, to see how the streaming fails when it cannot keep up.
    static constexpr float kStressSpeed = 300.0f;

    // `speedScale` multiplies every speed, for scenarios faster than the standard flight.
    explicit StreamFlightPath(int totalFrames, float speedScale = 1.0f);

    // Where the flight starts.
    Pose Start() const;
    // Moves the camera on by one frame of `deltaSeconds` and returns where it is now.
    Pose Step(float deltaSeconds);

    // The names of the legs, and the leg a frame (counted from the first step) belongs to.
    static const std::vector<std::string>& LegNames();
    // The speed of a leg in metres per second, before the speed scale.
    static float LegSpeed(size_t leg);
    // Whether a leg is the stress leg, which only has to survive rather than keep up.
    static bool IsStressLeg(size_t leg);
    size_t LegOfFrame(int frame) const;

    float DistanceFlown() const { return distance; }

private:
    float Progress() const;
    Pose PoseHere() const;

    int totalFrames;
    float speedScale;
    int frame = 0;
    glm::vec3 position{0.0f};
    float distance = 0.0f;
};
