#pragma once

#include <glm/glm.hpp>

struct GLFWwindow;

// A free-flying debug camera.
// Hold the right mouse button to look around; WASD moves, E/Q go up/down, Shift moves faster.
class FlyCamera {
public:
    FlyCamera(glm::vec3 position, float yawRadians, float pitchRadians);

    void Update(GLFWwindow* window, float deltaSeconds);

    // Places the camera directly, for scripted views such as screenshots and benchmarks.
    void SetPose(glm::vec3 newPosition, float yawRadians, float pitchRadians);
    void LookAt(glm::vec3 newPosition, glm::vec3 target);

    glm::mat4 ViewMatrix() const;
    // Vulkan-style projection: Y points down in clip space and depth is reversed
    // (1 at the near plane, 0 at the far plane) for much better depth precision far away.
    glm::mat4 ProjectionMatrix(float aspectRatio) const;
    glm::vec3 Position() const { return position; }
    glm::vec3 Forward() const;

private:
    void UpdateLook(GLFWwindow* window);

    glm::vec3 position;
    float yaw;    // 0 looks along +Z, positive turns towards +X
    float pitch;  // positive looks up
    bool isLooking = false;
    double lastCursorX = 0.0;
    double lastCursorY = 0.0;
};
