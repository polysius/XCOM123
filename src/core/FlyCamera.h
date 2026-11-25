#pragma once

#include <glm/glm.hpp>

struct GLFWwindow;

// A free-flying debug camera.
// Hold the right mouse button to look around; WASD moves, E/Q go up/down, Shift moves faster.
class FlyCamera {
public:
    FlyCamera(glm::vec3 position, float yawRadians, float pitchRadians);

    // Looks at the mouse and keys. Something drawn over the scene (the editor) may want them: `mayStartLook` false keeps a new
    // look from starting, and `mayMove` false leaves the movement keys alone. A look that is going always ends when the right
    // button comes up, whatever else is asked, so the cursor is never left hidden.
    void Update(GLFWwindow* window, float deltaSeconds, bool mayStartLook = true, bool mayMove = true);

    // Whether the camera is looking after a frame: a look that is going goes on while the button is held, and one that is not
    // starts when the button goes down, if it may.
    static constexpr bool IsLookingAfter(bool wasLooking, bool buttonHeld, bool mayStartLook) {
        return wasLooking ? buttonHeld : (buttonHeld && mayStartLook);
    }

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
    void UpdateLook(GLFWwindow* window, bool mayStartLook);

    glm::vec3 position;
    float yaw;    // 0 looks along +Z, positive turns towards +X
    float pitch;  // positive looks up
    bool isLooking = false;
    double lastCursorX = 0.0;
    double lastCursorY = 0.0;
};
