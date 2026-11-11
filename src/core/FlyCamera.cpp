#include "core/FlyCamera.h"

#include <algorithm>

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

namespace {
constexpr glm::vec3 kWorldUp(0.0f, 1.0f, 0.0f);
constexpr float kMouseSensitivity = 0.0025f;  // radians per pixel
constexpr float kWalkSpeed = 15.0f;           // metres per second
constexpr float kFastSpeed = 70.0f;
constexpr float kFieldOfView = glm::radians(60.0f);
constexpr float kNearPlane = 0.2f;
constexpr float kFarPlane = 4000.0f;

bool IsKeyDown(GLFWwindow* window, int key) {
    return glfwGetKey(window, key) == GLFW_PRESS;
}
}  // namespace

FlyCamera::FlyCamera(glm::vec3 startPosition, float yawRadians, float pitchRadians)
    : position(startPosition), yaw(yawRadians), pitch(pitchRadians) {}

glm::vec3 FlyCamera::Forward() const {
    return {std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw)};
}

void FlyCamera::Update(GLFWwindow* window, float deltaSeconds) {
    UpdateLook(window);

    const glm::vec3 forward = Forward();
    const glm::vec3 right = glm::normalize(glm::cross(forward, kWorldUp));

    glm::vec3 movement(0.0f);
    if (IsKeyDown(window, GLFW_KEY_W)) movement += forward;
    if (IsKeyDown(window, GLFW_KEY_S)) movement -= forward;
    if (IsKeyDown(window, GLFW_KEY_D)) movement += right;
    if (IsKeyDown(window, GLFW_KEY_A)) movement -= right;
    if (IsKeyDown(window, GLFW_KEY_E)) movement += kWorldUp;
    if (IsKeyDown(window, GLFW_KEY_Q)) movement -= kWorldUp;

    if (glm::length(movement) > 0.0f) {
        const float speed = IsKeyDown(window, GLFW_KEY_LEFT_SHIFT) ? kFastSpeed : kWalkSpeed;
        position += glm::normalize(movement) * speed * deltaSeconds;
    }
}

void FlyCamera::UpdateLook(GLFWwindow* window) {
    const bool rightButtonHeld = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    double cursorX = 0.0;
    double cursorY = 0.0;
    glfwGetCursorPos(window, &cursorX, &cursorY);

    if (rightButtonHeld && !isLooking) {
        // Hide and lock the cursor while looking so it never hits the edge of the screen.
        isLooking = true;
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        glfwGetCursorPos(window, &cursorX, &cursorY);
        lastCursorX = cursorX;
        lastCursorY = cursorY;
    } else if (!rightButtonHeld && isLooking) {
        isLooking = false;
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }

    if (!isLooking) {
        return;
    }

    // Moving the mouse right turns right, which means decreasing yaw in this right-handed setup.
    yaw -= static_cast<float>(cursorX - lastCursorX) * kMouseSensitivity;
    pitch -= static_cast<float>(cursorY - lastCursorY) * kMouseSensitivity;
    pitch = std::clamp(pitch, -1.55f, 1.55f);
    lastCursorX = cursorX;
    lastCursorY = cursorY;
}

glm::mat4 FlyCamera::ViewMatrix() const {
    return glm::lookAt(position, position + Forward(), kWorldUp);
}

glm::mat4 FlyCamera::ProjectionMatrix(float aspectRatio) const {
    // Passing far before near produces a reversed depth range.
    glm::mat4 projection = glm::perspectiveRH_ZO(kFieldOfView, aspectRatio, kFarPlane, kNearPlane);
    // Vulkan's clip space has Y pointing down, OpenGL-style math has it pointing up.
    projection[1][1] *= -1.0f;
    return projection;
}
