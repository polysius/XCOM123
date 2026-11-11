#pragma once

struct GLFWwindow;

// Shared services handed to every module when it starts.
struct EngineContext {
    GLFWwindow* window = nullptr;
    bool vsync = true;
};
