#pragma once

#include <array>
#include <string_view>
#include <vector>

#include "graph_engine/KeyNames.h"

struct GLFWwindow;

// Finds the keys that went down since the last look, as the names an "On Key Pressed" event waits for (graph_engine::kKeyNames).
// ImGui's GLFW backend owns the window's key callback, so the keys are looked at once per frame instead.

namespace editor_ui {

class KeyPoller {
public:
    explicit KeyPoller(GLFWwindow* window);

    // The names of the keys that are down now and were up at the last call. A key that is already down when the poller is made does
    // not count. Call it every frame, also when the result is not wanted, so that a key held while something else had the keyboard
    // does not count as a press when that ends.
    std::vector<std::string_view> NewlyPressed();

private:
    GLFWwindow* window;
    std::array<bool, graph_engine::kKeyNames.size()> wasDown{};
};

}  // namespace editor_ui
