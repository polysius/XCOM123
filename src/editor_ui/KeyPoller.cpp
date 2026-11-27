#include "editor_ui/KeyPoller.h"

#include <GLFW/glfw3.h>

namespace editor_ui {
namespace {

using graph_engine::kKeyNames;

// The GLFW key of the name at a place in kKeyNames, whose order is: the letters, the digits, Space, Enter, Tab, the arrows, F2 to F12.
constexpr size_t kFirstDigit = 26;
constexpr size_t kFirstFunctionKey = 43;
static_assert(kKeyNames.size() == 54 && kKeyNames[0] == "A" && kKeyNames[25] == "Z" && kKeyNames[kFirstDigit] == "0" && kKeyNames[35] == "9" &&
                  kKeyNames[36] == "Space" && kKeyNames[42] == "Right" && kKeyNames[kFirstFunctionKey] == "F2" && kKeyNames[53] == "F12",
              "the key names changed: update GlfwKeyAt");

int GlfwKeyAt(size_t index) {
    if (index < kFirstDigit) {
        return GLFW_KEY_A + static_cast<int>(index);
    }
    if (index < 36) {
        return GLFW_KEY_0 + static_cast<int>(index - kFirstDigit);
    }
    if (index < kFirstFunctionKey) {
        constexpr int others[] = {GLFW_KEY_SPACE, GLFW_KEY_ENTER, GLFW_KEY_TAB, GLFW_KEY_UP, GLFW_KEY_DOWN, GLFW_KEY_LEFT, GLFW_KEY_RIGHT};
        return others[index - 36];
    }
    return GLFW_KEY_F2 + static_cast<int>(index - kFirstFunctionKey);
}

}  // namespace

KeyPoller::KeyPoller(GLFWwindow* window) : window(window) {
    for (size_t index = 0; index < wasDown.size(); ++index) {
        wasDown[index] = glfwGetKey(window, GlfwKeyAt(index)) == GLFW_PRESS;
    }
}

std::vector<std::string_view> KeyPoller::NewlyPressed() {
    std::vector<std::string_view> pressed;
    for (size_t index = 0; index < wasDown.size(); ++index) {
        const bool down = glfwGetKey(window, GlfwKeyAt(index)) == GLFW_PRESS;
        if (down && !wasDown[index]) {
            pressed.push_back(kKeyNames[index]);
        }
        wasDown[index] = down;
    }
    return pressed;
}

}  // namespace editor_ui
