#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <string>
#include <string_view>
#include <utility>

#include "graph/Graph.h"
#include "graph/Interpreter.h"

// The names of the keys that an "On Key Pressed" event can wait for. They are plain words a person picks from a list, kept apart
// from GLFW so that the graph, the editor model and the tests need no window. The editor's key poller maps each name to a GLFW key.
// F1 and Escape are not here: the editor owns them.

namespace graph_engine {

inline constexpr const char* kKeyPressedEvent = "event.key_pressed";
inline constexpr const char* kKeyProperty = "Key";
inline constexpr const char* kDefaultKey = "Space";

inline constexpr std::array<std::string_view, 54> kKeyNames = {
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z",
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
    "Space", "Enter", "Tab", "Up", "Down", "Left", "Right",
    "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
};

inline bool SameKeyName(std::string_view a, std::string_view b) {
    return std::equal(a.begin(), a.end(), b.begin(), b.end(), [](char x, char y) {
        return std::tolower(static_cast<unsigned char>(x)) == std::tolower(static_cast<unsigned char>(y));
    });
}

// The event nodes that wait for this key. A node that holds no key (one added from the palette) waits for the default.
inline graph::Interpreter::EventFilter KeyFilter(std::string keyName) {
    return [keyName = std::move(keyName)](const graph::Node& node) {
        const auto held = node.properties.find(kKeyProperty);
        if (held != node.properties.end() && std::holds_alternative<std::string>(held->second)) {
            return SameKeyName(std::get<std::string>(held->second), keyName);
        }
        return SameKeyName(kDefaultKey, keyName);
    };
}

}  // namespace graph_engine
