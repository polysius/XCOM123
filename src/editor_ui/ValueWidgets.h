#pragma once

#include "graph/Value.h"

// The controls for typing a value of each data type: a check box, a number that can be dragged or typed, three numbers for a
// vector, a text box.

namespace editor_ui {

// Shows an editor for `value`, `width` pixels wide, and returns true in the frame in which it changed. The value is turned
// into the type first if it is not of it. `id` must be unique among the controls of the window.
bool EditValue(const char* id, graph::Value& value, graph::PinType type, float width);

}  // namespace editor_ui
