#pragma once

#include <imgui.h>

#include "graph/NodeRegistry.h"
#include "graph/Value.h"

// The colours of the graph editor. Nodes are coloured by what they do, as in Blueprints (events red, actions blue, pure
// computations green), and pins and wires by what they carry, so that a wire says at a glance whether it fits a pin.

namespace editor_ui {

// The colour of a pin and of the wire out of it.
ImU32 ColorOfPinType(graph::PinType type);
// The colour of the header of a node of a kind; grey for a node of a kind that is not known.
ImU32 ColorOfNodeKind(const graph::NodeDescription* description);
// A word for what a kind of node does, for tooltips and the details panel.
const char* NameOfNodeKind(const graph::NodeDescription* description);

// The same colour with another opacity, from 0 to 1.
ImU32 WithOpacity(ImU32 color, float opacity);

constexpr ImU32 kCanvasBackground = IM_COL32(24, 25, 28, 255);
constexpr ImU32 kGridDot = IM_COL32(70, 72, 78, 255);
constexpr ImU32 kGridDotMajor = IM_COL32(100, 103, 110, 255);
constexpr ImU32 kNodeBody = IM_COL32(36, 38, 43, 245);
constexpr ImU32 kNodeBorder = IM_COL32(12, 12, 14, 255);
constexpr ImU32 kSelectionColor = IM_COL32(255, 255, 255, 255);
constexpr ImU32 kRanColor = IM_COL32(255, 205, 70, 255);
constexpr ImU32 kErrorColor = IM_COL32(235, 60, 60, 255);
constexpr ImU32 kWarningColor = IM_COL32(235, 170, 50, 255);
constexpr ImU32 kTextColor = IM_COL32(225, 227, 232, 255);
constexpr ImU32 kDimTextColor = IM_COL32(150, 154, 163, 255);

// Rounds the corners and darkens the controls to match the canvas.
void ApplyEditorStyle();

}  // namespace editor_ui
