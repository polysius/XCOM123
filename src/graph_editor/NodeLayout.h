#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "graph/Graph.h"
#include "graph_editor/Viewport.h"

// Where the parts of a node sit on the canvas. The editor model hit-tests with these and the view draws with them, so what a
// person sees under the pointer is what a click finds.
//
// A node is a header with its title, then rows. Row i holds input i on the left and output i on the right. An input of a data
// type that nothing is wired into has room for a small box next to its name, to type a value in.

namespace graph_editor {

constexpr float kHeaderHeight = 28.0f;
constexpr float kRowHeight = 24.0f;
constexpr float kNodePadding = 10.0f;
constexpr float kPinRadius = 6.0f;
constexpr float kMinNodeWidth = 120.0f;
constexpr float kInlineValueWidth = 64.0f;  // the box for a number or a yes/no; text and vectors have wider ones, see InlineValueWidth
constexpr float kCharacterWidth = 7.0f;  // an estimate of the width of the font, so that a name does not run out of its node

struct PinSlot {
    graph::PinRef ref;
    bool isOutput = false;
    graph::PinType type = graph::PinType::Float;
    std::string name;
    glm::vec2 position{0.0f};  // the centre of the pin, on the canvas
};

struct NodeGeometry {
    Rect bounds;
    Rect header;
    std::vector<PinSlot> inputs;
    std::vector<PinSlot> outputs;
};

// How much room the box for a typed value takes beside the name of an input of this type.
float InlineValueWidth(graph::PinType type);

// What a node is called on the canvas: its kind's title, followed by the variable for the nodes that work on one.
std::string NodeTitle(const graph::NodeRegistry& registry, const graph::Node& node);

// The room a node takes on the canvas. A node of an unknown kind is an empty box with its kind as the title.
NodeGeometry LayoutNode(const graph::Graph& graph, const graph::NodeRegistry& registry, const graph::Node& node);

// The first and last control points of the curve a wire makes between two pins, with the two between them that make it leave
// and arrive sideways. The view draws the same curve, and a click on a wire is tested against it.
struct WireCurve {
    glm::vec2 start;
    glm::vec2 control1;
    glm::vec2 control2;
    glm::vec2 end;
    glm::vec2 PointAt(float t) const;
};
WireCurve MakeWireCurve(glm::vec2 from, glm::vec2 to);

}  // namespace graph_editor
