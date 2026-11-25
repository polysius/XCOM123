#include "graph_editor/NodeLayout.h"

#include <algorithm>

namespace graph_editor {
namespace {

float TextWidth(const std::string& text) {
    return static_cast<float>(text.size()) * kCharacterWidth;
}

bool NeedsValueBox(const graph::PinDescription& pin) {
    return pin.type != graph::PinType::Exec;
}

// The width that the names of one row and the box for a value need, so that no row runs into its neighbour.
float WidestRow(const graph::NodePins& pins) {
    float widest = 0.0f;
    const size_t rows = std::max(pins.inputs.size(), pins.outputs.size());
    for (size_t row = 0; row < rows; ++row) {
        float width = 0.0f;
        if (row < pins.inputs.size()) {
            width += 2.0f * kPinRadius + TextWidth(pins.inputs[row].name) + (NeedsValueBox(pins.inputs[row]) ? kInlineValueWidth : 0.0f);
        }
        if (row < pins.outputs.size()) {
            width += 2.0f * kPinRadius + TextWidth(pins.outputs[row].name);
        }
        widest = std::max(widest, width);
    }
    return widest + 3.0f * kNodePadding;
}

}  // namespace

std::string NodeTitle(const graph::NodeRegistry& registry, const graph::Node& node) {
    const graph::NodeDescription* description = registry.Find(node.type);
    if (description == nullptr) {
        return node.type;
    }
    if (description->usesVariable) {
        const auto named = node.properties.find("variable");
        if (named != node.properties.end() && std::holds_alternative<std::string>(named->second)) {
            return description->title + " " + std::get<std::string>(named->second);
        }
    }
    return description->title;
}

NodeGeometry LayoutNode(const graph::Graph& graph, const graph::NodeRegistry& registry, const graph::Node& node) {
    const graph::NodePins pins = graph::PinsOf(graph, registry, node);
    const size_t rows = std::max(pins.inputs.size(), pins.outputs.size());
    const float width = std::max({kMinNodeWidth, TextWidth(NodeTitle(registry, node)) + 2.0f * kNodePadding, WidestRow(pins)});
    const float height = kHeaderHeight + static_cast<float>(rows) * kRowHeight + kNodePadding * 0.5f;

    NodeGeometry geometry;
    geometry.bounds = {node.position, node.position + glm::vec2(width, height)};
    geometry.header = {node.position, node.position + glm::vec2(width, kHeaderHeight)};

    auto rowCentreY = [&](size_t row) { return node.position.y + kHeaderHeight + (static_cast<float>(row) + 0.5f) * kRowHeight; };
    for (size_t index = 0; index < pins.inputs.size(); ++index) {
        const glm::vec2 centre(node.position.x, rowCentreY(index));
        geometry.inputs.push_back({{node.id, static_cast<int>(index)}, false, pins.inputs[index].type, pins.inputs[index].name, centre});
    }
    for (size_t index = 0; index < pins.outputs.size(); ++index) {
        const glm::vec2 centre(node.position.x + width, rowCentreY(index));
        geometry.outputs.push_back({{node.id, static_cast<int>(index)}, true, pins.outputs[index].type, pins.outputs[index].name, centre});
    }
    return geometry;
}

glm::vec2 WireCurve::PointAt(float t) const {
    const float u = 1.0f - t;
    return u * u * u * start + 3.0f * u * u * t * control1 + 3.0f * u * t * t * control2 + t * t * t * end;
}

WireCurve MakeWireCurve(glm::vec2 from, glm::vec2 to) {
    // A wire leaves an output to the right and enters an input from the left. The handles are longer the farther apart the pins
    // are, but never so short that a wire between two close pins is a straight line.
    const float reach = std::max(50.0f, std::abs(to.x - from.x) * 0.5f);
    return {from, from + glm::vec2(reach, 0.0f), to - glm::vec2(reach, 0.0f), to};
}

}  // namespace graph_editor
