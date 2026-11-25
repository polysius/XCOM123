#include "graph_editor/EditorModel.h"

#include <algorithm>
#include <iterator>

#include "graph/GraphFile.h"
#include "graph_editor/AutoLayout.h"

namespace graph_editor {
namespace {

// How far from a pin or a wire, in screen pixels, a click still counts as on it.
constexpr float kPinGrabPixels = kPinRadius + 5.0f;
constexpr float kWireGrabPixels = 6.0f;
// How far a pasted copy of a duplicated node sits from the original, in canvas units.
constexpr glm::vec2 kDuplicateOffset(30.0f, 30.0f);
constexpr int kWireSamples = 24;

float DistanceToSegment(glm::vec2 point, glm::vec2 a, glm::vec2 b) {
    const glm::vec2 along = b - a;
    const float lengthSquared = glm::dot(along, along);
    const float t = lengthSquared > 0.0f ? glm::clamp(glm::dot(point - a, along) / lengthSquared, 0.0f, 1.0f) : 0.0f;
    return glm::length(point - (a + along * t));
}

bool SameWire(const graph::Connection& a, const graph::Connection& b) {
    return a.from == b.from && a.to == b.to;
}

}  // namespace

EditorModel::EditorModel(const graph::NodeRegistry& registry) : registry(registry) {
    savedText = graph::SaveGraph(graph);
    GraphChanged();
}

bool EditorModel::IsModified() const {
    return graph::SaveGraph(graph) != savedText;
}

// ---------------------------------------------------------------------------
// Where things are
// ---------------------------------------------------------------------------

NodeGeometry EditorModel::GeometryOf(uint32_t node) const {
    const graph::Node* found = graph.FindNode(node);
    return found != nullptr ? LayoutNode(graph, registry, *found) : NodeGeometry{};
}

uint32_t EditorModel::NodeAt(glm::vec2 canvasPoint) const {
    // Nodes later in the list are drawn over earlier ones, so they are asked first.
    const std::vector<graph::Node>& nodes = graph.Nodes();
    for (auto node = nodes.rbegin(); node != nodes.rend(); ++node) {
        if (LayoutNode(graph, registry, *node).bounds.Contains(canvasPoint)) {
            return node->id;
        }
    }
    return 0;
}

std::optional<PinHit> EditorModel::PinAt(glm::vec2 canvasPoint) const {
    const float grabRadius = kPinGrabPixels / viewport.Zoom();
    std::optional<PinHit> nearest;
    float nearestDistance = grabRadius;
    for (const graph::Node& node : graph.Nodes()) {
        const NodeGeometry geometry = LayoutNode(graph, registry, node);
        for (const std::vector<PinSlot>* side : {&geometry.inputs, &geometry.outputs}) {
            for (const PinSlot& slot : *side) {
                const float distance = glm::length(slot.position - canvasPoint);
                if (distance <= nearestDistance) {
                    nearestDistance = distance;
                    nearest = PinHit{slot.ref, slot.isOutput};
                }
            }
        }
    }
    return nearest;
}

glm::vec2 EditorModel::PinPosition(const PinHit& pin) const {
    const NodeGeometry geometry = GeometryOf(pin.pin.node);
    const std::vector<PinSlot>& slots = pin.isOutput ? geometry.outputs : geometry.inputs;
    const size_t index = static_cast<size_t>(pin.pin.pin);
    return index < slots.size() ? slots[index].position : glm::vec2(0.0f);
}

std::optional<graph::Connection> EditorModel::ConnectionAt(glm::vec2 canvasPoint) const {
    const float grabDistance = kWireGrabPixels / viewport.Zoom();
    for (const graph::Connection& connection : graph.Connections()) {
        const WireCurve curve = MakeWireCurve(PinPosition({connection.from, true}), PinPosition({connection.to, false}));
        glm::vec2 previous = curve.start;
        for (int step = 1; step <= kWireSamples; ++step) {
            const glm::vec2 next = curve.PointAt(static_cast<float>(step) / static_cast<float>(kWireSamples));
            if (DistanceToSegment(canvasPoint, previous, next) <= grabDistance) {
                return connection;
            }
            previous = next;
        }
    }
    return std::nullopt;
}

std::optional<Rect> EditorModel::GraphBounds() const {
    if (graph.Nodes().empty()) {
        return std::nullopt;
    }
    Rect bounds = LayoutNode(graph, registry, graph.Nodes().front()).bounds;
    for (const graph::Node& node : graph.Nodes()) {
        const Rect nodeBounds = LayoutNode(graph, registry, node).bounds;
        bounds.min = glm::min(bounds.min, nodeBounds.min);
        bounds.max = glm::max(bounds.max, nodeBounds.max);
    }
    return bounds;
}

// ---------------------------------------------------------------------------
// Selection
// ---------------------------------------------------------------------------

void EditorModel::SelectNode(uint32_t node, SelectMode mode) {
    if (graph.FindNode(node) == nullptr) {
        return;
    }
    selectedConnection.reset();
    switch (mode) {
    case SelectMode::Replace:
        selection = {node};
        break;
    case SelectMode::Add:
        selection.insert(node);
        break;
    case SelectMode::Toggle:
        if (selection.erase(node) == 0) {
            selection.insert(node);
        }
        break;
    }
}

void EditorModel::SelectInRect(const Rect& rect, SelectMode mode) {
    selectedConnection.reset();
    if (mode == SelectMode::Replace) {
        selection.clear();
    }
    for (const graph::Node& node : graph.Nodes()) {
        if (!LayoutNode(graph, registry, node).bounds.Overlaps(rect)) {
            continue;
        }
        if (mode == SelectMode::Toggle && selection.erase(node.id) != 0) {
            continue;
        }
        selection.insert(node.id);
    }
}

void EditorModel::SelectAll() {
    selectedConnection.reset();
    selection.clear();
    for (const graph::Node& node : graph.Nodes()) {
        selection.insert(node.id);
    }
}

void EditorModel::ClearSelection() {
    selection.clear();
    selectedConnection.reset();
}

// ---------------------------------------------------------------------------
// Changes to the graph
// ---------------------------------------------------------------------------

void EditorModel::BeginEdit(const std::string& mergeKey) {
    history.Record(graph::SaveGraph(graph), mergeKey);
}

void EditorModel::GraphChanged() {
    issues = graph::Validate(graph, registry);
}

void EditorModel::MoveSelection(glm::vec2 delta) {
    if (selection.empty() || delta == glm::vec2(0.0f)) {
        return;
    }
    BeginEdit("move");
    for (uint32_t id : selection) {
        if (graph::Node* node = graph.FindNode(id)) {
            node->position += delta;
        }
    }
}

uint32_t EditorModel::AddNode(const std::string& type, glm::vec2 position) {
    if (registry.Find(type) == nullptr) {
        status = "There is no kind of node called " + type + ".";
        return 0;
    }
    BeginEdit();
    const uint32_t id = graph.AddNode(type, position);
    SelectNode(id);
    GraphChanged();
    return id;
}

std::vector<PaletteEntry> EditorModel::SearchPalette(const std::string& query) const {
    return graph_editor::SearchPalette(graph, registry, query, pendingWire ? WishOf(pendingWire->start) : std::nullopt);
}

uint32_t EditorModel::AddNodeFromPalette(const PaletteEntry& entry, glm::vec2 position) {
    if (registry.Find(entry.typeId) == nullptr) {
        status = "There is no kind of node called " + entry.typeId + ".";
        return 0;
    }
    BeginEdit();
    const uint32_t id = graph.AddNode(entry.typeId, position);
    if (!entry.variable.empty()) {
        graph.FindNode(id)->properties["variable"] = entry.variable;
    }
    if (pendingWire && entry.connectPin >= 0) {
        const PinHit& start = pendingWire->start;
        const graph::PinRef newPin{id, entry.connectPin};
        const std::string why = start.isOutput ? graph.Connect(registry, start.pin, newPin) : graph.Connect(registry, newPin, start.pin);
        status = why.empty() ? "Added " + entry.title + " and wired it." : why;
    } else {
        status = "Added " + entry.title + ".";
    }
    pendingWire.reset();
    SelectNode(id);
    GraphChanged();
    return id;
}

void EditorModel::DeleteSelection() {
    if (!selection.empty()) {
        BeginEdit();
        for (uint32_t id : selection) {
            graph.RemoveNode(id);
        }
        status = "Deleted " + std::to_string(selection.size()) + (selection.size() == 1 ? " node." : " nodes.");
        ClearSelection();
        GraphChanged();
    } else if (selectedConnection) {
        DeleteConnection(*selectedConnection);
    }
}

void EditorModel::DeleteConnection(const graph::Connection& connection) {
    if (graph.ConnectionInto(connection.to) == nullptr) {
        return;
    }
    BeginEdit();
    graph.DisconnectInput(connection.to);
    if (selectedConnection && SameWire(*selectedConnection, connection)) {
        selectedConnection.reset();
    }
    status = "Deleted a wire.";
    GraphChanged();
}

void EditorModel::BreakLinks(const PinHit& pin) {
    const bool hasLinks = pin.isOutput ? !graph.ConnectionsFrom(pin.pin).empty() : graph.ConnectionInto(pin.pin) != nullptr;
    if (!hasLinks) {
        return;
    }
    BeginEdit();
    if (pin.isOutput) {
        graph.DisconnectOutput(pin.pin);
    } else {
        graph.DisconnectInput(pin.pin);
    }
    KeepOnlyThingsThatExist();
    GraphChanged();
}

// ---------------------------------------------------------------------------
// Copying
// ---------------------------------------------------------------------------

EditorModel::Clipboard EditorModel::CollectSelection() const {
    Clipboard contents;
    for (const graph::Node& node : graph.Nodes()) {
        if (IsSelected(node.id)) {
            contents.nodes.push_back(node);
        }
    }
    for (const graph::Connection& connection : graph.Connections()) {
        if (IsSelected(connection.from.node) && IsSelected(connection.to.node)) {
            contents.connections.push_back(connection);
        }
    }
    return contents;
}

void EditorModel::PasteFrom(const Clipboard& contents, glm::vec2 anchor) {
    if (contents.nodes.empty()) {
        return;
    }
    glm::vec2 topLeft = contents.nodes.front().position;
    for (const graph::Node& node : contents.nodes) {
        topLeft = glm::min(topLeft, node.position);
    }

    BeginEdit();
    std::map<uint32_t, uint32_t> newIds;
    for (const graph::Node& original : contents.nodes) {
        const uint32_t id = graph.AddNode(original.type, original.position - topLeft + anchor);
        graph::Node* copy = graph.FindNode(id);
        copy->inputValues = original.inputValues;
        copy->properties = original.properties;
        newIds[original.id] = id;
    }
    for (const graph::Connection& connection : contents.connections) {
        graph.Connect(registry, {newIds.at(connection.from.node), connection.from.pin}, {newIds.at(connection.to.node), connection.to.pin});
    }
    selection.clear();
    selectedConnection.reset();
    for (const auto& [oldId, newId] : newIds) {
        selection.insert(newId);
    }
    GraphChanged();
}

void EditorModel::Copy() {
    Clipboard contents = CollectSelection();
    if (contents.nodes.empty()) {
        return;
    }
    status = "Copied " + std::to_string(contents.nodes.size()) + (contents.nodes.size() == 1 ? " node." : " nodes.");
    clipboard = std::move(contents);
}

void EditorModel::Cut() {
    Copy();
    DeleteSelection();
}

void EditorModel::Paste(glm::vec2 anchor) {
    PasteFrom(clipboard, anchor);
}

void EditorModel::Duplicate() {
    const Clipboard contents = CollectSelection();
    if (contents.nodes.empty()) {
        return;
    }
    glm::vec2 topLeft = contents.nodes.front().position;
    for (const graph::Node& node : contents.nodes) {
        topLeft = glm::min(topLeft, node.position);
    }
    PasteFrom(contents, topLeft + kDuplicateOffset);
}

// ---------------------------------------------------------------------------
// Values and variables
// ---------------------------------------------------------------------------

void EditorModel::SetInputValue(uint32_t node, int input, const graph::Value& value) {
    graph::Node* target = graph.FindNode(node);
    if (target == nullptr) {
        return;
    }
    const graph::NodePins pins = graph::PinsOf(graph, registry, *target);
    if (input < 0 || input >= static_cast<int>(pins.inputs.size()) || pins.inputs[static_cast<size_t>(input)].type == graph::PinType::Exec) {
        return;
    }
    BeginEdit("value:" + std::to_string(node) + ":" + std::to_string(input));
    target->inputValues[input] = graph::Convert(value, pins.inputs[static_cast<size_t>(input)].type);
    GraphChanged();
}

void EditorModel::SetProperty(uint32_t node, const std::string& name, const graph::Value& value) {
    graph::Node* target = graph.FindNode(node);
    if (target == nullptr) {
        return;
    }
    BeginEdit("property:" + std::to_string(node) + ":" + name);
    target->properties[name] = value;
    GraphChanged();
}

void EditorModel::RemoveWiresThatNoLongerFit(uint32_t node) {
    std::vector<graph::Connection> doomed;
    for (const graph::Connection& connection : graph.Connections()) {
        if (connection.from.node != node && connection.to.node != node) {
            continue;
        }
        const graph::Node* from = graph.FindNode(connection.from.node);
        const graph::Node* to = graph.FindNode(connection.to.node);
        const graph::NodePins fromPins = graph::PinsOf(graph, registry, *from);
        const graph::NodePins toPins = graph::PinsOf(graph, registry, *to);
        const bool pinsExist = connection.from.pin < static_cast<int>(fromPins.outputs.size()) && connection.to.pin < static_cast<int>(toPins.inputs.size());
        const bool fits = pinsExist && graph::CanConvert(fromPins.outputs[static_cast<size_t>(connection.from.pin)].type,
                                                         toPins.inputs[static_cast<size_t>(connection.to.pin)].type);
        if (!fits) {
            doomed.push_back(connection);
        }
    }
    for (const graph::Connection& connection : doomed) {
        graph.DisconnectInput(connection.to);
    }
}

void EditorModel::SetNodeVariable(uint32_t node, const std::string& variable) {
    graph::Node* target = graph.FindNode(node);
    const graph::NodeDescription* description = target != nullptr ? registry.Find(target->type) : nullptr;
    if (description == nullptr || !description->usesVariable || graph.FindVariable(variable) == nullptr) {
        return;
    }
    BeginEdit();
    target->properties["variable"] = variable;
    target->inputValues.clear();  // what was typed was for another type
    RemoveWiresThatNoLongerFit(node);
    KeepOnlyThingsThatExist();
    GraphChanged();
}

std::string EditorModel::AddVariable(const std::string& name, graph::PinType type) {
    if (name.empty()) {
        return "A variable needs a name.";
    }
    if (graph.FindVariable(name) != nullptr) {
        return "There is already a variable called " + name + ".";
    }
    if (type == graph::PinType::Exec) {
        return "A variable cannot hold execution.";
    }
    BeginEdit();
    graph.Variables().push_back({name, type, graph::DefaultValue(type)});
    GraphChanged();
    return {};
}

std::string EditorModel::RemoveVariable(const std::string& name) {
    std::vector<graph::Variable>& variables = graph.Variables();
    const auto found = std::find_if(variables.begin(), variables.end(), [&](const graph::Variable& v) { return v.name == name; });
    if (found == variables.end()) {
        return "There is no variable called " + name + ".";
    }
    BeginEdit();
    variables.erase(found);
    GraphChanged();  // nodes that used it now say so
    return {};
}

std::string EditorModel::RenameVariable(const std::string& name, const std::string& newName) {
    if (newName.empty()) {
        return "A variable needs a name.";
    }
    if (name == newName) {
        return {};
    }
    if (graph.FindVariable(newName) != nullptr) {
        return "There is already a variable called " + newName + ".";
    }
    if (graph.FindVariable(name) == nullptr) {
        return "There is no variable called " + name + ".";
    }
    BeginEdit();
    for (graph::Variable& variable : graph.Variables()) {
        if (variable.name == name) {
            variable.name = newName;
        }
    }
    for (const graph::Node& node : graph.Nodes()) {
        const auto named = node.properties.find("variable");
        if (named != node.properties.end() && std::holds_alternative<std::string>(named->second) && std::get<std::string>(named->second) == name) {
            graph.FindNode(node.id)->properties["variable"] = newName;
        }
    }
    GraphChanged();
    return {};
}

std::string EditorModel::SetVariableType(const std::string& name, graph::PinType type) {
    const graph::Variable* existing = graph.FindVariable(name);
    if (existing == nullptr) {
        return "There is no variable called " + name + ".";
    }
    if (type == graph::PinType::Exec) {
        return "A variable cannot hold execution.";
    }
    BeginEdit();
    for (graph::Variable& variable : graph.Variables()) {
        if (variable.name == name) {
            variable.type = type;
            variable.initial = graph::Convert(variable.initial, type);
        }
    }
    for (const graph::Node& node : graph.Nodes()) {
        const auto named = node.properties.find("variable");
        if (named != node.properties.end() && std::holds_alternative<std::string>(named->second) && std::get<std::string>(named->second) == name) {
            graph.FindNode(node.id)->inputValues.clear();
            RemoveWiresThatNoLongerFit(node.id);
        }
    }
    GraphChanged();
    return {};
}

void EditorModel::SetVariableInitial(const std::string& name, const graph::Value& value) {
    for (graph::Variable& variable : graph.Variables()) {
        if (variable.name == name) {
            BeginEdit("variable:" + name);
            variable.initial = graph::Convert(value, variable.type);
            GraphChanged();
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// Undo
// ---------------------------------------------------------------------------

void EditorModel::RestoreSnapshot(const std::string& snapshot) {
    graph::LoadGraph(snapshot, graph);
    gesture = Gesture::None;
    wireDrag.reset();
    pendingWire.reset();
    KeepOnlyThingsThatExist();
    GraphChanged();
}

void EditorModel::KeepOnlyThingsThatExist() {
    for (auto id = selection.begin(); id != selection.end();) {
        id = graph.FindNode(*id) == nullptr ? selection.erase(id) : std::next(id);
    }
    if (selectedConnection) {
        const graph::Connection* wire = graph.ConnectionInto(selectedConnection->to);
        if (wire == nullptr || !SameWire(*wire, *selectedConnection)) {
            selectedConnection.reset();
        }
    }
}

void EditorModel::Undo() {
    const std::optional<std::string> before = history.Undo(graph::SaveGraph(graph));
    if (!before) {
        status = "Nothing to undo.";
        return;
    }
    RestoreSnapshot(*before);
    status = "Undone.";
}

void EditorModel::Redo() {
    const std::optional<std::string> after = history.Redo(graph::SaveGraph(graph));
    if (!after) {
        status = "Nothing to redo.";
        return;
    }
    RestoreSnapshot(*after);
    status = "Redone.";
}

// ---------------------------------------------------------------------------
// Layout and checking
// ---------------------------------------------------------------------------

void EditorModel::AutoLayoutAll() {
    if (graph.Nodes().empty()) {
        return;
    }
    BeginEdit();
    AutoLayout(graph, registry);
    status = "Arranged the nodes.";
}

std::vector<const graph::Issue*> EditorModel::IssuesFor(uint32_t node) const {
    std::vector<const graph::Issue*> found;
    for (const graph::Issue& issue : issues) {
        if (issue.node == node) {
            found.push_back(&issue);
        }
    }
    return found;
}

}  // namespace graph_editor
