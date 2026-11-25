// The mouse side of the editor model: what pressing, dragging and releasing the primary button does.

#include "graph_editor/EditorModel.h"

namespace graph_editor {
namespace {

// A press that moves less than this many screen pixels before it is released is a click, not a drag.
constexpr float kDragThresholdPixels = 3.0f;

}  // namespace

std::optional<graph::PinType> EditorModel::TypeOfPin(const PinHit& pin) const {
    const NodeGeometry geometry = GeometryOf(pin.pin.node);
    const std::vector<PinSlot>& slots = pin.isOutput ? geometry.outputs : geometry.inputs;
    const size_t index = static_cast<size_t>(pin.pin.pin);
    if (index >= slots.size()) {
        return std::nullopt;
    }
    return slots[index].type;
}

std::optional<WireWish> EditorModel::WishOf(const PinHit& pin) const {
    const std::optional<graph::PinType> type = TypeOfPin(pin);
    if (!type) {
        return std::nullopt;
    }
    return WireWish{*type, pin.isOutput};
}

WireVerdict EditorModel::CheckWire(const PinHit& a, const PinHit& b) const {
    // The graph asks for the output first. It cannot tell an output from an input by its number alone, so two pins on the same
    // side are refused here, before they are given to it.
    if (a.isOutput == b.isOutput) {
        return {false, a.isOutput ? "An output cannot be wired to another output. Wires run from an output to an input."
                                  : "An input cannot be wired to another input. Wires run from an output to an input."};
    }
    const PinHit& output = a.isOutput ? a : b;
    const PinHit& input = a.isOutput ? b : a;
    const std::string why = graph.WhyNotConnect(registry, output.pin, input.pin);
    return {why.empty(), why};
}

WireVerdict EditorModel::ConnectPins(const PinHit& a, const PinHit& b) {
    return MakeWire(a, b, {});
}

WireVerdict EditorModel::MakeWire(const PinHit& a, const PinHit& b, const std::string& mergeKey) {
    const WireVerdict verdict = CheckWire(a, b);
    if (!verdict.accepted) {
        status = verdict.reason;
        return verdict;
    }
    const PinHit& output = a.isOutput ? a : b;
    const PinHit& input = a.isOutput ? b : a;
    BeginEdit(mergeKey);
    graph.Connect(registry, output.pin, input.pin);
    status = "Connected.";
    GraphChanged();
    return verdict;
}

void EditorModel::StartWire(const PinHit& start, glm::vec2 point) {
    PinHit dragged = start;
    if (!start.isOutput) {
        // An input has room for one wire. Pressing on a wired input takes the wire up by its other end, to put it somewhere else.
        if (const graph::Connection* wire = graph.ConnectionInto(start.pin)) {
            dragged = {wire->from, true};
            BeginEdit("wire");
            graph.DisconnectInput(start.pin);
            KeepOnlyThingsThatExist();
            GraphChanged();
        }
    }
    wireDrag = WireDrag{dragged, point, std::nullopt, {}};
    gesture = Gesture::DraggingWire;
}

void EditorModel::UpdateWireDrag(glm::vec2 point) {
    wireDrag->end = point;
    wireDrag->hover = PinAt(point);
    if (wireDrag->hover && *wireDrag->hover == wireDrag->start) {
        wireDrag->hover.reset();  // still over the pin it came from
    }
    if (wireDrag->hover) {
        wireDrag->verdict = CheckWire(wireDrag->start, *wireDrag->hover);
    }
}

ReleaseOutcome EditorModel::FinishWire(glm::vec2 point) {
    UpdateWireDrag(point);
    const WireDrag drag = *wireDrag;
    wireDrag.reset();
    gesture = Gesture::None;

    if (drag.hover) {
        // Taking a wire up and putting it down is one gesture, and one undo step.
        return MakeWire(drag.start, *drag.hover, "wire").accepted ? ReleaseOutcome::WireConnected : ReleaseOutcome::WireRefused;
    }
    const bool barelyMoved = glm::length(point - pressPoint) * viewport.Zoom() < kDragThresholdPixels;
    if (barelyMoved) {
        return ReleaseOutcome::Nothing;
    }
    pendingWire = PendingWire{drag.start, point};
    return ReleaseOutcome::WireDroppedOnEmpty;
}

void EditorModel::PointerPressed(glm::vec2 canvasPoint, bool additive) {
    pressPoint = canvasPoint;
    lastPointer = canvasPoint;
    pressedAdditive = additive;
    movedDuringGesture = false;
    pressedNode.reset();
    pendingWire.reset();

    if (const std::optional<PinHit> pin = PinAt(canvasPoint)) {
        StartWire(*pin, canvasPoint);
        return;
    }
    if (const uint32_t node = NodeAt(canvasPoint)) {
        pressedNode = node;
        if (additive) {
            SelectNode(node, SelectMode::Toggle);
            gesture = IsSelected(node) ? Gesture::MovingNodes : Gesture::None;
        } else {
            if (!IsSelected(node)) {
                SelectNode(node, SelectMode::Replace);
            }
            selectedConnection.reset();
            gesture = Gesture::MovingNodes;
        }
        return;
    }
    if (const std::optional<graph::Connection> wire = ConnectionAt(canvasPoint)) {
        if (!additive) {
            selection.clear();
        }
        selectedConnection = wire;
        gesture = Gesture::None;
        return;
    }
    if (!additive) {
        ClearSelection();
    }
    gesture = Gesture::BoxSelecting;
    boxEnd = canvasPoint;
}

void EditorModel::PointerMoved(glm::vec2 canvasPoint) {
    switch (gesture) {
    case Gesture::MovingNodes: {
        // Nothing moves until the pointer has gone a few pixels, so that a click that wobbles does not nudge the node.
        const bool startedMoving = movedDuringGesture || glm::length(canvasPoint - pressPoint) * viewport.Zoom() >= kDragThresholdPixels;
        if (!startedMoving) {
            break;
        }
        movedDuringGesture = true;
        MoveSelection(canvasPoint - lastPointer);
        lastPointer = canvasPoint;
        break;
    }
    case Gesture::DraggingWire:
        UpdateWireDrag(canvasPoint);
        break;
    case Gesture::BoxSelecting:
        boxEnd = canvasPoint;
        break;
    case Gesture::None:
        break;
    }
}

ReleaseOutcome EditorModel::PointerReleased(glm::vec2 canvasPoint) {
    ReleaseOutcome outcome = ReleaseOutcome::Nothing;
    switch (gesture) {
    case Gesture::MovingNodes:
        PointerMoved(canvasPoint);
        if (movedDuringGesture) {
            outcome = ReleaseOutcome::MovedNodes;
        } else {
            // A click on a node of a selection of several makes it the only one selected, as a click does in a file manager.
            if (!pressedAdditive && pressedNode) {
                SelectNode(*pressedNode, SelectMode::Replace);
            }
            outcome = ReleaseOutcome::Clicked;
        }
        break;
    case Gesture::DraggingWire:
        outcome = FinishWire(canvasPoint);
        break;
    case Gesture::BoxSelecting: {
        boxEnd = canvasPoint;
        const bool wasClick = glm::length(boxEnd - pressPoint) * viewport.Zoom() < kDragThresholdPixels;
        if (!wasClick) {
            SelectInRect(RectFromCorners(pressPoint, boxEnd), pressedAdditive ? SelectMode::Add : SelectMode::Replace);
        }
        outcome = wasClick ? ReleaseOutcome::Clicked : ReleaseOutcome::BoxSelected;
        break;
    }
    case Gesture::None:
        outcome = pressedNode || selectedConnection ? ReleaseOutcome::Clicked : ReleaseOutcome::Nothing;
        break;
    }
    gesture = Gesture::None;
    history.EndGesture();
    return outcome;
}

void EditorModel::CancelGesture() {
    gesture = Gesture::None;
    wireDrag.reset();
    history.EndGesture();
}

std::optional<Rect> EditorModel::ActiveSelectionBox() const {
    if (gesture != Gesture::BoxSelecting) {
        return std::nullopt;
    }
    return RectFromCorners(pressPoint, boxEnd);
}

}  // namespace graph_editor
