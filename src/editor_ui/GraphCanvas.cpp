#include "editor_ui/GraphCanvas.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "editor_ui/EditorTheme.h"
#include "editor_ui/ValueWidgets.h"

namespace editor_ui {
namespace {

using graph_editor::kHeaderHeight;
using graph_editor::kPinRadius;

constexpr float kGridSpacing = 24.0f;       // canvas units between dots
constexpr int kMajorGridEvery = 5;          // every fifth dot is larger
constexpr float kMinimumDotSpacing = 9.0f;  // dots closer than this on screen are thinned out
constexpr float kWheelZoomStep = 1.12f;
constexpr float kRightDragThreshold = 4.0f;  // pixels the pointer must move with the right button down to pan and not open a menu
constexpr float kRoundingCanvasUnits = 6.0f;
constexpr float kTooltipDelaySeconds = 0.4f;
constexpr float kInlineBoxMinimumZoom = 0.55f;  // below this the boxes for typing are too small to use and are left out
constexpr float kLabelMinimumZoom = 0.4f;
constexpr ImVec2 kMinimapSize(190.0f, 120.0f);
constexpr float kMinimapMargin = 12.0f;
constexpr float kPasteStepWithoutPointer = 40.0f;

ImVec2 operator+(ImVec2 a, ImVec2 b) { return {a.x + b.x, a.y + b.y}; }
ImVec2 operator-(ImVec2 a, ImVec2 b) { return {a.x - b.x, a.y - b.y}; }
ImVec2 operator*(ImVec2 a, float s) { return {a.x * s, a.y * s}; }
float Length(ImVec2 v) { return std::sqrt(v.x * v.x + v.y * v.y); }

// A pin is a circle for data and a triangle for execution; filled while a wire is on it.
void DrawPinShape(ImDrawList* list, ImVec2 centre, graph::PinType type, bool connected, float radius, ImU32 color) {
    if (type == graph::PinType::Exec) {
        const ImVec2 top(centre.x - radius * 0.8f, centre.y - radius);
        const ImVec2 tip(centre.x + radius, centre.y);
        const ImVec2 bottom(centre.x - radius * 0.8f, centre.y + radius);
        if (connected) {
            list->AddTriangleFilled(top, tip, bottom, color);
        } else {
            list->AddTriangle(top, tip, bottom, color, std::max(1.0f, radius * 0.28f));
        }
        return;
    }
    if (connected) {
        list->AddCircleFilled(centre, radius, color);
    } else {
        list->AddCircle(centre, radius, color, 0, std::max(1.0f, radius * 0.3f));
    }
}

std::string DescribeValueType(graph::PinType type) {
    switch (type) {
    case graph::PinType::Exec: return "Execution";
    case graph::PinType::Bool: return "Boolean (true or false)";
    case graph::PinType::Int: return "Whole number";
    case graph::PinType::Float: return "Decimal number";
    case graph::PinType::Vec3: return "Vector of three numbers";
    case graph::PinType::String: return "Text";
    }
    return "";
}

}  // namespace

// ---------------------------------------------------------------------------
// Frame set-up
// ---------------------------------------------------------------------------

ImVec2 GraphCanvas::ToScreen(glm::vec2 canvasPoint) const {
    const glm::vec2 local = model->View().ToScreen(canvasPoint);
    return {origin.x + local.x, origin.y + local.y};
}

glm::vec2 GraphCanvas::PointerCanvasPosition() const {
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    return model->View().ToCanvas({mouse.x - origin.x, mouse.y - origin.y});
}

void GraphCanvas::ApplyFrameRequest() {
    if (frameRequest == FrameRequest::None) {
        return;
    }
    std::optional<graph_editor::Rect> area;
    for (const graph::Node& node : model->GetGraph().Nodes()) {
        if (frameRequest == FrameRequest::Selection && !model->IsSelected(node.id)) {
            continue;
        }
        const graph_editor::Rect bounds = model->GeometryOf(node.id).bounds;
        if (!area) {
            area = bounds;
        }
        area->min = glm::min(area->min, bounds.min);
        area->max = glm::max(area->max, bounds.max);
    }
    if (area) {
        model->View().Frame(*area, {canvasSize.x, canvasSize.y});
    }
    frameRequest = FrameRequest::None;
}

void GraphCanvas::Draw(graph_editor::EditorModel& editorModel, ImVec2 size, CanvasRequests& requests) {
    model = &editorModel;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::BeginChild("graph canvas", size, ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();

    list = ImGui::GetWindowDrawList();
    origin = ImGui::GetWindowPos();
    canvasSize = ImGui::GetWindowSize();
    ApplyFrameRequest();
    zoom = model->View().Zoom();
    hovered = ImGui::IsWindowHovered();
    controlBusy = ImGui::IsAnyItemHovered() || ImGui::IsAnyItemActive();

    // A value that was being dragged ends its undo step when the control is let go of.
    if (editingValue && !ImGui::IsAnyItemActive()) {
        model->EndGesture();
        editingValue = false;
    }

    geometry.clear();
    for (const graph::Node& node : model->GetGraph().Nodes()) {
        geometry[node.id] = model->GeometryOf(node.id);
    }

    list->PushClipRect(origin, origin + canvasSize, true);
    HandleInput(requests);
    zoom = model->View().Zoom();  // the input may have changed the view

    DrawBackground();
    DrawWires();
    DrawNodes();
    DrawWireInProgress();
    DrawSelectionBox();
    DrawMinimap();
    DrawOverlayText();
    DrawTooltips();
    list->PopClipRect();

    DrawContextMenu();
    ImGui::EndChild();
}

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------

void GraphCanvas::HandleInput(CanvasRequests& requests) {
    const ImGuiIO& io = ImGui::GetIO();
    if (draggingMinimap) {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            MoveViewWithMinimap(io.MousePos);
        } else {
            draggingMinimap = false;
        }
        return;
    }
    if (hovered && !controlBusy && IsOverMinimap(io.MousePos) && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        draggingMinimap = true;
        MoveViewWithMinimap(io.MousePos);
        return;
    }
    HandleViewControls();
    HandlePrimaryButton(requests);
    HandleSecondaryButton(requests);
    HandleKeys(requests);
}

// The wheel zooms toward the pointer, and the middle button or the space bar with the left button drags the canvas.
void GraphCanvas::HandleViewControls() {
    const ImGuiIO& io = ImGui::GetIO();
    const glm::vec2 local(io.MousePos.x - origin.x, io.MousePos.y - origin.y);
    if (hovered && !controlBusy && io.MouseWheel != 0.0f) {
        model->View().ZoomAt(local, std::pow(kWheelZoomStep, io.MouseWheel));
    }
    if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.0f)) {
        model->View().PanByScreen({io.MouseDelta.x, io.MouseDelta.y});
    }
    if (panningWithSpace) {
        panningWithSpace = ImGui::IsMouseDown(ImGuiMouseButton_Left);
        model->View().PanByScreen({io.MouseDelta.x, io.MouseDelta.y});
    }
}

void GraphCanvas::HandlePrimaryButton(CanvasRequests& requests) {
    const ImGuiIO& io = ImGui::GetIO();
    const glm::vec2 point = PointerCanvasPosition();

    const bool canStartPress = hovered && !controlBusy && !pressedInCanvas && !panningWithSpace && ImGui::IsMouseClicked(ImGuiMouseButton_Left);
    if (canStartPress && ImGui::IsKeyDown(ImGuiKey_Space)) {
        panningWithSpace = true;
    } else if (canStartPress) {
        pressedInCanvas = true;
        const bool onEmptyCanvas = !model->PinAt(point) && model->NodeAt(point) == 0 && !model->ConnectionAt(point);
        if (onEmptyCanvas && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            requests.openPalette = true;
            requests.paletteCanvasPosition = point;
        }
        model->PointerPressed(point, io.KeyShift || io.KeyCtrl);
    }
    if (!pressedInCanvas) {
        return;
    }
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        model->PointerMoved(point);
        return;
    }
    pressedInCanvas = false;
    const graph_editor::ReleaseOutcome outcome = model->PointerReleased(point);
    if (outcome == graph_editor::ReleaseOutcome::WireDroppedOnEmpty) {
        requests.openPalette = true;
        requests.paletteCanvasPosition = point;
    }
}

void GraphCanvas::HandleSecondaryButton(CanvasRequests& requests) {
    const ImGuiIO& io = ImGui::GetIO();
    if (hovered && !controlBusy && !IsOverMinimap(io.MousePos) && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        secondaryDown = true;
        secondaryDragged = false;
        secondaryPressPosition = io.MousePos;
    }
    if (!secondaryDown) {
        return;
    }
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        secondaryDragged = secondaryDragged || Length(io.MousePos - secondaryPressPosition) > kRightDragThreshold;
        if (secondaryDragged) {
            model->View().PanByScreen({io.MouseDelta.x, io.MouseDelta.y});
        }
        return;
    }
    secondaryDown = false;
    if (secondaryDragged) {
        return;
    }

    // A right click without a drag opens a menu for what is under the pointer, or the palette on empty canvas.
    const glm::vec2 point = PointerCanvasPosition();
    contextTarget = {};
    contextTarget.pin = model->PinAt(point);
    contextTarget.node = contextTarget.pin ? 0 : model->NodeAt(point);
    if (!contextTarget.pin && contextTarget.node == 0) {
        contextTarget.wire = model->ConnectionAt(point);
    }
    if (contextTarget.node != 0 && !model->IsSelected(contextTarget.node)) {
        model->SelectNode(contextTarget.node);
    }
    if (!contextTarget.pin && contextTarget.node == 0 && !contextTarget.wire) {
        model->CancelPendingWire();
        requests.openPalette = true;
        requests.paletteCanvasPosition = point;
        return;
    }
    ImGui::OpenPopup("canvas context");
}

void GraphCanvas::DrawContextMenu() {
    if (!ImGui::BeginPopup("canvas context")) {
        return;
    }
    if (contextTarget.pin) {
        const graph_editor::PinHit& pin = *contextTarget.pin;
        const bool linked = pin.isOutput ? !model->GetGraph().ConnectionsFrom(pin.pin).empty() : model->GetGraph().ConnectionInto(pin.pin) != nullptr;
        if (ImGui::MenuItem("Break links", nullptr, false, linked)) {
            model->BreakLinks(pin);
        }
        if (!linked) {
            ImGui::TextDisabled("No wires on this pin");
        }
    } else if (contextTarget.node != 0) {
        if (ImGui::MenuItem("Delete", "Del")) {
            model->DeleteSelection();
        }
        if (ImGui::MenuItem("Duplicate", "Ctrl+D")) {
            model->Duplicate();
        }
        if (ImGui::MenuItem("Copy", "Ctrl+C")) {
            model->Copy();
        }
        if (ImGui::MenuItem("Cut", "Ctrl+X")) {
            model->Cut();
        }
    } else if (contextTarget.wire) {
        if (ImGui::MenuItem("Delete wire")) {
            model->DeleteConnection(*contextTarget.wire);
        }
    }
    ImGui::EndPopup();
}

void GraphCanvas::HandleKeys(CanvasRequests& requests) {
    const ImGuiIO& io = ImGui::GetIO();
    const bool windowFocused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
    if (io.WantTextInput || !(hovered || windowFocused)) {
        return;
    }
    const bool control = io.KeyCtrl;
    // Pasted nodes go where the pointer is, or else a little way from the middle of the view.
    const glm::vec2 pasteAt = hovered ? PointerCanvasPosition()
                                      : model->View().ToCanvas({canvasSize.x * 0.5f, canvasSize.y * 0.5f}) + glm::vec2(kPasteStepWithoutPointer);

    if (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) {
        model->DeleteSelection();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        model->CancelGesture();
        pressedInCanvas = false;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F) && !control) {
        RequestFrame(!model->Selection().empty());
    }
    if (!control) {
        return;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Z)) {
        io.KeyShift ? model->Redo() : model->Undo();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Y)) {
        model->Redo();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_C)) {
        model->Copy();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_X)) {
        model->Cut();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_V)) {
        model->Paste(pasteAt);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_D)) {
        model->Duplicate();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_A)) {
        model->SelectAll();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_S)) {
        requests.saveRequested = true;
    }
}

// ---------------------------------------------------------------------------
// Background and wires
// ---------------------------------------------------------------------------

void GraphCanvas::DrawBackground() {
    list->AddRectFilled(origin, origin + canvasSize, kCanvasBackground);
    DrawGrid();
}

// Dots rather than lines, so that the grid is a quiet guide and not a pattern that competes with the wires.
void GraphCanvas::DrawGrid() {
    float step = kGridSpacing * zoom;
    int thinning = 1;
    while (step < kMinimumDotSpacing) {
        step *= 2.0f;
        thinning *= 2;
    }
    const glm::vec2 viewOrigin = model->View().Origin();
    const float spacingInCanvas = kGridSpacing * static_cast<float>(thinning);
    const int firstX = static_cast<int>(std::floor(viewOrigin.x / spacingInCanvas));
    const int firstY = static_cast<int>(std::floor(viewOrigin.y / spacingInCanvas));
    const int columns = static_cast<int>(canvasSize.x / step) + 2;
    const int rows = static_cast<int>(canvasSize.y / step) + 2;
    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < columns; ++column) {
            const int gridX = firstX + column;
            const int gridY = firstY + row;
            const bool major = (gridX % kMajorGridEvery == 0) && (gridY % kMajorGridEvery == 0);
            const ImVec2 at = ToScreen({static_cast<float>(gridX) * spacingInCanvas, static_cast<float>(gridY) * spacingInCanvas});
            list->AddCircleFilled(at, (major ? 1.8f : 1.1f), major ? kGridDotMajor : kGridDot, 6);
        }
    }
}

void GraphCanvas::DrawWires() {
    const graph::Graph& graph = model->GetGraph();
    const double time = ImGui::GetTime();
    for (const graph::Connection& connection : graph.Connections()) {
        const auto from = geometry.find(connection.from.node);
        const auto to = geometry.find(connection.to.node);
        if (from == geometry.end() || to == geometry.end()) {
            continue;
        }
        const size_t outputIndex = static_cast<size_t>(connection.from.pin);
        const size_t inputIndex = static_cast<size_t>(connection.to.pin);
        if (outputIndex >= from->second.outputs.size() || inputIndex >= to->second.inputs.size()) {
            continue;
        }
        const graph_editor::PinSlot& output = from->second.outputs[outputIndex];
        const graph_editor::PinSlot& input = to->second.inputs[inputIndex];
        const graph_editor::WireCurve curve = graph_editor::MakeWireCurve(output.position, input.position);

        const bool isExec = output.type == graph::PinType::Exec;
        const bool selected = model->SelectedConnection() && model->SelectedConnection()->from == connection.from && model->SelectedConnection()->to == connection.to;
        const float thickness = (isExec ? 3.2f : 2.0f) * std::max(zoom, 0.5f);
        const float justRan = isExec ? model->NodeGlow(connection.from.node) : 0.0f;
        const ImU32 color = ColorOfPinType(output.type);

        const ImVec2 p0 = ToScreen(curve.start);
        const ImVec2 p1 = ToScreen(curve.control1);
        const ImVec2 p2 = ToScreen(curve.control2);
        const ImVec2 p3 = ToScreen(curve.end);
        if (selected) {
            list->AddBezierCubic(p0, p1, p2, p3, kSelectionColor, thickness + 4.0f);
        }
        if (justRan > 0.0f) {
            list->AddBezierCubic(p0, p1, p2, p3, WithOpacity(kRanColor, justRan * 0.8f), thickness + 5.0f);
        }
        list->AddBezierCubic(p0, p1, p2, p3, color, thickness);

        // While the graph runs, dots travel along the execution wires in the direction that execution goes.
        if (isExec && model->IsPlaying()) {
            const float length = Length(p3 - p0);
            const int dots = std::max(2, static_cast<int>(length / 70.0f));
            for (int dot = 0; dot < dots; ++dot) {
                const float t = static_cast<float>(std::fmod(time * 0.7 + static_cast<double>(dot) / static_cast<double>(dots), 1.0));
                const glm::vec2 at = curve.PointAt(t);
                list->AddCircleFilled(ToScreen(at), (2.4f + 1.6f * justRan) * std::max(zoom, 0.6f), WithOpacity(IM_COL32(255, 255, 255, 255), 0.55f + 0.45f * justRan));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Nodes
// ---------------------------------------------------------------------------

void GraphCanvas::DrawNodes() {
    const ImVec2 windowMax = origin + canvasSize;
    ImGui::SetWindowFontScale(1.0f);
    for (const graph::Node& node : model->GetGraph().Nodes()) {
        const graph_editor::NodeGeometry& nodeGeometry = geometry[node.id];
        const ImVec2 low = ToScreen(nodeGeometry.bounds.min);
        const ImVec2 high = ToScreen(nodeGeometry.bounds.max);
        if (high.x < origin.x || high.y < origin.y || low.x > windowMax.x || low.y > windowMax.y) {
            continue;
        }
        DrawNode(node);
    }
}

void GraphCanvas::DrawNode(const graph::Node& node) {
    const graph_editor::NodeGeometry& nodeGeometry = geometry[node.id];
    const graph::NodeDescription* description = model->Registry().Find(node.type);
    const graph::NodePins pins = graph::PinsOf(model->GetGraph(), model->Registry(), node);
    const ImVec2 low = ToScreen(nodeGeometry.bounds.min);
    const ImVec2 high = ToScreen(nodeGeometry.bounds.max);
    const float rounding = kRoundingCanvasUnits * zoom;

    DrawNodeStatusMarks(node, nodeGeometry);
    list->AddRectFilled(low, high, kNodeBody, rounding);
    DrawNodeHeader(node, nodeGeometry, description);
    list->AddRect(low, high, model->IsSelected(node.id) ? kSelectionColor : kNodeBorder, rounding, 0, model->IsSelected(node.id) ? 2.5f : 1.0f);
    DrawNodePins(node, nodeGeometry, pins);
}

// The outline that says what is happening to a node: lit while it ran a moment ago, red or amber for a problem with it.
void GraphCanvas::DrawNodeStatusMarks(const graph::Node& node, const graph_editor::NodeGeometry& nodeGeometry) {
    const ImVec2 low = ToScreen(nodeGeometry.bounds.min);
    const ImVec2 high = ToScreen(nodeGeometry.bounds.max);
    const float rounding = kRoundingCanvasUnits * zoom;
    const float glow = model->NodeGlow(node.id);
    if (glow > 0.0f) {
        const float reach = (3.0f + 7.0f * glow) * std::max(zoom, 0.6f);
        list->AddRectFilled(low - ImVec2(reach, reach), high + ImVec2(reach, reach), WithOpacity(kRanColor, 0.25f * glow), rounding + reach);
        list->AddRect(low - ImVec2(2.0f, 2.0f), high + ImVec2(2.0f, 2.0f), WithOpacity(kRanColor, glow), rounding + 2.0f, 0, 3.0f);
    }
    bool hasError = false;
    bool hasWarning = false;
    for (const graph::Issue* issue : model->IssuesFor(node.id)) {
        (issue->isError ? hasError : hasWarning) = true;
    }
    if (hasError) {
        list->AddRect(low - ImVec2(2.0f, 2.0f), high + ImVec2(2.0f, 2.0f), kErrorColor, rounding + 2.0f, 0, 3.0f);
    } else if (hasWarning) {
        list->AddRect(low - ImVec2(1.0f, 1.0f), high + ImVec2(1.0f, 1.0f), kWarningColor, rounding + 1.0f, 0, 1.6f);
    }
}

void GraphCanvas::DrawNodeHeader(const graph::Node& node, const graph_editor::NodeGeometry& nodeGeometry, const graph::NodeDescription* description) {
    const ImVec2 low = ToScreen(nodeGeometry.header.min);
    const ImVec2 high = ToScreen(nodeGeometry.header.max);
    list->AddRectFilled(low, high, ColorOfNodeKind(description), kRoundingCanvasUnits * zoom, ImDrawFlags_RoundCornersTop);

    const float fontSize = ImGui::GetFontSize() * zoom;
    const std::string title = graph_editor::NodeTitle(model->Registry(), node);
    if (zoom >= kLabelMinimumZoom) {
        const ImVec2 textAt(low.x + graph_editor::kNodePadding * zoom, low.y + (high.y - low.y - fontSize) * 0.5f);
        list->AddText(ImGui::GetFont(), fontSize, textAt, IM_COL32(255, 255, 255, 255), title.c_str());
    }

    // A badge in the header for a node with a problem, so that it is seen even when the outline is hidden by another node.
    bool hasError = false;
    bool hasProblem = false;
    for (const graph::Issue* issue : model->IssuesFor(node.id)) {
        hasProblem = true;
        hasError = hasError || issue->isError;
    }
    if (hasProblem) {
        const float radius = 8.0f * zoom;
        const ImVec2 centre(high.x - radius - 5.0f * zoom, (low.y + high.y) * 0.5f);
        list->AddCircleFilled(centre, radius, hasError ? kErrorColor : kWarningColor);
        if (zoom >= kLabelMinimumZoom) {
            list->AddText(ImGui::GetFont(), fontSize, centre - ImVec2(fontSize * 0.2f, fontSize * 0.5f), IM_COL32(20, 20, 20, 255), "!");
        }
    }
}

graph_editor::WireVerdict GraphCanvas::VerdictForPin(const graph_editor::PinHit& pin) const {
    const std::optional<graph_editor::WireDrag>& drag = model->ActiveWireDrag();
    return drag ? model->CheckWire(drag->start, pin) : graph_editor::WireVerdict{true, {}};
}

void GraphCanvas::DrawNodePins(const graph::Node& node, const graph_editor::NodeGeometry& nodeGeometry, const graph::NodePins& pins) {
    const graph::Graph& graph = model->GetGraph();
    const bool dragging = model->ActiveWireDrag().has_value();
    const float fontSize = ImGui::GetFont()->FontSize * zoom;
    const float radius = kPinRadius * zoom;
    const float labelGap = (kPinRadius + 4.0f) * zoom;

    auto drawSide = [&](const std::vector<graph_editor::PinSlot>& slots, const std::vector<graph::PinDescription>& descriptions) {
        for (size_t index = 0; index < slots.size(); ++index) {
            const graph_editor::PinSlot& slot = slots[index];
            const bool connected = slot.isOutput ? !graph.ConnectionsFrom(slot.ref).empty() : graph.ConnectionInto(slot.ref) != nullptr;
            const ImVec2 centre = ToScreen(slot.position);
            ImU32 color = ColorOfPinType(slot.type);
            if (dragging) {
                const bool fits = VerdictForPin({slot.ref, slot.isOutput}).accepted;
                color = fits ? color : WithOpacity(color, 0.25f);
                if (fits && model->ActiveWireDrag()->start.pin.node != node.id) {
                    list->AddCircle(centre, radius + 3.0f, kSelectionColor, 0, 1.5f);  // a ring on every pin the wire can end at
                }
            }
            DrawPinShape(list, centre, slot.type, connected, radius, color);
            if (zoom < kLabelMinimumZoom) {
                continue;
            }
            const char* name = slot.name.c_str();
            const ImVec2 textSize = ImGui::GetFont()->CalcTextSizeA(fontSize, FLT_MAX, 0.0f, name);
            const float textY = centre.y - textSize.y * 0.5f;
            if (slot.isOutput) {
                list->AddText(ImGui::GetFont(), fontSize, ImVec2(centre.x - labelGap - textSize.x, textY), kTextColor, name);
            } else {
                list->AddText(ImGui::GetFont(), fontSize, ImVec2(centre.x + labelGap, textY), kTextColor, name);
                if (!connected && slot.type != graph::PinType::Exec && zoom >= kInlineBoxMinimumZoom && index < descriptions.size()) {
                    DrawInlineValue(node, slot, descriptions[index], textSize.x);
                }
            }
        }
    };
    drawSide(nodeGeometry.inputs, pins.inputs);
    drawSide(nodeGeometry.outputs, pins.outputs);
}

// The box beside an input that nothing is wired into, to type its value in, as Blueprints does.
void GraphCanvas::DrawInlineValue(const graph::Node& node, const graph_editor::PinSlot& slot, const graph::PinDescription& pin, float labelWidth) {
    // A node on top of this one hides the box, and so must not leave a control that still answers to the pointer.
    const float boxWidth = graph_editor::InlineValueWidth(slot.type);
    const glm::vec2 boxMiddle = slot.position + glm::vec2((kPinRadius + 4.0f) + labelWidth / zoom + 6.0f + boxWidth * 0.5f, 0.0f);
    if (model->NodeAt(boxMiddle) != node.id) {
        return;
    }

    const auto typed = node.inputValues.find(slot.ref.pin);
    graph::Value value = typed != node.inputValues.end() ? typed->second : pin.defaultValue;

    const ImGuiStyle& style = ImGui::GetStyle();
    const float frameHeight = (ImGui::GetFontSize() + 2.0f * 2.0f) * zoom;
    const ImVec2 centre = ToScreen(slot.position);
    const float x = centre.x + (kPinRadius + 4.0f) * zoom + labelWidth + 6.0f * zoom;

    ImGui::SetWindowFontScale(zoom);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(3.0f * zoom, 2.0f * zoom));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemInnerSpacing, ImVec2(2.0f * zoom, style.ItemInnerSpacing.y));
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 3.0f * zoom);
    ImGui::PushID(static_cast<int>(node.id));
    ImGui::PushID(slot.ref.pin);
    ImGui::SetCursorScreenPos(ImVec2(x, centre.y - frameHeight * 0.5f));
    if (EditValue("value", value, slot.type, boxWidth * zoom)) {
        model->SetInputValue(node.id, slot.ref.pin, value);
        editingValue = true;
    }
    ImGui::PopID();
    ImGui::PopID();
    ImGui::PopStyleVar(3);
    ImGui::SetWindowFontScale(1.0f);
}

// ---------------------------------------------------------------------------
// What the pointer is doing
// ---------------------------------------------------------------------------

void GraphCanvas::DrawWireInProgress() {
    const std::optional<graph_editor::WireDrag>& drag = model->ActiveWireDrag();
    if (!drag) {
        return;
    }
    const graph_editor::PinHit& start = drag->start;
    const graph_editor::NodeGeometry startGeometry = model->GeometryOf(start.pin.node);
    const std::vector<graph_editor::PinSlot>& startSlots = start.isOutput ? startGeometry.outputs : startGeometry.inputs;
    if (static_cast<size_t>(start.pin.pin) >= startSlots.size()) {
        return;
    }
    const graph_editor::PinSlot& startSlot = startSlots[static_cast<size_t>(start.pin.pin)];

    // The wire ends on the pin it would join, or follows the pointer.
    glm::vec2 end = drag->end;
    if (drag->hover) {
        const graph_editor::NodeGeometry hoverGeometry = model->GeometryOf(drag->hover->pin.node);
        const std::vector<graph_editor::PinSlot>& hoverSlots = drag->hover->isOutput ? hoverGeometry.outputs : hoverGeometry.inputs;
        if (static_cast<size_t>(drag->hover->pin.pin) < hoverSlots.size()) {
            end = hoverSlots[static_cast<size_t>(drag->hover->pin.pin)].position;
        }
    }
    const graph_editor::WireCurve curve = start.isOutput ? graph_editor::MakeWireCurve(startSlot.position, end) : graph_editor::MakeWireCurve(end, startSlot.position);
    const bool refused = drag->hover && !drag->verdict.accepted;
    const ImU32 color = refused ? kErrorColor : ColorOfPinType(startSlot.type);
    const float thickness = (startSlot.type == graph::PinType::Exec ? 3.2f : 2.0f) * std::max(zoom, 0.5f);
    list->AddBezierCubic(ToScreen(curve.start), ToScreen(curve.control1), ToScreen(curve.control2), ToScreen(curve.end), color, thickness);

    // Says what letting go would do, beside the pointer.
    std::string message = "Release on empty canvas to add a node that fits";
    ImU32 messageColor = kDimTextColor;
    if (drag->hover) {
        message = drag->verdict.accepted ? "Connect" : drag->verdict.reason;
        messageColor = drag->verdict.accepted ? IM_COL32(120, 230, 130, 255) : kErrorColor;
    }
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const ImVec2 textSize = ImGui::CalcTextSize(message.c_str());
    const ImVec2 boxLow = mouse + ImVec2(16.0f, 14.0f);
    list->AddRectFilled(boxLow - ImVec2(5.0f, 3.0f), boxLow + textSize + ImVec2(5.0f, 3.0f), IM_COL32(15, 15, 18, 235), 4.0f);
    list->AddText(boxLow, messageColor, message.c_str());
}

void GraphCanvas::DrawSelectionBox() {
    const std::optional<graph_editor::Rect> box = model->ActiveSelectionBox();
    if (!box) {
        return;
    }
    const ImVec2 low = ToScreen(box->min);
    const ImVec2 high = ToScreen(box->max);
    list->AddRectFilled(low, high, IM_COL32(90, 150, 255, 45));
    list->AddRect(low, high, IM_COL32(120, 175, 255, 230), 0.0f, 0, 1.2f);
}

// ---------------------------------------------------------------------------
// Minimap, text on the canvas and tooltips
// ---------------------------------------------------------------------------

bool GraphCanvas::IsOverMinimap(ImVec2 point) const {
    const ImVec2 high = origin + canvasSize - ImVec2(kMinimapMargin, kMinimapMargin);
    const ImVec2 low = high - kMinimapSize;
    return point.x >= low.x && point.x <= high.x && point.y >= low.y && point.y <= high.y;
}

namespace {

// How the canvas maps into the minimap: its bounds are the nodes and the part of the canvas in view together.
struct MinimapMapping {
    graph_editor::Rect world;
    ImVec2 low;
    float scale = 1.0f;
    ImVec2 ToMinimap(glm::vec2 canvasPoint) const { return {low.x + (canvasPoint.x - world.min.x) * scale, low.y + (canvasPoint.y - world.min.y) * scale}; }
};

}  // namespace

static MinimapMapping MapMinimap(const graph_editor::EditorModel& model, ImVec2 low, ImVec2 canvasSize) {
    graph_editor::Rect world = model.GraphBounds().value_or(graph_editor::Rect{});
    const graph_editor::Viewport& view = model.View();
    const graph_editor::Rect visible{view.ToCanvas({0.0f, 0.0f}), view.ToCanvas({canvasSize.x, canvasSize.y})};
    if (model.GraphBounds()) {
        world.min = glm::min(world.min, visible.min);
        world.max = glm::max(world.max, visible.max);
    } else {
        world = visible;
    }
    const glm::vec2 size = glm::max(world.Size(), glm::vec2(1.0f));
    MinimapMapping mapping;
    mapping.world = world;
    mapping.scale = std::min(kMinimapSize.x / size.x, kMinimapSize.y / size.y);
    const ImVec2 used(size.x * mapping.scale, size.y * mapping.scale);
    mapping.low = low + (kMinimapSize - used) * 0.5f;
    return mapping;
}

void GraphCanvas::MoveViewWithMinimap(ImVec2 point) {
    const ImVec2 high = origin + canvasSize - ImVec2(kMinimapMargin, kMinimapMargin);
    const MinimapMapping mapping = MapMinimap(*model, high - kMinimapSize, canvasSize);
    const glm::vec2 canvasPoint(mapping.world.min.x + (point.x - mapping.low.x) / mapping.scale, mapping.world.min.y + (point.y - mapping.low.y) / mapping.scale);
    model->View().CenterOn(canvasPoint, {canvasSize.x, canvasSize.y});
}

void GraphCanvas::DrawMinimap() {
    const ImVec2 high = origin + canvasSize - ImVec2(kMinimapMargin, kMinimapMargin);
    const ImVec2 low = high - kMinimapSize;
    list->AddRectFilled(low, high, IM_COL32(14, 15, 18, 215), 4.0f);
    list->AddRect(low, high, IM_COL32(90, 94, 104, 255), 4.0f);

    const MinimapMapping mapping = MapMinimap(*model, low, canvasSize);
    list->PushClipRect(low, high, true);
    for (const graph::Node& node : model->GetGraph().Nodes()) {
        const graph_editor::Rect bounds = geometry[node.id].bounds;
        const ImU32 color = ColorOfNodeKind(model->Registry().Find(node.type));
        list->AddRectFilled(mapping.ToMinimap(bounds.min), mapping.ToMinimap(bounds.max), model->IsSelected(node.id) ? kSelectionColor : color);
    }
    const graph_editor::Viewport& view = model->View();
    list->AddRect(mapping.ToMinimap(view.ToCanvas({0.0f, 0.0f})), mapping.ToMinimap(view.ToCanvas({canvasSize.x, canvasSize.y})), IM_COL32(120, 175, 255, 255));
    list->PopClipRect();
}

void GraphCanvas::DrawOverlayText() {
    char zoomText[48];
    std::snprintf(zoomText, sizeof(zoomText), "Zoom %d%%", static_cast<int>(zoom * 100.0f + 0.5f));
    const std::string& status = model->StatusMessage();
    const float lineHeight = ImGui::GetTextLineHeight();
    ImVec2 at(origin.x + 10.0f, origin.y + canvasSize.y - lineHeight - 8.0f);
    list->AddText(at, kDimTextColor, zoomText);
    if (!status.empty()) {
        list->AddText(ImVec2(at.x + 90.0f, at.y), kTextColor, status.c_str());
    }
    if (model->IsPlaying()) {
        list->AddText(ImVec2(origin.x + 12.0f, origin.y + 8.0f), IM_COL32(255, 110, 100, 255), "PLAYING");
    }
    if (model->GetGraph().Nodes().empty()) {
        const char* hint = "Right-click or double-click to add a node. Drag from a pin to make a wire.";
        const ImVec2 size = ImGui::CalcTextSize(hint);
        list->AddText(origin + (canvasSize - size) * 0.5f, kDimTextColor, hint);
    }
}

void GraphCanvas::DrawTooltips() {
    const bool pointerFree = hovered && !controlBusy && model->CurrentGesture() == graph_editor::Gesture::None && !secondaryDown &&
                             !ImGui::IsPopupOpen("canvas context") && !IsOverMinimap(ImGui::GetIO().MousePos);
    if (!pointerFree) {
        hoverKey.clear();
        return;
    }
    const glm::vec2 point = PointerCanvasPosition();
    const std::optional<graph_editor::PinHit> pin = model->PinAt(point);
    const uint32_t nodeId = model->NodeAt(point);
    const std::optional<graph::Connection> wire = (pin || nodeId != 0) ? std::nullopt : model->ConnectionAt(point);

    std::string key;
    if (pin) {
        key = "pin:" + std::to_string(pin->pin.node) + ":" + std::to_string(pin->pin.pin) + (pin->isOutput ? "o" : "i");
    } else if (nodeId != 0) {
        key = "node:" + std::to_string(nodeId);
    } else if (wire) {
        key = "wire:" + std::to_string(wire->to.node) + ":" + std::to_string(wire->to.pin);
    }
    if (key != hoverKey) {
        hoverKey = key;
        hoverSeconds = 0.0f;
    }
    hoverSeconds += ImGui::GetIO().DeltaTime;
    if (key.empty() || hoverSeconds < kTooltipDelaySeconds) {
        return;
    }

    const graph::Graph& graph = model->GetGraph();
    ImGui::BeginTooltip();
    if (pin) {
        const graph::Node* node = graph.FindNode(pin->pin.node);
        const graph::NodePins pins = graph::PinsOf(graph, model->Registry(), *node);
        const auto& descriptions = pin->isOutput ? pins.outputs : pins.inputs;
        const graph::PinDescription& description = descriptions[static_cast<size_t>(pin->pin.pin)];
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(ColorOfPinType(description.type)));
        ImGui::TextUnformatted(description.name.c_str());
        ImGui::PopStyleColor();
        ImGui::TextUnformatted(DescribeValueType(description.type).c_str());
        if (!description.tooltip.empty()) {
            ImGui::TextUnformatted(description.tooltip.c_str());
        }
        ImGui::TextDisabled("%s", pin->isOutput ? "Drag from here to a pin on the left of another node." : "Drag from here to an output, or type a value in the box.");
    } else if (nodeId != 0) {
        const graph::Node* node = graph.FindNode(nodeId);
        const graph::NodeDescription* description = model->Registry().Find(node->type);
        ImGui::TextUnformatted(graph_editor::NodeTitle(model->Registry(), *node).c_str());
        ImGui::TextDisabled("%s", NameOfNodeKind(description));
        if (description != nullptr) {
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 24.0f);
            ImGui::TextUnformatted(description->tooltip.c_str());
            ImGui::PopTextWrapPos();
        }
        for (const graph::Issue* issue : model->IssuesFor(nodeId)) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(issue->isError ? kErrorColor : kWarningColor));
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 24.0f);
            ImGui::TextUnformatted(issue->message.c_str());
            ImGui::PopTextWrapPos();
            ImGui::PopStyleColor();
        }
    } else if (wire) {
        ImGui::TextUnformatted("Wire");
        ImGui::TextDisabled("Click to select it, then press Delete. Right-click for a menu.");
    }
    ImGui::EndTooltip();
}

}  // namespace editor_ui
