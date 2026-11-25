// Tests of the graph editor's model (no window, no GPU). Like the other test programs there is no test framework: each test is a
// function that calls Check, and main reports every failure it finds.
//
// What the editor must do, and what these tests hold it to: selection that follows the mouse, nodes that move together as one
// undo step, wires that are made only between pins that fit and that say why when they are refused, a palette that finds what is
// typed and offers a dropped wire only what it can join, copies that keep their wires, undo and redo of every kind of edit with a
// limit, a run on a copy of the graph that leaves a trail to look at, and files that give back what was saved.
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "graph/GraphFile.h"
#include "graph_editor/AutoLayout.h"
#include "graph_editor/EditorModel.h"
#include "graph_editor/UndoHistory.h"
#include "graph_editor/Viewport.h"

namespace {

using namespace graph_editor;
using graph::PinRef;
using graph::PinType;

int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

bool Near(float a, float b, float tolerance = 1e-3f) {
    return std::abs(a - b) <= tolerance;
}

bool NearPoint(glm::vec2 a, glm::vec2 b, float tolerance = 1e-3f) {
    return Near(a.x, b.x, tolerance) && Near(a.y, b.y, tolerance);
}

bool Contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

graph::NodeRegistry MakeRegistry() {
    graph::NodeRegistry registry;
    graph::RegisterStandardNodes(registry);
    return registry;
}

// An editor with the standard nodes, and short ways to do with it what a person does with the mouse.
struct Editor {
    graph::NodeRegistry registry;
    EditorModel model;

    Editor() : registry(MakeRegistry()), model(registry) {}

    uint32_t Add(const std::string& type, float x, float y) {
        const uint32_t id = model.AddNode(type, {x, y});
        Check(id != 0, "adding " + type);
        return id;
    }

    glm::vec2 Output(uint32_t node, int pin) const { return model.GeometryOf(node).outputs.at(static_cast<size_t>(pin)).position; }
    glm::vec2 Input(uint32_t node, int pin) const { return model.GeometryOf(node).inputs.at(static_cast<size_t>(pin)).position; }
    // A point on the title of a node, which no pin is near.
    glm::vec2 Title(uint32_t node) const {
        const Rect header = model.GeometryOf(node).header;
        return (header.min + header.max) * 0.5f;
    }

    void Click(glm::vec2 point, bool additive = false) {
        model.PointerPressed(point, additive);
        model.PointerReleased(point);
    }

    ReleaseOutcome Drag(glm::vec2 from, glm::vec2 to, bool additive = false) {
        model.PointerPressed(from, additive);
        model.PointerMoved((from + to) * 0.5f);
        model.PointerMoved(to);
        return model.PointerReleased(to);
    }

    void Link(uint32_t from, int output, uint32_t to, int input) {
        const WireVerdict verdict = model.ConnectPins({{from, output}, true}, {{to, input}, false});
        Check(verdict.accepted, "linking " + std::to_string(from) + ":" + std::to_string(output) + " to " + std::to_string(to) + ":" +
                                    std::to_string(input) + " was refused: " + verdict.reason);
    }

    bool Wired(uint32_t from, int output, uint32_t to, int input) const {
        const graph::Connection* wire = model.GetGraph().ConnectionInto({to, input});
        return wire != nullptr && wire->from == PinRef{from, output};
    }

    size_t NodeCount() const { return model.GetGraph().Nodes().size(); }
    size_t WireCount() const { return model.GetGraph().Connections().size(); }
    glm::vec2 PositionOf(uint32_t node) const { return model.GetGraph().FindNode(node)->position; }
    std::string Text() const { return graph::SaveGraph(model.GetGraph()); }
};

std::string TemporaryPath(const std::string& name) {
    return (std::filesystem::temp_directory_path() / name).string();
}

// ---------------------------------------------------------------------------
// The view of the canvas
// ---------------------------------------------------------------------------

void ZoomingKeepsTheCanvasUnderThePointerWhereItIs() {
    Viewport viewport;
    viewport.PanByScreen({-120.0f, 45.0f});
    const glm::vec2 pointers[] = {{0.0f, 0.0f}, {300.0f, 200.0f}, {799.0f, 10.0f}};
    const float factors[] = {1.1f, 0.8f, 1.5f, 0.5f};
    for (const glm::vec2 pointer : pointers) {
        for (const float factor : factors) {
            const glm::vec2 canvasBefore = viewport.ToCanvas(pointer);
            viewport.ZoomAt(pointer, factor);
            Check(NearPoint(viewport.ToScreen(canvasBefore), pointer), "the point under the pointer moved when zooming by " + std::to_string(factor));
            Check(NearPoint(viewport.ToCanvas(viewport.ToScreen({12.0f, 34.0f})), {12.0f, 34.0f}), "ToCanvas and ToScreen are not inverses");
        }
    }
}

void ZoomStaysWithinItsLimitsAndStillAnchorsThere() {
    Viewport viewport;
    const glm::vec2 pointer(400.0f, 300.0f);
    for (int i = 0; i < 60; ++i) {
        viewport.ZoomAt(pointer, 1.3f);
    }
    Check(Near(viewport.Zoom(), kMaxZoom), "zoom went past its upper limit: " + std::to_string(viewport.Zoom()));
    const glm::vec2 canvasUnderPointer = viewport.ToCanvas(pointer);
    viewport.ZoomAt(pointer, 1.3f);  // already at the limit
    Check(NearPoint(viewport.ToCanvas(pointer), canvasUnderPointer), "zooming at the limit moved the canvas");
    for (int i = 0; i < 120; ++i) {
        viewport.ZoomAt(pointer, 0.7f);
    }
    Check(Near(viewport.Zoom(), kMinZoom), "zoom went below its lower limit: " + std::to_string(viewport.Zoom()));
}

void PanningMovesTheCanvasWithThePointer() {
    Viewport viewport;
    viewport.ZoomAt({0.0f, 0.0f}, 2.0f);
    const glm::vec2 canvasPoint(30.0f, 40.0f);
    const glm::vec2 before = viewport.ToScreen(canvasPoint);
    viewport.PanByScreen({10.0f, -6.0f});
    Check(NearPoint(viewport.ToScreen(canvasPoint), before + glm::vec2(10.0f, -6.0f)), "the canvas did not follow the pointer when panning");
}

void FramingShowsAnAreaCentredAndInsideTheWindow() {
    Viewport viewport;
    const Rect area{{100.0f, 100.0f}, {900.0f, 500.0f}};
    const glm::vec2 window(1000.0f, 600.0f);
    viewport.Frame(area, window);
    const glm::vec2 low = viewport.ToScreen(area.min);
    const glm::vec2 high = viewport.ToScreen(area.max);
    Check(low.x >= 0.0f && low.y >= 0.0f && high.x <= window.x && high.y <= window.y, "the framed area does not fit the window");
    Check(NearPoint((low + high) * 0.5f, window * 0.5f), "the framed area is not in the middle of the window");
}

// ---------------------------------------------------------------------------
// Where things are
// ---------------------------------------------------------------------------

void PinsSitOnTheEdgesOfTheirNodeInTheOrderOfTheirRows() {
    Editor editor;
    const uint32_t add = editor.Add("math.add", 100.0f, 50.0f);
    const NodeGeometry geometry = editor.model.GeometryOf(add);
    Check(geometry.inputs.size() == 2 && geometry.outputs.size() == 1, "math.add should have two inputs and one output");
    for (const PinSlot& pin : geometry.inputs) {
        Check(Near(pin.position.x, geometry.bounds.min.x), "an input is not on the left edge");
    }
    Check(Near(geometry.outputs[0].position.x, geometry.bounds.max.x), "an output is not on the right edge");
    Check(geometry.inputs[1].position.y > geometry.inputs[0].position.y, "the second input is not below the first");
    Check(geometry.inputs[0].position.y > geometry.header.max.y, "a pin is inside the header");
    Check(geometry.bounds.Contains(geometry.inputs[1].position) && geometry.bounds.Contains(geometry.outputs[0].position), "a pin is outside its node");
    Check(editor.model.GeometryOf(9999).bounds.Size() == glm::vec2(0.0f), "a node that is not there has a size");
}

void ANodeIsAsWideAsItsNamesNeed() {
    Editor editor;
    const uint32_t print = editor.Add("debug.print", 0.0f, 0.0f);
    const NodeGeometry geometry = editor.model.GeometryOf(print);
    // Row 1 has the input Text and its box for a typed value; both have to fit between the edges.
    Check(geometry.bounds.Size().x >= 2.0f * kPinRadius + 4.0f * kCharacterWidth + kInlineValueWidth, "the node is narrower than its row");
}

// ---------------------------------------------------------------------------
// Selection
// ---------------------------------------------------------------------------

void AClickSelectsTheTopmostNodeAndEmptyCanvasClearsTheSelection() {
    Editor editor;
    const uint32_t a = editor.Add("math.add", 0.0f, 0.0f);
    const uint32_t b = editor.Add("math.add", 400.0f, 0.0f);
    editor.Click(editor.Title(a));
    Check(editor.model.Selection() == std::set<uint32_t>{a}, "clicking a node did not select it alone");
    editor.Click(editor.Title(b));
    Check(editor.model.Selection() == std::set<uint32_t>{b}, "clicking another node did not move the selection");
    editor.Click({200.0f, 600.0f});
    Check(editor.model.Selection().empty(), "clicking empty canvas did not clear the selection");

    const uint32_t over = editor.Add("math.add", 0.0f, 0.0f);  // exactly over a
    editor.Click({10.0f, 10.0f});
    Check(editor.model.Selection() == std::set<uint32_t>{over}, "the click did not find the node on top");
}

void AdditiveClicksAddAndToggle() {
    Editor editor;
    const uint32_t a = editor.Add("math.add", 0.0f, 0.0f);
    const uint32_t b = editor.Add("math.add", 400.0f, 0.0f);
    editor.Click(editor.Title(a));
    editor.Click(editor.Title(b), true);
    Check(editor.model.Selection() == (std::set<uint32_t>{a, b}), "an additive click did not add");
    editor.Click(editor.Title(a), true);
    Check(editor.model.Selection() == std::set<uint32_t>{b}, "an additive click on a selected node did not deselect it");
    editor.model.SelectAll();
    Check(editor.model.Selection().size() == 2, "SelectAll did not select every node");
    editor.model.ClearSelection();
    Check(editor.model.Selection().empty(), "ClearSelection did not clear");
}

void ABoxSelectsWhatItTouchesAndAddsWhenAdditive() {
    Editor editor;
    const uint32_t a = editor.Add("math.add", 0.0f, 0.0f);
    const uint32_t b = editor.Add("math.add", 400.0f, 0.0f);
    const uint32_t c = editor.Add("math.add", 0.0f, 300.0f);

    editor.model.PointerPressed({-50.0f, -50.0f}, false);
    editor.model.PointerMoved({60.0f, 40.0f});
    Check(editor.model.CurrentGesture() == Gesture::BoxSelecting, "pressing empty canvas did not start a box");
    const std::optional<Rect> box = editor.model.ActiveSelectionBox();
    Check(box && NearPoint(box->min, {-50.0f, -50.0f}) && NearPoint(box->max, {60.0f, 40.0f}), "the box is not between the press and the pointer");
    const ReleaseOutcome outcome = editor.model.PointerReleased({60.0f, 40.0f});
    Check(outcome == ReleaseOutcome::BoxSelected, "letting go of a box did not say it selected");
    Check(editor.model.Selection() == std::set<uint32_t>{a}, "the box did not select the node it touched, and only it");
    Check(!editor.model.ActiveSelectionBox(), "the box stayed after it was let go");

    editor.Drag({380.0f, -50.0f}, {450.0f, 30.0f}, true);
    Check(editor.model.Selection() == (std::set<uint32_t>{a, b}), "an additive box did not add to the selection");

    editor.Drag({-80.0f, -80.0f}, {900.0f, 500.0f});
    Check(editor.model.Selection() == (std::set<uint32_t>{a, b, c}), "a box around everything did not select everything");

    // A box drawn from the lower right corner works the same as from the upper left.
    editor.Drag({200.0f, 100.0f}, {-50.0f, -50.0f});
    Check(editor.model.Selection() == std::set<uint32_t>{a}, "a box dragged up and to the left selected the wrong nodes");
}

void ANodeSelectedBySelectInRectOnlyIfItTouchesIt() {
    Editor editor;
    const uint32_t a = editor.Add("math.add", 0.0f, 0.0f);
    const uint32_t b = editor.Add("math.add", 1000.0f, 0.0f);
    editor.model.SelectInRect({{-10.0f, -10.0f}, {10.0f, 10.0f}});
    Check(editor.model.IsSelected(a) && !editor.model.IsSelected(b), "SelectInRect selected the wrong nodes");
    editor.model.SelectInRect({{990.0f, 0.0f}, {1010.0f, 10.0f}}, SelectMode::Toggle);
    Check(editor.model.IsSelected(a) && editor.model.IsSelected(b), "toggling a rectangle did not add the node inside it");
    editor.model.SelectInRect({{-10.0f, -10.0f}, {10.0f, 10.0f}}, SelectMode::Toggle);
    Check(!editor.model.IsSelected(a) && editor.model.IsSelected(b), "toggling a rectangle did not remove the selected node inside it");
}

// ---------------------------------------------------------------------------
// Moving
// ---------------------------------------------------------------------------

void DraggingANodeMovesTheWholeSelectionAsOneUndoStep() {
    Editor editor;
    const uint32_t a = editor.Add("math.add", 0.0f, 0.0f);
    const uint32_t b = editor.Add("math.add", 400.0f, 0.0f);
    const uint32_t c = editor.Add("math.add", 0.0f, 300.0f);
    editor.model.SelectNode(a);
    editor.model.SelectNode(b, SelectMode::Add);
    const size_t stepsBefore = editor.model.UndoCount();

    const glm::vec2 grab = editor.Title(a);
    editor.model.PointerPressed(grab, false);
    for (int i = 1; i <= 20; ++i) {
        editor.model.PointerMoved(grab + glm::vec2(5.0f * static_cast<float>(i), 2.0f * static_cast<float>(i)));
    }
    const ReleaseOutcome outcome = editor.model.PointerReleased(grab + glm::vec2(100.0f, 40.0f));
    Check(outcome == ReleaseOutcome::MovedNodes, "a drag of a node did not say the nodes moved");
    Check(NearPoint(editor.PositionOf(a), {100.0f, 40.0f}), "the pressed node did not follow the pointer");
    Check(NearPoint(editor.PositionOf(b), {500.0f, 40.0f}), "the other selected node did not move with it");
    Check(NearPoint(editor.PositionOf(c), {0.0f, 300.0f}), "a node that was not selected moved");
    Check(editor.model.UndoCount() == stepsBefore + 1, "a drag was not exactly one undo step but " + std::to_string(editor.model.UndoCount() - stepsBefore));

    editor.model.Undo();
    Check(NearPoint(editor.PositionOf(a), {0.0f, 0.0f}) && NearPoint(editor.PositionOf(b), {400.0f, 0.0f}), "undo did not put the moved nodes back");
    editor.model.Redo();
    Check(NearPoint(editor.PositionOf(a), {100.0f, 40.0f}), "redo did not move them again");
}

void PressingAnUnselectedNodeSelectsItAndPressingOneOfSeveralKeepsThemUntilAClick() {
    Editor editor;
    const uint32_t a = editor.Add("math.add", 0.0f, 0.0f);
    const uint32_t b = editor.Add("math.add", 400.0f, 0.0f);
    editor.model.SelectAll();
    editor.model.PointerPressed(editor.Title(a), false);
    Check(editor.model.Selection().size() == 2, "pressing a selected node dropped the rest of the selection, so it could not be dragged along");
    editor.model.PointerReleased(editor.Title(a));
    Check(editor.model.Selection() == std::set<uint32_t>{a}, "a click on one of several selected nodes did not select only it");
    (void)b;
}

void AClickThatWobblesDoesNotMoveTheNode() {
    Editor editor;
    const uint32_t a = editor.Add("math.add", 0.0f, 0.0f);
    const size_t stepsBefore = editor.model.UndoCount();
    const glm::vec2 grab = editor.Title(a);
    editor.model.PointerPressed(grab, false);
    editor.model.PointerMoved(grab + glm::vec2(1.0f, 1.0f));
    const ReleaseOutcome outcome = editor.model.PointerReleased(grab + glm::vec2(1.0f, 0.0f));
    Check(outcome == ReleaseOutcome::Clicked, "a click that moved one pixel was not a click");
    Check(NearPoint(editor.PositionOf(a), {0.0f, 0.0f}), "a click moved the node");
    Check(editor.model.UndoCount() == stepsBefore, "a click made an undo step");
}

void TwoSeparateDragsAreTwoUndoSteps() {
    Editor editor;
    const uint32_t a = editor.Add("math.add", 0.0f, 0.0f);
    editor.Drag(editor.Title(a), editor.Title(a) + glm::vec2(50.0f, 0.0f));
    editor.Drag(editor.Title(a), editor.Title(a) + glm::vec2(0.0f, 70.0f));
    const size_t stepsAfterTwoDrags = editor.model.UndoCount();
    editor.model.Undo();
    Check(NearPoint(editor.PositionOf(a), {50.0f, 0.0f}), "undoing the second drag did not leave the first in place");
    editor.model.Undo();
    Check(NearPoint(editor.PositionOf(a), {0.0f, 0.0f}), "undoing the first drag did not put the node back");
    Check(stepsAfterTwoDrags == 3, "expected the add and two drags to be three steps, got " + std::to_string(stepsAfterTwoDrags));
}

// ---------------------------------------------------------------------------
// Wires
// ---------------------------------------------------------------------------

void DraggingFromAnOutputPinToAnInputPinMakesTheWire() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    const uint32_t print = editor.Add("debug.print", 300.0f, 0.0f);

    editor.model.PointerPressed(editor.Output(start, 0), false);
    Check(editor.model.CurrentGesture() == Gesture::DraggingWire, "pressing on a pin did not start a wire");
    editor.model.PointerMoved(editor.Input(print, 0));
    const std::optional<WireDrag>& drag = editor.model.ActiveWireDrag();
    Check(drag && drag->hover && drag->hover->pin == PinRef{print, 0} && !drag->hover->isOutput, "the pin under the wire was not found");
    Check(drag && drag->verdict.accepted && drag->verdict.reason.empty(), "a wire that fits was not accepted while dragging");
    const ReleaseOutcome outcome = editor.model.PointerReleased(editor.Input(print, 0));
    Check(outcome == ReleaseOutcome::WireConnected, "letting go on a pin that fits did not connect");
    Check(editor.Wired(start, 0, print, 0), "the wire is not in the graph");
    Check(!editor.model.ActiveWireDrag(), "the wire is still being dragged");
}

void DraggingFromAnInputToAnOutputMakesTheSameWire() {
    Editor editor;
    const uint32_t add = editor.Add("math.add", 0.0f, 0.0f);
    const uint32_t print = editor.Add("debug.print", 300.0f, 0.0f);
    const ReleaseOutcome outcome = editor.Drag(editor.Input(print, 1), editor.Output(add, 0));
    Check(outcome == ReleaseOutcome::WireConnected, "a wire from an input to an output was not connected");
    Check(editor.Wired(add, 0, print, 1), "the wire does not run from the output to the input");
}

void ARefusedWireSaysWhyInTheWordsOfTheGraph() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    const uint32_t add = editor.Add("math.add", 0.0f, 200.0f);
    const uint32_t print = editor.Add("debug.print", 400.0f, 0.0f);
    const uint32_t other = editor.Add("debug.print", 400.0f, 300.0f);

    // A number into an execution pin.
    const WireDrag* seen = nullptr;
    editor.model.PointerPressed(editor.Output(add, 0), false);
    editor.model.PointerMoved(editor.Input(print, 0));
    seen = editor.model.ActiveWireDrag() ? &*editor.model.ActiveWireDrag() : nullptr;
    const std::string expected = editor.model.GetGraph().WhyNotConnect(editor.registry, {add, 0}, {print, 0});
    Check(!expected.empty(), "the graph should refuse a number into an execution pin");
    Check(seen != nullptr && seen->hover && !seen->verdict.accepted && seen->verdict.reason == expected, "the verdict while dragging is not what WhyNotConnect says");
    const ReleaseOutcome outcome = editor.model.PointerReleased(editor.Input(print, 0));
    Check(outcome == ReleaseOutcome::WireRefused, "a refused wire was not reported as refused");
    Check(editor.model.StatusMessage() == expected, "the status line does not give the reason");
    Check(editor.WireCount() == 0, "a refused wire was made anyway");

    // A wire to its own node.
    const WireVerdict self = editor.model.CheckWire({{print, 1}, false}, {{print, 0}, true});
    Check(!self.accepted && Contains(self.reason, "itself"), "a wire from a node to itself was not refused for that reason: " + self.reason);

    // An output to an output, which has to be refused here because the graph reads pin numbers as one side only.
    const WireVerdict outputs = editor.model.CheckWire({{start, 0}, true}, {{add, 0}, true});
    Check(!outputs.accepted && Contains(outputs.reason, "output"), "an output wired to an output was not refused: " + outputs.reason);
    const WireVerdict inputs = editor.model.CheckWire({{print, 1}, false}, {{other, 1}, false});
    Check(!inputs.accepted && Contains(inputs.reason, "input"), "an input wired to an input was not refused: " + inputs.reason);

    // A loop.
    editor.Link(print, 0, other, 0);
    const WireVerdict loop = editor.model.CheckWire({{other, 0}, true}, {{print, 0}, false});
    Check(!loop.accepted && Contains(loop.reason, "loop"), "a wire that closes a loop was not refused for that: " + loop.reason);
}

void ADroppedWireOnItsOwnPinIsAClickAndMakesNothing() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    const size_t stepsBefore = editor.model.UndoCount();
    editor.model.PointerPressed(editor.Output(start, 0), false);
    const ReleaseOutcome outcome = editor.model.PointerReleased(editor.Output(start, 0));
    Check(outcome == ReleaseOutcome::Nothing, "a press and release on a pin did not come to nothing");
    Check(!editor.model.GetPendingWire(), "a click on a pin opened the palette");
    Check(editor.model.UndoCount() == stepsBefore, "a click on a pin made an undo step");
}

void AWiredInputTakesItsWireUpToMoveItAsOneUndoStep() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    const uint32_t first = editor.Add("debug.print", 300.0f, 0.0f);
    const uint32_t second = editor.Add("debug.print", 300.0f, 250.0f);
    editor.Link(start, 0, first, 0);
    const size_t stepsBefore = editor.model.UndoCount();

    const ReleaseOutcome outcome = editor.Drag(editor.Input(first, 0), editor.Input(second, 0));
    Check(outcome == ReleaseOutcome::WireConnected, "moving a wire to another input did not connect it");
    Check(editor.Wired(start, 0, second, 0) && !editor.Wired(start, 0, first, 0), "the wire did not move from the first input to the second");
    Check(editor.model.UndoCount() == stepsBefore + 1, "moving a wire was not one undo step");
    editor.model.Undo();
    Check(editor.Wired(start, 0, first, 0) && !editor.Wired(start, 0, second, 0), "undo did not put the wire back where it was");
}

void ATakenUpWireLetGoOnEmptyCanvasOpensThePaletteForItsOutput() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    const uint32_t print = editor.Add("debug.print", 300.0f, 0.0f);
    editor.Link(start, 0, print, 0);
    const ReleaseOutcome outcome = editor.Drag(editor.Input(print, 0), {600.0f, 500.0f});
    Check(outcome == ReleaseOutcome::WireDroppedOnEmpty, "a wire let go on empty canvas did not ask for a palette");
    Check(editor.WireCount() == 0, "the wire that was taken up is still in the graph");
    const std::optional<PendingWire>& pending = editor.model.GetPendingWire();
    Check(pending && pending->start.isOutput && pending->start.pin == PinRef{start, 0}, "the palette is not for the output the wire came from");
    Check(pending && NearPoint(pending->dropPosition, {600.0f, 500.0f}), "the palette does not know where the wire was dropped");
}

void EscapeGivesUpADraggedWire() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    editor.model.PointerPressed(editor.Output(start, 0), false);
    editor.model.PointerMoved({500.0f, 500.0f});
    editor.model.CancelGesture();
    Check(editor.model.CurrentGesture() == Gesture::None && !editor.model.ActiveWireDrag(), "cancelling did not end the wire");
    Check(!editor.model.GetPendingWire(), "cancelling opened a palette");
    Check(editor.model.PointerReleased({500.0f, 500.0f}) == ReleaseOutcome::Nothing, "letting go after cancelling did something");
}

void ClickingAWireSelectsItAndDeleteTakesItAway() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    const uint32_t print = editor.Add("debug.print", 400.0f, 100.0f);
    editor.Link(start, 0, print, 0);
    const WireCurve curve = MakeWireCurve(editor.Output(start, 0), editor.Input(print, 0));
    const glm::vec2 onTheWire = curve.PointAt(0.5f);
    Check(editor.model.ConnectionAt(onTheWire).has_value(), "the middle of a wire is not found");
    Check(!editor.model.ConnectionAt(onTheWire + glm::vec2(0.0f, 60.0f)).has_value(), "a point far from the wire is found on it");

    editor.Click(onTheWire);
    Check(editor.model.SelectedConnection().has_value() && editor.model.Selection().empty(), "clicking a wire did not select it");
    editor.model.DeleteSelection();
    Check(editor.WireCount() == 0 && editor.NodeCount() == 2, "deleting the selected wire did not remove just the wire");
    Check(!editor.model.SelectedConnection(), "the deleted wire is still selected");
    editor.model.Undo();
    Check(editor.Wired(start, 0, print, 0), "undo did not bring the wire back");
}

void BreakingLinksOfAPinTakesAwayAllWiresOnIt() {
    Editor editor;
    const uint32_t add = editor.Add("math.add", 0.0f, 0.0f);
    const uint32_t second = editor.Add("math.add", 300.0f, 0.0f);
    const uint32_t third = editor.Add("math.add", 300.0f, 200.0f);
    editor.Link(add, 0, second, 0);
    editor.Link(add, 0, third, 1);
    editor.model.BreakLinks({{add, 0}, true});
    Check(editor.WireCount() == 0, "breaking the links of an output left wires");
    editor.model.Undo();
    editor.model.BreakLinks({{second, 0}, false});
    Check(editor.WireCount() == 1 && editor.Wired(add, 0, third, 1), "breaking the link of one input took away the wrong wires");
}

// ---------------------------------------------------------------------------
// The palette
// ---------------------------------------------------------------------------

bool HasEntry(const std::vector<PaletteEntry>& entries, const std::string& typeId) {
    return std::any_of(entries.begin(), entries.end(), [&](const PaletteEntry& entry) { return entry.typeId == typeId; });
}

const PaletteEntry* FindEntry(const std::vector<PaletteEntry>& entries, const std::string& typeId) {
    for (const PaletteEntry& entry : entries) {
        if (entry.typeId == typeId) {
            return &entry;
        }
    }
    return nullptr;
}

bool HasPinOfType(const graph::NodeRegistry& registry, const std::string& typeId, PinType type) {
    const graph::NodeDescription* description = registry.Find(typeId);
    if (description == nullptr) {
        return false;
    }
    for (const auto* pins : {&description->inputs, &description->outputs}) {
        for (const graph::PinDescription& pin : *pins) {
            if (pin.type == type) {
                return true;
            }
        }
    }
    return false;
}

void TheEmptyQueryListsEveryKindByCategoryAndTitle() {
    Editor editor;
    const std::vector<PaletteEntry> entries = editor.model.SearchPalette("");
    // The nodes of a variable are offered once for each variable, and this graph has none.
    const std::vector<const graph::NodeDescription*> kinds = editor.registry.All();
    const size_t kindsWithoutVariable =
        static_cast<size_t>(std::count_if(kinds.begin(), kinds.end(), [](const graph::NodeDescription* kind) { return !kind->usesVariable; }));
    Check(entries.size() == kindsWithoutVariable, "the empty query did not list every kind of node");
    for (size_t i = 1; i < entries.size(); ++i) {
        const bool inOrder = entries[i - 1].category < entries[i].category || (entries[i - 1].category == entries[i].category && entries[i - 1].title <= entries[i].title);
        Check(inOrder, "the palette is not ordered by category and title at " + entries[i].title);
    }
}

void TypingFiltersByNameCategoryAndPinTypes() {
    Editor editor;

    const std::vector<PaletteEntry> add = editor.model.SearchPalette("add");
    Check(!add.empty() && add.front().typeId == "math.add", "\"add\" does not find Add first");

    Check(editor.model.SearchPalette("ADD").size() == add.size(), "the search depends on upper and lower case");

    const std::vector<PaletteEntry> flow = editor.model.SearchPalette("flow");
    Check(!flow.empty(), "\"flow\" finds nothing");
    for (const PaletteEntry& entry : flow) {
        Check(entry.category == "Flow" || Contains(entry.typeId, "flow"), "\"flow\" found " + entry.title + " in " + entry.category);
    }

    const std::vector<PaletteEntry> vectors = editor.model.SearchPalette("vec3");
    Check(!vectors.empty(), "\"vec3\" finds nothing, so pin types are not searched");
    for (const PaletteEntry& entry : vectors) {
        Check(HasPinOfType(editor.registry, entry.typeId, PinType::Vec3), "\"vec3\" found " + entry.title + ", which has no vec3 pin");
    }

    const std::vector<PaletteEntry> exec = editor.model.SearchPalette("exec");
    Check(HasEntry(exec, "event.start") && HasEntry(exec, "debug.print") && !HasEntry(exec, "math.add"), "\"exec\" does not find exactly the nodes with execution pins");

    const std::vector<PaletteEntry> twoWords = editor.model.SearchPalette("math float");
    Check(HasEntry(twoWords, "math.add") && !HasEntry(twoWords, "text.join"), "two words did not both have to match");

    Check(editor.model.SearchPalette("qqqqzzz").empty(), "a word that fits nothing found something");
}

void ATitleMatchRanksAboveAPinMatch() {
    Editor editor;
    const std::vector<PaletteEntry> found = editor.model.SearchPalette("random");
    Check(!found.empty() && found.front().typeId == "math.random", "\"random\" does not put Random number first");
}

void VariablesGetAndSetNodesAppearOncePerVariable() {
    Editor editor;
    Check(!HasEntry(editor.model.SearchPalette(""), "variable.get"), "a variable node is offered while there are no variables");
    Check(editor.model.AddVariable("Health", PinType::Float).empty(), "adding a variable failed");
    const std::vector<PaletteEntry> found = editor.model.SearchPalette("health");
    Check(found.size() == 2 && found[0].variable == "Health" && found[1].variable == "Health", "\"health\" does not give a get and a set node");

    const PaletteEntry* get = FindEntry(found, "variable.get");
    Check(get != nullptr, "no get node for the variable");
    const uint32_t node = editor.model.AddNodeFromPalette(*get, {10.0f, 10.0f});
    const graph::Node* added = editor.model.GetGraph().FindNode(node);
    Check(added != nullptr && std::get<std::string>(added->properties.at("variable")) == "Health", "the node does not name its variable");
    Check(NodeTitle(editor.registry, *added) == "Get variable Health", "the node title does not name its variable: " + NodeTitle(editor.registry, *added));
}

void ADroppedExecutionWireOffersOnlyNodesWithAnExecutionInput() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    editor.Drag(editor.Output(start, 0), {500.0f, 400.0f});
    Check(editor.model.GetPendingWire().has_value(), "the drop did not leave a pending wire");

    const std::vector<PaletteEntry> entries = editor.model.SearchPalette("");
    Check(!entries.empty(), "nothing is offered to an execution wire");
    Check(HasEntry(entries, "debug.print") && HasEntry(entries, "flow.branch") && HasEntry(entries, "flow.sequence"), "an execution wire is not offered the actions");
    Check(!HasEntry(entries, "math.add") && !HasEntry(entries, "event.start") && !HasEntry(entries, "event.tick"), "an execution wire is offered a node it cannot join");
    for (const PaletteEntry& entry : entries) {
        const graph::NodeDescription* description = editor.registry.Find(entry.typeId);
        const bool pinIsExecutionInput = entry.connectPin >= 0 && entry.connectPin < static_cast<int>(description->inputs.size()) &&
                                         description->inputs[static_cast<size_t>(entry.connectPin)].type == PinType::Exec;
        Check(pinIsExecutionInput, entry.title + " does not point to an execution input");
    }

    // The query still narrows the offer.
    const std::vector<PaletteEntry> printing = editor.model.SearchPalette("print");
    Check(printing.size() == 1 && printing[0].typeId == "debug.print", "a query did not narrow the offer for a wire");

    const uint32_t print = editor.model.AddNodeFromPalette(printing[0], {500.0f, 400.0f});
    Check(editor.Wired(start, 0, print, 0), "the new node was not wired to the pin the wire came from");
    Check(!editor.model.GetPendingWire(), "the wire is still pending after a node was chosen");
    Check(editor.model.Selection() == std::set<uint32_t>{print}, "the new node is not selected");
}

void ADroppedDataWireOffersNodesThatCanTakeItAndWiresTheFirstPinThatFits() {
    Editor editor;
    const uint32_t add = editor.Add("math.add", 0.0f, 0.0f);
    editor.Drag(editor.Output(add, 0), {500.0f, 400.0f});
    const std::vector<PaletteEntry> entries = editor.model.SearchPalette("");

    Check(HasEntry(entries, "math.add") && HasEntry(entries, "compare.less") && HasEntry(entries, "debug.print"), "a number wire is not offered the nodes that take a number or text");
    Check(!HasEntry(entries, "event.start") && !HasEntry(entries, "event.tick"), "a number wire is offered an event, which has no input");
    const PaletteEntry* print = FindEntry(entries, "debug.print");
    Check(print != nullptr && print->connectPin == 1, "the number wire is not pointed at the text input of Print, after its execution input");
    const PaletteEntry* sum = FindEntry(entries, "math.add");
    Check(sum != nullptr && sum->connectPin == 0, "the number wire is not pointed at the first input of Add");

    // Every offer has a pin that fits, and no pin before it does.
    for (const PaletteEntry& entry : entries) {
        const graph::NodeDescription* description = editor.registry.Find(entry.typeId);
        bool earlierPinFits = false;
        for (int pin = 0; pin < static_cast<int>(description->inputs.size()); ++pin) {
            const PinType type = description->inputs[static_cast<size_t>(pin)].type;
            const bool fits = type != PinType::Exec && graph::CanConvert(PinType::Float, type);
            if (pin == entry.connectPin) {
                Check(fits, entry.title + " points to a pin the wire cannot join");
                break;
            }
            earlierPinFits = earlierPinFits || fits;
        }
        Check(!earlierPinFits, entry.title + " does not point to the first pin that fits");
    }

    const uint32_t print2 = editor.model.AddNodeFromPalette(*print, {500.0f, 400.0f});
    Check(editor.Wired(add, 0, print2, 1), "the wire did not end at the text input");
}

void ADroppedWireFromAnInputOffersNodesWithAnOutputThatFitsIt() {
    Editor editor;
    const uint32_t print = editor.Add("debug.print", 400.0f, 0.0f);
    editor.Drag(editor.Input(print, 1), {100.0f, 400.0f});
    const std::vector<PaletteEntry> entries = editor.model.SearchPalette("");
    Check(HasEntry(entries, "math.add") && HasEntry(entries, "event.tick") && HasEntry(entries, "text.join"), "a text input is not offered nodes that give a value");
    Check(!HasEntry(entries, "debug.print") && !HasEntry(entries, "event.start"), "a text input is offered a node with no value to give");
    const PaletteEntry* tick = FindEntry(entries, "event.tick");
    Check(tick != nullptr && tick->connectPin == 1, "the input is not pointed at the number of the every-frame event, after its execution output");

    const uint32_t made = editor.model.AddNodeFromPalette(*FindEntry(entries, "math.add"), {100.0f, 400.0f});
    Check(editor.Wired(made, 0, print, 1), "the new node's output was not wired to the input the wire came from");
}

void ADroppedExecutionInputIsOfferedEventsAndActions() {
    Editor editor;
    const uint32_t print = editor.Add("debug.print", 400.0f, 0.0f);
    editor.Drag(editor.Input(print, 0), {100.0f, 400.0f});
    const std::vector<PaletteEntry> entries = editor.model.SearchPalette("");
    Check(HasEntry(entries, "event.start") && HasEntry(entries, "flow.branch"), "an execution input is not offered what can lead to it");
    Check(!HasEntry(entries, "math.add"), "an execution input is offered a pure node");
    const uint32_t start = editor.model.AddNodeFromPalette(*FindEntry(entries, "event.start"), {100.0f, 400.0f});
    Check(editor.Wired(start, 0, print, 0), "the event was not wired to the execution input");
}

void APressClearsThePendingWireSoThePaletteIsNotNarrowedByAnOldOne() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    editor.Drag(editor.Output(start, 0), {500.0f, 400.0f});
    Check(editor.model.GetPendingWire().has_value(), "no pending wire to start with");
    editor.Click({900.0f, 900.0f});
    Check(!editor.model.GetPendingWire(), "a click elsewhere left the wire pending");
    Check(HasEntry(editor.model.SearchPalette(""), "math.add"), "the palette is still narrowed after the wire was given up");
}

void ANodeFromThePaletteWithoutAWireIsJustAdded() {
    Editor editor;
    const std::vector<PaletteEntry> entries = editor.model.SearchPalette("branch");
    Check(!entries.empty(), "no Branch in the palette");
    const uint32_t branch = editor.model.AddNodeFromPalette(entries[0], {80.0f, 90.0f});
    Check(NearPoint(editor.PositionOf(branch), {80.0f, 90.0f}), "the node is not where it was dropped");
    Check(editor.WireCount() == 0, "a wire appeared from nowhere");
    editor.model.Undo();
    Check(editor.NodeCount() == 0, "undo did not take the new node away");
}

void AddingAKindThatDoesNotExistIsRefused() {
    Editor editor;
    Check(editor.model.AddNode("nothing.here", {0.0f, 0.0f}) == 0 && editor.NodeCount() == 0, "a node of an unknown kind was added");
    Check(!editor.model.StatusMessage().empty(), "the refusal did not say anything");
}

// ---------------------------------------------------------------------------
// Deleting, copying and pasting
// ---------------------------------------------------------------------------

void DeletingNodesRemovesTheirWiresAndUndoBringsBackTheSameNumbers() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    const uint32_t print = editor.Add("debug.print", 300.0f, 0.0f);
    const uint32_t print2 = editor.Add("debug.print", 600.0f, 0.0f);
    editor.Link(start, 0, print, 0);
    editor.Link(print, 0, print2, 0);

    editor.model.SelectNode(print);
    editor.model.DeleteSelection();
    Check(editor.NodeCount() == 2 && editor.WireCount() == 0, "deleting a node did not remove it and its wires");
    Check(editor.model.Selection().empty(), "a deleted node is still selected");

    editor.model.Undo();
    Check(editor.NodeCount() == 3 && editor.WireCount() == 2, "undo did not restore the node and its wires");
    Check(editor.model.GetGraph().FindNode(print) != nullptr && editor.Wired(start, 0, print, 0) && editor.Wired(print, 0, print2, 0), "the restored node or its wires have other numbers");
    const uint32_t fresh = editor.Add("math.add", 0.0f, 300.0f);
    Check(fresh != start && fresh != print && fresh != print2, "a new node was given a number that is already used");
}

void DuplicatingKeepsTheWiresBetweenCopiedNodesAndLeavesOutTheOthers() {
    Editor editor;
    const uint32_t outside = editor.Add("math.add", 0.0f, 400.0f);
    const uint32_t a = editor.Add("math.add", 0.0f, 0.0f);
    const uint32_t b = editor.Add("math.add", 300.0f, 0.0f);
    editor.Link(a, 0, b, 0);
    editor.Link(outside, 0, a, 1);
    editor.model.SetInputValue(b, 1, 4.5f);
    editor.model.SelectNode(a);
    editor.model.SelectNode(b, SelectMode::Add);
    const size_t wiresBefore = editor.WireCount();

    editor.model.Duplicate();
    Check(editor.NodeCount() == 5, "duplicating two nodes did not add two");
    Check(editor.model.Selection().size() == 2 && !editor.model.IsSelected(a) && !editor.model.IsSelected(b), "the copies are not the selection");
    Check(editor.WireCount() == wiresBefore + 1, "the copies should have one wire between them and none to outside nodes, but the graph went from " +
                                                     std::to_string(wiresBefore) + " wires to " + std::to_string(editor.WireCount()));

    uint32_t copyOfA = 0;
    uint32_t copyOfB = 0;
    for (uint32_t id : editor.model.Selection()) {
        const graph::Node* node = editor.model.GetGraph().FindNode(id);
        if (NearPoint(node->position, editor.PositionOf(a) + glm::vec2(30.0f, 30.0f))) {
            copyOfA = id;
        } else if (NearPoint(node->position, editor.PositionOf(b) + glm::vec2(30.0f, 30.0f))) {
            copyOfB = id;
        }
    }
    Check(copyOfA != 0 && copyOfB != 0, "the copies are not a little to the side of the originals, keeping their distance");
    Check(editor.Wired(copyOfA, 0, copyOfB, 0), "the wire between the copies is missing");
    Check(editor.model.GetGraph().ConnectionInto({copyOfA, 1}) == nullptr, "a wire from a node that was not copied reached the copy");
    const graph::Node* copiedB = editor.model.GetGraph().FindNode(copyOfB);
    Check(copiedB != nullptr && copiedB->inputValues.count(1) == 1 && std::get<float>(copiedB->inputValues.at(1)) == 4.5f, "the typed value was not copied");
    Check(!editor.model.CanPaste(), "duplicating used the clipboard");

    editor.model.Undo();
    Check(editor.NodeCount() == 3, "a duplicate was not one undo step");
}

void CopyAndPasteTwiceGivesUniqueNumbersAtTheAnchor() {
    Editor editor;
    const uint32_t a = editor.Add("math.add", 100.0f, 100.0f);
    const uint32_t b = editor.Add("math.add", 400.0f, 130.0f);
    editor.Link(a, 0, b, 0);
    editor.model.SelectAll();
    editor.model.Copy();
    Check(editor.model.CanPaste(), "nothing to paste after a copy");

    editor.model.Paste({1000.0f, 1000.0f});
    Check(editor.model.Selection().size() == 2, "the pasted nodes are not selected");
    glm::vec2 topLeft(1e9f);
    for (uint32_t id : editor.model.Selection()) {
        topLeft = glm::min(topLeft, editor.PositionOf(id));
    }
    Check(NearPoint(topLeft, {1000.0f, 1000.0f}), "the pasted nodes are not at the anchor");
    editor.model.Paste({2000.0f, 1000.0f});
    Check(editor.NodeCount() == 6 && editor.WireCount() == 3, "pasting twice did not give three copies of the pair and their wire");

    std::set<uint32_t> ids;
    for (const graph::Node& node : editor.model.GetGraph().Nodes()) {
        ids.insert(node.id);
    }
    Check(ids.size() == 6, "pasting gave two nodes the same number");
    // Each wire runs between two nodes of the same copy.
    for (const graph::Connection& wire : editor.model.GetGraph().Connections()) {
        const float fromX = editor.PositionOf(wire.from.node).x;
        const float toX = editor.PositionOf(wire.to.node).x;
        Check(std::abs(toX - fromX - 300.0f) < 1e-3f, "a pasted wire joins nodes of different copies");
    }
    editor.model.Undo();
    Check(editor.NodeCount() == 4, "a paste was not one undo step");
}

void CutRemovesTheNodesAndPasteBringsThemBack() {
    Editor editor;
    const uint32_t a = editor.Add("math.add", 100.0f, 100.0f);
    editor.model.SelectNode(a);
    editor.model.Cut();
    Check(editor.NodeCount() == 0, "cut did not remove the node");
    editor.model.Paste({10.0f, 10.0f});
    Check(editor.NodeCount() == 1, "paste did not bring the cut node back");
}

void PastingWithNothingCopiedDoesNothing() {
    Editor editor;
    editor.Add("math.add", 0.0f, 0.0f);
    const size_t steps = editor.model.UndoCount();
    editor.model.ClearSelection();
    editor.model.Paste({0.0f, 0.0f});
    editor.model.Duplicate();
    Check(editor.NodeCount() == 1 && editor.model.UndoCount() == steps, "pasting nothing changed the graph");
}

// ---------------------------------------------------------------------------
// Typed values and variables
// ---------------------------------------------------------------------------

void ATypedValueIsConvertedToThePinTypeAndExecutionPinsTakeNone() {
    Editor editor;
    const uint32_t add = editor.Add("math.add", 0.0f, 0.0f);
    const uint32_t print = editor.Add("debug.print", 300.0f, 0.0f);
    editor.model.SetInputValue(add, 0, 3);
    const graph::Node* node = editor.model.GetGraph().FindNode(add);
    Check(std::holds_alternative<float>(node->inputValues.at(0)) && std::get<float>(node->inputValues.at(0)) == 3.0f, "a whole number typed into a decimal input was not turned into one");
    editor.model.SetInputValue(print, 0, true);
    Check(editor.model.GetGraph().FindNode(print)->inputValues.count(0) == 0, "a value was put on an execution pin");
    editor.model.SetInputValue(print, 7, std::string("x"));
    Check(editor.model.GetGraph().FindNode(print)->inputValues.count(7) == 0, "a value was put on a pin that is not there");
}

void VariablesAreCheckedWhenMadeRenamedAndRetyped() {
    Editor editor;
    Check(editor.model.AddVariable("Score", PinType::Float).empty(), "adding Score failed");
    Check(!editor.model.AddVariable("Score", PinType::Int).empty(), "a second variable with the same name was accepted");
    Check(!editor.model.AddVariable("", PinType::Int).empty(), "a variable with no name was accepted");
    Check(!editor.model.AddVariable("Flow", PinType::Exec).empty(), "a variable that holds execution was accepted");
    Check(editor.model.AddVariable("Other", PinType::Int).empty(), "adding Other failed");

    const uint32_t get = editor.model.AddNodeFromPalette(*FindEntry(editor.model.SearchPalette("score"), "variable.get"), {0.0f, 0.0f});
    const uint32_t add = editor.Add("math.add", 300.0f, 0.0f);
    editor.Link(get, 0, add, 0);

    Check(!editor.model.RenameVariable("Score", "Other").empty(), "a variable was renamed to a name that is taken");
    Check(editor.model.RenameVariable("Score", "Points").empty(), "renaming failed");
    Check(std::get<std::string>(editor.model.GetGraph().FindNode(get)->properties.at("variable")) == "Points", "the node still names the old variable");
    Check(!editor.model.HasErrors(), "renaming a variable left its nodes pointing at nothing");

    Check(editor.model.SetVariableType("Points", PinType::Vec3).empty(), "changing the type failed");
    Check(editor.WireCount() == 0, "a wire that no longer fits the type of the variable was kept");
    Check(!editor.model.HasErrors(), "changing the type left an error");
    Check(std::holds_alternative<glm::vec3>(editor.model.GetGraph().FindVariable("Points")->initial), "the starting value was not turned into the new type");

    editor.model.SetVariableInitial("Points", glm::vec3(1.0f, 2.0f, 3.0f));
    Check(std::get<glm::vec3>(editor.model.GetGraph().FindVariable("Points")->initial) == glm::vec3(1.0f, 2.0f, 3.0f), "the starting value was not set");

    Check(editor.model.RemoveVariable("Points").empty(), "removing failed");
    Check(editor.model.HasErrors() && !editor.model.IssuesFor(get).empty(), "a node of a removed variable is not flagged");
    Check(!editor.model.RemoveVariable("Points").empty(), "removing a variable twice said nothing");
    editor.model.Undo();
    Check(!editor.model.HasErrors(), "undoing the removal did not bring the variable back");
}

void ChoosingAnotherVariableForANodeDropsWiresThatNoLongerFit() {
    Editor editor;
    editor.model.AddVariable("Count", PinType::Float);
    editor.model.AddVariable("Where", PinType::Vec3);
    const uint32_t get = editor.model.AddNodeFromPalette(*FindEntry(editor.model.SearchPalette("count"), "variable.get"), {0.0f, 0.0f});
    const uint32_t add = editor.Add("math.add", 300.0f, 0.0f);
    editor.Link(get, 0, add, 0);
    editor.model.SetNodeVariable(get, "Where");
    Check(editor.WireCount() == 0, "a vector output is still wired into a number input");
    editor.model.SetNodeVariable(get, "Nothing");
    Check(std::get<std::string>(editor.model.GetGraph().FindNode(get)->properties.at("variable")) == "Where", "a variable that does not exist was chosen");
}

// ---------------------------------------------------------------------------
// Undo and redo
// ---------------------------------------------------------------------------

void UndoAndRedoWalkBackAndForthThroughEveryKindOfEdit() {
    Editor editor;
    std::vector<std::string> states = {editor.Text()};
    auto remember = [&] { states.push_back(editor.Text()); };

    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    remember();
    const uint32_t print = editor.Add("debug.print", 300.0f, 0.0f);
    remember();
    editor.Link(start, 0, print, 0);
    remember();
    editor.model.SetInputValue(print, 1, std::string("Hi"));
    remember();
    editor.model.AddVariable("V", PinType::Int);
    remember();
    editor.model.SelectNode(print);
    editor.model.MoveSelection({5.0f, 5.0f});
    editor.model.EndGesture();
    remember();
    editor.model.Duplicate();
    remember();
    editor.model.DeleteSelection();
    remember();
    editor.model.AutoLayoutAll();
    remember();
    editor.model.BreakLinks({{start, 0}, true});
    remember();

    for (size_t step = states.size() - 1; step > 0; --step) {
        editor.model.Undo();
        Check(editor.Text() == states[step - 1], "undo to step " + std::to_string(step - 1) + " did not restore the graph");
    }
    Check(!editor.model.CanUndo(), "there is more to undo than was done");
    for (size_t step = 1; step < states.size(); ++step) {
        editor.model.Redo();
        Check(editor.Text() == states[step], "redo to step " + std::to_string(step) + " did not restore the graph");
    }
    Check(!editor.model.CanRedo(), "there is more to redo than was undone");
}

void AnEditAfterUndoMakesTheUndoneStepsUnreachable() {
    Editor editor;
    editor.Add("math.add", 0.0f, 0.0f);
    editor.Add("math.add", 100.0f, 0.0f);
    editor.model.Undo();
    Check(editor.model.CanRedo(), "nothing to redo after an undo");
    editor.Add("math.add", 200.0f, 0.0f);
    Check(!editor.model.CanRedo(), "a new edit left the undone step reachable");
    editor.model.Redo();
    Check(editor.NodeCount() == 2, "redo after a new edit changed the graph");
}

void UndoKeepsOnlyTheLastHundredSteps() {
    Editor editor;
    for (int i = 0; i < 150; ++i) {
        editor.Add("math.add", static_cast<float>(i) * 10.0f, 0.0f);
    }
    Check(editor.model.UndoCount() == kMaxUndoSteps, "the history holds " + std::to_string(editor.model.UndoCount()) + " steps, not " + std::to_string(kMaxUndoSteps));
    int undone = 0;
    while (editor.model.CanUndo()) {
        editor.model.Undo();
        ++undone;
    }
    Check(undone == 100, "could undo " + std::to_string(undone) + " steps");
    Check(editor.NodeCount() == 50, "after undoing everything that was kept " + std::to_string(editor.NodeCount()) + " nodes are left, not 50");
    for (int i = 0; i < 100; ++i) {
        editor.model.Redo();
    }
    Check(editor.NodeCount() == 150, "redoing everything did not bring back all nodes");
}

void ChangesThatShareAMergeKeyAreOneStepUntilTheGestureEnds() {
    UndoHistory history;
    history.Record("a", "drag");
    history.Record("b", "drag");
    history.Record("c", "drag");
    Check(history.UndoCount() == 1, "three records with one key made " + std::to_string(history.UndoCount()) + " steps");
    history.EndGesture();
    history.Record("d", "drag");
    Check(history.UndoCount() == 2, "a record after the gesture ended did not start a new step");
    history.Record("e", "other");
    Check(history.UndoCount() == 3, "a record with another key did not start a new step");
    history.Record("f");
    history.Record("g");
    Check(history.UndoCount() == 5, "records without a key were merged");
    const std::optional<std::string> back = history.Undo("now");
    Check(back && *back == "g", "undo did not give the latest state recorded");
    const std::optional<std::string> forward = history.Redo("g");
    Check(forward && *forward == "now", "redo did not give back the state undone from");
}

void TypingAValueIsOneStepAndSeparateEditsAreSeparate() {
    Editor editor;
    const uint32_t add = editor.Add("math.add", 0.0f, 0.0f);
    const size_t steps = editor.model.UndoCount();
    for (int i = 1; i <= 10; ++i) {
        editor.model.SetInputValue(add, 0, static_cast<float>(i));
    }
    Check(editor.model.UndoCount() == steps + 1, "dragging a value was not one undo step");
    editor.model.EndGesture();
    editor.model.SetInputValue(add, 1, 8.0f);
    Check(editor.model.UndoCount() == steps + 2, "a value on another pin did not start another step");
    editor.model.Undo();
    Check(editor.model.GetGraph().FindNode(add)->inputValues.count(1) == 0 && std::get<float>(editor.model.GetGraph().FindNode(add)->inputValues.at(0)) == 10.0f,
          "undo did not give back the value from before the second edit");
    editor.model.Undo();
    Check(editor.model.GetGraph().FindNode(add)->inputValues.empty(), "undo did not give back the value from before the drag");
}

void UndoDropsFromTheSelectionWhatIsNoLongerThere() {
    Editor editor;
    const uint32_t a = editor.Add("math.add", 0.0f, 0.0f);
    Check(editor.model.IsSelected(a), "a new node is not selected");
    editor.model.Undo();
    Check(editor.model.Selection().empty(), "an undone node is still selected");

    const uint32_t b = editor.Add("math.add", 0.0f, 0.0f);
    const uint32_t c = editor.Add("math.add", 300.0f, 0.0f);
    editor.Link(b, 0, c, 0);
    const WireCurve curve = MakeWireCurve(editor.Output(b, 0), editor.Input(c, 0));
    editor.Click(curve.PointAt(0.5f));
    Check(editor.model.SelectedConnection().has_value(), "the wire was not selected");
    editor.model.Undo();
    Check(!editor.model.SelectedConnection(), "an undone wire is still selected");
}

// ---------------------------------------------------------------------------
// Running
// ---------------------------------------------------------------------------

// Start leads to a Print of the text. Returns the Print.
uint32_t StartThenPrint(Editor& editor, const std::string& text, uint32_t* startNode = nullptr) {
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    const uint32_t print = editor.Add("debug.print", 300.0f, 0.0f);
    editor.Link(start, 0, print, 0);
    editor.model.SetInputValue(print, 1, text);
    if (startNode != nullptr) {
        *startNode = start;
    }
    return print;
}

void PlayRunsTheGraphAndLightsTheNodesThatRanForAMoment() {
    Editor editor;
    uint32_t start = 0;
    const uint32_t print = StartThenPrint(editor, "Hello", &start);
    Check(!editor.model.IsPlaying(), "playing before Play");
    Check(editor.model.Play() && editor.model.IsPlaying(), "Play did not start a graph that has no errors");
    Check(editor.model.LogLines() == std::vector<std::string>{"Hello"}, "the log does not hold what Print printed");
    Check(Near(editor.model.NodeGlow(start), 1.0f) && Near(editor.model.NodeGlow(print), 1.0f), "the nodes that ran are not fully lit just after");

    editor.model.Tick(0.4f);
    const float dimmed = editor.model.NodeGlow(print);
    Check(dimmed > 0.0f && dimmed < 1.0f, "a node did not dim as time passed: " + std::to_string(dimmed));
    editor.model.Tick(0.5f);
    Check(editor.model.NodeGlow(print) == 0.0f, "a node stayed lit for longer than the trail lasts");

    editor.model.Stop();
    Check(!editor.model.IsPlaying() && editor.model.NodeGlow(start) == 0.0f, "Stop left the graph playing or lit");
    Check(editor.model.LogLines() == std::vector<std::string>{"Hello"}, "Stop threw away the log");

    Check(editor.model.Play(), "playing a second time failed");
    Check(editor.model.LogLines().size() == 1, "a second play did not begin with a fresh log");
}

void EveryFrameEventsRunOnEachTickAndLightTheirNodesAgain() {
    Editor editor;
    const uint32_t tick = editor.Add("event.tick", 0.0f, 0.0f);
    const uint32_t print = editor.Add("debug.print", 300.0f, 0.0f);
    editor.Link(tick, 0, print, 0);
    editor.model.SetInputValue(print, 1, std::string("tick"));
    editor.model.Play();
    Check(editor.model.LogLines().empty() && editor.model.NodeGlow(print) == 0.0f, "an every-frame event ran at the start");
    for (int i = 0; i < 3; ++i) {
        editor.model.Tick(0.1f);
    }
    Check(editor.model.LogLines().size() == 3, "an every-frame event did not run once for each tick");
    Check(Near(editor.model.NodeGlow(print), 1.0f), "the node that ran in the last tick is not lit");
    editor.model.Stop();
    editor.model.Tick(0.1f);
    Check(editor.model.LogLines().size() == 3, "ticking after Stop ran the graph");
}

void ARunIsOnACopySoEditingWhilePlayingDoesNotTouchIt() {
    Editor editor;
    const uint32_t tick = editor.Add("event.tick", 0.0f, 0.0f);
    const uint32_t print = editor.Add("debug.print", 300.0f, 0.0f);
    editor.Link(tick, 0, print, 0);
    editor.model.SetInputValue(print, 1, std::string("tick"));
    editor.model.Play();
    editor.model.Tick(0.1f);

    editor.model.SelectNode(print);
    editor.model.DeleteSelection();
    editor.model.Tick(0.1f);
    Check(editor.model.IsPlaying(), "editing stopped the run");
    Check(editor.model.LogLines().size() == 2, "the run did not go on with the graph as it was when it started");

    editor.model.Undo();
    editor.model.Tick(0.1f);
    Check(editor.model.LogLines().size() == 3, "undoing while playing disturbed the run");
}

void PlayRefusesAGraphWithErrorsAndTheNodeIsFlagged() {
    Editor editor;
    editor.model.AddVariable("Gone", PinType::Float);
    const uint32_t get = editor.model.AddNodeFromPalette(*FindEntry(editor.model.SearchPalette("gone"), "variable.get"), {0.0f, 0.0f});
    Check(!editor.model.HasErrors(), "a valid graph has errors");
    editor.model.RemoveVariable("Gone");

    Check(editor.model.HasErrors(), "a node of a variable that is gone is not an error");
    const std::vector<const graph::Issue*> issues = editor.model.IssuesFor(get);
    Check(!issues.empty() && issues[0]->isError, "the node is not flagged with the error");
    Check(!editor.model.Play(), "a graph with errors was played");
    Check(!editor.model.IsPlaying(), "a graph with errors is playing");
    Check(Contains(editor.model.StatusMessage(), "Cannot play"), "the refusal does not say so: " + editor.model.StatusMessage());
}

void IssuesAreCheckedAgainAfterEveryEdit() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    const uint32_t print = editor.Add("debug.print", 300.0f, 0.0f);
    const bool warnedOfUnrunAction = std::any_of(editor.model.Issues().begin(), editor.model.Issues().end(),
                                                 [&](const graph::Issue& issue) { return issue.node == print && !issue.isError; });
    Check(warnedOfUnrunAction, "an action that nothing runs is not flagged");
    editor.Link(start, 0, print, 0);
    Check(editor.model.IssuesFor(print).empty(), "the warning stayed after the action was wired");
    editor.model.Undo();
    Check(!editor.model.IssuesFor(print).empty(), "the warning did not come back after undo");
}

void AnErrorThatStopsARunIsKeptAfterStop() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    const uint32_t loop = editor.Add("flow.for", 300.0f, 0.0f);
    editor.Link(start, 0, loop, 0);
    editor.model.SetInputValue(loop, 2, 2000000000);
    const uint32_t body = editor.Add("debug.print", 700.0f, 0.0f);
    editor.Link(loop, 0, body, 0);
    editor.model.Play();
    Check(!editor.model.RuntimeError().empty(), "a loop that would not end did not give a runtime error");
    editor.model.Stop();
    Check(!editor.model.RuntimeError().empty(), "the runtime error was lost when the run was stopped");
}

// ---------------------------------------------------------------------------
// Files
// ---------------------------------------------------------------------------

void ASavedFileLoadsBackAsTheSameGraphWithAFreshHistory() {
    const std::string path = TemporaryPath("xcom_graph_editor_roundtrip.json");
    std::string saved;
    {
        Editor editor;
        StartThenPrint(editor, "Saved");
        editor.model.AddVariable("Lives", PinType::Int);
        Check(editor.model.IsModified(), "an edited graph is not marked as modified");
        Check(editor.model.SaveToFile(path).empty(), "saving failed");
        Check(!editor.model.IsModified() && editor.model.FilePath() == path, "saving did not clear the modified mark and remember the path");
        saved = editor.Text();
    }
    Editor other;
    other.Add("math.add", 0.0f, 0.0f);
    Check(other.model.LoadFromFile(path).empty(), "loading failed: " + other.model.StatusMessage());
    Check(other.Text() == saved, "the loaded graph is not the one that was saved");
    Check(!other.model.CanUndo() && other.model.Selection().empty(), "loading kept the history or the selection of the old graph");
    Check(!other.model.IsModified() && other.model.FilePath() == path, "a loaded graph is marked as modified");

    other.model.NewGraph();
    Check(other.NodeCount() == 0 && other.model.FilePath().empty() && !other.model.CanUndo(), "a new graph is not empty and unnamed");
    std::filesystem::remove(path);
}

void ALoadThatFailsLeavesTheGraphAsItWasAndSaysWhy() {
    Editor editor;
    StartThenPrint(editor, "Keep");
    const std::string before = editor.Text();

    const std::string missing = TemporaryPath("xcom_graph_editor_missing.json");
    std::filesystem::remove(missing);
    const std::string cannotOpen = editor.model.LoadFromFile(missing);
    Check(!cannotOpen.empty() && editor.Text() == before, "a missing file changed the graph or gave no reason");

    const std::string spoiled = TemporaryPath("xcom_graph_editor_spoiled.json");
    std::ofstream(spoiled) << "this is not a graph";
    const std::string cannotRead = editor.model.LoadFromFile(spoiled);
    Check(!cannotRead.empty() && editor.Text() == before, "a spoiled file changed the graph or gave no reason");
    Check(editor.model.CanUndo(), "a failed load threw away the history");
    std::filesystem::remove(spoiled);

    Check(!editor.model.SaveToFile("").empty(), "saving to no name did not say anything");
}

void ANodePlacementlessFileIsArrangedWhenLoaded() {
    // A graph made by a program puts every node at the origin; loaded as is it would be a pile.
    Editor maker;
    const uint32_t start = maker.Add("event.start", 0.0f, 0.0f);
    const uint32_t first = maker.Add("debug.print", 0.0f, 0.0f);
    const uint32_t second = maker.Add("debug.print", 0.0f, 0.0f);
    maker.Link(start, 0, first, 0);
    maker.Link(first, 0, second, 0);
    const std::string path = TemporaryPath("xcom_graph_editor_pile.json");
    maker.model.SaveToFile(path);

    Editor editor;
    Check(editor.model.LoadFromFile(path).empty(), "loading the pile failed");
    Check(editor.PositionOf(start).x < editor.PositionOf(first).x && editor.PositionOf(first).x < editor.PositionOf(second).x, "the loaded nodes are not arranged in the order of the wires");
    std::filesystem::remove(path);
}

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------

void AutoLayoutPutsNodesInColumnsInOrderOfTheWiresWithoutOverlap() {
    Editor editor;
    const uint32_t start = editor.Add("event.start", 500.0f, 500.0f);
    const uint32_t branch = editor.Add("flow.branch", 500.0f, 500.0f);
    const uint32_t yes = editor.Add("debug.print", 500.0f, 500.0f);
    const uint32_t no = editor.Add("debug.print", 500.0f, 500.0f);
    const uint32_t less = editor.Add("compare.less", 500.0f, 500.0f);
    editor.Link(start, 0, branch, 0);
    editor.Link(branch, 0, yes, 0);
    editor.Link(branch, 1, no, 0);
    editor.Link(less, 0, branch, 1);

    editor.model.AutoLayoutAll();
    Check(editor.PositionOf(start).x < editor.PositionOf(branch).x, "the event is not left of the node it leads to");
    Check(editor.PositionOf(less).x < editor.PositionOf(branch).x, "a value is not left of the node that reads it");
    Check(editor.PositionOf(branch).x < editor.PositionOf(yes).x && editor.PositionOf(branch).x < editor.PositionOf(no).x, "a branch is not left of what it leads to");

    const std::vector<graph::Node>& nodes = editor.model.GetGraph().Nodes();
    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = i + 1; j < nodes.size(); ++j) {
            const Rect a = editor.model.GeometryOf(nodes[i].id).bounds;
            const Rect b = editor.model.GeometryOf(nodes[j].id).bounds;
            Check(!a.Overlaps(b), "two nodes overlap after the layout");
        }
    }
    Check(editor.model.GraphBounds().has_value(), "a graph with nodes has no bounds");

    editor.model.Undo();
    Check(NearPoint(editor.PositionOf(branch), {500.0f, 500.0f}), "undo did not put the nodes back where they were");
}

void AutoLayoutTerminatesWhenExecutionAndDataWiresFormACycle() {
    // An action's result may feed a pure node that feeds an earlier action. That is no loop for the graph, but it is a cycle
    // for the layout, which must still finish.
    Editor editor;
    const uint32_t start = editor.Add("event.start", 0.0f, 0.0f);
    const uint32_t first = editor.Add("debug.print", 0.0f, 0.0f);
    editor.model.AddVariable("X", PinType::Float);
    const uint32_t set = editor.model.AddNodeFromPalette(*FindEntry(editor.model.SearchPalette("set x"), "variable.set"), {0.0f, 0.0f});
    const uint32_t add = editor.Add("math.add", 0.0f, 0.0f);
    editor.Link(start, 0, first, 0);
    editor.Link(first, 0, set, 0);
    editor.Link(set, 1, add, 0);
    editor.Link(add, 0, first, 1);
    editor.model.AutoLayoutAll();
    Check(editor.model.GraphBounds().has_value(), "the layout of a cyclic graph did not finish");
}

}  // namespace

int main() {
    struct Test {
        const char* name;
        std::function<void()> run;
    };
    const Test tests[] = {
        {"ZoomingKeepsTheCanvasUnderThePointerWhereItIs", ZoomingKeepsTheCanvasUnderThePointerWhereItIs},
        {"ZoomStaysWithinItsLimitsAndStillAnchorsThere", ZoomStaysWithinItsLimitsAndStillAnchorsThere},
        {"PanningMovesTheCanvasWithThePointer", PanningMovesTheCanvasWithThePointer},
        {"FramingShowsAnAreaCentredAndInsideTheWindow", FramingShowsAnAreaCentredAndInsideTheWindow},
        {"PinsSitOnTheEdgesOfTheirNodeInTheOrderOfTheirRows", PinsSitOnTheEdgesOfTheirNodeInTheOrderOfTheirRows},
        {"ANodeIsAsWideAsItsNamesNeed", ANodeIsAsWideAsItsNamesNeed},
        {"AClickSelectsTheTopmostNodeAndEmptyCanvasClearsTheSelection", AClickSelectsTheTopmostNodeAndEmptyCanvasClearsTheSelection},
        {"AdditiveClicksAddAndToggle", AdditiveClicksAddAndToggle},
        {"ABoxSelectsWhatItTouchesAndAddsWhenAdditive", ABoxSelectsWhatItTouchesAndAddsWhenAdditive},
        {"ANodeSelectedBySelectInRectOnlyIfItTouchesIt", ANodeSelectedBySelectInRectOnlyIfItTouchesIt},
        {"DraggingANodeMovesTheWholeSelectionAsOneUndoStep", DraggingANodeMovesTheWholeSelectionAsOneUndoStep},
        {"PressingAnUnselectedNodeSelectsItAndPressingOneOfSeveralKeepsThemUntilAClick", PressingAnUnselectedNodeSelectsItAndPressingOneOfSeveralKeepsThemUntilAClick},
        {"AClickThatWobblesDoesNotMoveTheNode", AClickThatWobblesDoesNotMoveTheNode},
        {"TwoSeparateDragsAreTwoUndoSteps", TwoSeparateDragsAreTwoUndoSteps},
        {"DraggingFromAnOutputPinToAnInputPinMakesTheWire", DraggingFromAnOutputPinToAnInputPinMakesTheWire},
        {"DraggingFromAnInputToAnOutputMakesTheSameWire", DraggingFromAnInputToAnOutputMakesTheSameWire},
        {"ARefusedWireSaysWhyInTheWordsOfTheGraph", ARefusedWireSaysWhyInTheWordsOfTheGraph},
        {"ADroppedWireOnItsOwnPinIsAClickAndMakesNothing", ADroppedWireOnItsOwnPinIsAClickAndMakesNothing},
        {"AWiredInputTakesItsWireUpToMoveItAsOneUndoStep", AWiredInputTakesItsWireUpToMoveItAsOneUndoStep},
        {"ATakenUpWireLetGoOnEmptyCanvasOpensThePaletteForItsOutput", ATakenUpWireLetGoOnEmptyCanvasOpensThePaletteForItsOutput},
        {"EscapeGivesUpADraggedWire", EscapeGivesUpADraggedWire},
        {"ClickingAWireSelectsItAndDeleteTakesItAway", ClickingAWireSelectsItAndDeleteTakesItAway},
        {"BreakingLinksOfAPinTakesAwayAllWiresOnIt", BreakingLinksOfAPinTakesAwayAllWiresOnIt},
        {"TheEmptyQueryListsEveryKindByCategoryAndTitle", TheEmptyQueryListsEveryKindByCategoryAndTitle},
        {"TypingFiltersByNameCategoryAndPinTypes", TypingFiltersByNameCategoryAndPinTypes},
        {"ATitleMatchRanksAboveAPinMatch", ATitleMatchRanksAboveAPinMatch},
        {"VariablesGetAndSetNodesAppearOncePerVariable", VariablesGetAndSetNodesAppearOncePerVariable},
        {"ADroppedExecutionWireOffersOnlyNodesWithAnExecutionInput", ADroppedExecutionWireOffersOnlyNodesWithAnExecutionInput},
        {"ADroppedDataWireOffersNodesThatCanTakeItAndWiresTheFirstPinThatFits", ADroppedDataWireOffersNodesThatCanTakeItAndWiresTheFirstPinThatFits},
        {"ADroppedWireFromAnInputOffersNodesWithAnOutputThatFitsIt", ADroppedWireFromAnInputOffersNodesWithAnOutputThatFitsIt},
        {"ADroppedExecutionInputIsOfferedEventsAndActions", ADroppedExecutionInputIsOfferedEventsAndActions},
        {"APressClearsThePendingWireSoThePaletteIsNotNarrowedByAnOldOne", APressClearsThePendingWireSoThePaletteIsNotNarrowedByAnOldOne},
        {"ANodeFromThePaletteWithoutAWireIsJustAdded", ANodeFromThePaletteWithoutAWireIsJustAdded},
        {"AddingAKindThatDoesNotExistIsRefused", AddingAKindThatDoesNotExistIsRefused},
        {"DeletingNodesRemovesTheirWiresAndUndoBringsBackTheSameNumbers", DeletingNodesRemovesTheirWiresAndUndoBringsBackTheSameNumbers},
        {"DuplicatingKeepsTheWiresBetweenCopiedNodesAndLeavesOutTheOthers", DuplicatingKeepsTheWiresBetweenCopiedNodesAndLeavesOutTheOthers},
        {"CopyAndPasteTwiceGivesUniqueNumbersAtTheAnchor", CopyAndPasteTwiceGivesUniqueNumbersAtTheAnchor},
        {"CutRemovesTheNodesAndPasteBringsThemBack", CutRemovesTheNodesAndPasteBringsThemBack},
        {"PastingWithNothingCopiedDoesNothing", PastingWithNothingCopiedDoesNothing},
        {"ATypedValueIsConvertedToThePinTypeAndExecutionPinsTakeNone", ATypedValueIsConvertedToThePinTypeAndExecutionPinsTakeNone},
        {"VariablesAreCheckedWhenMadeRenamedAndRetyped", VariablesAreCheckedWhenMadeRenamedAndRetyped},
        {"ChoosingAnotherVariableForANodeDropsWiresThatNoLongerFit", ChoosingAnotherVariableForANodeDropsWiresThatNoLongerFit},
        {"UndoAndRedoWalkBackAndForthThroughEveryKindOfEdit", UndoAndRedoWalkBackAndForthThroughEveryKindOfEdit},
        {"AnEditAfterUndoMakesTheUndoneStepsUnreachable", AnEditAfterUndoMakesTheUndoneStepsUnreachable},
        {"UndoKeepsOnlyTheLastHundredSteps", UndoKeepsOnlyTheLastHundredSteps},
        {"ChangesThatShareAMergeKeyAreOneStepUntilTheGestureEnds", ChangesThatShareAMergeKeyAreOneStepUntilTheGestureEnds},
        {"TypingAValueIsOneStepAndSeparateEditsAreSeparate", TypingAValueIsOneStepAndSeparateEditsAreSeparate},
        {"UndoDropsFromTheSelectionWhatIsNoLongerThere", UndoDropsFromTheSelectionWhatIsNoLongerThere},
        {"PlayRunsTheGraphAndLightsTheNodesThatRanForAMoment", PlayRunsTheGraphAndLightsTheNodesThatRanForAMoment},
        {"EveryFrameEventsRunOnEachTickAndLightTheirNodesAgain", EveryFrameEventsRunOnEachTickAndLightTheirNodesAgain},
        {"ARunIsOnACopySoEditingWhilePlayingDoesNotTouchIt", ARunIsOnACopySoEditingWhilePlayingDoesNotTouchIt},
        {"PlayRefusesAGraphWithErrorsAndTheNodeIsFlagged", PlayRefusesAGraphWithErrorsAndTheNodeIsFlagged},
        {"IssuesAreCheckedAgainAfterEveryEdit", IssuesAreCheckedAgainAfterEveryEdit},
        {"AnErrorThatStopsARunIsKeptAfterStop", AnErrorThatStopsARunIsKeptAfterStop},
        {"ASavedFileLoadsBackAsTheSameGraphWithAFreshHistory", ASavedFileLoadsBackAsTheSameGraphWithAFreshHistory},
        {"ALoadThatFailsLeavesTheGraphAsItWasAndSaysWhy", ALoadThatFailsLeavesTheGraphAsItWasAndSaysWhy},
        {"ANodePlacementlessFileIsArrangedWhenLoaded", ANodePlacementlessFileIsArrangedWhenLoaded},
        {"AutoLayoutPutsNodesInColumnsInOrderOfTheWiresWithoutOverlap", AutoLayoutPutsNodesInColumnsInOrderOfTheWiresWithoutOverlap},
        {"AutoLayoutTerminatesWhenExecutionAndDataWiresFormACycle", AutoLayoutTerminatesWhenExecutionAndDataWiresFormACycle},
    };
    for (const Test& test : tests) {
        const int failuresBefore = failureCount;
        try {
            test.run();
        } catch (const std::exception& error) {
            ++failureCount;
            std::cerr << "  threw: " << error.what() << "\n";
        }
        std::cout << (failureCount == failuresBefore ? "PASS  " : "FAIL  ") << test.name << "\n";
    }
    std::cout << (failureCount == 0 ? "all graph editor tests passed\n" : "graph editor tests failed\n");
    return failureCount == 0 ? 0 : 1;
}
