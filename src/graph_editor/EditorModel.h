#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "graph/Graph.h"
#include "graph/Interpreter.h"
#include "graph/NodeRegistry.h"
#include "graph_editor/NodeLayout.h"
#include "graph_editor/Palette.h"
#include "graph_editor/UndoHistory.h"
#include "graph_editor/Viewport.h"

// Everything the graph editor does, apart from drawing it: what is selected, moving nodes, dragging wires, adding nodes from the
// palette, copying and pasting, undo and redo, running the graph and saving it. The window only draws what this holds and
// tells it what the pointer and the keys did, which is why all of it can be tested without a window or a GPU.
//
// Positions given to the model are canvas points (see Viewport), and the pointer functions take the pointer's position as one.

namespace graph_editor {

enum class SelectMode {
    Replace,  // only this
    Add,      // this too
    Toggle,   // this too, or not any more if it was selected
};

// A pin of a node, and which side of the node it is on.
struct PinHit {
    graph::PinRef pin;
    bool isOutput = false;
    bool operator==(const PinHit& other) const { return pin == other.pin && isOutput == other.isOutput; }
};

// Whether a wire between two pins can be made, and if not, why, in words for a person.
struct WireVerdict {
    bool accepted = false;
    std::string reason;  // empty when accepted
};

// A wire that is being dragged from a pin, for the view to draw.
struct WireDrag {
    PinHit start;
    glm::vec2 end{0.0f};            // where the pointer is
    std::optional<PinHit> hover;    // the pin under the pointer, if any
    WireVerdict verdict;            // for the pin under the pointer; meaningless without one
};

// A wire that was let go on empty canvas: a palette opens, and the node chosen from it is wired to the pin the wire came from.
struct PendingWire {
    PinHit start;
    glm::vec2 dropPosition{0.0f};
};

enum class Gesture { None, MovingNodes, DraggingWire, BoxSelecting };

// What letting go of the pointer button came to.
enum class ReleaseOutcome {
    Nothing,
    Clicked,            // a click that selected something, or nothing
    MovedNodes,
    WireConnected,
    WireRefused,        // the wire was dropped on a pin it cannot join; the status message says why
    WireDroppedOnEmpty, // the view opens the palette, see PendingWire
    BoxSelected,
};

class EditorModel {
public:
    // The registry must outlive the model.
    explicit EditorModel(const graph::NodeRegistry& registry);

    const graph::Graph& GetGraph() const { return graph; }
    const graph::NodeRegistry& Registry() const { return registry; }
    Viewport& View() { return viewport; }
    const Viewport& View() const { return viewport; }

    // ----- Where things are -----
    // The place of a node on the canvas, or an empty box for a node that is not there.
    NodeGeometry GeometryOf(uint32_t node) const;
    // The topmost node at a point, or 0.
    uint32_t NodeAt(glm::vec2 canvasPoint) const;
    // The pin at a point. The grab area is a few screen pixels larger than the pin, whatever the zoom.
    std::optional<PinHit> PinAt(glm::vec2 canvasPoint) const;
    // The wire at a point, if it passes within a few screen pixels of it.
    std::optional<graph::Connection> ConnectionAt(glm::vec2 canvasPoint) const;
    // The canvas rectangle that holds every node, or nothing for an empty graph.
    std::optional<Rect> GraphBounds() const;

    // ----- Selection -----
    const std::set<uint32_t>& Selection() const { return selection; }
    bool IsSelected(uint32_t node) const { return selection.count(node) != 0; }
    const std::optional<graph::Connection>& SelectedConnection() const { return selectedConnection; }
    void SelectNode(uint32_t node, SelectMode mode = SelectMode::Replace);
    // Every node that touches the rectangle.
    void SelectInRect(const Rect& rect, SelectMode mode = SelectMode::Replace);
    void SelectAll();
    void ClearSelection();

    // ----- The mouse: press, drag and release the primary button -----
    // A press on a pin starts a wire (on a wired input it takes the wire up, to move it), on a node selects it and starts
    // moving the selection, on a wire selects the wire, and on empty canvas starts a selection box. `additive` is the shift or
    // control key: it adds to the selection and toggles, and a press on a pin or empty canvas then leaves the selection as it is.
    void PointerPressed(glm::vec2 canvasPoint, bool additive);
    void PointerMoved(glm::vec2 canvasPoint);
    ReleaseOutcome PointerReleased(glm::vec2 canvasPoint);
    // Gives up what is being dragged, as Escape does. Nodes that were moved stay moved, and a wire taken up stays taken up.
    void CancelGesture();

    Gesture CurrentGesture() const { return gesture; }
    const std::optional<WireDrag>& ActiveWireDrag() const { return wireDrag; }
    std::optional<Rect> ActiveSelectionBox() const;
    // Ends a run of changes that were merged into one undo step. The view calls it when a value that is being dragged is let go.
    void EndGesture() { history.EndGesture(); }

    // Whether a wire could be made between two pins, in whichever order they are given.
    WireVerdict CheckWire(const PinHit& a, const PinHit& b) const;
    // Makes the wire, or says why not.
    WireVerdict ConnectPins(const PinHit& a, const PinHit& b);

    // ----- Moving, adding and removing -----
    // Moves the selected nodes. Many calls in a row are one undo step.
    void MoveSelection(glm::vec2 delta);
    uint32_t AddNode(const std::string& type, glm::vec2 position);

    // The palette for what is typed, narrowed to nodes the pending wire can join when there is one.
    std::vector<PaletteEntry> SearchPalette(const std::string& query) const;
    // Adds the node of an entry, selects it, and when a wire is pending wires it to the pin the entry names. Returns its number.
    uint32_t AddNodeFromPalette(const PaletteEntry& entry, glm::vec2 position);
    const std::optional<PendingWire>& GetPendingWire() const { return pendingWire; }
    void CancelPendingWire() { pendingWire.reset(); }

    // Deletes the selected nodes with their wires, or else the selected wire.
    void DeleteSelection();
    void DeleteConnection(const graph::Connection& connection);
    // Takes away every wire on a pin.
    void BreakLinks(const PinHit& pin);

    // ----- Copying -----
    // Keeps the selected nodes and the wires between them. Paste adds them again, with new numbers, so that their top-left
    // corner is at `anchor`, and selects the copies. Duplicate is Copy and Paste a little to the side.
    void Copy();
    void Cut();
    void Paste(glm::vec2 anchor);
    void Duplicate();
    bool CanPaste() const { return !clipboard.nodes.empty(); }

    // ----- Values and variables -----
    // What a person typed into an input that nothing is wired into.
    void SetInputValue(uint32_t node, int input, const graph::Value& value);
    void SetProperty(uint32_t node, const std::string& name, const graph::Value& value);
    // Points a variable node at another variable. Wires that no longer fit the types of its pins are removed.
    void SetNodeVariable(uint32_t node, const std::string& variable);
    // Each returns why a change was refused, or an empty string.
    std::string AddVariable(const std::string& name, graph::PinType type);
    std::string RemoveVariable(const std::string& name);
    std::string RenameVariable(const std::string& name, const std::string& newName);
    std::string SetVariableType(const std::string& name, graph::PinType type);
    void SetVariableInitial(const std::string& name, const graph::Value& value);

    // ----- Undo -----
    void Undo();
    void Redo();
    bool CanUndo() const { return history.CanUndo(); }
    bool CanRedo() const { return history.CanRedo(); }
    size_t UndoCount() const { return history.UndoCount(); }

    // ----- Layout -----
    void AutoLayoutAll();

    // ----- Checking and running -----
    const graph::ValidationReport& Issues() const { return issues; }
    std::vector<const graph::Issue*> IssuesFor(uint32_t node) const;
    bool HasErrors() const { return graph::HasErrors(issues); }

    // Runs the graph's Start events, and from then on Tick runs its every-frame events. The run is on a copy of the graph, so the
    // graph can go on being edited. Returns false, with the reason as the status message, when the graph has errors.
    bool Play();
    void Stop();
    bool IsPlaying() const { return interpreter != nullptr; }
    // Does nothing unless playing.
    void Tick(float deltaSeconds);
    // How brightly a node should light up, from 1 when it has just run to 0 a moment later.
    float NodeGlow(uint32_t node) const;
    // The lines of the log of the current or last run, and what stopped it if something did.
    const std::vector<std::string>& LogLines() const;
    const std::string& RuntimeError() const;

    // ----- Files -----
    // Each returns why it failed, or an empty string.
    std::string SaveToFile(const std::string& path);
    std::string LoadFromFile(const std::string& path);
    // Loads a graph from its text. `path` is where it came from, or empty; it becomes the path that Save writes to.
    std::string LoadFromText(const std::string& text, const std::string& path);
    void NewGraph();
    const std::string& FilePath() const { return filePath; }
    bool IsModified() const { return modified; }

    // The last thing the editor has to say about what it did, for a status line: why a wire was refused, that a file was saved.
    const std::string& StatusMessage() const { return status; }
    void SetStatus(std::string message) { status = std::move(message); }

private:
    struct Clipboard {
        std::vector<graph::Node> nodes;
        std::vector<graph::Connection> connections;
    };

    // Records the state before a change, for Undo. The merge key makes a run of changes with the same key one step.
    void BeginEdit(const std::string& mergeKey = {});
    // Brings what is derived from the graph (the issues) up to date after a change.
    void GraphChanged();
    void RestoreSnapshot(const std::string& snapshot);
    void KeepOnlyThingsThatExist();
    void ClearRunState();
    void NoteNodesThatRan();

    Clipboard CollectSelection() const;
    void PasteFrom(const Clipboard& contents, glm::vec2 anchor);
    glm::vec2 PinPosition(const PinHit& pin) const;

    WireVerdict MakeWire(const PinHit& a, const PinHit& b, const std::string& mergeKey);
    void StartWire(const PinHit& start, glm::vec2 point);
    void UpdateWireDrag(glm::vec2 point);
    ReleaseOutcome FinishWire(glm::vec2 point);
    void RemoveWiresThatNoLongerFit(uint32_t node);
    std::optional<WireWish> WishOf(const PinHit& pin) const;
    std::optional<graph::PinType> TypeOfPin(const PinHit& pin) const;

    const graph::NodeRegistry& registry;
    graph::Graph graph;
    Viewport viewport;
    UndoHistory history;
    graph::ValidationReport issues;

    std::set<uint32_t> selection;
    std::optional<graph::Connection> selectedConnection;
    Clipboard clipboard;

    Gesture gesture = Gesture::None;
    glm::vec2 pressPoint{0.0f};    // where the primary button went down
    glm::vec2 lastPointer{0.0f};   // where the selection was last moved to, while moving
    glm::vec2 boxEnd{0.0f};
    bool pressedAdditive = false;
    bool movedDuringGesture = false;
    std::optional<uint32_t> pressedNode;  // the node a press began on, for collapsing the selection to it on a click
    std::optional<WireDrag> wireDrag;
    std::optional<PendingWire> pendingWire;

    // A run: the interpreter reads its own copy of the graph, because it must not see edits.
    std::unique_ptr<graph::Graph> runGraph;
    std::unique_ptr<graph::Interpreter> interpreter;
    std::map<uint32_t, float> secondsSinceRun;  // by node, for the glow of a node that ran a moment ago
    std::vector<std::string> lastLog;           // kept after Stop so that the output can still be read
    std::string lastRuntimeError;

    std::string filePath;
    bool modified = false;
    std::string status;
};

}  // namespace graph_editor
