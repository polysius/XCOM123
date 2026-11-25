#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>

#include <glm/glm.hpp>
#include <imgui.h>

#include "graph_editor/EditorModel.h"

// The canvas of the graph editor: the grid, the nodes and wires, and the mouse and keys that edit them. It draws with ImDrawList
// and puts the inline value boxes on the nodes as ImGui controls. What an edit does is the model's work; this only tells the
// model what the pointer and the keys did, and draws what the model holds.

namespace editor_ui {

// What the canvas asks of the window around it.
struct CanvasRequests {
    bool openPalette = false;
    glm::vec2 paletteCanvasPosition{0.0f};  // where the new node goes
    bool saveRequested = false;             // Ctrl+S
};

class GraphCanvas {
public:
    // Draws into a child window of `size` at the cursor and handles its input.
    void Draw(graph_editor::EditorModel& model, ImVec2 size, CanvasRequests& requests);

    // Shows the selected nodes, or all nodes, at the next Draw.
    void RequestFrame(bool onlySelection) { frameRequest = onlySelection ? FrameRequest::Selection : FrameRequest::All; }

private:
    enum class FrameRequest { None, All, Selection };

    // What a right click landed on, for the menu it opens.
    struct ContextTarget {
        std::optional<graph_editor::PinHit> pin;
        std::optional<graph::Connection> wire;
        uint32_t node = 0;
    };

    // Everything below runs inside Draw, between BeginChild and EndChild.
    ImVec2 ToScreen(glm::vec2 canvasPoint) const;
    glm::vec2 PointerCanvasPosition() const;
    void ApplyFrameRequest();

    void HandleInput(CanvasRequests& requests);
    void HandleViewControls();
    void HandlePrimaryButton(CanvasRequests& requests);
    void HandleSecondaryButton(CanvasRequests& requests);
    void HandleKeys(CanvasRequests& requests);
    void DrawContextMenu();

    void DrawBackground();
    void DrawGrid();
    void DrawWires();
    void DrawNodes();
    void DrawNode(const graph::Node& node);
    void DrawNodeHeader(const graph::Node& node, const graph_editor::NodeGeometry& geometry, const graph::NodeDescription* description);
    void DrawNodePins(const graph::Node& node, const graph_editor::NodeGeometry& geometry, const graph::NodePins& pins);
    void DrawInlineValue(const graph::Node& node, const graph_editor::PinSlot& slot, const graph::PinDescription& pin, float labelWidth);
    void DrawWireInProgress();
    void DrawSelectionBox();
    void DrawMinimap();
    void DrawOverlayText();
    void DrawTooltips();

    // The glow, outline and badge a node shows for having run or having a problem.
    void DrawNodeStatusMarks(const graph::Node& node, const graph_editor::NodeGeometry& geometry);
    graph_editor::WireVerdict VerdictForPin(const graph_editor::PinHit& pin) const;

    bool IsOverMinimap(ImVec2 point) const;
    void MoveViewWithMinimap(ImVec2 point);

    // The model and the drawing context of the frame that is being drawn.
    graph_editor::EditorModel* model = nullptr;
    ImDrawList* list = nullptr;
    ImVec2 origin{0.0f, 0.0f};  // the top-left corner of the canvas on the screen
    ImVec2 canvasSize{0.0f, 0.0f};
    float zoom = 1.0f;
    bool hovered = false;       // the pointer is over the canvas and nothing covers it
    bool controlBusy = false;   // the pointer is over an inline control, or one is being used
    std::map<uint32_t, graph_editor::NodeGeometry> geometry;

    // Input that lasts from one frame to the next.
    FrameRequest frameRequest = FrameRequest::All;
    bool pressedInCanvas = false;     // the primary button went down on the canvas and has not come up
    bool panningWithSpace = false;
    bool secondaryDown = false;
    bool secondaryDragged = false;
    ImVec2 secondaryPressPosition{0.0f, 0.0f};
    bool draggingMinimap = false;
    bool editingValue = false;        // an inline control changed a value that has not been let go of yet
    ContextTarget contextTarget;

    // What the pointer rests on, for the tooltip that appears after a short wait.
    std::string hoverKey;
    float hoverSeconds = 0.0f;
};

}  // namespace editor_ui
