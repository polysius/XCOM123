#pragma once

#include "editor_ui/EditorPanels.h"
#include "editor_ui/GraphCanvas.h"
#include "graph_editor/EditorModel.h"

// The window of the graph editor: it lays out the toolbar, the canvas, the log and the panels, and passes on what the canvas
// asks for. It only exists inside an ImGui frame.

namespace editor_ui {

class GraphEditorWindow {
public:
    // Draws the whole editor for one frame.
    void Draw(graph_editor::EditorModel& model);

    // Whether the Close button was pressed since the last call.
    bool TakeCloseRequest();
    bool IsPaletteOpen() const { return state.paletteOpen; }

private:
    PanelState state;
    GraphCanvas canvas;
};

}  // namespace editor_ui
