#pragma once

#include <string>

#include <glm/glm.hpp>
#include <imgui.h>

#include "editor_ui/GraphCanvas.h"
#include "graph_editor/EditorModel.h"

// The panels around the canvas: the toolbar on the left, the details of the selected node and the variables on the right, the
// log and problems at the bottom, and the palette that pops up to add a node. Each draws into the window that is current.

namespace editor_ui {

// What the panels remember between frames: text being typed, and whether the palette is open.
struct PanelState {
    char filePath[260] = "graph.json";
    bool fullHeight = false;      // the editor covers the whole window, and not the lower part
    bool closeRequested = false;

    bool paletteOpen = false;
    glm::vec2 paletteCanvasPosition{0.0f};
    ImVec2 paletteScreenPosition{0.0f, 0.0f};
    char paletteQuery[96] = "";
    int paletteHighlight = 0;
    bool paletteJustOpened = false;

    int renamingVariable = -1;
    char renameBuffer[64] = "";
    char newVariableName[64] = "";
    int newVariableType = 2;      // the index of a type in the list of types a variable can hold; 2 is float
    std::string variableMessage;  // why the last change to a variable was refused

    bool editingValue = false;    // a control in a panel changed a value that has not been let go of yet
};

void DrawToolbar(graph_editor::EditorModel& model, PanelState& state, GraphCanvas& canvas);
void DrawDetails(graph_editor::EditorModel& model, PanelState& state, GraphCanvas& canvas);
void DrawVariables(graph_editor::EditorModel& model, PanelState& state);
void DrawLogAndProblems(graph_editor::EditorModel& model, GraphCanvas& canvas);

// Opens the palette at a canvas position, for the node that is chosen to be put there.
void OpenPalette(PanelState& state, glm::vec2 canvasPosition);
// Draws the palette if it is open. Closing it without choosing gives up a wire that was waiting for a node.
void DrawPalette(graph_editor::EditorModel& model, PanelState& state);

}  // namespace editor_ui
