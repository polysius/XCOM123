#pragma once

#include <string>

#include "graph_editor/EditorModel.h"

// The menu bar across the top of the window: File (new, open, save), Edit (undo, redo), Graph (play, stop), View (which windows are
// shown) and Help (the list of shortcuts). Each item does what a button of the toolbar or a key of the canvas does, so that a person
// can find it where they expect it.

namespace editor_ui {

// Which of the editor's windows are shown.
struct WindowVisibility {
    bool graphEditor = true;
    bool characterGenerator = false;
    bool materialGenerator = false;
    bool shortcuts = false;
};

// What the menu asks of the rest of the editor, which the menu cannot do itself.
struct MenuRequests {
    bool showWholeGraph = false;  // a graph was opened: zoom to see all of it
    bool closeEditor = false;
};

// What the menu bar remembers between frames: which file dialog is open, and the text typed in it.
struct MenuState {
    enum class Dialog { None, Open, SaveAs };
    Dialog dialog = Dialog::None;
    bool dialogJustRequested = false;
    char path[260] = "graph.json";
    std::string message;  // why the last open or save did not work
};

// Draws the menu bar and the file dialogs. Only inside an ImGui frame. The menu bar takes room from the top of the main viewport.
MenuRequests DrawMenuBar(graph_editor::EditorModel& model, WindowVisibility& windows, MenuState& state);

// The window that lists the shortcuts, while `open` is true.
void DrawShortcutsWindow(bool& open);

}  // namespace editor_ui
