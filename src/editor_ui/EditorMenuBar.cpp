#include "editor_ui/EditorMenuBar.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <vector>

#include <imgui.h>

namespace editor_ui {
namespace {

constexpr const char* kDefaultGraphFile = "graph.json";
constexpr const char* kExampleFolder = "assets/graphs";  // where the example graphs live, from the folder the engine runs in
constexpr const char* kDialogTitles[] = {"", "Open graph", "Save graph as"};

// The example graphs that can be found from where the engine runs, by file name.
std::vector<std::string> FindExamples() {
    std::vector<std::string> examples;
    std::error_code error;
    for (const auto& entry : std::filesystem::directory_iterator(kExampleFolder, error)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            examples.push_back(entry.path().generic_string());
        }
    }
    std::sort(examples.begin(), examples.end());
    return examples;
}

void OpenDialog(MenuState& state, MenuState::Dialog dialog, const std::string& suggestedPath) {
    state.dialog = dialog;
    state.dialogJustRequested = true;
    std::snprintf(state.path, sizeof(state.path), "%s", suggestedPath.c_str());
}

std::string CurrentFileOrDefault(const graph_editor::EditorModel& model) {
    return model.FilePath().empty() ? kDefaultGraphFile : model.FilePath();
}

void DrawFileMenu(graph_editor::EditorModel& model, MenuState& state, MenuRequests& requests) {
    if (!ImGui::BeginMenu("File")) {
        return;
    }
    if (ImGui::MenuItem("New")) {
        model.NewGraph();
    }
    if (ImGui::MenuItem("Open...")) {
        OpenDialog(state, MenuState::Dialog::Open, CurrentFileOrDefault(model));
    }
    if (ImGui::BeginMenu("Open example")) {
        const std::vector<std::string> examples = FindExamples();
        if (examples.empty()) {
            ImGui::MenuItem("No examples found next to the program", nullptr, false, false);
        }
        for (const std::string& example : examples) {
            if (ImGui::MenuItem(example.c_str())) {
                state.message = model.LoadFromFile(example);
                requests.showWholeGraph = state.message.empty();
            }
        }
        ImGui::EndMenu();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Save", "Ctrl+S")) {
        state.message = model.SaveToFile(CurrentFileOrDefault(model));
    }
    if (ImGui::MenuItem("Save as...")) {
        OpenDialog(state, MenuState::Dialog::SaveAs, CurrentFileOrDefault(model));
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Close editor", "F1")) {
        requests.closeEditor = true;
    }
    ImGui::EndMenu();
}

void DrawEditMenu(graph_editor::EditorModel& model) {
    if (!ImGui::BeginMenu("Edit")) {
        return;
    }
    if (ImGui::MenuItem("Undo", "Ctrl+Z", false, model.CanUndo())) {
        model.Undo();
    }
    if (ImGui::MenuItem("Redo", "Ctrl+Y", false, model.CanRedo())) {
        model.Redo();
    }
    ImGui::Separator();
    const bool hasSelection = !model.Selection().empty();
    if (ImGui::MenuItem("Duplicate", "Ctrl+D", false, hasSelection)) {
        model.Duplicate();
    }
    if (ImGui::MenuItem("Delete", "Delete", false, hasSelection || model.SelectedConnection().has_value())) {
        model.DeleteSelection();
    }
    if (ImGui::MenuItem("Select all", "Ctrl+A")) {
        model.SelectAll();
    }
    ImGui::EndMenu();
}

void DrawGraphMenu(graph_editor::EditorModel& model) {
    if (!ImGui::BeginMenu("Graph")) {
        return;
    }
    if (ImGui::MenuItem("Play", nullptr, false, !model.IsPlaying())) {
        model.Play();
    }
    if (ImGui::MenuItem("Stop", nullptr, false, model.IsPlaying())) {
        model.Stop();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Arrange nodes")) {
        model.AutoLayoutAll();
    }
    ImGui::EndMenu();
}

void DrawViewMenu(WindowVisibility& windows) {
    if (!ImGui::BeginMenu("View")) {
        return;
    }
    ImGui::MenuItem("Graph editor", nullptr, &windows.graphEditor);
    ImGui::MenuItem("Character generator", nullptr, &windows.characterGenerator);
    ImGui::MenuItem("Material generator", nullptr, &windows.materialGenerator);
    ImGui::EndMenu();
}

void DrawHelpMenu(WindowVisibility& windows) {
    if (!ImGui::BeginMenu("Help")) {
        return;
    }
    ImGui::MenuItem("Shortcuts", nullptr, &windows.shortcuts);
    ImGui::EndMenu();
}

// The dialog that asks for a file name, for Open and Save as.
void DrawDialog(graph_editor::EditorModel& model, MenuState& state, MenuRequests& requests) {
    if (state.dialog == MenuState::Dialog::None) {
        return;
    }
    const char* title = kDialogTitles[static_cast<int>(state.dialog)];
    if (state.dialogJustRequested) {
        ImGui::OpenPopup(title);
        state.dialogJustRequested = false;
    }
    bool open = true;
    if (!ImGui::BeginPopupModal(title, &open, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings)) {
        state.dialog = open ? state.dialog : MenuState::Dialog::None;
        return;
    }
    ImGui::SetNextItemWidth(360.0f);
    const bool entered = ImGui::InputText("##path", state.path, sizeof(state.path), ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SetItemTooltip("A file name, from the folder the engine runs in.");
    const bool isOpen = state.dialog == MenuState::Dialog::Open;
    if (ImGui::Button(isOpen ? "Open" : "Save") || entered) {
        state.message = isOpen ? model.LoadFromFile(state.path) : model.SaveToFile(state.path);
        requests.showWholeGraph = isOpen && state.message.empty();
        if (state.message.empty()) {
            state.dialog = MenuState::Dialog::None;
            ImGui::CloseCurrentPopup();
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
        state.dialog = MenuState::Dialog::None;
        ImGui::CloseCurrentPopup();
    }
    if (!state.message.empty()) {
        ImGui::TextWrapped("%s", state.message.c_str());
    }
    ImGui::EndPopup();
}

}  // namespace

MenuRequests DrawMenuBar(graph_editor::EditorModel& model, WindowVisibility& windows, MenuState& state) {
    MenuRequests requests;
    if (ImGui::BeginMainMenuBar()) {
        DrawFileMenu(model, state, requests);
        DrawEditMenu(model);
        DrawGraphMenu(model);
        DrawViewMenu(windows);
        DrawHelpMenu(windows);
        ImGui::EndMainMenuBar();
    }
    DrawDialog(model, state, requests);
    return requests;
}

void DrawShortcutsWindow(bool& open) {
    if (!open) {
        return;
    }
    struct Shortcut {
        const char* keys;
        const char* action;
    };
    static const Shortcut kGraphShortcuts[] = {
        {"Right-click, or double-click", "Add a node (type to search)"},
        {"Drag from a pin", "Make a wire; let go on empty canvas to add a node that fits"},
        {"Delete, Backspace", "Delete the selected nodes or wire"},
        {"Ctrl+Z, Ctrl+Y", "Undo, redo"},
        {"Ctrl+C, Ctrl+X, Ctrl+V", "Copy, cut, paste"},
        {"Ctrl+D", "Duplicate"},
        {"Ctrl+A", "Select all"},
        {"Ctrl+S", "Save"},
        {"F", "Zoom to the selection"},
        {"Escape", "Give up what is being dragged"},
        {"Mouse wheel", "Zoom"},
        {"Middle or right drag, or Space and drag", "Move the view"},
    };
    static const Shortcut kSceneShortcuts[] = {
        {"F1", "Show or hide the editor"},
        {"Right mouse button and move", "Look around (when the editor is not under the pointer)"},
        {"W A S D, E Q", "Fly the camera, up and down"},
        {"Shift", "Fly faster"},
        {"Z, X", "Lower, raise the sun"},
        {"Escape", "Quit (while the editor is closed)"},
    };
    ImGui::SetNextWindowSize(ImVec2(560.0f, 0.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Shortcuts", &open, ImGuiWindowFlags_NoSavedSettings)) {
        const auto draw = [](const char* heading, const Shortcut* list, size_t count) {
            ImGui::SeparatorText(heading);
            if (ImGui::BeginTable(heading, 2, ImGuiTableFlags_SizingStretchProp)) {
                for (size_t index = 0; index < count; ++index) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(list[index].keys);
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(list[index].action);
                }
                ImGui::EndTable();
            }
        };
        draw("In the graph", kGraphShortcuts, sizeof(kGraphShortcuts) / sizeof(kGraphShortcuts[0]));
        draw("In the scene", kSceneShortcuts, sizeof(kSceneShortcuts) / sizeof(kSceneShortcuts[0]));
    }
    ImGui::End();
}

}  // namespace editor_ui
