#include "editor_ui/GraphEditorWindow.h"

#include <algorithm>
#include <cmath>

namespace editor_ui {
namespace {

constexpr float kLowerPartOfTheWindow = 0.62f;  // how much of the height the editor takes unless it is made larger
constexpr float kToolbarWidth = 150.0f;
constexpr float kSidePanelWidth = 290.0f;
constexpr float kLogHeight = 150.0f;

}  // namespace

bool GraphEditorWindow::TakeCloseRequest() {
    const bool requested = state.closeRequested;
    state.closeRequested = false;
    return requested;
}

void GraphEditorWindow::Draw(graph_editor::EditorModel& model) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float height = state.fullHeight ? viewport->WorkSize.y : std::floor(viewport->WorkSize.y * kLowerPartOfTheWindow);
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + viewport->WorkSize.y - height));
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, height));
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
    ImGui::Begin("Graph editor", nullptr, flags);
    focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);

    // A value dragged in a panel is one undo step, which ends when the control is let go of.
    if (state.editingValue && !ImGui::IsAnyItemActive()) {
        model.EndGesture();
        state.editingValue = false;
    }

    const ImVec2 available = ImGui::GetContentRegionAvail();
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const float centreWidth = std::max(available.x - kToolbarWidth - kSidePanelWidth - 2.0f * spacing, 100.0f);

    ImGui::BeginChild("toolbar", ImVec2(kToolbarWidth, available.y), ImGuiChildFlags_Borders);
    DrawToolbar(model, state, canvas);
    ImGui::EndChild();
    ImGui::SameLine();

    CanvasRequests requests;
    ImGui::BeginGroup();
    canvas.Draw(model, ImVec2(centreWidth, std::max(available.y - kLogHeight - spacing, 60.0f)), requests);
    ImGui::BeginChild("log panel", ImVec2(centreWidth, kLogHeight), ImGuiChildFlags_Borders);
    DrawLogAndProblems(model, canvas);
    ImGui::EndChild();
    ImGui::EndGroup();
    ImGui::SameLine();

    ImGui::BeginChild("side panel", ImVec2(kSidePanelWidth, available.y), ImGuiChildFlags_Borders);
    DrawDetails(model, state, canvas);
    DrawVariables(model, state);
    ImGui::EndChild();

    if (requests.saveRequested) {
        model.SaveToFile(state.filePath);
    }
    if (requests.openPalette) {
        OpenPalette(state, requests.paletteCanvasPosition);
    }
    DrawPalette(model, state);
    ImGui::End();
}

}  // namespace editor_ui
