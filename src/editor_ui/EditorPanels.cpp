#include "editor_ui/EditorPanels.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "editor_ui/EditorTheme.h"
#include "editor_ui/FilePathSync.h"
#include "editor_ui/ValueWidgets.h"

namespace editor_ui {
namespace {

constexpr graph::PinType kVariableTypes[] = {graph::PinType::Bool, graph::PinType::Int, graph::PinType::Float, graph::PinType::Vec3, graph::PinType::String};
constexpr int kVariableTypeCount = static_cast<int>(sizeof(kVariableTypes) / sizeof(kVariableTypes[0]));
constexpr ImVec2 kPaletteSize(360.0f, 380.0f);

ImVec4 ToVec4(ImU32 color) {
    return ImGui::ColorConvertU32ToFloat4(color);
}

int VariableTypeIndex(graph::PinType type) {
    for (int index = 0; index < kVariableTypeCount; ++index) {
        if (kVariableTypes[index] == type) {
            return index;
        }
    }
    return 2;
}

// A combo box of the types a variable can have. Returns the new type when one was chosen.
std::optional<graph::PinType> TypeCombo(const char* id, graph::PinType current, float width) {
    std::optional<graph::PinType> chosen;
    ImGui::SetNextItemWidth(width);
    if (ImGui::BeginCombo(id, graph::PinTypeName(current))) {
        for (graph::PinType type : kVariableTypes) {
            if (ImGui::Selectable(graph::PinTypeName(type), type == current)) {
                chosen = type;
            }
        }
        ImGui::EndCombo();
    }
    return chosen;
}

void ColoredText(ImU32 color, const char* text) {
    ImGui::PushStyleColor(ImGuiCol_Text, ToVec4(color));
    ImGui::TextUnformatted(text);
    ImGui::PopStyleColor();
}

void SectionTitle(const char* text) {
    ImGui::SeparatorText(text);
}

// The text of a node for a list: its title and number.
std::string NodeLabel(const graph_editor::EditorModel& model, uint32_t nodeId) {
    const graph::Node* node = model.GetGraph().FindNode(nodeId);
    if (node == nullptr) {
        return "the graph";
    }
    return graph_editor::NodeTitle(model.Registry(), *node) + " #" + std::to_string(nodeId);
}

// Adds the get or set node of a variable beside the nodes that are there, selected, ready to be moved to where it is wanted.
void AddVariableNode(graph_editor::EditorModel& model, const std::string& kindId, const std::string& variableName) {
    const graph_editor::Rect bounds = model.GraphBounds().value_or(graph_editor::Rect{});
    for (const graph_editor::PaletteEntry& entry : model.SearchPalette("variable")) {
        if (entry.typeId == kindId && entry.variable == variableName) {
            model.AddNodeFromPalette(entry, {bounds.max.x + 40.0f, bounds.min.y});
            return;
        }
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Toolbar
// ---------------------------------------------------------------------------

void DrawToolbar(graph_editor::EditorModel& model, PanelState& state, GraphCanvas& canvas) {
    if (FileFieldMustFollowModel(model.FilePath(), state.lastModelPath)) {
        std::snprintf(state.filePath, sizeof(state.filePath), "%s", model.FilePath().c_str());
    }
    const ImVec2 fullWidth(-1.0f, 0.0f);

    SectionTitle("Run");
    if (!model.IsPlaying()) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.14f, 0.45f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.2f, 0.58f, 0.28f, 1.0f));
        if (ImGui::Button("Play", fullWidth)) {
            model.Play();
        }
        ImGui::PopStyleColor(2);
        ImGui::SetItemTooltip("Runs the graph: the Start events once, then the Every frame events each frame.\nThe nodes that run light up.");
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.17f, 0.17f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.24f, 0.24f, 1.0f));
        if (ImGui::Button("Stop", fullWidth)) {
            model.Stop();
        }
        ImGui::PopStyleColor(2);
        ImGui::SetItemTooltip("Stops the run. The log stays to be read.");
    }

    SectionTitle("Edit");
    ImGui::BeginDisabled(!model.CanUndo());
    if (ImGui::Button("Undo", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 3.0f, 0.0f))) {
        model.Undo();
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Takes back the last change (Ctrl+Z).");
    ImGui::SameLine();
    ImGui::BeginDisabled(!model.CanRedo());
    if (ImGui::Button("Redo", fullWidth)) {
        model.Redo();
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Does again what Undo took back (Ctrl+Y).");

    ImGui::BeginDisabled(model.Selection().empty() && !model.SelectedConnection());
    if (ImGui::Button("Delete", fullWidth)) {
        model.DeleteSelection();
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Deletes the selected nodes, or the selected wire (Delete).");
    ImGui::BeginDisabled(model.Selection().empty());
    if (ImGui::Button("Duplicate", fullWidth)) {
        model.Duplicate();
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Copies the selected nodes and the wires between them (Ctrl+D).");

    SectionTitle("File");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputText("##file path", state.filePath, sizeof(state.filePath));
    ImGui::SetItemTooltip("The file that Save and Load use.");
    if (ImGui::Button("Save", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 3.0f, 0.0f))) {
        model.SaveToFile(state.filePath);
    }
    ImGui::SetItemTooltip("Writes the graph to the file (Ctrl+S).");
    ImGui::SameLine();
    if (ImGui::Button("Load", fullWidth)) {
        if (model.LoadFromFile(state.filePath).empty()) {
            canvas.RequestFrame(false);
        }
    }
    ImGui::SetItemTooltip("Replaces the graph with the one in the file.");
    if (ImGui::Button("New", fullWidth)) {
        model.NewGraph();
    }
    ImGui::SetItemTooltip("Starts an empty graph.");
    if (model.IsModified()) {
        ImGui::TextDisabled("Changed since saved");
    }

    SectionTitle("View");
    if (ImGui::Button("Show all", fullWidth)) {
        canvas.RequestFrame(false);
    }
    ImGui::SetItemTooltip("Zooms to show every node (F shows the selection).");
    if (ImGui::Button("Arrange", fullWidth)) {
        model.AutoLayoutAll();
        canvas.RequestFrame(false);
    }
    ImGui::SetItemTooltip("Puts the nodes in columns in the order that things run.");
    if (ImGui::Button(state.fullHeight ? "Smaller" : "Larger", fullWidth)) {
        state.fullHeight = !state.fullHeight;
    }
    ImGui::SetItemTooltip("Lets the editor cover the whole window, or only the lower part.");
    if (ImGui::Button("Close (F1)", fullWidth)) {
        state.closeRequested = true;
    }
    ImGui::SetItemTooltip("Hides the editor. F1 brings it back; the graph stays as it is.");
}

// ---------------------------------------------------------------------------
// Details of the selection
// ---------------------------------------------------------------------------

namespace {

// One setting of a node (see PropertyDescription): a list when only some texts mean something, otherwise the control of its type.
void DrawProperty(graph_editor::EditorModel& model, PanelState& state, const graph::Node& node, const graph::NodeDescription& description,
                  const graph::PropertyDescription& property) {
    ImGui::PushID(property.name.c_str());
    ColoredText(ColorOfPinType(property.type), property.name.c_str());
    ImGui::SetItemTooltip("%s", property.tooltip.c_str());
    graph::Value value = graph::PropertyValue(node, description, property.name);
    if (!property.choices.empty()) {
        const std::string& current = std::get<std::string>(value);
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginCombo("##choice", current.c_str())) {
            for (const std::string& choice : property.choices) {
                if (ImGui::Selectable(choice.c_str(), choice == current)) {
                    model.SetProperty(node.id, property.name, choice);
                }
            }
            ImGui::EndCombo();
        }
    } else if (EditValue("setting", value, property.type, -1.0f)) {
        model.SetProperty(node.id, property.name, value);
        state.editingValue = true;
    }
    ImGui::PopID();
}

void DrawNodeDetails(graph_editor::EditorModel& model, PanelState& state, const graph::Node& node) {
    const graph::NodeDescription* description = model.Registry().Find(node.type);
    ImGui::PushTextWrapPos(0.0f);
    ColoredText(ColorOfNodeKind(description), graph_editor::NodeTitle(model.Registry(), node).c_str());
    ImGui::TextDisabled("%s, number %u", NameOfNodeKind(description), node.id);
    if (description != nullptr) {
        ImGui::TextUnformatted(description->tooltip.c_str());
    } else {
        ImGui::TextUnformatted("The program has no node of this kind, so the node is kept as it is and cannot run.");
    }
    ImGui::PopTextWrapPos();

    if (description != nullptr && description->usesVariable) {
        const auto named = node.properties.find("variable");
        const std::string current = named != node.properties.end() && std::holds_alternative<std::string>(named->second) ? std::get<std::string>(named->second) : "";
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginCombo("##variable", current.empty() ? "Choose a variable" : current.c_str())) {
            for (const graph::Variable& variable : model.GetGraph().Variables()) {
                if (ImGui::Selectable(variable.name.c_str(), variable.name == current)) {
                    model.SetNodeVariable(node.id, variable.name);
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SetItemTooltip("The variable this node works on.");
    }

    if (description != nullptr && !description->properties.empty()) {
        SectionTitle("Settings");
        for (const graph::PropertyDescription& property : description->properties) {
            DrawProperty(model, state, node, *description, property);
        }
    }

    const graph::NodePins pins = graph::PinsOf(model.GetGraph(), model.Registry(), node);
    bool shownHeading = false;
    for (size_t index = 0; index < pins.inputs.size(); ++index) {
        const graph::PinDescription& pin = pins.inputs[index];
        if (pin.type == graph::PinType::Exec) {
            continue;
        }
        if (!shownHeading) {
            SectionTitle("Inputs");
            shownHeading = true;
        }
        ImGui::PushID(static_cast<int>(index));
        ColoredText(ColorOfPinType(pin.type), pin.name.c_str());
        if (!pin.tooltip.empty()) {
            ImGui::SetItemTooltip("%s", pin.tooltip.c_str());
        }
        if (model.GetGraph().ConnectionInto({node.id, static_cast<int>(index)}) != nullptr) {
            ImGui::SameLine();
            ImGui::TextDisabled("wired in");
        } else {
            const auto typed = node.inputValues.find(static_cast<int>(index));
            graph::Value value = typed != node.inputValues.end() ? typed->second : pin.defaultValue;
            if (EditValue("value", value, pin.type, -1.0f)) {
                model.SetInputValue(node.id, static_cast<int>(index), value);
                state.editingValue = true;
            }
        }
        ImGui::PopID();
    }

    const std::vector<const graph::Issue*> issues = model.IssuesFor(node.id);
    if (!issues.empty()) {
        SectionTitle("Problems");
        ImGui::PushTextWrapPos(0.0f);
        for (const graph::Issue* issue : issues) {
            ColoredText(issue->isError ? kErrorColor : kWarningColor, issue->message.c_str());
        }
        ImGui::PopTextWrapPos();
    }
}

void DrawHelp() {
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled("Nothing is selected.");
    ImGui::Spacing();
    ImGui::TextUnformatted("Right-click empty canvas, or double-click it, to add a node. Type to search by name, category or pin type.");
    ImGui::Spacing();
    ImGui::TextUnformatted("Drag from a pin to another pin to make a wire. Let go on empty canvas to add a node that fits.");
    ImGui::Spacing();
    ImGui::TextUnformatted("Click a node to select it, drag to move it, or drag on empty canvas to select several. Hold Shift to add.");
    ImGui::Spacing();
    ImGui::TextUnformatted("The wheel zooms. Drag with the middle or right button, or with Space held, to move the view.");
    ImGui::PopTextWrapPos();
}

}  // namespace

void DrawDetails(graph_editor::EditorModel& model, PanelState& state, GraphCanvas& canvas) {
    (void)canvas;
    SectionTitle("Details");
    const std::set<uint32_t>& selection = model.Selection();
    if (selection.size() == 1) {
        if (const graph::Node* node = model.GetGraph().FindNode(*selection.begin())) {
            DrawNodeDetails(model, state, *node);
        }
    } else if (selection.size() > 1) {
        ImGui::Text("%zu nodes selected", selection.size());
        ImGui::TextDisabled("Drag one to move them all.");
    } else if (const auto& wire = model.SelectedConnection()) {
        ImGui::TextUnformatted("A wire");
        ImGui::TextDisabled("From %s", NodeLabel(model, wire->from.node).c_str());
        ImGui::TextDisabled("To %s", NodeLabel(model, wire->to.node).c_str());
        if (ImGui::Button("Delete wire")) {
            model.DeleteSelection();
        }
    } else {
        DrawHelp();
    }
}

// ---------------------------------------------------------------------------
// Variables
// ---------------------------------------------------------------------------

void DrawVariables(graph_editor::EditorModel& model, PanelState& state) {
    SectionTitle("Variables");
    std::vector<graph::Variable> variables = model.GetGraph().Variables();  // a copy, as the model changes while this is drawn
    for (size_t index = 0; index < variables.size(); ++index) {
        const graph::Variable& variable = variables[index];
        ImGui::PushID(static_cast<int>(index));

        char name[sizeof(state.renameBuffer)];
        std::snprintf(name, sizeof(name), "%s", state.renamingVariable == static_cast<int>(index) ? state.renameBuffer : variable.name.c_str());
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
        ImGui::InputText("##name", name, sizeof(name));
        if (ImGui::IsItemActivated()) {
            state.renamingVariable = static_cast<int>(index);
        }
        if (ImGui::IsItemActive()) {
            std::snprintf(state.renameBuffer, sizeof(state.renameBuffer), "%s", name);
        }
        if (ImGui::IsItemDeactivated()) {
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                state.variableMessage = model.RenameVariable(variable.name, name);
            }
            state.renamingVariable = -1;
        }
        ImGui::SetItemTooltip("The name of the variable. Nodes that use it follow a new name.");
        ImGui::SameLine();
        if (const std::optional<graph::PinType> type = TypeCombo("##type", variable.type, -1.0f)) {
            state.variableMessage = model.SetVariableType(variable.name, *type);
        }

        graph::Value initial = variable.initial;
        if (EditValue("initial", initial, variable.type, ImGui::GetContentRegionAvail().x - 120.0f)) {
            model.SetVariableInitial(variable.name, initial);
            state.editingValue = true;
        }
        ImGui::SetItemTooltip("What the variable holds when a run starts.");
        ImGui::SameLine();
        if (ImGui::SmallButton("Get")) {
            AddVariableNode(model, "variable.get", variable.name);
        }
        ImGui::SetItemTooltip("Adds a node that gives the value of the variable.");
        ImGui::SameLine();
        if (ImGui::SmallButton("Set")) {
            AddVariableNode(model, "variable.set", variable.name);
        }
        ImGui::SetItemTooltip("Adds a node that gives the variable a new value.");
        ImGui::SameLine();
        if (ImGui::SmallButton("X")) {
            state.variableMessage = model.RemoveVariable(variable.name);
        }
        ImGui::SetItemTooltip("Removes the variable. Nodes that used it are flagged.");
        ImGui::PopID();
    }

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
    ImGui::InputTextWithHint("##new variable", "New variable name", state.newVariableName, sizeof(state.newVariableName));
    ImGui::SameLine();
    if (const std::optional<graph::PinType> type = TypeCombo("##new type", kVariableTypes[state.newVariableType], 70.0f)) {
        state.newVariableType = VariableTypeIndex(*type);
    }
    ImGui::SameLine();
    if (ImGui::Button("Add")) {
        std::string name = state.newVariableName;
        if (name.empty()) {
            name = "Variable" + std::to_string(model.GetGraph().Variables().size() + 1);
        }
        state.variableMessage = model.AddVariable(name, kVariableTypes[state.newVariableType]);
        if (state.variableMessage.empty()) {
            state.newVariableName[0] = '\0';
        }
    }
    ImGui::SetItemTooltip("Adds a variable of the chosen type.");
    if (!state.variableMessage.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ColoredText(kErrorColor, state.variableMessage.c_str());
        ImGui::PopTextWrapPos();
    }
}

// ---------------------------------------------------------------------------
// Log and problems
// ---------------------------------------------------------------------------

void DrawLogAndProblems(graph_editor::EditorModel& model, GraphCanvas& canvas) {
    const graph::ValidationReport& issues = model.Issues();
    const std::string problemsTitle = "Problems (" + std::to_string(issues.size()) + ")###problems";
    if (!ImGui::BeginTabBar("log tabs")) {
        return;
    }
    if (ImGui::BeginTabItem("Log")) {
        ImGui::BeginChild("log lines", ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
        const std::vector<std::string>& lines = model.LogLines();
        for (const std::string& line : lines) {
            ImGui::TextUnformatted(line.c_str());
        }
        if (!model.RuntimeError().empty()) {
            ColoredText(kErrorColor, ("Stopped: " + model.RuntimeError()).c_str());
        }
        if (lines.empty() && model.RuntimeError().empty()) {
            ImGui::TextDisabled("What Print nodes write, and what stops a run, appears here.");
        }
        if (model.IsPlaying() && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f) {
            ImGui::SetScrollHereY(1.0f);  // follows the output while the end of it is in view
        }
        ImGui::EndChild();
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem(problemsTitle.c_str())) {
        ImGui::BeginChild("problem lines", ImVec2(0.0f, 0.0f), ImGuiChildFlags_None);
        for (const graph::Issue& issue : issues) {
            const std::string line = (issue.isError ? "Error: " : "Warning: ") + (issue.node != 0 ? NodeLabel(model, issue.node) + ": " : std::string()) + issue.message;
            ImGui::PushStyleColor(ImGuiCol_Text, ToVec4(issue.isError ? kErrorColor : kWarningColor));
            if (ImGui::Selectable(line.c_str(), false) && issue.node != 0) {
                model.SelectNode(issue.node);
                canvas.RequestFrame(true);
            }
            ImGui::PopStyleColor();
        }
        if (issues.empty()) {
            ImGui::TextDisabled("No problems.");
        }
        ImGui::EndChild();
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
}

// ---------------------------------------------------------------------------
// Palette
// ---------------------------------------------------------------------------

void OpenPalette(PanelState& state, glm::vec2 canvasPosition) {
    state.paletteOpen = true;
    state.paletteCanvasPosition = canvasPosition;
    state.paletteScreenPosition = ImGui::GetIO().MousePos;
    state.paletteQuery[0] = '\0';
    state.paletteHighlight = 0;
    state.paletteJustOpened = true;
    ImGui::OpenPopup("add node palette");
}

void DrawPalette(graph_editor::EditorModel& model, PanelState& state) {
    if (!state.paletteOpen) {
        return;
    }
    ImGui::SetNextWindowPos(state.paletteScreenPosition, ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(kPaletteSize, ImGuiCond_Appearing);
    if (!ImGui::BeginPopup("add node palette")) {
        // Closed by a click elsewhere or by Escape: a wire that was waiting for a node is given up.
        state.paletteOpen = false;
        model.CancelPendingWire();
        return;
    }

    if (const auto& pending = model.GetPendingWire()) {
        ImGui::TextDisabled("Nodes that a wire from %s can join", NodeLabel(model, pending->start.pin.node).c_str());
    } else {
        ImGui::TextDisabled("Add a node");
    }
    if (state.paletteJustOpened) {
        ImGui::SetKeyboardFocusHere();
    }
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputTextWithHint("##palette query", "Type a name, category or pin type", state.paletteQuery, sizeof(state.paletteQuery));
    state.paletteJustOpened = false;

    const std::vector<graph_editor::PaletteEntry> entries = model.SearchPalette(state.paletteQuery);
    const int count = static_cast<int>(entries.size());
    const bool moved = ImGui::IsKeyPressed(ImGuiKey_DownArrow) || ImGui::IsKeyPressed(ImGuiKey_UpArrow);
    if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
        ++state.paletteHighlight;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
        --state.paletteHighlight;
    }
    state.paletteHighlight = count > 0 ? std::clamp(state.paletteHighlight, 0, count - 1) : 0;
    // The first entry is chosen while typing, since the list is ordered best match first.
    if (!moved && ImGui::IsItemEdited()) {
        state.paletteHighlight = 0;
    }

    int chosen = -1;
    ImGui::BeginChild("palette results", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);
    std::string lastCategory;
    for (int index = 0; index < count; ++index) {
        const graph_editor::PaletteEntry& entry = entries[static_cast<size_t>(index)];
        if (state.paletteQuery[0] == '\0' && entry.category != lastCategory) {
            ImGui::SeparatorText(entry.category.c_str());
            lastCategory = entry.category;
        }
        ImGui::PushID(index);
        if (ImGui::Selectable(entry.title.c_str(), index == state.paletteHighlight)) {
            chosen = index;
        }
        if (moved && index == state.paletteHighlight) {
            ImGui::SetScrollHereY(0.5f);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            ImGui::TextUnformatted(entry.title.c_str());
            ImGui::TextDisabled("%s", entry.category.c_str());
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 22.0f);
            ImGui::TextUnformatted(entry.tooltip.c_str());
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
        ImGui::PopID();
    }
    if (count == 0) {
        ImGui::TextDisabled("Nothing matches.");
    }
    ImGui::EndChild();

    if (chosen < 0 && count > 0 && (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter))) {
        chosen = state.paletteHighlight;
    }
    if (chosen >= 0) {
        model.AddNodeFromPalette(entries[static_cast<size_t>(chosen)], state.paletteCanvasPosition);
        state.paletteOpen = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

}  // namespace editor_ui
