#include "editor_ui/EditorTheme.h"

#include <algorithm>

namespace editor_ui {

ImU32 ColorOfPinType(graph::PinType type) {
    switch (type) {
    case graph::PinType::Exec: return IM_COL32(245, 245, 245, 255);
    case graph::PinType::Bool: return IM_COL32(200, 40, 40, 255);
    case graph::PinType::Int: return IM_COL32(40, 205, 200, 255);
    case graph::PinType::Float: return IM_COL32(125, 220, 70, 255);
    case graph::PinType::Vec3: return IM_COL32(250, 200, 50, 255);
    case graph::PinType::String: return IM_COL32(235, 80, 215, 255);
    }
    return IM_COL32(180, 180, 180, 255);
}

ImU32 ColorOfNodeKind(const graph::NodeDescription* description) {
    if (description == nullptr) {
        return IM_COL32(95, 95, 100, 255);
    }
    switch (description->kind) {
    case graph::NodeKind::Event: return IM_COL32(170, 45, 50, 255);
    case graph::NodeKind::Action: return IM_COL32(45, 100, 175, 255);
    case graph::NodeKind::Pure: return IM_COL32(50, 135, 70, 255);
    }
    return IM_COL32(95, 95, 100, 255);
}

const char* NameOfNodeKind(const graph::NodeDescription* description) {
    if (description == nullptr) {
        return "Unknown kind of node";
    }
    switch (description->kind) {
    case graph::NodeKind::Event: return "Event: starts a run";
    case graph::NodeKind::Action: return "Action: runs in turn";
    case graph::NodeKind::Pure: return "Pure: only computes a value";
    }
    return "";
}

ImU32 WithOpacity(ImU32 color, float opacity) {
    const float alpha = static_cast<float>((color >> IM_COL32_A_SHIFT) & 0xFF) * std::clamp(opacity, 0.0f, 1.0f);
    return (color & ~IM_COL32_A_MASK) | (static_cast<ImU32>(alpha) << IM_COL32_A_SHIFT);
}

void ApplyEditorStyle() {
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.PopupRounding = 4.0f;
    style.GrabRounding = 3.0f;
    style.ChildRounding = 3.0f;
    style.WindowBorderSize = 1.0f;
    style.ItemSpacing = ImVec2(6.0f, 6.0f);
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.11f, 0.115f, 0.13f, 0.97f);
    style.Colors[ImGuiCol_ChildBg] = ImVec4(0.09f, 0.095f, 0.11f, 1.0f);
    style.Colors[ImGuiCol_PopupBg] = ImVec4(0.12f, 0.125f, 0.14f, 0.98f);
    style.Colors[ImGuiCol_FrameBg] = ImVec4(0.17f, 0.18f, 0.2f, 1.0f);
    style.Colors[ImGuiCol_Button] = ImVec4(0.2f, 0.22f, 0.27f, 1.0f);
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.32f, 0.4f, 1.0f);
    style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.2f, 0.4f, 0.65f, 1.0f);
}

}  // namespace editor_ui
