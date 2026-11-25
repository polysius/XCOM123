#include "editor_ui/ValueWidgets.h"

#include <cstdio>

#include <imgui.h>

namespace editor_ui {
namespace {

constexpr size_t kTextBufferSize = 512;
// How much a drag of one pixel changes a number.
constexpr float kFloatDragSpeed = 0.02f;
constexpr float kIntDragSpeed = 0.2f;

bool EditVector(const char* id, glm::vec3& vector, float width) {
    const float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
    const float each = (width - 2.0f * spacing) / 3.0f;
    bool changed = false;
    ImGui::PushID(id);
    for (int axis = 0; axis < 3; ++axis) {
        if (axis > 0) {
            ImGui::SameLine(0.0f, spacing);
        }
        ImGui::PushID(axis);
        ImGui::SetNextItemWidth(each);
        changed |= ImGui::DragFloat("##axis", &vector[axis], kFloatDragSpeed, 0.0f, 0.0f, "%.3g");
        ImGui::PopID();
    }
    ImGui::PopID();
    return changed;
}

bool EditText(const char* id, std::string& text, float width) {
    char buffer[kTextBufferSize];
    std::snprintf(buffer, kTextBufferSize, "%s", text.c_str());
    ImGui::SetNextItemWidth(width);
    if (!ImGui::InputText(id, buffer, kTextBufferSize)) {
        return false;
    }
    text = buffer;
    return true;
}

}  // namespace

bool EditValue(const char* id, graph::Value& value, graph::PinType type, float width) {
    if (graph::TypeOf(value) != type) {
        value = graph::Convert(value, type);
    }
    // The control gets a label that is only an id, because its name is shown beside it by the caller.
    const std::string hiddenLabel = std::string("##") + id;
    switch (type) {
    case graph::PinType::Bool:
        return ImGui::Checkbox(hiddenLabel.c_str(), &std::get<bool>(value));
    case graph::PinType::Int:
        ImGui::SetNextItemWidth(width);
        return ImGui::DragInt(hiddenLabel.c_str(), &std::get<int>(value), kIntDragSpeed);
    case graph::PinType::Float:
        ImGui::SetNextItemWidth(width);
        return ImGui::DragFloat(hiddenLabel.c_str(), &std::get<float>(value), kFloatDragSpeed, 0.0f, 0.0f, "%.3g");
    case graph::PinType::Vec3:
        return EditVector(id, std::get<glm::vec3>(value), width);
    case graph::PinType::String:
        return EditText(hiddenLabel.c_str(), std::get<std::string>(value), width);
    case graph::PinType::Exec:
        break;
    }
    return false;
}

}  // namespace editor_ui
