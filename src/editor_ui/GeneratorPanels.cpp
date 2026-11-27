#include "editor_ui/GeneratorPanels.h"

#include <algorithm>
#include <cfloat>
#include <cstdio>

#include "editor_ui/EditorTheme.h"

namespace editor_ui {
namespace {

constexpr float kPanelWidth = 360.0f;
constexpr float kPanelMargin = 16.0f;
constexpr float kSwatchPixels = 192.0f;
constexpr float kSpawnDistance = 4.0f;  // metres in front of the camera
constexpr size_t kOutfitBufferSize = 64;

// A window that starts at the right of the screen, `column` windows from the edge, and stays wherever the person moves it.
bool BeginPanelWindow(const char* title, bool& open, int column) {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float right = viewport->WorkPos.x + viewport->WorkSize.x - kPanelMargin - static_cast<float>(column) * (kPanelWidth + kPanelMargin);
    ImGui::SetNextWindowPos(ImVec2(right - kPanelWidth, viewport->WorkPos.y + kPanelMargin), ImGuiCond_FirstUseEver);
    // The window is as high as its content, which changes when a result or a preview appears, and always as wide as kPanelWidth.
    ImGui::SetNextWindowSizeConstraints(ImVec2(kPanelWidth, 0.0f), ImVec2(kPanelWidth, FLT_MAX));
    // NoSavedSettings: the engine keeps no layout file next to the program.
    return ImGui::Begin(title, &open, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
}

void ColouredText(ImU32 colour, const std::string& text) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(colour));
    ImGui::TextWrapped("%s", text.c_str());
    ImGui::PopStyleColor();
}

// A bar that moves back and forth for work whose end cannot be told: the generators report no progress while they run.
void BusyBar(const char* label, double secondsRunning) {
    char text[96];
    std::snprintf(text, sizeof(text), "%s %.1f s", label, secondsRunning);
    ImGui::ProgressBar(-1.0f * static_cast<float>(ImGui::GetTime()), ImVec2(-1.0f, 0.0f), text);
}

}  // namespace

void GeneratorPanels::Update() {
    focused = false;
    if (std::optional<panel_model::CharacterGeneratorModel::Finished> finished = characters.Poll()) {
        if (host == nullptr) {
            spawnMessage = "Made, not spawned: there is no engine.";
        } else {
            lastSpawnedId = host->SpawnCharacter(finished->spec, PointInFrontOfCamera());
            spawnMessage = lastSpawnedId != 0 ? "Spawned as person " + std::to_string(lastSpawnedId) + ", " + std::to_string(static_cast<int>(kSpawnDistance)) +
                                                    " metres in front of the camera."
                                              : "The engine did not take the person (too many at once).";
        }
    }
    materials.Update(materialWindowWasOpen);
}

glm::vec3 GeneratorPanels::PointInFrontOfCamera() const {
    return host->CameraPosition() + host->CameraDirection() * kSpawnDistance;
}

// ----- The character window -----

void GeneratorPanels::DrawCharacterWindow(bool& open) {
    if (!open) {
        return;
    }
    if (BeginPanelWindow("Character generator", open, 0)) {
        focused = focused || ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
        DrawCharacterControls();
        DrawCharacterResult();
    }
    ImGui::End();
}

void GeneratorPanels::DrawCharacterControls() {
    panel_model::CharacterPanelParameters& parameters = characters.parameters;
    ImGui::SliderFloat("Stature", &parameters.statureMeters, graph_engine::kStatureMin, graph_engine::kStatureMax, "%.2f m");
    ImGui::SliderFloat("Body mass index", &parameters.bodyMassIndex, panel_model::kBodyMassIndexMin, panel_model::kBodyMassIndexMax, "%.1f");
    ImGui::SetItemTooltip("Kilograms per square metre of stature: 18.5 to 25 is the usual range of an adult.");
    ImGui::TextDisabled("Mass: %.1f kg", panel_model::MassKg(parameters));
    ImGui::SliderFloat("Age", &parameters.ageYears, graph_engine::kAgeMin, graph_engine::kAgeMax, "%.0f years");
    ImGui::SliderFloat("Sex", &parameters.masculinity, 0.0f, 1.0f, "%.2f");
    ImGui::SetItemTooltip("0 is a female body, 1 a male one, and between is a blend.");
    ImGui::SliderFloat("Muscularity", &parameters.muscularity, 0.0f, 1.0f, "%.2f");
    ImGui::SetItemTooltip("0 is slight, 1 is a trained athlete.");

    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt("Seed", &parameters.seed, 1, 100);
    ImGui::SetItemTooltip("The same numbers and seed always make the same person.");
    ImGui::SameLine();
    if (ImGui::Button("Random person")) {
        const std::string outfit = parameters.outfit;
        parameters = panel_model::RandomPanelParameters((parameters.seed + 1) % (panel_model::kSeedMax + 1));
        parameters.outfit = outfit;
    }
    ImGui::SetItemTooltip("Draws a person from the usual spread of adults, with the next seed.");

    char outfit[kOutfitBufferSize];
    std::snprintf(outfit, sizeof(outfit), "%s", parameters.outfit.c_str());
    if (ImGui::InputText("Outfit", outfit, sizeof(outfit))) {
        parameters.outfit = outfit;
    }
    ImGui::SetItemTooltip("The name of the clothes. The generator makes no clothes yet: the name only picks the colour of the stand-in.");

    for (const std::string& problem : panel_model::Problems(parameters)) {
        ColouredText(kWarningColor, problem);
    }
    parameters = panel_model::Clamped(parameters);  // so that the sliders show what will be made

    ImGui::BeginDisabled(characters.IsRunning());
    if (ImGui::Button("Generate and spawn", ImVec2(-1.0f, 0.0f))) {
        characters.Generate();
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Makes the person in the background and puts them in front of the camera when they are ready.");
}

void GeneratorPanels::DrawCharacterResult() {
    if (characters.IsRunning()) {
        BusyBar("Making the character...", characters.SecondsRunning());
    }
    if (!characters.LastError().empty()) {
        ColouredText(kErrorColor, characters.LastError());
    }
    if (const std::optional<panel_model::CharacterReport>& report = characters.LastReport()) {
        ImGui::SeparatorText("Last person");
        ImGui::Text("Vertices: %zu", report->vertexCount);
        ImGui::Text("Triangles: %zu in %zu pieces", report->triangleCount, report->pieceCount);
        ImGui::Text("Stature %.2f m, mass %.1f kg", static_cast<double>(report->statureMeters), static_cast<double>(report->massKg));
        ImGui::Text("Made in %.0f ms", report->milliseconds);
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("%s", spawnMessage.c_str());
        ImGui::TextDisabled("The person is shown as a stand-in until the skinned mesh can be drawn.");
        ImGui::PopTextWrapPos();
    }
}

// ----- The material window -----

void GeneratorPanels::DrawMaterialWindow(bool& open) {
    materialWindowWasOpen = open;
    if (!open) {
        return;
    }
    if (BeginPanelWindow("Material generator", open, 1)) {
        focused = focused || ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
        DrawMaterialControls();
        DrawMaterialPreview();
    }
    ImGui::End();
}

void GeneratorPanels::DrawMaterialControls() {
    panel_model::MaterialPanelParameters& parameters = materials.parameters;
    if (ImGui::BeginCombo("Kind", panel_model::DisplayName(parameters.kind).c_str())) {
        for (material::MaterialKind kind : panel_model::AllKinds()) {
            if (ImGui::Selectable(panel_model::DisplayName(kind).c_str(), kind == parameters.kind)) {
                parameters.kind = kind;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SetItemTooltip("The material shown in the preview.");
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt("Seed", &parameters.seed, 1, 100);
    ImGui::SliderFloat("Wear", &parameters.wear, 0.0f, 1.0f, "%.2f");
    ImGui::SetItemTooltip("0 is new, 1 is old: bright worn edges, dirt in the pits, stains and chips.");
    ImGui::SliderFloat("Scan detail", &parameters.scanDetail, 0.0f, 1.0f, "%.2f");
    ImGui::SetItemTooltip("0 is smooth, 1 is the finest grain and the most scanner noise.");
    parameters = panel_model::Clamped(parameters);

    ImGui::BeginDisabled(materials.IsApplying());
    if (ImGui::Button("Apply to scan materials", ImVec2(-1.0f, 0.0f))) {
        materials.Apply();
        uploadedMilliseconds.reset();
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Builds all ten materials of the city from this seed, wear and scan detail, and gives them to the renderer. "
                          "The kind only chooses what the preview shows.");
    if (materials.IsApplying()) {
        BusyBar("Building the materials...", materials.SecondsApplying());
    } else if (uploadedMilliseconds) {
        ImGui::TextDisabled("Applied: built in %.0f ms and uploaded in %.0f ms.", materials.LastLibraryMilliseconds(), *uploadedMilliseconds);
    }
    if (!materials.LastError().empty()) {
        ColouredText(kErrorColor, materials.LastError());
    }
}

void GeneratorPanels::DrawMaterialPreview() {
    ImGui::SeparatorText("Preview");
    const std::optional<panel_model::MaterialPreview>& preview = materials.Preview();
    if (!preview) {
        ImGui::TextDisabled("Making the preview...");
        return;
    }
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(kSwatchPixels, kSwatchPixels));
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const float cell = kSwatchPixels / static_cast<float>(panel_model::kPreviewCells);
    for (int y = 0; y < panel_model::kPreviewCells; ++y) {
        for (int x = 0; x < panel_model::kPreviewCells; ++x) {
            const glm::vec3 colour = preview->colours[static_cast<size_t>(y) * panel_model::kPreviewCells + static_cast<size_t>(x)];
            const ImVec2 low(origin.x + static_cast<float>(x) * cell, origin.y + static_cast<float>(y) * cell);
            draw->AddRectFilled(low, ImVec2(low.x + cell, low.y + cell), ImGui::ColorConvertFloat4ToU32(ImVec4(colour.r, colour.g, colour.b, 1.0f)));
        }
    }
    draw->AddRect(origin, ImVec2(origin.x + kSwatchPixels, origin.y + kSwatchPixels), IM_COL32(255, 255, 255, 60));
    if (hovered) {
        ImGui::SetTooltip("One tile of the material, %.2f m wide.", static_cast<double>(preview->tileMeters));
    }
    ImGui::SameLine();
    ImGui::BeginGroup();
    const glm::vec3 average = preview->averageColour;
    ImGui::ColorButton("##average", ImVec4(average.r, average.g, average.b, 1.0f), ImGuiColorEditFlags_NoTooltip, ImVec2(24.0f, 24.0f));
    ImGui::SameLine();
    ImGui::TextDisabled("average colour");
    ImGui::Text("Roughness %.2f", static_cast<double>(preview->averageRoughness));
    ImGui::Text("Metallic %.2f", static_cast<double>(preview->averageMetallic));
    ImGui::Text("Relief %.1f mm", static_cast<double>(preview->reliefMillimeters));
    ImGui::Text("Made in %.0f ms", preview->milliseconds);
    if (materials.IsPreviewRunning()) {
        ImGui::TextDisabled("updating...");
    }
    ImGui::EndGroup();
}

}  // namespace editor_ui
