#pragma once

#include <optional>
#include <string>

#include <imgui.h>

#include "graph_engine/EngineHost.h"
#include "material/MaterialLibrary.h"
#include "panel_model/CharacterPanelModel.h"
#include "panel_model/MaterialPanelModel.h"

// The windows of the generators: one makes a person from sliders and puts them in front of the camera, the other previews a material
// and builds the scan materials of the renderer from it. What they hold and compute is in src/panel_model; this draws it.

namespace editor_ui {

class GeneratorPanels {
public:
    // The engine that a generated person is spawned in. Null (the start): the person is made and measured, and not spawned.
    void SetHost(graph_engine::EngineHost* engineHost) { host = engineHost; }

    // Collects the work that has finished since the last frame, and spawns what is ready. Call once a frame, also while the windows are hidden,
    // so that work that was started is not left waiting.
    void Update();

    // Each draws its window while `open` is true, and sets it to false when the window is closed. Only inside an ImGui frame.
    void DrawCharacterWindow(bool& open);
    void DrawMaterialWindow(bool& open);

    // Whether one of the windows has the keyboard focus, as of the last draw.
    bool IsFocused() const { return focused; }

    // A library that Apply built and that the renderer has not been given yet, once.
    std::optional<material::MaterialLibrary> TakeLibraryToUpload() { return materials.TakeLibrary(); }
    // Tells the panel the renderer took the library, for its status line.
    void NoteLibraryUploaded(double milliseconds) { uploadedMilliseconds = milliseconds; }

private:
    void DrawCharacterControls();
    void DrawCharacterResult();
    void DrawMaterialControls();
    void DrawMaterialPreview();
    // The point four metres in front of the camera, where a person is put.
    glm::vec3 PointInFrontOfCamera() const;

    graph_engine::EngineHost* host = nullptr;
    panel_model::CharacterGeneratorModel characters;
    panel_model::MaterialGeneratorModel materials;
    bool focused = false;
    bool materialWindowWasOpen = false;  // the material preview is only made while its window is open

    uint32_t lastSpawnedId = 0;       // 0 when the last person made was not spawned
    std::string spawnMessage;
    std::optional<double> uploadedMilliseconds;
};

}  // namespace editor_ui
