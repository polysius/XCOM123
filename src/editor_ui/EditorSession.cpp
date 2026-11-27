#include "editor_ui/EditorSession.h"

#include <chrono>
#include <iostream>

#include <GLFW/glfw3.h>

namespace editor_ui {

EditorSession::EditorSession(IRenderModule& renderer, GLFWwindow* window, const Options& options, engine_host::LiveEngineHost* host)
    : renderer(renderer), window(window), editor(CreateGraphEditorUi()), keyPoller(window) {
    renderer.SetOverlay(editor.get());
    // The host comes before the graph is loaded and played, so that the Start events of a graph that plays at once reach the engine.
    editor->SetHost(host);
    editor->SetScreenMessages(host != nullptr ? &host->ScreenMessages() : nullptr);
    if (!options.editorGraphPath.empty()) {
        startupError = editor->LoadGraphFile(options.editorGraphPath);
        if (!startupError.empty()) {
            // An interactive run goes on with the starter graph, and the status line of the editor says why.
            std::cerr << "Editor: " << startupError << "\n";
        }
    }
    for (const std::string& panel : options.editorPanels) {
        editor->OpenPanel(panel);
    }
    if (options.editorPlay) {
        editor->Play();
    }
    editor->SetVisible(options.editor);
    // A key that is already down when the editor starts must not count as a press.
    f1WasDown = glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS;
}

EditorSession::~EditorSession() {
    renderer.SetOverlay(nullptr);  // detaches the editor, with the GPU idle, before it is destroyed
}

// The renderer takes a library between frames: uploading waits for the GPU, so it is not done while a frame is being recorded.
void EditorSession::UploadFinishedMaterials() {
    std::optional<material::MaterialLibrary> library = editor->TakeMaterialLibraryToUpload();
    if (!library) {
        return;
    }
    const auto begin = std::chrono::steady_clock::now();
    renderer.UploadMaterials(*library);
    editor->NoteMaterialLibraryUploaded(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count());
}

void EditorSession::BeginFrame(float deltaSeconds) {
    // Toggle on the press, not while the key is held.
    const bool f1Down = glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS;
    if (f1Down && !f1WasDown) {
        editor->SetVisible(!editor->IsVisible());
    }
    f1WasDown = f1Down;
    editor->BeginFrame(deltaSeconds);
    UploadFinishedMaterials();

    // The keys are looked at every frame, but only handed to the graph while the editor is not using the keyboard.
    const std::vector<std::string_view> pressed = keyPoller.NewlyPressed();
    if (!editor->CapturesKeyboard()) {
        for (std::string_view key : pressed) {
            editor->SendKey(std::string(key));
        }
    }
}

}  // namespace editor_ui
