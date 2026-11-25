#include "editor_ui/EditorSession.h"

#include <iostream>

#include <GLFW/glfw3.h>

namespace editor_ui {

EditorSession::EditorSession(IRenderModule& renderer, GLFWwindow* window, const Options& options)
    : renderer(renderer), window(window), editor(CreateGraphEditorUi()) {
    renderer.SetOverlay(editor.get());
    if (!options.editorGraphPath.empty()) {
        startupError = editor->LoadGraphFile(options.editorGraphPath);
        if (!startupError.empty()) {
            // An interactive run goes on with the starter graph, and the status line of the editor says why.
            std::cerr << "Editor: " << startupError << "\n";
        }
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

void EditorSession::BeginFrame(float deltaSeconds) {
    // Toggle on the press, not while the key is held.
    const bool f1Down = glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS;
    if (f1Down && !f1WasDown) {
        editor->SetVisible(!editor->IsVisible());
    }
    f1WasDown = f1Down;
    editor->BeginFrame(deltaSeconds);
}

}  // namespace editor_ui
