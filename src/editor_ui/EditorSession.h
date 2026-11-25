#pragma once

#include <memory>
#include <string>

#include "core/CommandLine.h"
#include "editor_ui/IEditorUi.h"
#include "render/IRenderModule.h"

struct GLFWwindow;

// The graph editor for one run of the engine: it makes the editor, hands it to the renderer to draw over the frames, opens and
// closes it with F1, and takes it off the renderer again when the run ends, so that the renderer never holds one that is gone.
// A run that does not make a session never creates the editor, and so never touches ImGui.

namespace editor_ui {

class EditorSession {
public:
    EditorSession(IRenderModule& renderer, GLFWwindow* window, const Options& options);
    ~EditorSession();
    EditorSession(const EditorSession&) = delete;
    EditorSession& operator=(const EditorSession&) = delete;

    // Call once per frame before drawing: looks at F1, and builds the editor's interface for the frame when it is open.
    void BeginFrame(float deltaSeconds);

    // Why the graph given by --editor-graph could not be loaded, or empty. The editor then holds its starter graph.
    const std::string& StartupError() const { return startupError; }

    bool IsVisible() const { return editor->IsVisible(); }
    // Whether the editor is using the pointer or the keys, so that the scene's controls should leave them alone.
    bool CapturesMouse() const { return editor->CapturesMouse(); }
    bool CapturesKeyboard() const { return editor->CapturesKeyboard(); }

private:
    IRenderModule& renderer;
    GLFWwindow* window;
    std::unique_ptr<IEditorUi> editor;
    std::string startupError;
    bool f1WasDown = false;
};

}  // namespace editor_ui
