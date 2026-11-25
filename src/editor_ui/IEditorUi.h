#pragma once

#include <memory>
#include <string>

#include "render/IFrameOverlay.h"

// The hook between the engine and an editor drawn over the scene. The engine only knows this interface: it hands the editor to
// the renderer as an overlay, calls BeginFrame once per frame, and asks whether the editor wants the mouse or the keyboard
// so that the camera does not move while a person types into it.

namespace editor_ui {

class IEditorUi : public IFrameOverlay {
public:
    virtual void SetVisible(bool visible) = 0;
    virtual bool IsVisible() const = 0;

    // Builds the editor's interface for this frame, if it is visible. Call it before the renderer draws the frame.
    virtual void BeginFrame(float deltaSeconds) = 0;

    // Whether the pointer is over the editor, or a person is typing into it, as of the last frame.
    virtual bool CapturesMouse() const = 0;
    virtual bool CapturesKeyboard() const = 0;

    // Loads a graph into the editor. Returns why it could not, or an empty string.
    virtual std::string LoadGraphFile(const std::string& path) = 0;
    // Runs the graph that is in the editor.
    virtual void Play() = 0;
};

// The graph editor: a visual node graph in an ImGui window. It creates no ImGui objects until it is first shown.
std::unique_ptr<IEditorUi> CreateGraphEditorUi();

}  // namespace editor_ui
