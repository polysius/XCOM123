#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "engine_host/LiveEngineHost.h"
#include "material/MaterialLibrary.h"
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

    // Opens a window of the editor by name ("character", "material" or "shortcuts"). A name the editor does not know is ignored.
    virtual void OpenPanel(const std::string& name) = 0;

    // The engine that the nodes of the graph act on, for runs started after this call; null for none. It must outlive the editor.
    virtual void SetHost(graph_engine::EngineHost* host) = 0;
    // A key went down while the editor was not using the keyboard: runs the "On Key Pressed" events of a graph that is playing.
    virtual void SendKey(const std::string& keyName) = 0;
    // The lines that graphs print on the screen. The editor draws them over the scene, also when it is closed, for as long as
    // the list holds any. The list must outlive the editor; null for none.
    virtual void SetScreenMessages(const std::vector<engine_host::ScreenMessage>* messages) = 0;

    // The material generator's Apply makes a library of materials in the background. When one is ready this gives it, once, for the
    // caller to hand to the renderer (which must not be in the middle of a frame), and the caller then says how long that took.
    virtual std::optional<material::MaterialLibrary> TakeMaterialLibraryToUpload() = 0;
    virtual void NoteMaterialLibraryUploaded(double milliseconds) = 0;
};

// The graph editor: a visual node graph in an ImGui window. It creates no ImGui objects until it is first shown.
std::unique_ptr<IEditorUi> CreateGraphEditorUi();

}  // namespace editor_ui
