#pragma once

// Who gets the keys and the mouse when the editor is open: the editor or the scene behind it. These are plain decisions on facts
// that ImGui reports, kept apart from ImGui so that they can be tested.

namespace editor_ui {

// What ImGui says as of the last frame.
struct EditorInputState {
    bool visible = false;              // the editor is open
    bool windowFocused = false;        // the editor window, or a part of it, has the keyboard focus
    bool wantCaptureKeyboard = false;  // ImGui's own flag
    bool wantTextInput = false;        // a text box is being typed into
    bool paletteOpen = false;
    bool wantCaptureMouse = false;     // the pointer is over the editor, or a control of it is being dragged
};

// Whether the editor keeps the keys from the scene (the camera, the sun). A focused editor keeps all of them: Ctrl+Z, Ctrl+X and
// the keys of a text box must not also move the sun or the camera. Clicking the scene takes the focus away.
inline bool EditorCapturesKeyboard(const EditorInputState& state) {
    return state.visible && (state.windowFocused || state.wantCaptureKeyboard || state.wantTextInput || state.paletteOpen);
}

inline bool EditorCapturesMouse(const EditorInputState& state) {
    return state.visible && state.wantCaptureMouse;
}

// Whether the keys of the canvas (Delete, Ctrl+Z, Ctrl+S and the others) act. The editor window must be focused, so that keys
// pressed while the scene has the focus are left to the scene, and the canvas must be the part in use: focused, or under the
// pointer. Never while a text box is being typed into.
inline bool CanvasShortcutsActive(bool editorWindowFocused, bool canvasFocused, bool canvasHovered, bool wantTextInput) {
    return !wantTextInput && editorWindowFocused && (canvasFocused || canvasHovered);
}

}  // namespace editor_ui
