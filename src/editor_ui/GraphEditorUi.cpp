#include <iostream>
#include <memory>

#include <imgui.h>

#include "editor_ui/GraphEditorWindow.h"
#include "editor_ui/IEditorUi.h"
#include "editor_ui/InputPolicy.h"
#include "editor_ui/ImGuiHost.h"
#include "graph/GraphFile.h"
#include "graph_editor/EditorModel.h"

namespace editor_ui {
namespace {

// A small graph to open the editor on, so that there is something to look at, run and change: a message when the graph starts,
// and a variable that counts the seconds, which makes the nodes of an every-frame event light up.
std::string StarterGraphText(const graph::NodeRegistry& registry) {
    graph::Graph graph;
    graph.Variables().push_back({"Elapsed", graph::PinType::Float, 0.0f});

    const uint32_t start = graph.AddNode("event.start", {40.0f, 40.0f});
    const uint32_t print = graph.AddNode("debug.print", {300.0f, 40.0f});
    graph.FindNode(print)->inputValues[1] = std::string("Hello from the graph editor");

    const uint32_t tick = graph.AddNode("event.tick", {40.0f, 240.0f});
    const uint32_t current = graph.AddNode("variable.get", {40.0f, 380.0f});
    graph.FindNode(current)->properties["variable"] = std::string("Elapsed");
    const uint32_t sum = graph.AddNode("math.add", {320.0f, 340.0f});
    const uint32_t store = graph.AddNode("variable.set", {560.0f, 240.0f});
    graph.FindNode(store)->properties["variable"] = std::string("Elapsed");

    graph.Connect(registry, {start, 0}, {print, 0});
    graph.Connect(registry, {tick, 0}, {store, 0});
    graph.Connect(registry, {current, 0}, {sum, 0});
    graph.Connect(registry, {tick, 1}, {sum, 1});
    graph.Connect(registry, {sum, 0}, {store, 1});
    return graph::SaveGraph(graph);
}

// The graph editor as an overlay on the renderer's frames. ImGui is created the first time the editor is shown, and not before.
class GraphEditorUi final : public IEditorUi {
public:
    GraphEditorUi() : model(registry) {
        graph::RegisterStandardNodes(registry);
        model.LoadFromText(StarterGraphText(registry), "");
        model.SetStatus("");
    }

    // ----- IFrameOverlay -----
    void Attach(const OverlayTarget& newTarget) override {
        target = newTarget;
        attached = true;
    }

    void Detach() override {
        host.Destroy();
        attached = false;
        frameReady = false;
    }

    bool IsActive() const override { return frameReady; }

    void Record(VkCommandBuffer commandBuffer, VkImage /*image*/, VkImageView view, VkExtent2D extent) override {
        host.Record(commandBuffer, view, extent);
    }

    // ----- IEditorUi -----
    void SetVisible(bool shown) override { visible = shown; }
    bool IsVisible() const override { return visible; }

    void BeginFrame(float deltaSeconds) override {
        frameReady = false;
        // A graph that is playing goes on running while the editor is hidden, so that hiding it to look at the scene does not
        // stop what the graph does. Its log and glow are there when the editor is opened again.
        model.Tick(deltaSeconds);
        if (host.IsCreated()) {
            host.SetInputConnected(visible);  // while hidden, ImGui must not be fed events that no frame will use
        }
        if (!visible || !attached) {
            return;
        }
        if (!host.IsCreated()) {
            if (!host.Create(target)) {
                visible = false;  // the editor cannot be shown on this device
                return;
            }
            host.SetInputConnected(true);
        }
        host.BeginFrame();
        window.Draw(model);
        if (window.TakeCloseRequest()) {
            visible = false;
        }
        host.EndFrame();
        frameReady = true;
    }

    bool CapturesMouse() const override { return EditorCapturesMouse(InputState()); }
    bool CapturesKeyboard() const override { return EditorCapturesKeyboard(InputState()); }

    std::string LoadGraphFile(const std::string& path) override { return model.LoadFromFile(path); }
    void Play() override { model.Play(); }

private:
    EditorInputState InputState() const {
        EditorInputState state;
        state.visible = visible && host.IsCreated();
        if (state.visible) {
            const ImGuiIO& io = ImGui::GetIO();
            state.windowFocused = window.IsFocused();
            state.wantCaptureKeyboard = io.WantCaptureKeyboard;
            state.wantTextInput = io.WantTextInput;
            state.paletteOpen = window.IsPaletteOpen();
            state.wantCaptureMouse = io.WantCaptureMouse;
        }
        return state;
    }

    // The registry comes first: the model keeps a reference to it.
    graph::NodeRegistry registry;
    graph_editor::EditorModel model;
    GraphEditorWindow window;
    ImGuiHost host;
    OverlayTarget target;
    bool attached = false;
    bool visible = false;
    bool frameReady = false;  // the interface of this frame is built and waiting to be recorded
};

}  // namespace

std::unique_ptr<IEditorUi> CreateGraphEditorUi() {
    return std::make_unique<GraphEditorUi>();
}

}  // namespace editor_ui
