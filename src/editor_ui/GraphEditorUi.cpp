#include <iostream>
#include <memory>

#include <imgui.h>

#include "editor_ui/EditorMenuBar.h"
#include "editor_ui/GeneratorPanels.h"
#include "editor_ui/GraphEditorWindow.h"
#include "editor_ui/IEditorUi.h"
#include "editor_ui/InputPolicy.h"
#include "editor_ui/ImGuiHost.h"
#include "graph/GraphFile.h"
#include "graph_editor/EditorModel.h"
#include "graph_engine/EngineNodes.h"

namespace editor_ui {
namespace {

constexpr float kScreenMessageMargin = 12.0f;

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
        graph_engine::RegisterEngineNodes(registry);
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
        generatorPanels.Update();  // work started in a panel is collected and spawned also while the editor is closed
        if (host.IsCreated()) {
            host.SetInputConnected(visible);  // while hidden, ImGui must not be fed events that no frame will use
        }
        // Lines that a graph prints are drawn over the scene when the editor is closed too, but only while there are any, so that a run
        // that prints nothing never creates ImGui.
        const bool hasScreenMessages = screenMessages != nullptr && !screenMessages->empty();
        if (!attached || (!visible && !hasScreenMessages)) {
            return;
        }
        if (!host.IsCreated()) {
            if (!host.Create(target)) {
                visible = false;  // the editor cannot be shown on this device
                return;
            }
            host.SetInputConnected(visible);
        }
        host.BeginFrame();
        if (visible) {
            DrawEditor();
        }
        if (hasScreenMessages) {
            DrawScreenMessages(*screenMessages);
        }
        host.EndFrame();
        frameReady = true;
    }

    bool CapturesMouse() const override { return EditorCapturesMouse(InputState()); }
    bool CapturesKeyboard() const override { return EditorCapturesKeyboard(InputState()); }

    std::string LoadGraphFile(const std::string& path) override { return model.LoadFromFile(path); }
    void Play() override { model.Play(); }

    void SetHost(graph_engine::EngineHost* engineHost) override {
        model.SetHost(engineHost);
        generatorPanels.SetHost(engineHost);
    }
    void OpenPanel(const std::string& name) override {
        windows.characterGenerator = windows.characterGenerator || name == "character";
        windows.materialGenerator = windows.materialGenerator || name == "material";
        windows.shortcuts = windows.shortcuts || name == "shortcuts";
    }
    void SendKey(const std::string& keyName) override { model.SendKey(keyName); }
    void SetScreenMessages(const std::vector<engine_host::ScreenMessage>* messages) override { screenMessages = messages; }
    std::optional<material::MaterialLibrary> TakeMaterialLibraryToUpload() override { return generatorPanels.TakeLibraryToUpload(); }
    void NoteMaterialLibraryUploaded(double milliseconds) override { generatorPanels.NoteLibraryUploaded(milliseconds); }

private:
    // The menu bar, the graph editor and the other windows that the View menu shows.
    void DrawEditor() {
        const MenuRequests requests = DrawMenuBar(model, windows, menuState);
        if (windows.graphEditor) {
            window.Draw(model);
            if (window.TakeCloseRequest()) {
                visible = false;
            }
        }
        if (requests.showWholeGraph) {
            window.ShowWholeGraph();
        }
        if (requests.closeEditor) {
            visible = false;
        }
        generatorPanels.DrawCharacterWindow(windows.characterGenerator);
        generatorPanels.DrawMaterialWindow(windows.materialGenerator);
        DrawShortcutsWindow(windows.shortcuts);
    }

    // The lines that graphs print, at the top left of the window, over the scene. They take no clicks.
    static void DrawScreenMessages(const std::vector<engine_host::ScreenMessage>& messages) {
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + kScreenMessageMargin, viewport->WorkPos.y + kScreenMessageMargin));
        ImGui::SetNextWindowBgAlpha(0.45f);
        const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings |
                                       ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_AlwaysAutoResize;
        if (ImGui::Begin("##screen messages", nullptr, flags)) {
            for (const engine_host::ScreenMessage& message : messages) {
                ImGui::TextUnformatted(message.text.c_str());
            }
        }
        ImGui::End();
    }

    EditorInputState InputState() const {
        EditorInputState state;
        state.visible = visible && host.IsCreated();
        if (state.visible) {
            const ImGuiIO& io = ImGui::GetIO();
            state.windowFocused = window.IsFocused() || generatorPanels.IsFocused();
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
    GeneratorPanels generatorPanels;
    WindowVisibility windows;
    MenuState menuState;
    ImGuiHost host;
    const std::vector<engine_host::ScreenMessage>* screenMessages = nullptr;
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
