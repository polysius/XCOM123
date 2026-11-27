// The running and saving side of the editor model: play and stop, what ran, and files.

#include <algorithm>
#include <fstream>
#include <iterator>
#include <sstream>

#include "graph/GraphFile.h"
#include "graph_editor/AutoLayout.h"
#include "graph_editor/EditorModel.h"
#include "graph_engine/EngineHost.h"
#include "graph_engine/KeyNames.h"

namespace graph_editor {
namespace {

// How long a node stays lit after it ran.
constexpr float kGlowSeconds = 0.8f;

}  // namespace

bool EditorModel::Play() {
    Stop();
    if (HasErrors()) {
        for (const graph::Issue& issue : issues) {
            if (issue.isError) {
                status = "Cannot play: " + issue.message;
                break;
            }
        }
        return false;
    }
    runGraph = std::make_unique<graph::Graph>(graph);
    interpreter = std::make_unique<graph::Interpreter>(*runGraph, registry);
    interpreter->SetHost(host);
    lastLog.clear();
    lastRuntimeError.clear();
    interpreter->Start();
    NoteNodesThatRan();
    status = "Playing.";
    return true;
}

void EditorModel::Stop() {
    if (interpreter == nullptr) {
        return;
    }
    lastLog = interpreter->Log();
    lastRuntimeError = interpreter->RuntimeError();
    interpreter.reset();
    runGraph.reset();
    secondsSinceRun.clear();
    status = "Stopped.";
}

void EditorModel::NoteNodesThatRan() {
    for (uint32_t id : interpreter->ExecutedNodes()) {
        secondsSinceRun[id] = 0.0f;
    }
}

void EditorModel::Tick(float deltaSeconds) {
    if (interpreter == nullptr) {
        return;
    }
    for (auto it = secondsSinceRun.begin(); it != secondsSinceRun.end();) {
        it->second += deltaSeconds;
        it = it->second > kGlowSeconds ? secondsSinceRun.erase(it) : std::next(it);
    }
    interpreter->Tick(deltaSeconds);
    NoteNodesThatRan();
}

void EditorModel::SendKey(const std::string& keyName) {
    if (interpreter == nullptr) {
        return;
    }
    interpreter->FireEvent(graph_engine::kKeyPressedEvent, graph_engine::KeyFilter(keyName));
    NoteNodesThatRan();
}

float EditorModel::NodeGlow(uint32_t node) const {
    const auto found = secondsSinceRun.find(node);
    return found == secondsSinceRun.end() ? 0.0f : glm::clamp(1.0f - found->second / kGlowSeconds, 0.0f, 1.0f);
}

const std::vector<std::string>& EditorModel::LogLines() const {
    return interpreter != nullptr ? interpreter->Log() : lastLog;
}

const std::string& EditorModel::RuntimeError() const {
    return interpreter != nullptr ? interpreter->RuntimeError() : lastRuntimeError;
}

void EditorModel::ClearRunState() {
    Stop();
    lastLog.clear();
    lastRuntimeError.clear();
}

std::string EditorModel::SaveToFile(const std::string& path) {
    if (path.empty()) {
        return "There is no file name to save to.";
    }
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file << graph::SaveGraph(graph);
    file.close();
    if (!file) {
        status = "Could not write " + path + ".";
        return status;
    }
    filePath = path;
    savedText = graph::SaveGraph(graph);
    status = "Saved " + path + ".";
    return {};
}

std::string EditorModel::LoadFromFile(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        status = "Could not open " + path + ".";
        return status;
    }
    std::stringstream text;
    text << file.rdbuf();
    return LoadFromText(text.str(), path);
}

std::string EditorModel::LoadFromText(const std::string& text, const std::string& path) {
    const std::string name = path.empty() ? "the graph" : path;
    graph::Graph loaded;
    const graph::LoadResult result = graph::LoadGraph(text, loaded);
    if (!result.ok) {
        status = name + " is not a graph this editor can read: " + result.error;
        return status;
    }

    ClearRunState();
    graph = std::move(loaded);
    history.Clear();
    ClearSelection();
    pendingWire.reset();
    wireDrag.reset();
    gesture = Gesture::None;
    filePath = path;
    savedText = graph::SaveGraph(graph);

    // A graph made by a program or by hand may not have places for its nodes. Nodes that all stand on one spot are arranged.
    const bool allOnOneSpot = std::all_of(graph.Nodes().begin(), graph.Nodes().end(),
                                          [&](const graph::Node& node) { return node.position == graph.Nodes().front().position; });
    if (graph.Nodes().size() > 1 && allOnOneSpot) {
        AutoLayout(graph, registry);
    }
    GraphChanged();
    status = "Loaded " + name + (result.warnings.empty() ? "." : " with " + std::to_string(result.warnings.size()) + " warning(s): " + result.warnings.front());
    return {};
}

void EditorModel::NewGraph() {
    ClearRunState();
    graph = graph::Graph();
    history.Clear();
    ClearSelection();
    pendingWire.reset();
    wireDrag.reset();
    gesture = Gesture::None;
    filePath.clear();
    savedText = graph::SaveGraph(graph);
    GraphChanged();
    status = "Started a new graph.";
}

}  // namespace graph_editor
