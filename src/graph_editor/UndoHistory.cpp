#include "graph_editor/UndoHistory.h"

namespace graph_editor {

void UndoHistory::Record(const std::string& before, const std::string& mergeKey) {
    const bool continuesGesture = !mergeKey.empty() && mergeKey == lastMergeKey;
    lastMergeKey = mergeKey;
    if (continuesGesture) {
        return;
    }
    undoSteps.push_back(before);
    if (undoSteps.size() > kMaxUndoSteps) {
        undoSteps.pop_front();
    }
    redoSteps.clear();
}

std::optional<std::string> UndoHistory::Undo(const std::string& current) {
    if (undoSteps.empty()) {
        return std::nullopt;
    }
    std::string previous = std::move(undoSteps.back());
    undoSteps.pop_back();
    redoSteps.push_back(current);
    lastMergeKey.clear();
    return previous;
}

std::optional<std::string> UndoHistory::Redo(const std::string& current) {
    if (redoSteps.empty()) {
        return std::nullopt;
    }
    std::string next = std::move(redoSteps.back());
    redoSteps.pop_back();
    undoSteps.push_back(current);
    lastMergeKey.clear();
    return next;
}

void UndoHistory::Clear() {
    undoSteps.clear();
    redoSteps.clear();
    lastMergeKey.clear();
}

}  // namespace graph_editor
