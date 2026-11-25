#pragma once

#include <cstddef>
#include <deque>
#include <optional>
#include <string>

// The steps that Undo and Redo move through, kept as snapshots of the whole graph (the text that saving it gives). A graph is
// small, and a snapshot cannot forget to restore something that a smarter record of changes would.
//
// Before each change the editor records the state it is about to leave. Changes that belong together, such as the many small
// moves of one drag, give the same merge key and are recorded once, until something ends the gesture.

namespace graph_editor {

constexpr size_t kMaxUndoSteps = 100;

class UndoHistory {
public:
    // Records `before` as the state to come back to. With a merge key equal to that of the previous record (and no EndGesture
    // since), nothing is recorded, as the earlier snapshot already holds the state from before the gesture began.
    // A new change makes the steps that were undone unreachable.
    void Record(const std::string& before, const std::string& mergeKey = {});

    // Ends a run of changes that share a merge key, so the next change with that key starts a step of its own.
    void EndGesture() { lastMergeKey.clear(); }

    bool CanUndo() const { return !undoSteps.empty(); }
    bool CanRedo() const { return !redoSteps.empty(); }

    // Gives the state to go back to and keeps `current` for Redo; empty when there is nothing to undo.
    std::optional<std::string> Undo(const std::string& current);
    std::optional<std::string> Redo(const std::string& current);

    size_t UndoCount() const { return undoSteps.size(); }
    size_t RedoCount() const { return redoSteps.size(); }
    void Clear();

private:
    std::deque<std::string> undoSteps;
    std::deque<std::string> redoSteps;
    std::string lastMergeKey;
};

}  // namespace graph_editor
