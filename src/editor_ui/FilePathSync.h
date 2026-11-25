#pragma once

#include <string>

// The file name box of the toolbar shows the path of the graph in the model, so that Save writes to the file the graph came from,
// and a person can still type another name. The model's path is the one that is true: the box follows it whenever it changes.

namespace editor_ui {

// Call each frame with the model's path and the last model path that was seen. Returns true when the box must be set to the
// model's path: the path changed, and is not empty (a graph that has no file yet leaves the box as it is).
inline bool FileFieldMustFollowModel(const std::string& modelPath, std::string& lastSeenModelPath) {
    if (modelPath == lastSeenModelPath) {
        return false;
    }
    lastSeenModelPath = modelPath;
    return !modelPath.empty();
}

}  // namespace editor_ui
