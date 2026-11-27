#pragma once

#include "graph/NodeRegistry.h"
#include "graph_engine/EngineHost.h"

// The nodes that make engine features part of a graph: the sun and the time of day, the camera, props and their physics, people,
// the river and the on-screen log, and the events that start a run (a key, a timer).
//
// A node that acts on the engine reaches it through NodeContext::Host(), which must be an EngineHost (give the interpreter
// `static_cast<EngineHost*>(&host)` converted to void*, as EditorModel::SetHost does). With no host such a node does nothing, so
// a graph can be checked and run in a program that has no engine. Every argument is made finite and kept inside the limits of
// EngineHost.h before it reaches the host. Actions that change the engine are never Pure nodes, which the interpreter may run
// more than once or not at all; the Pure nodes here only read.
//
// There is no Delay node: the interpreter runs a graph to its end within one step of the frame and cannot pause in the middle
// of one. "After N Seconds" and "Every N Seconds" events cover what a delay is used for.

namespace graph_engine {

// The categories of the nodes, in the order a palette lists them.
inline constexpr const char* kEngineCategories[] = {"World", "Camera", "Characters", "Physics", "Water", "Debug"};

void RegisterEngineNodes(graph::NodeRegistry& registry);

}  // namespace graph_engine
