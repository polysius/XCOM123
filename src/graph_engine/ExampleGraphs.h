#pragma once

#include <string>

#include "graph/NodeRegistry.h"

// The example graphs that ship in assets/graphs. They are built here, in code, and saved with SaveGraph, so that the files are the
// canonical text of a graph and the tests can check that the files and the code agree. `registry` must hold the standard nodes and
// the engine nodes.

namespace graph_engine {

// The sun goes round the day: the hour advances every frame and sets the time of day. R starts the day again at sunrise.
std::string DayNightCycleGraphText(const graph::NodeRegistry& registry);

// Keys that act on the world: N makes a big crowd, M a small one, C drops a crate in front of the camera and B puts a person there.
std::string SpawnCrowdOnKeyGraphText(const graph::NodeRegistry& registry);

}  // namespace graph_engine
