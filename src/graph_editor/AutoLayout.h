#pragma once

#include "graph/Graph.h"
#include "graph/NodeRegistry.h"

// Places the nodes of a graph from left to right in the order that things run and values flow: a node stands in a column to
// the right of every node that feeds it, and the nodes of a column are stacked in the order of the nodes that feed them, so
// wires run forward and rarely cross. The top-left corner of the result stays where the top-left corner of the graph was.

namespace graph_editor {

void AutoLayout(graph::Graph& graph, const graph::NodeRegistry& registry);

}  // namespace graph_editor
