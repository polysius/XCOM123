#pragma once

#include <string>
#include <vector>

#include "graph/Graph.h"

// Saving a graph as text and reading it back. The text is JSON, written in a fixed order, so that saving a graph that did not
// change gives the same file, and a graph with the same wires saved twice gives files a person can compare.

namespace graph {

// The graph as text.
std::string SaveGraph(const Graph& graph);

struct LoadResult {
    bool ok = false;
    std::string error;                  // why the text could not be read at all, when ok is false
    std::vector<std::string> warnings;  // what was left out of a file that could be read: wires to nodes that are not there
};

// Replaces the contents of `graph` with the graph in the text. A text that is not a graph of this version leaves the graph
// as it was and says why. Nodes of kinds the program does not know are kept, so that saving the graph again loses nothing.
LoadResult LoadGraph(const std::string& text, Graph& graph);

}  // namespace graph
