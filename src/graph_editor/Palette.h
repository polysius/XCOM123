#pragma once

#include <optional>
#include <string>
#include <vector>

#include "graph/Graph.h"
#include "graph/NodeRegistry.h"

// The list of nodes a person can add, filtered by what they type. It is built from the registry, plus one "get" and one "set"
// node for each variable of the graph.

namespace graph_editor {

// What a wire that was dragged off a pin and let go on empty canvas is looking for: a node with a pin it can join.
struct WireWish {
    graph::PinType type = graph::PinType::Float;
    bool fromOutput = true;  // the wire was dragged from an output, so it needs a node with an input; otherwise an output
};

struct PaletteEntry {
    std::string typeId;
    std::string title;      // "Add", or "Get Health" for a variable
    std::string category;
    std::string tooltip;
    std::string variable;   // for the nodes of a variable: which one; otherwise empty
    int connectPin = -1;    // with a WireWish: the first pin of the new node that the wire fits; otherwise -1
};

// Every entry that matches the query, best first. The query is split at spaces and each word must match something of the entry:
// its title, its category, the id of its kind or the name or type of one of its pins ("float", "exec"), so that "math float" and
// "vec" both find what a person means. An empty query matches everything, ordered by category and title.
// With a wish, only nodes with a pin that the wire can join are listed, and each says which pin it is.
std::vector<PaletteEntry> SearchPalette(const graph::Graph& graph, const graph::NodeRegistry& registry, const std::string& query,
                                        const std::optional<WireWish>& wish = std::nullopt);

}  // namespace graph_editor
