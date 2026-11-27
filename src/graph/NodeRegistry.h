#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "graph/Value.h"

// What kinds of node there are, and what each one does. A graph only names the kind of a node ("math.add"); the registry
// says what pins that kind has and what it runs, so the same registry builds the palette that a person picks nodes
// from, checks the wires of a graph, and gives the interpreter the code to run.

namespace graph {

class NodeContext;

struct PinDescription {
    std::string name;
    PinType type = PinType::Float;
    Value defaultValue;       // what an input holds while nothing is wired into it; ignored for Exec and for outputs
    std::string tooltip;
};

// How a node takes part in a run.
enum class NodeKind {
    Event,   // starts a run: no way in, and an Exec out, such as "when the game starts"
    Action,  // runs in turn, when the wire before it reaches it, and passes on to the wire after it
    Pure,    // only computes: it has no Exec pins and runs whenever something that needs its result asks
};

using NodeFunction = std::function<void(NodeContext&)>;

// A setting of a node that is not a pin: a value a person picks once, such as the key an event waits for. It is kept in the
// node's properties; a node that has none of it (one added from the palette) uses the default.
struct PropertyDescription {
    std::string name;
    PinType type = PinType::Float;
    Value defaultValue;
    std::string tooltip;
    // For a String property: the only values that mean something. The editor offers them in a list, and validation warns of
    // any other text. Empty means any text will do.
    std::vector<std::string> choices;
};

struct NodeDescription {
    std::string id;        // "math.add"; also what a saved graph calls the kind
    std::string title;     // "Add"
    std::string category;  // "Math", for the palette
    std::string tooltip;   // what it does, in a sentence
    NodeKind kind = NodeKind::Pure;
    std::vector<PinDescription> inputs;
    std::vector<PinDescription> outputs;
    NodeFunction run;
    // A node that decides for itself which of its Exec outputs go on, by calling NodeContext::Execute (a branch, a loop),
    // sets this to false. Every other action goes on to its first Exec output when it is done.
    bool continuesAutomatically = true;
    // A variable node works on the variable named in its "variable" property. Its pins are those of the variable's type.
    bool usesVariable = false;
    // The settings a person can change on the node, besides its pins.
    std::vector<PropertyDescription> properties;
    // An event that waits for time to pass runs when the seconds in this Float property have gone by since the graph started (or
    // since it last ran, when it repeats). Empty for every other event.
    std::string timerSecondsProperty;
    bool timerRepeats = false;
};

class NodeRegistry {
public:
    // Adds a kind of node. A second kind with the same id replaces the first.
    void Register(NodeDescription description);

    // The kind with this id, or null.
    const NodeDescription* Find(const std::string& id) const;

    // Every kind, in the order of id, for building a palette.
    std::vector<const NodeDescription*> All() const;

private:
    std::map<std::string, NodeDescription> kinds;
};

// Adds the kinds that every graph can use: events for the start and for every frame, branches, sequences and loops,
// arithmetic, comparisons, logic, vectors, text, variables and a print for debugging.
void RegisterStandardNodes(NodeRegistry& registry);

}  // namespace graph
