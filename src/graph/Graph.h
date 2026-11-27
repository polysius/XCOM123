#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "graph/NodeRegistry.h"
#include "graph/Value.h"

// A graph: nodes placed on a canvas, and wires between their pins. This is the data a person edits and saves; running it
// is the interpreter's job.
//
// A pin is named by its node and its place in the lists of inputs or outputs of that node's kind. A wire runs from an
// output pin to an input pin. Execution wires (Exec) say what runs next, and each Exec output has at most one wire, as in
// a flow chart. Data wires carry a value, and each input takes at most one, while an output may feed many.

namespace graph {

struct PinRef {
    uint32_t node = 0;
    int pin = 0;
    bool operator==(const PinRef& other) const { return node == other.node && pin == other.pin; }
};

struct Connection {
    PinRef from;  // an output pin
    PinRef to;    // an input pin
};

struct Node {
    uint32_t id = 0;
    std::string type;  // the id of its kind in the registry
    glm::vec2 position{0.0f};
    std::map<int, Value> inputValues;               // what a person typed into an input that has no wire, by input
    std::map<std::string, Value> properties;        // settings of the node that are not pins, such as a variable name
};

// A named value that every node of the graph can read and set.
struct Variable {
    std::string name;
    PinType type = PinType::Float;
    Value initial;
};

class Graph {
public:
    // Adds a node of a kind and returns its number, which stays the same for the life of the node. The kind need not be
    // in a registry yet; validation says when it is not.
    uint32_t AddNode(const std::string& type, glm::vec2 position);

    // Takes a node away with all the wires on it. Returns false if there is no such node.
    bool RemoveNode(uint32_t id);

    Node* FindNode(uint32_t id);
    const Node* FindNode(uint32_t id) const;
    const std::vector<Node>& Nodes() const { return nodes; }
    const std::vector<Connection>& Connections() const { return connections; }

    // Draws a wire. Returns an empty string when it was made, or says why not: the pins are not there, they are both on
    // one node, an output is wired to an output, the types do not fit (see CanConvert), or the wire would close a loop.
    // A wire already in the input, and one already out of an Exec output, are replaced.
    std::string Connect(const NodeRegistry& registry, PinRef from, PinRef to);

    // Why a wire between these pins cannot be made, or an empty string if it can. Connect does not change the graph when
    // this says no, and the editor asks it while a wire is being dragged.
    std::string WhyNotConnect(const NodeRegistry& registry, PinRef from, PinRef to) const;

    // Takes away the wire into an input, or all the wires out of an output.
    void DisconnectInput(PinRef input);
    void DisconnectOutput(PinRef output);

    const Connection* ConnectionInto(PinRef input) const;
    std::vector<const Connection*> ConnectionsFrom(PinRef output) const;

    std::vector<Variable>& Variables() { return variables; }
    const std::vector<Variable>& Variables() const { return variables; }
    const Variable* FindVariable(const std::string& name) const;

    // The node number the next AddNode gives. Saved with a graph, so numbers are not used again after a reload.
    uint32_t NextId() const { return nextId; }
    void SetNextId(uint32_t id) { nextId = id; }

    // Replaces the nodes and wires, as loading does.
    void Clear();
    void AddLoadedNode(Node node);
    void AddLoadedConnection(const Connection& connection);

private:
    // Whether a wire from `from` to `to` would give a node a path back to itself, along Exec wires or along the data wires
    // between Pure nodes (which would be asked for their own result without end).
    bool WouldCloseLoop(const NodeRegistry& registry, PinRef from, PinRef to, bool exec) const;

    std::vector<Node> nodes;
    std::vector<Connection> connections;
    std::vector<Variable> variables;
    uint32_t nextId = 1;
};

// The pins of a node, as the registry says for its kind. For a variable node they are those of the variable's type. Null
// kinds give empty lists.
struct NodePins {
    std::vector<PinDescription> inputs;
    std::vector<PinDescription> outputs;
    const NodeDescription* description = nullptr;
};
NodePins PinsOf(const Graph& graph, const NodeRegistry& registry, const Node& node);

// The value of a setting of a node (see PropertyDescription): what the node holds, or the default of its kind when it holds
// none or one of another type. A name the kind does not list gives the default of a decimal number, 0.
Value PropertyValue(const Node& node, const NodeDescription& description, const std::string& name);

}  // namespace graph
