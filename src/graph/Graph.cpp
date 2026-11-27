#include "graph/Graph.h"

#include <algorithm>

namespace graph {

NodePins PinsOf(const Graph& graph, const NodeRegistry& registry, const Node& node) {
    NodePins pins;
    pins.description = registry.Find(node.type);
    if (pins.description == nullptr) {
        return pins;
    }
    pins.inputs = pins.description->inputs;
    pins.outputs = pins.description->outputs;
    if (pins.description->usesVariable) {
        // The pins that carry the value are those of the variable's type; the Exec pins stay as they are.
        PinType type = PinType::Float;
        const auto named = node.properties.find("variable");
        if (named != node.properties.end() && std::holds_alternative<std::string>(named->second)) {
            if (const Variable* variable = graph.FindVariable(std::get<std::string>(named->second))) {
                type = variable->type;
            }
        }
        for (std::vector<PinDescription>* list : {&pins.inputs, &pins.outputs}) {
            for (PinDescription& pin : *list) {
                if (pin.type != PinType::Exec) {
                    pin.type = type;
                    pin.defaultValue = DefaultValue(type);
                }
            }
        }
    }
    return pins;
}

Value PropertyValue(const Node& node, const NodeDescription& description, const std::string& name) {
    const auto listed = std::find_if(description.properties.begin(), description.properties.end(),
                                     [&](const PropertyDescription& property) { return property.name == name; });
    if (listed == description.properties.end()) {
        return DefaultValue(PinType::Float);
    }
    const auto held = node.properties.find(name);
    if (held != node.properties.end() && TypeOf(held->second) == listed->type) {
        return held->second;
    }
    return listed->defaultValue;
}

uint32_t Graph::AddNode(const std::string& type, glm::vec2 position) {
    Node node;
    node.id = nextId++;
    node.type = type;
    node.position = position;
    nodes.push_back(std::move(node));
    return nodes.back().id;
}

bool Graph::RemoveNode(uint32_t id) {
    const auto found = std::find_if(nodes.begin(), nodes.end(), [id](const Node& node) { return node.id == id; });
    if (found == nodes.end()) {
        return false;
    }
    nodes.erase(found);
    connections.erase(std::remove_if(connections.begin(), connections.end(), [id](const Connection& c) { return c.from.node == id || c.to.node == id; }),
                      connections.end());
    return true;
}

Node* Graph::FindNode(uint32_t id) {
    for (Node& node : nodes) {
        if (node.id == id) {
            return &node;
        }
    }
    return nullptr;
}

const Node* Graph::FindNode(uint32_t id) const {
    for (const Node& node : nodes) {
        if (node.id == id) {
            return &node;
        }
    }
    return nullptr;
}

const Variable* Graph::FindVariable(const std::string& name) const {
    for (const Variable& variable : variables) {
        if (variable.name == name) {
            return &variable;
        }
    }
    return nullptr;
}

const Connection* Graph::ConnectionInto(PinRef input) const {
    for (const Connection& connection : connections) {
        if (connection.to == input) {
            return &connection;
        }
    }
    return nullptr;
}

std::vector<const Connection*> Graph::ConnectionsFrom(PinRef output) const {
    std::vector<const Connection*> result;
    for (const Connection& connection : connections) {
        if (connection.from == output) {
            result.push_back(&connection);
        }
    }
    return result;
}

bool Graph::WouldCloseLoop(const NodeRegistry& registry, PinRef from, PinRef to, bool exec) const {
    // The wire runs from.node to to.node. It closes a loop when to.node already leads back to from.node. Walk forward from
    // to.node along the wires of the same sort, and see whether from.node is reached.
    std::vector<uint32_t> stack = {to.node};
    std::vector<uint32_t> seen;
    while (!stack.empty()) {
        const uint32_t current = stack.back();
        stack.pop_back();
        if (current == from.node) {
            return true;
        }
        if (std::find(seen.begin(), seen.end(), current) != seen.end()) {
            continue;
        }
        seen.push_back(current);
        for (const Connection& connection : connections) {
            if (connection.from.node != current) {
                continue;
            }
            const Node* target = FindNode(connection.to.node);
            const Node* source = FindNode(connection.from.node);
            if (target == nullptr || source == nullptr) {
                continue;
            }
            const NodePins sourcePins = PinsOf(*this, registry, *source);
            if (connection.from.pin < 0 || connection.from.pin >= static_cast<int>(sourcePins.outputs.size())) {
                continue;
            }
            const bool isExec = sourcePins.outputs[connection.from.pin].type == PinType::Exec;
            if (isExec != exec) {
                continue;
            }
            if (!exec) {
                // Only Pure nodes ask one another for results without end; an action's outputs are stored and read.
                const NodePins targetPins = PinsOf(*this, registry, *target);
                if (sourcePins.description == nullptr || targetPins.description == nullptr || sourcePins.description->kind != NodeKind::Pure ||
                    targetPins.description->kind != NodeKind::Pure) {
                    continue;
                }
            }
            stack.push_back(connection.to.node);
        }
    }
    return false;
}

std::string Graph::WhyNotConnect(const NodeRegistry& registry, PinRef from, PinRef to) const {
    const Node* source = FindNode(from.node);
    const Node* target = FindNode(to.node);
    if (source == nullptr || target == nullptr) {
        return "One of the nodes is not there.";
    }
    if (from.node == to.node) {
        return "A node cannot be wired to itself.";
    }
    const NodePins sourcePins = PinsOf(*this, registry, *source);
    const NodePins targetPins = PinsOf(*this, registry, *target);
    if (sourcePins.description == nullptr || targetPins.description == nullptr) {
        return "A node of an unknown kind has no pins.";
    }
    if (from.pin < 0 || from.pin >= static_cast<int>(sourcePins.outputs.size())) {
        return "The wire has to start at an output pin.";
    }
    if (to.pin < 0 || to.pin >= static_cast<int>(targetPins.inputs.size())) {
        return "The wire has to end at an input pin.";
    }
    const PinType fromType = sourcePins.outputs[from.pin].type;
    const PinType toType = targetPins.inputs[to.pin].type;
    if ((fromType == PinType::Exec) != (toType == PinType::Exec)) {
        return "Execution pins only join other execution pins.";
    }
    if (!CanConvert(fromType, toType)) {
        return std::string("A ") + PinTypeName(fromType) + " cannot be turned into a " + PinTypeName(toType) + ".";
    }
    if (WouldCloseLoop(registry, from, to, fromType == PinType::Exec)) {
        return "That wire would make a loop.";
    }
    return {};
}

std::string Graph::Connect(const NodeRegistry& registry, PinRef from, PinRef to) {
    std::string why = WhyNotConnect(registry, from, to);
    if (!why.empty()) {
        return why;
    }
    const NodePins sourcePins = PinsOf(*this, registry, *FindNode(from.node));
    DisconnectInput(to);
    if (sourcePins.outputs[from.pin].type == PinType::Exec) {
        DisconnectOutput(from);  // an execution pin leads to one place only
    }
    connections.push_back({from, to});
    return {};
}

void Graph::DisconnectInput(PinRef input) {
    connections.erase(std::remove_if(connections.begin(), connections.end(), [&](const Connection& c) { return c.to == input; }), connections.end());
}

void Graph::DisconnectOutput(PinRef output) {
    connections.erase(std::remove_if(connections.begin(), connections.end(), [&](const Connection& c) { return c.from == output; }), connections.end());
}

void Graph::Clear() {
    nodes.clear();
    connections.clear();
    variables.clear();
    nextId = 1;
}

void Graph::AddLoadedNode(Node node) {
    nextId = std::max(nextId, node.id + 1);
    nodes.push_back(std::move(node));
}

void Graph::AddLoadedConnection(const Connection& connection) {
    connections.push_back(connection);
}

}  // namespace graph
