#include "graph/Interpreter.h"

#include <algorithm>

namespace graph {
namespace {

// How many nodes one run may run before it is stopped as a loop that will not end. The limit is generous: a person's graph
// does a few hundred steps, and a loop over a few thousand things a few thousand.
constexpr int kMaxSteps = 100000;
// How deep branches and loops may be nested inside one another.
constexpr int kMaxDepth = 64;
// Pure nodes ask for their inputs by calling each other, so a long chain of them uses a deep stack; this keeps well inside the
// smallest stack a thread may have.
constexpr int kMaxPureDepth = 200;

constexpr const char* kStartEvent = "event.start";
constexpr const char* kTickEvent = "event.tick";

int FirstExecOutput(const NodePins& pins) {
    for (size_t i = 0; i < pins.outputs.size(); ++i) {
        if (pins.outputs[i].type == PinType::Exec) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

}  // namespace

bool HasErrors(const ValidationReport& report) {
    return std::any_of(report.begin(), report.end(), [](const Issue& issue) { return issue.isError; });
}

ValidationReport Validate(const Graph& graph, const NodeRegistry& registry) {
    ValidationReport report;
    const auto error = [&](uint32_t node, std::string message) { report.push_back({node, std::move(message), true}); };
    const auto warning = [&](uint32_t node, std::string message) { report.push_back({node, std::move(message), false}); };

    bool hasEvent = false;
    for (const Node& node : graph.Nodes()) {
        const NodePins pins = PinsOf(graph, registry, node);
        if (pins.description == nullptr) {
            error(node.id, "This kind of node, '" + node.type + "', is not known.");
            continue;
        }
        hasEvent = hasEvent || pins.description->kind == NodeKind::Event;
        if (pins.description->usesVariable) {
            const auto named = node.properties.find("variable");
            if (named == node.properties.end() || !std::holds_alternative<std::string>(named->second) ||
                graph.FindVariable(std::get<std::string>(named->second)) == nullptr) {
                error(node.id, "Choose a variable that exists.");
            }
        }
    }

    for (const Connection& connection : graph.Connections()) {
        const std::string why = graph.WhyNotConnect(registry, connection.from, connection.to);
        if (!why.empty()) {
            error(connection.to.node, "A wire does not work: " + why);
        }
    }
    // Two wires into one input, or two out of one Exec output, are wires that the editor never draws: they come from a damaged file.
    for (const Connection& connection : graph.Connections()) {
        int into = 0;
        int outOf = 0;
        for (const Connection& other : graph.Connections()) {
            into += other.to == connection.to ? 1 : 0;
            outOf += other.from == connection.from ? 1 : 0;
        }
        if (into > 1) {
            error(connection.to.node, "More than one wire goes into the same input.");
        }
        const Node* source = graph.FindNode(connection.from.node);
        if (source != nullptr && outOf > 1) {
            const NodePins pins = PinsOf(graph, registry, *source);
            if (connection.from.pin >= 0 && connection.from.pin < static_cast<int>(pins.outputs.size()) &&
                pins.outputs[static_cast<size_t>(connection.from.pin)].type == PinType::Exec) {
                error(connection.from.node, "An execution output leads to more than one node.");
            }
        }
    }

    if (!hasEvent && !graph.Nodes().empty()) {
        warning(0, "Nothing starts this graph. Add a 'Start' or an 'Every frame' event.");
    }

    // Actions that no event leads to never run.
    std::vector<uint32_t> reached;
    std::vector<uint32_t> frontier;
    for (const Node& node : graph.Nodes()) {
        const NodePins pins = PinsOf(graph, registry, node);
        if (pins.description != nullptr && pins.description->kind == NodeKind::Event) {
            frontier.push_back(node.id);
        }
    }
    while (!frontier.empty()) {
        const uint32_t current = frontier.back();
        frontier.pop_back();
        if (std::find(reached.begin(), reached.end(), current) != reached.end()) {
            continue;
        }
        reached.push_back(current);
        const Node* node = graph.FindNode(current);
        const NodePins pins = PinsOf(graph, registry, *node);
        for (int i = 0; i < static_cast<int>(pins.outputs.size()); ++i) {
            if (pins.outputs[static_cast<size_t>(i)].type != PinType::Exec) {
                continue;
            }
            for (const Connection* connection : graph.ConnectionsFrom({current, i})) {
                frontier.push_back(connection->to.node);
            }
        }
    }
    for (const Node& node : graph.Nodes()) {
        const NodePins pins = PinsOf(graph, registry, node);
        if (pins.description != nullptr && pins.description->kind == NodeKind::Action && std::find(reached.begin(), reached.end(), node.id) == reached.end()) {
            warning(node.id, "This node never runs: no event leads to it.");
        }
    }
    return report;
}

// ---------------------------------------------------------------------------------------------------------------------
// NodeContext
// ---------------------------------------------------------------------------------------------------------------------

void NodeContext::Execute(int execOutput) {
    if (interpreter.depth >= kMaxDepth) {
        interpreter.stopped = true;
        if (interpreter.runtimeError.empty()) {
            interpreter.runtimeError = "Branches and loops are nested more than 64 deep.";
        }
        return;
    }
    // Every run of the output is a step, even when nothing is wired to it, so a loop with an empty body still comes to an end.
    interpreter.CountStep();
    if (interpreter.stopped) {
        return;
    }
    ++interpreter.depth;
    interpreter.Follow({node.id, execOutput});
    --interpreter.depth;
}

void NodeContext::Set(int output, Value value) {
    if (storesAtOnce) {
        interpreter.outputs[{node.id, output}] = value;
    }
    outputs[static_cast<size_t>(output)] = std::move(value);
}

bool NodeContext::Stopped() const {
    return interpreter.stopped;
}

Value NodeContext::GetVariable(const std::string& name) const {
    const auto found = interpreter.variables.find(name);
    return found != interpreter.variables.end() ? found->second : DefaultValue(PinType::Float);
}

void NodeContext::SetVariable(const std::string& name, Value value) {
    const Variable* declared = interpreter.graph.FindVariable(name);
    interpreter.variables[name] = declared != nullptr ? Convert(value, declared->type) : value;
}

void NodeContext::Log(const std::string& line) {
    interpreter.log.push_back(line);
}

float NodeContext::DeltaSeconds() const {
    return interpreter.currentDelta;
}

float NodeContext::Random() {
    return interpreter.NextRandom();
}

void* NodeContext::Host() const {
    return interpreter.host;
}

const Value* NodeContext::Property(const std::string& name) const {
    const auto found = node.properties.find(name);
    return found != node.properties.end() ? &found->second : nullptr;
}

// ---------------------------------------------------------------------------------------------------------------------
// Interpreter
// ---------------------------------------------------------------------------------------------------------------------

Interpreter::Interpreter(const Graph& graph, const NodeRegistry& registry) : graph(graph), registry(registry), report(Validate(graph, registry)) {}

void Interpreter::Start() {
    if (!CanRun()) {
        return;
    }
    variables.clear();
    for (const Variable& variable : graph.Variables()) {
        variables[variable.name] = Convert(variable.initial, variable.type);
    }
    outputs.clear();
    pureResults.clear();
    log.clear();
    runtimeError.clear();
    randomState = seed;
    RunEvents(kStartEvent, 0.0f);
}

void Interpreter::Tick(float deltaSeconds) {
    if (!CanRun()) {
        return;
    }
    RunEvents(kTickEvent, deltaSeconds);
}

void Interpreter::RunEvents(const std::string& eventType, float deltaSeconds) {
    executed.clear();
    for (const Node& node : graph.Nodes()) {
        if (node.type != eventType) {
            continue;
        }
        stepsLeft = kMaxSteps;
        depth = 0;
        stopped = false;
        currentDelta = deltaSeconds;
        const int next = RunNode(node);
        if (next >= 0) {
            Follow({node.id, next});
        }
    }
    currentDelta = 0.0f;
}

void Interpreter::Follow(PinRef execOutput) {
    PinRef current = execOutput;
    while (!stopped) {
        const std::vector<const Connection*> wires = graph.ConnectionsFrom(current);
        if (wires.empty()) {
            return;
        }
        const Node* target = graph.FindNode(wires.front()->to.node);
        if (target == nullptr) {
            return;
        }
        const NodePins pins = PinsOf(graph, registry, *target);
        if (pins.description == nullptr || pins.description->kind != NodeKind::Action) {
            return;
        }
        const int next = RunNode(*target);
        if (next < 0) {
            return;
        }
        current = {target->id, next};
    }
}

int Interpreter::RunNode(const Node& node) {
    CountStep();
    if (stopped) {
        return -1;
    }
    const NodePins pins = PinsOf(graph, registry, node);
    executed.push_back(node.id);
    ++epoch;  // what Pure nodes worked out for the step before may have changed

    NodeContext context(*this, node);
    context.storesAtOnce = true;
    context.inputs.resize(pins.inputs.size());
    for (size_t i = 0; i < pins.inputs.size(); ++i) {
        context.inputs[i] = pins.inputs[i].type == PinType::Exec ? Value(false) : EvaluateInput(node, pins, static_cast<int>(i));
    }
    if (stopped) {
        return -1;
    }
    context.outputs.resize(pins.outputs.size());
    for (size_t i = 0; i < pins.outputs.size(); ++i) {
        context.outputs[i] = DefaultValue(pins.outputs[i].type);
    }
    if (pins.description->usesVariable) {
        const auto named = node.properties.find("variable");
        if (named != node.properties.end() && std::holds_alternative<std::string>(named->second)) {
            context.variableName = std::get<std::string>(named->second);
        }
    }
    if (pins.description->run) {
        pins.description->run(context);
    }
    for (size_t i = 0; i < pins.outputs.size(); ++i) {
        if (pins.outputs[i].type != PinType::Exec) {
            outputs[{node.id, static_cast<int>(i)}] = context.outputs[i];
        }
    }
    return pins.description->continuesAutomatically ? FirstExecOutput(pins) : -1;
}

Value Interpreter::EvaluateInput(const Node& node, const NodePins& pins, int inputIndex) {
    const PinType type = pins.inputs[static_cast<size_t>(inputIndex)].type;
    const Connection* wire = graph.ConnectionInto({node.id, inputIndex});
    if (wire != nullptr) {
        return Convert(ReadOutput(wire->from), type);
    }
    const auto typed = node.inputValues.find(inputIndex);
    if (typed != node.inputValues.end()) {
        return Convert(typed->second, type);
    }
    return Convert(pins.inputs[static_cast<size_t>(inputIndex)].defaultValue, type);
}

Value Interpreter::ReadOutput(PinRef from) {
    const Node* source = graph.FindNode(from.node);
    const NodePins pins = PinsOf(graph, registry, *source);
    if (pins.description->kind == NodeKind::Pure) {
        const std::vector<Value>& values = EvaluatePure(*source);
        return values[static_cast<size_t>(from.pin)];
    }
    const auto stored = outputs.find({from.node, from.pin});
    return stored != outputs.end() ? stored->second : DefaultValue(pins.outputs[static_cast<size_t>(from.pin)].type);
}

const std::vector<Value>& Interpreter::EvaluatePure(const Node& node) {
    PureResult& result = pureResults[node.id];
    if (result.epoch == epoch && !result.values.empty()) {
        return result.values;
    }
    const NodePins pins = PinsOf(graph, registry, node);
    NodeContext context(*this, node);
    context.outputs.resize(pins.outputs.size());
    for (size_t i = 0; i < pins.outputs.size(); ++i) {
        context.outputs[i] = DefaultValue(pins.outputs[i].type);
    }
    CountStep();
    if (pureDepth >= kMaxPureDepth) {
        stopped = true;
        if (runtimeError.empty()) {
            runtimeError = "Nodes that compute a value are chained more than " + std::to_string(kMaxPureDepth) + " deep.";
        }
    }
    if (!stopped) {
        ++pureDepth;
        context.inputs.resize(pins.inputs.size());
        for (size_t i = 0; i < pins.inputs.size(); ++i) {
            context.inputs[i] = EvaluateInput(node, pins, static_cast<int>(i));
        }
        --pureDepth;
        if (pins.description->usesVariable) {
            const auto named = node.properties.find("variable");
            if (named != node.properties.end() && std::holds_alternative<std::string>(named->second)) {
                context.variableName = std::get<std::string>(named->second);
            }
        }
        if (!stopped && pins.description->run) {
            pins.description->run(context);
        }
    }
    result.values = std::move(context.outputs);
    result.epoch = epoch;
    return result.values;
}

void Interpreter::CountStep() {
    if (stepsLeft <= 0) {
        stopped = true;
        if (runtimeError.empty()) {
            runtimeError = "Stopped after " + std::to_string(kMaxSteps) + " steps. Is there a loop that never ends?";
        }
        return;
    }
    --stepsLeft;
}

float Interpreter::NextRandom() {
    // SplitMix64, taking the top 24 bits so that every value is exact in a float.
    randomState += 0x9E3779B97F4A7C15ull;
    uint64_t z = randomState;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z ^= z >> 31;
    return static_cast<float>(z >> 40) * (1.0f / 16777216.0f);
}

const Value* Interpreter::LastOutput(PinRef output) const {
    const auto found = outputs.find({output.node, output.pin});
    return found != outputs.end() ? &found->second : nullptr;
}

const Value* Interpreter::FindVariable(const std::string& name) const {
    const auto found = variables.find(name);
    return found != variables.end() ? &found->second : nullptr;
}

}  // namespace graph
