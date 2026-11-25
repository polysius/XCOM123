#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "graph/Graph.h"
#include "graph/NodeRegistry.h"

// Checking a graph and running it.
//
// A run starts at an event node (when the game starts, every frame). From its Exec output the interpreter follows the wires
// from node to node, running each one in turn. When a node needs a value, it asks the node that is wired into that input:
// a Pure node is worked out then and there from its own inputs, and the output of an action is the value it left when it
// last ran. An input that nothing is wired into holds what its pin says, or what a person typed into it.

namespace graph {

struct Issue {
    uint32_t node = 0;  // the node it is about, or 0 for the graph as a whole
    std::string message;
    bool isError = true;  // an error stops the graph from running; a warning only tells a person something
};
using ValidationReport = std::vector<Issue>;

bool HasErrors(const ValidationReport& report);

// Looks for everything that is wrong with a graph: nodes of unknown kinds, wires that do not fit, two wires into one input,
// loops, variable nodes that name no variable. Also warns of what is probably not meant: actions that nothing runs, and a
// graph with no event to start it.
ValidationReport Validate(const Graph& graph, const NodeRegistry& registry);

class Interpreter;

// What a node can see and do while it runs: the values on its inputs, where its results go, the variables, and a way to
// run the wire that leaves one of its Exec outputs (for a branch, a loop).
class NodeContext {
public:
    // The value on an input, already turned into the type of the pin.
    const Value& Input(int index) const { return inputs[static_cast<size_t>(index)]; }
    template <typename T>
    T Get(int index) const {
        return std::get<T>(inputs[static_cast<size_t>(index)]);
    }

    // Leaves a result on an output. A node that runs a wire (a loop) can set an output before it does, and the nodes
    // on that wire will read it, so an action's results are visible at once.
    void Set(int output, Value value);

    // Whether the run has been stopped (it ran too long, or too deep), so that a loop can give up.
    bool Stopped() const;

    // Runs everything that follows the Exec output, and comes back when it has finished.
    void Execute(int execOutput);

    // The variable this node works on (for variable nodes), and the variables by name.
    const std::string& VariableName() const { return variableName; }
    Value GetVariable(const std::string& name) const;
    void SetVariable(const std::string& name, Value value);

    // Adds a line to the log of the run.
    void Log(const std::string& line);

    // The seconds since the last frame, in the nodes that run every frame; 0 otherwise.
    float DeltaSeconds() const;

    // A number from 0 up to but not including 1, from a generator that starts the same on every run.
    float Random();

    // The program around the graph, for nodes that act on it. It is what was given to Interpreter::SetHost.
    void* Host() const;

    // A setting of the node that is not a pin.
    const Value* Property(const std::string& name) const;

private:
    friend class Interpreter;
    NodeContext(Interpreter& interpreter, const Node& node) : interpreter(interpreter), node(node) {}

    Interpreter& interpreter;
    const Node& node;
    bool storesAtOnce = false;  // for actions: results go to where other nodes read them as soon as they are set
    std::vector<Value> inputs;
    std::vector<Value> outputs;
    std::string variableName;
};

class Interpreter {
public:
    // Checks the graph. The graph and the registry must outlive the interpreter, and the graph must not change while it
    // is in use; to run an edited graph, make another interpreter.
    Interpreter(const Graph& graph, const NodeRegistry& registry);

    const ValidationReport& Report() const { return report; }
    bool CanRun() const { return !HasErrors(report); }

    // The program around the graph, handed to the nodes through NodeContext::Host.
    void SetHost(void* hostProgram) { host = hostProgram; }
    // Where the random numbers start. The same seed gives the same run.
    void SetSeed(uint64_t newSeed) { seed = newSeed; randomState = newSeed; }

    // Sets every variable to its starting value, clears the log and runs every "start" event. Does nothing when the graph
    // has errors.
    void Start();
    // Runs every "every frame" event.
    void Tick(float deltaSeconds);

    const std::vector<std::string>& Log() const { return log; }
    // What stopped a run early, such as a loop that does not end; empty while all is well.
    const std::string& RuntimeError() const { return runtimeError; }
    // The nodes that ran in the most recent Start or Tick, in order, for the editor to light up.
    const std::vector<uint32_t>& ExecutedNodes() const { return executed; }
    // The value a pin of an action or event last left, or null if it has not run.
    const Value* LastOutput(PinRef output) const;
    const Value* FindVariable(const std::string& name) const;

private:
    friend class NodeContext;

    void RunEvents(const std::string& eventType, float deltaSeconds);
    // Runs the nodes along the wire that leaves an Exec output, one after another, until a node does not go on.
    void Follow(PinRef execOutput);
    // Runs one action or event node. Returns the Exec output to go on from, or -1 if the node does not go on.
    int RunNode(const Node& node);
    Value EvaluateInput(const Node& node, const NodePins& pins, int inputIndex);
    Value ReadOutput(PinRef from);
    const std::vector<Value>& EvaluatePure(const Node& node);
    void CountStep();
    float NextRandom();

    const Graph& graph;
    const NodeRegistry& registry;
    ValidationReport report;
    void* host = nullptr;
    uint64_t seed = 1;         // where Start() sets the random numbers back to
    uint64_t randomState = 1;

    std::map<std::string, Value> variables;
    std::map<std::pair<uint32_t, int>, Value> outputs;  // what actions and events last left on their outputs

    // The results of Pure nodes for one run of an action: asking again in the same step gives the same answer and does the work once.
    struct PureResult {
        uint64_t epoch = 0;
        std::vector<Value> values;
    };
    std::map<uint32_t, PureResult> pureResults;
    uint64_t epoch = 0;

    std::vector<std::string> log;
    std::vector<uint32_t> executed;
    std::string runtimeError;
    float currentDelta = 0.0f;
    int stepsLeft = 0;
    int depth = 0;
    int pureDepth = 0;  // how many Pure nodes wait on the ones before them
    bool stopped = false;
};

}  // namespace graph
