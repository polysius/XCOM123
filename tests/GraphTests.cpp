// Tests of the visual graph (no window, no GPU). Like the other test programs there is no test framework: each test is a
// function that calls Check, and main reports every failure it finds.
//
// What the graph must do, and what these tests hold it to: wires that fit and only wires that fit, never a loop, nodes that
// run in the order the wires say with values that come from where the wires say, branches and loops that behave, a run that
// cannot hang the program, clear reports of what is wrong, and a file that gives back the very graph that was saved.
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "graph/Graph.h"
#include "graph/GraphFile.h"
#include "graph/Interpreter.h"
#include "graph/Json.h"
#include "graph/NodeRegistry.h"
#include "graph/Value.h"

namespace {

using namespace graph;

int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

// A graph and a registry with the standard nodes, and short ways to build a graph with them.
struct Scene {
    Graph graph;
    NodeRegistry registry;

    Scene() { RegisterStandardNodes(registry); }

    uint32_t Add(const std::string& type) { return graph.AddNode(type, glm::vec2(0.0f)); }

    // Wires an output pin to an input pin and checks that it was accepted.
    void Wire(uint32_t from, int output, uint32_t to, int input) {
        const std::string why = graph.Connect(registry, {from, output}, {to, input});
        Check(why.empty(), "wiring " + std::to_string(from) + ":" + std::to_string(output) + " to " + std::to_string(to) + ":" + std::to_string(input) + " was refused: " + why);
    }

    // A Start event that leads to the node, which is returned.
    uint32_t AfterStart(const std::string& type) {
        const uint32_t start = Add("event.start");
        const uint32_t node = Add(type);
        Wire(start, 0, node, 0);
        return node;
    }

    void Type(uint32_t node, int input, Value value) { graph.FindNode(node)->inputValues[input] = std::move(value); }
};

// Runs the Start of a graph, and returns what it printed.
std::vector<std::string> RunStart(Scene& scene) {
    Interpreter interpreter(scene.graph, scene.registry);
    interpreter.Start();
    return interpreter.Log();
}

// JSON -----------------------------------------------------------------------------------------

void JsonWritesAndReadsEverything() {
    Json root = Json::MakeObject();
    root.Set("text", Json::FromString("a \"quoted\" line\nwith a tab\t and a back\\slash"));
    root.Set("flag", Json::FromBool(true));
    root.Set("whole", Json::FromNumber(-42));
    root.Set("fraction", Json::FromNumber(0.1f));
    Json list = Json::MakeArray();
    list.Push(Json::FromNumber(1));
    list.Push(Json());
    list.Push(Json::MakeObject());
    root.Set("list", std::move(list));
    for (const int indent : {0, 2}) {
        Json read;
        std::string error;
        Check(Json::Parse(root.Write(indent), read, error), "the text written with indent " + std::to_string(indent) + " reads back: " + error);
        Check(read.Write(indent) == root.Write(indent), "and writes the same text again");
        Check(read.Find("text") != nullptr && read.Find("text")->AsString() == root.Find("text")->AsString(), "with the same text, escapes and all");
        Check(read.Find("fraction") != nullptr && static_cast<float>(read.Find("fraction")->AsNumber()) == 0.1f, "and the float that was written");
    }
    Check(root.Write(2).find("\"whole\": -42") != std::string::npos, "a whole number is written without a point");
}

void JsonRefusesWhatIsNotJsonAndSaysWhere() {
    Json read;
    std::string error;
    Check(!Json::Parse("{\"a\": }", read, error) && error.find("line 1") != std::string::npos, "a missing value is refused with a place: " + error);
    Check(!Json::Parse("{\"a\": 1} extra", read, error), "text after the document is refused");
    Check(!Json::Parse("[1, 2", read, error), "an array that is not closed is refused");
    Check(!Json::Parse("\"abc", read, error), "so is a string that is not");
    Check(!Json::Parse("", read, error), "so is nothing at all");
    Check(!Json::Parse("{\n\n  \"a\" 1}", read, error) && error.find("line 3") != std::string::npos, "the line is counted: " + error);
    std::string deep(200, '[');
    deep += std::string(200, ']');
    Check(!Json::Parse(deep, read, error), "nesting two hundred deep is refused instead of running out of stack");
    Json infinite = Json::FromNumber(std::numeric_limits<double>::infinity());
    Check(infinite.Write(0) == "null", "infinity is written as null, which is valid");
}

// Values ------------------------------------------------------------------------------------------

void ValuesConvertWhereTheyShouldAndOnlyThere() {
    Check(CanConvert(PinType::Bool, PinType::Int) && CanConvert(PinType::Int, PinType::Float) && CanConvert(PinType::Float, PinType::Int), "numbers convert between each other");
    Check(!CanConvert(PinType::Int, PinType::Bool) && !CanConvert(PinType::Float, PinType::Bool), "a number does not become a bool");
    Check(!CanConvert(PinType::Vec3, PinType::Float) && !CanConvert(PinType::Float, PinType::Vec3), "a vector is not a number");
    Check(CanConvert(PinType::Vec3, PinType::String) && CanConvert(PinType::Bool, PinType::String), "anything becomes text");
    Check(!CanConvert(PinType::String, PinType::Float), "text does not become a number");
    Check(!CanConvert(PinType::Exec, PinType::Bool) && !CanConvert(PinType::Bool, PinType::Exec) && CanConvert(PinType::Exec, PinType::Exec), "execution only joins execution");
    Check(std::get<int>(Convert(Value(3.9f), PinType::Int)) == 3 && std::get<int>(Convert(Value(-3.9f), PinType::Int)) == -3, "a decimal loses what is after the point");
    Check(std::get<int>(Convert(Value(1e30f), PinType::Int)) == 2147483647 && std::get<int>(Convert(Value(-1e30f), PinType::Int)) == -2147483647 - 1, "and one that is too big is held at the largest");
    Check(std::get<int>(Convert(Value(std::numeric_limits<float>::quiet_NaN()), PinType::Int)) == 0, "a number that is not a number is 0");
    Check(ToText(Value(0.5f)) == "0.5" && ToText(Value(true)) == "true" && ToText(Value(glm::vec3(1.0f, 2.0f, 3.0f))) == "(1, 2, 3)" && ToText(Value(7)) == "7", "text of values");
    Check(std::get<float>(Convert(Value(std::string("x")), PinType::Float)) == 0.0f, "an impossible conversion gives the default");
}

// The graph and its wires -------------------------------------------------------------------------

void WiresMustFitAndCarryTheRightKindOfThing() {
    Scene scene;
    const uint32_t add = scene.Add("math.add");
    const uint32_t print = scene.Add("debug.print");
    const uint32_t make = scene.Add("vec3.make");
    const uint32_t branch = scene.Add("flow.branch");
    // A number into a text input is fine. A vector into a number input is not.
    Check(scene.graph.Connect(scene.registry, {add, 0}, {print, 1}).empty(), "a number may go into text");
    Check(!scene.graph.Connect(scene.registry, {make, 0}, {add, 0}).empty(), "a vector may not go into a number: wrong output pin, vector is output 0");
    Check(!scene.graph.Connect(scene.registry, {make, 0}, {add, 1}).empty(), "nor into B");
    Check(!scene.graph.Connect(scene.registry, {add, 0}, {branch, 0}).empty(), "data does not go into an execution pin");
    Check(!scene.graph.Connect(scene.registry, {print, 0}, {add, 0}).empty(), "execution does not go into a data pin");
    Check(!scene.graph.Connect(scene.registry, {add, 0}, {add, 0}).empty(), "a node is not wired to itself");
    Check(!scene.graph.Connect(scene.registry, {add, 5}, {print, 1}).empty(), "a pin that is not there is refused");
    Check(!scene.graph.Connect(scene.registry, {999, 0}, {print, 1}).empty(), "so is a node that is not there");
    const size_t wires = scene.graph.Connections().size();
    scene.graph.Connect(scene.registry, {make, 0}, {add, 0});
    Check(scene.graph.Connections().size() == wires, "a refused wire changes nothing");
    Check(!scene.graph.WhyNotConnect(scene.registry, {make, 0}, {add, 0}).empty() && scene.graph.WhyNotConnect(scene.registry, {add, 0}, {print, 1}).empty(), "the editor can ask beforehand");
}

void AnInputTakesOneWireAndAnExecutionOutputLeadsOneWay() {
    Scene scene;
    const uint32_t a = scene.Add("math.add");
    const uint32_t b = scene.Add("math.add");
    const uint32_t c = scene.Add("math.add");
    scene.Wire(a, 0, c, 0);
    scene.Wire(b, 0, c, 0);
    int into = 0;
    for (const Connection& wire : scene.graph.Connections()) {
        into += wire.to == PinRef{c, 0} ? 1 : 0;
    }
    Check(into == 1 && scene.graph.ConnectionInto({c, 0})->from.node == b, "a second wire into an input replaces the first");
    scene.Wire(a, 0, c, 1);
    Check(scene.graph.ConnectionsFrom({a, 0}).size() == 1, "an output may feed several inputs");
    scene.Wire(b, 0, a, 1);
    Check(scene.graph.ConnectionsFrom({b, 0}).size() == 2, "and a, c both take from b");

    const uint32_t start = scene.Add("event.start");
    const uint32_t first = scene.Add("debug.print");
    const uint32_t second = scene.Add("debug.print");
    scene.Wire(start, 0, first, 0);
    scene.Wire(start, 0, second, 0);
    Check(scene.graph.ConnectionsFrom({start, 0}).size() == 1 && scene.graph.ConnectionsFrom({start, 0})[0]->to.node == second, "an execution output leads to one node: the new wire replaces the old");
}

void NoWireMayMakeALoop() {
    Scene scene;
    const uint32_t start = scene.Add("event.start");
    const uint32_t first = scene.Add("debug.print");
    const uint32_t second = scene.Add("debug.print");
    scene.Wire(start, 0, first, 0);
    scene.Wire(first, 0, second, 0);
    Check(!scene.graph.Connect(scene.registry, {second, 0}, {first, 0}).empty(), "execution may not run back to a node that ran before");
    // Pure nodes asking each other for results in a circle would never end.
    const uint32_t a = scene.Add("math.add");
    const uint32_t b = scene.Add("math.multiply");
    scene.Wire(a, 0, b, 0);
    Check(!scene.graph.Connect(scene.registry, {b, 0}, {a, 0}).empty(), "two computing nodes may not feed each other");
    // An action reading its own earlier result is not a loop: the result is whatever it left last time.
    const uint32_t set = scene.Add("variable.set");
    const uint32_t sum = scene.Add("math.add");
    scene.Wire(sum, 0, set, 1);
    Check(scene.graph.Connect(scene.registry, {set, 1}, {sum, 0}).empty(), "an action's result may feed a computation that feeds the action");
}

void RemovingANodeRemovesItsWires() {
    Scene scene;
    const uint32_t start = scene.Add("event.start");
    const uint32_t print = scene.Add("debug.print");
    const uint32_t add = scene.Add("math.add");
    scene.Wire(start, 0, print, 0);
    scene.Wire(add, 0, print, 1);
    Check(scene.graph.RemoveNode(print), "a node is removed");
    Check(scene.graph.Connections().empty(), "and its wires with it");
    Check(!scene.graph.RemoveNode(print) && scene.graph.FindNode(print) == nullptr, "a node that is gone is gone");
    const uint32_t again = scene.Add("debug.print");
    Check(again != print && again != start && again != add, "its number is not used again");
}

// Running ---------------------------------------------------------------------------------------

void ArithmeticFollowsTheWiresAndTheTypedValues() {
    Scene scene;
    const uint32_t print = scene.AfterStart("debug.print");
    const uint32_t add = scene.Add("math.add");
    const uint32_t multiply = scene.Add("math.multiply");
    scene.Type(add, 0, 2.0f);
    scene.Type(add, 1, 3.0f);
    scene.Type(multiply, 1, 4.0f);
    scene.Wire(add, 0, multiply, 0);  // (2 + 3) * 4
    scene.Wire(multiply, 0, print, 1);
    const std::vector<std::string> log = RunStart(scene);
    Check(log.size() == 1 && log[0] == "20", "(2 + 3) * 4 printed " + (log.empty() ? std::string("nothing") : log[0]));
}

void ABranchGoesOneWayAndASequenceInOrder() {
    for (const bool condition : {true, false}) {
        Scene scene;
        const uint32_t branch = scene.AfterStart("flow.branch");
        const uint32_t yes = scene.Add("debug.print");
        const uint32_t no = scene.Add("debug.print");
        scene.Type(branch, 1, condition);
        scene.Type(yes, 1, std::string("yes"));
        scene.Type(no, 1, std::string("no"));
        scene.Wire(branch, 0, yes, 0);
        scene.Wire(branch, 1, no, 0);
        const std::vector<std::string> log = RunStart(scene);
        Check(log.size() == 1 && log[0] == (condition ? "yes" : "no"), std::string("a branch on ") + (condition ? "true" : "false") + " goes the right way");
    }
    Scene scene;
    const uint32_t sequence = scene.AfterStart("flow.sequence");
    for (int i = 0; i < 3; ++i) {
        const uint32_t print = scene.Add("debug.print");
        scene.Type(print, 1, std::string("step ") + std::to_string(i));
        scene.Wire(sequence, i, print, 0);
    }
    const std::vector<std::string> log = RunStart(scene);
    Check(log == std::vector<std::string>({"step 0", "step 1", "step 2"}), "a sequence runs its outputs in order");
}

void AForLoopRunsItsBodyOnceForEveryNumberAndThenCompletes() {
    Scene scene;
    scene.graph.Variables().push_back({"total", PinType::Int, 0});
    const uint32_t loop = scene.AfterStart("flow.for");
    scene.Type(loop, 1, 1);
    scene.Type(loop, 2, 10);
    const uint32_t get = scene.Add("variable.get");
    const uint32_t add = scene.Add("int.add");
    const uint32_t set = scene.Add("variable.set");
    const uint32_t done = scene.Add("debug.print");
    for (const uint32_t node : {get, set}) {
        scene.graph.FindNode(node)->properties["variable"] = std::string("total");
    }
    scene.Wire(get, 0, add, 0);
    scene.Wire(loop, 1, add, 1);  // the index
    scene.Wire(add, 0, set, 1);
    scene.Wire(loop, 0, set, 0);
    scene.Wire(loop, 2, done, 0);
    scene.Wire(get, 0, done, 1);
    const std::vector<std::string> log = RunStart(scene);
    Check(log.size() == 1 && log[0] == "55", "a loop from 1 to 10 adds up to 55 and then completes: " + (log.empty() ? std::string("nothing") : log[0]));

    // A loop that does not run: First above Last.
    Scene empty;
    const uint32_t none = empty.AfterStart("flow.for");
    empty.Type(none, 1, 5);
    empty.Type(none, 2, 1);
    const uint32_t body = empty.Add("debug.print");
    const uint32_t after = empty.Add("debug.print");
    empty.Type(after, 1, std::string("after"));
    empty.Wire(none, 0, body, 0);
    empty.Wire(none, 2, after, 0);
    Check(RunStart(empty) == std::vector<std::string>({"after"}), "a loop with First above Last skips the body and completes");
}

void ARunThatWouldNeverEndIsStoppedAndSaysSo() {
    Scene scene;
    const uint32_t loop = scene.AfterStart("flow.for");
    scene.Type(loop, 1, 0);
    scene.Type(loop, 2, std::numeric_limits<int>::max());
    const uint32_t print = scene.Add("debug.print");
    scene.Wire(loop, 0, print, 0);
    Interpreter interpreter(scene.graph, scene.registry);
    interpreter.Start();
    Check(!interpreter.RuntimeError().empty(), "a loop of two billion steps is stopped: " + interpreter.RuntimeError());
    Check(interpreter.Log().size() < 100000, "after a bounded number of steps: " + std::to_string(interpreter.Log().size()));
    interpreter.Tick(0.016f);  // still usable
    Check(interpreter.CanRun(), "the graph can run again afterwards");

    // Nested a hundred deep: stopped too, without running out of stack.
    Scene deep;
    uint32_t previous = deep.AfterStart("flow.branch");
    deep.Type(previous, 1, true);
    for (int i = 0; i < 100; ++i) {
        const uint32_t next = deep.Add("flow.branch");
        deep.Type(next, 1, true);
        deep.Wire(previous, 0, next, 0);
        previous = next;
    }
    Interpreter nested(deep.graph, deep.registry);
    nested.Start();
    Check(!nested.RuntimeError().empty(), "branches nested a hundred deep are stopped: " + nested.RuntimeError());
}

void EveryFrameEventsGetTheTimeAndVariablesKeepTheirValuesBetweenFrames() {
    Scene scene;
    scene.graph.Variables().push_back({"seconds", PinType::Float, 0.0f});
    const uint32_t tick = scene.Add("event.tick");
    const uint32_t set = scene.Add("variable.set");
    const uint32_t get = scene.Add("variable.get");
    const uint32_t add = scene.Add("math.add");
    scene.graph.FindNode(set)->properties["variable"] = std::string("seconds");
    scene.graph.FindNode(get)->properties["variable"] = std::string("seconds");
    scene.Wire(tick, 0, set, 0);
    scene.Wire(get, 0, add, 0);
    scene.Wire(tick, 1, add, 1);
    scene.Wire(add, 0, set, 1);
    Interpreter interpreter(scene.graph, scene.registry);
    interpreter.Start();
    for (int i = 0; i < 4; ++i) {
        interpreter.Tick(0.25f);
    }
    const Value* seconds = interpreter.FindVariable("seconds");
    Check(seconds != nullptr && std::get<float>(*seconds) == 1.0f, "four frames of a quarter second add up to a second");
    interpreter.Start();
    Check(std::get<float>(*interpreter.FindVariable("seconds")) == 0.0f, "and Start sets the variable back");
}

void ASharedComputationIsWorkedOutOnceForEachStep() {
    // The same random number node feeds both inputs of an addition: the sum is twice that one number.
    Scene scene;
    const uint32_t print = scene.AfterStart("debug.print");
    const uint32_t random = scene.Add("math.random");
    const uint32_t add = scene.Add("math.add");
    scene.Wire(random, 0, add, 0);
    scene.Wire(random, 0, add, 1);
    scene.Wire(add, 0, print, 1);
    const uint32_t print2 = scene.Add("debug.print");
    scene.Wire(print, 0, print2, 0);
    scene.Wire(random, 0, print2, 1);
    const std::vector<std::string> log = RunStart(scene);
    Check(log.size() == 2, "two lines");
    if (log.size() == 2) {
        const float sum = std::stof(log[0]);
        const float again = std::stof(log[1]);
        Check(sum >= 0.0f && sum < 2.0f, "the sum is of two copies of one number");
        Check(again != sum / 2.0f, "and the next step draws a fresh number");
    }
}

void RandomNumbersRepeatForTheSameSeedAndDifferWithAnother() {
    Scene scene;
    const uint32_t print = scene.AfterStart("debug.print");
    const uint32_t random = scene.Add("math.random");
    scene.Wire(random, 0, print, 1);
    Interpreter first(scene.graph, scene.registry);
    Interpreter second(scene.graph, scene.registry);
    Interpreter third(scene.graph, scene.registry);
    first.SetSeed(5);
    second.SetSeed(5);
    third.SetSeed(6);
    first.Start();
    second.Start();
    third.Start();
    Check(first.Log() == second.Log(), "the same seed gives the same number");
    Check(first.Log() != third.Log(), "another seed gives another");
}

void ALoopWithAnEmptyBodyAndALongChainOfComputationsAreStoppedToo() {
    Scene empty;
    const uint32_t loop = empty.AfterStart("flow.for");
    empty.Type(loop, 1, 0);
    empty.Type(loop, 2, std::numeric_limits<int>::max());
    Interpreter emptyBody(empty.graph, empty.registry);
    emptyBody.Start();
    Check(!emptyBody.RuntimeError().empty(), "a loop with nothing in its body is stopped too: " + emptyBody.RuntimeError());

    // A thousand additions, each waiting on the one before: stopped, where it used to run out of stack.
    Scene chain;
    const uint32_t print = chain.AfterStart("debug.print");
    uint32_t previous = chain.Add("math.add");
    chain.Wire(previous, 0, print, 1);
    for (int i = 0; i < 1000; ++i) {
        const uint32_t next = chain.Add("math.add");
        chain.Wire(next, 0, previous, 0);
        previous = next;
    }
    Interpreter chained(chain.graph, chain.registry);
    chained.Start();
    Check(!chained.RuntimeError().empty(), "a very long chain of computing nodes is stopped: " + chained.RuntimeError());
}

void RandomNumbersRepeatWhenAGraphStartsAgain() {
    Scene scene;
    const uint32_t print = scene.AfterStart("debug.print");
    scene.Wire(scene.Add("math.random"), 0, print, 1);
    Interpreter interpreter(scene.graph, scene.registry);
    interpreter.SetSeed(5);
    interpreter.Start();
    const std::vector<std::string> first = interpreter.Log();
    interpreter.Start();
    Check(first == interpreter.Log(), "the second Start gives the numbers of the first");
}

void NumbersThatJsonCannotHoldAreSavedAsTheyWillBeRead() {
    Scene scene;
    const uint32_t add = scene.Add("math.add");
    scene.Type(add, 0, std::numeric_limits<float>::max());
    scene.Type(add, 1, std::numeric_limits<float>::quiet_NaN());
    const std::string text = SaveGraph(scene.graph);
    Graph loaded;
    Check(LoadGraph(text, loaded).ok && SaveGraph(loaded) == text, "the largest float and not-a-number survive a save, a load and a save");
    Check(std::get<float>(loaded.FindNode(add)->inputValues.at(0)) == std::numeric_limits<float>::max(), "the largest float stays the largest float");

    Graph large;
    const LoadResult result = LoadGraph("{\"version\": 1, \"nodes\": [{\"id\": 4294967294, \"type\": \"event.start\"}]}", large);
    Check(result.ok && large.Nodes().empty() && !result.warnings.empty(), "a node id so large that ids could wrap round is left out");
    Check(!LoadGraph("{\"version\": 1e300}", large).ok, "a version far out of range is refused");
}

void UnsafeMathGivesZeroNotACrash() {
    Scene scene;
    const uint32_t print = scene.AfterStart("debug.print");
    const uint32_t chain = scene.Add("debug.print");
    const uint32_t third = scene.Add("debug.print");
    const uint32_t fourth = scene.Add("debug.print");
    scene.Wire(print, 0, chain, 0);
    scene.Wire(chain, 0, third, 0);
    scene.Wire(third, 0, fourth, 0);
    const uint32_t divide = scene.Add("math.divide");
    scene.Type(divide, 0, 5.0f);
    scene.Type(divide, 1, 0.0f);
    scene.Wire(divide, 0, print, 1);
    const uint32_t intDivide = scene.Add("int.divide");
    scene.Type(intDivide, 0, std::numeric_limits<int>::min());
    scene.Type(intDivide, 1, -1);
    scene.Wire(intDivide, 0, chain, 1);
    const uint32_t modulo = scene.Add("int.modulo");
    scene.Type(modulo, 0, 7);
    scene.Type(modulo, 1, 0);
    scene.Wire(modulo, 0, third, 1);
    const uint32_t root = scene.Add("math.sqrt");
    scene.Type(root, 0, -4.0f);
    scene.Wire(root, 0, fourth, 1);
    const std::vector<std::string> log = RunStart(scene);
    Check(log == std::vector<std::string>({"0", "0", "0", "0"}), "5 / 0, INT_MIN / -1, 7 % 0 and the root of -4 are all 0");
    Scene wrap;
    const uint32_t out = wrap.AfterStart("debug.print");
    const uint32_t add = wrap.Add("int.add");
    wrap.Type(add, 0, std::numeric_limits<int>::max());
    wrap.Type(add, 1, 1);
    wrap.Wire(add, 0, out, 1);
    Check(RunStart(wrap) == std::vector<std::string>({"-2147483648"}), "whole numbers wrap round instead of overflowing");
}

void VariablesHaveTypesAndConvertWhatIsSetIntoThem() {
    Scene scene;
    scene.graph.Variables().push_back({"count", PinType::Int, 0});
    scene.graph.Variables().push_back({"where", PinType::Vec3, glm::vec3(1.0f, 2.0f, 3.0f)});
    const uint32_t set = scene.AfterStart("variable.set");
    scene.graph.FindNode(set)->properties["variable"] = std::string("count");
    scene.Type(set, 1, 7.9f);  // a decimal into a whole number variable
    const uint32_t get = scene.Add("variable.get");
    scene.graph.FindNode(get)->properties["variable"] = std::string("where");
    const uint32_t print = scene.Add("debug.print");
    scene.Wire(set, 0, print, 0);
    scene.Wire(get, 0, print, 1);
    Interpreter interpreter(scene.graph, scene.registry);
    interpreter.Start();
    Check(interpreter.CanRun(), "a graph with typed variables can run");
    Check(std::get<int>(*interpreter.FindVariable("count")) == 7, "a decimal set into a whole number variable is cut to 7");
    Check(interpreter.Log() == std::vector<std::string>({"(1, 2, 3)"}), "and a vector variable prints as a vector");
    const NodePins pins = PinsOf(scene.graph, scene.registry, *scene.graph.FindNode(get));
    Check(pins.outputs.size() == 1 && pins.outputs[0].type == PinType::Vec3, "the pin of a variable node has the type of the variable");
}

void TheNodesThatRanAreRecordedForTheEditor() {
    Scene scene;
    const uint32_t start = scene.Add("event.start");
    const uint32_t first = scene.Add("debug.print");
    const uint32_t second = scene.Add("debug.print");
    const uint32_t never = scene.Add("debug.print");
    scene.Wire(start, 0, first, 0);
    scene.Wire(first, 0, second, 0);
    Interpreter interpreter(scene.graph, scene.registry);
    interpreter.Start();
    Check(interpreter.ExecutedNodes() == std::vector<uint32_t>({start, first, second}), "the nodes that ran are listed in the order they ran");
    Check(std::find(interpreter.ExecutedNodes().begin(), interpreter.ExecutedNodes().end(), never) == interpreter.ExecutedNodes().end(), "and the one nothing led to is not there");
    Check(interpreter.LastOutput({first, 0}) == nullptr, "an execution pin has no value");
}

// Checking ---------------------------------------------------------------------------------------

bool HasMessage(const ValidationReport& report, uint32_t node, bool error, const std::string& part) {
    for (const Issue& issue : report) {
        if (issue.node == node && issue.isError == error && issue.message.find(part) != std::string::npos) {
            return true;
        }
    }
    return false;
}

void ValidationSaysWhatIsWrongWithWhichNode() {
    Scene scene;
    const uint32_t start = scene.Add("event.start");
    const uint32_t print = scene.Add("debug.print");
    const uint32_t stray = scene.Add("debug.print");
    const uint32_t alien = scene.Add("no.such.node");
    const uint32_t variable = scene.Add("variable.get");
    scene.Wire(start, 0, print, 0);
    ValidationReport report = Validate(scene.graph, scene.registry);
    Check(HasMessage(report, alien, true, "not known"), "a node of an unknown kind is an error");
    Check(HasMessage(report, variable, true, "variable"), "a variable node with no variable is an error");
    Check(HasMessage(report, stray, false, "never runs"), "an action that nothing leads to is a warning");
    Check(!HasMessage(report, print, false, "never runs"), "and the one that is led to is not");
    Check(HasErrors(report), "so the graph cannot run");
    Interpreter interpreter(scene.graph, scene.registry);
    interpreter.Start();
    Check(interpreter.Log().empty() && interpreter.ExecutedNodes().empty(), "and Start does nothing");

    Scene quiet;
    quiet.Add("debug.print");
    report = Validate(quiet.graph, quiet.registry);
    Check(HasMessage(report, 0, false, "Nothing starts") && !HasErrors(report), "a graph with no event is only a warning, and an empty graph says nothing");
    Check(Validate(Scene().graph, Scene().registry).empty(), "an empty graph has nothing to report");
}

void ValidationCatchesTheWiresThatAFileCanHoldButTheEditorCannotDraw() {
    // Build the damaged graph by hand, the way a damaged file would.
    Scene scene;
    const uint32_t start = scene.Add("event.start");
    const uint32_t print = scene.Add("debug.print");
    const uint32_t add = scene.Add("math.add");
    const uint32_t other = scene.Add("math.add");
    const uint32_t make = scene.Add("vec3.make");
    scene.graph.AddLoadedConnection({{start, 0}, {print, 0}});
    scene.graph.AddLoadedConnection({{add, 0}, {print, 1}});
    scene.graph.AddLoadedConnection({{other, 0}, {print, 1}});  // two wires into one input
    scene.graph.AddLoadedConnection({{make, 0}, {add, 0}});      // a vector into a number
    scene.graph.AddLoadedConnection({{add, 0}, {other, 0}});     // and a loop of computing nodes: add feeds other
    scene.graph.AddLoadedConnection({{other, 0}, {add, 1}});     // other feeds add
    const ValidationReport report = Validate(scene.graph, scene.registry);
    Check(HasMessage(report, print, true, "More than one wire"), "two wires into one input are found");
    Check(HasMessage(report, add, true, "vec3"), "a vector wired to a number is found: and it says what: " + (report.empty() ? std::string("") : report[0].message));
    Check(HasMessage(report, other, true, "loop") || HasMessage(report, add, true, "loop"), "a loop of computing nodes is found");
    Check(!Interpreter(scene.graph, scene.registry).CanRun(), "and the graph will not run");
}

// Files ---------------------------------------------------------------------------------------------

Scene BuildExample() {
    Scene scene;
    scene.graph.Variables().push_back({"total", PinType::Float, 1.5f});
    scene.graph.Variables().push_back({"where", PinType::Vec3, glm::vec3(1.0f, -2.0f, 0.5f)});
    scene.graph.Variables().push_back({"label", PinType::String, std::string("a \"name\"\nwith lines")});
    const uint32_t start = scene.graph.AddNode("event.start", glm::vec2(-120.5f, 40.0f));
    const uint32_t print = scene.graph.AddNode("debug.print", glm::vec2(80.0f, 0.1f));
    const uint32_t add = scene.graph.AddNode("math.add", glm::vec2(-10.0f, 90.0f));
    scene.graph.FindNode(add)->inputValues[0] = 2.5f;
    scene.graph.FindNode(add)->inputValues[1] = 3;
    scene.graph.FindNode(print)->inputValues[1] = std::string("hello");
    const uint32_t get = scene.graph.AddNode("variable.get", glm::vec2(0.0f, 200.0f));
    scene.graph.FindNode(get)->properties["variable"] = std::string("total");
    scene.Wire(start, 0, print, 0);
    scene.Wire(add, 0, print, 1);
    return scene;
}

void ASavedGraphLoadsBackAsTheSameGraphAndTheSameText() {
    const Scene original = BuildExample();
    const std::string text = SaveGraph(original.graph);
    Graph loaded;
    const LoadResult result = LoadGraph(text, loaded);
    Check(result.ok && result.warnings.empty(), "a saved graph loads: " + result.error);
    Check(SaveGraph(loaded) == text, "and saves to the very same text");
    Check(loaded.Nodes().size() == original.graph.Nodes().size() && loaded.Connections().size() == original.graph.Connections().size() &&
              loaded.Variables().size() == original.graph.Variables().size(),
          "with the same nodes, wires and variables");
    const Node* add = loaded.FindNode(3);
    Check(add != nullptr && add->type == "math.add" && std::get<float>(add->inputValues.at(0)) == 2.5f && std::get<int>(add->inputValues.at(1)) == 3, "typed input values keep their types");
    Check(loaded.FindNode(1)->position == glm::vec2(-120.5f, 40.0f) && loaded.FindNode(2)->position == glm::vec2(80.0f, 0.1f), "positions come back exactly");
    Check(std::get<std::string>(loaded.Variables()[2].initial) == "a \"name\"\nwith lines", "a text variable keeps its quotes and lines");
    Check(loaded.AddNode("debug.print", glm::vec2(0.0f)) == original.graph.NextId(), "numbers are not reused after a reload");
}

void TheTextOfAGraphDoesNotDependOnTheOrderItWasBuiltIn() {
    Scene first;
    const uint32_t a = first.Add("event.start");
    const uint32_t b = first.Add("debug.print");
    first.Wire(a, 0, b, 0);
    // The same nodes added to a graph in the opposite order.
    Graph reversed;
    Node second;
    second.id = 2;
    second.type = "debug.print";
    Node firstNode;
    firstNode.id = 1;
    firstNode.type = "event.start";
    reversed.AddLoadedNode(second);
    reversed.AddLoadedNode(firstNode);
    reversed.AddLoadedConnection({{1, 0}, {2, 0}});
    Check(SaveGraph(reversed) == SaveGraph(first.graph), "nodes added in another order save to the same text");
}

void LoadingASpoiledFileKeepsTheGraphAndSaysWhy() {
    const Scene original = BuildExample();
    Graph graph;
    graph = original.graph;
    const std::string before = SaveGraph(graph);
    Check(!LoadGraph("not json at all", graph).ok && SaveGraph(graph) == before, "text that is not JSON is refused and the graph is as it was");
    Check(!LoadGraph("{\"nodes\": []}", graph).ok && SaveGraph(graph) == before, "a file with no version is refused");
    const LoadResult newer = LoadGraph("{\"version\": 99}", graph);
    Check(!newer.ok && newer.error.find("version") != std::string::npos && SaveGraph(graph) == before, "a file of another version is refused, naming the version: " + newer.error);

    const std::string damaged =
        "{\"version\": 1, \"nodes\": [{\"id\": 1, \"type\": \"event.start\", \"x\": 0, \"y\": 0}, {\"id\": 1, \"type\": \"debug.print\"}, {\"type\": \"x\"}, "
        "{\"id\": 5, \"type\": \"the.future.node\"}], \"connections\": [{\"from\": [1, 0], \"to\": [77, 0]}, {\"from\": \"oops\", \"to\": [1, 0]}]}";
    Graph partial;
    const LoadResult result = LoadGraph(damaged, partial);
    Check(result.ok && result.warnings.size() == 4, "a damaged file loads what it can, with a warning for each loss: " + std::to_string(result.warnings.size()));
    Check(partial.Nodes().size() == 2 && partial.FindNode(5) != nullptr && partial.FindNode(5)->type == "the.future.node", "a node of an unknown kind is kept");
    Check(partial.NextId() == 6, "and the next number is above all that were read");
    Check(SaveGraph(partial).find("the.future.node") != std::string::npos, "so that saving again loses nothing");
}

// The library of nodes ---------------------------------------------------------------------------------

void EveryStandardNodeIsWellFormed() {
    NodeRegistry registry;
    RegisterStandardNodes(registry);
    const std::vector<const NodeDescription*> all = registry.All();
    Check(all.size() >= 50, "there are at least fifty standard nodes: " + std::to_string(all.size()));
    std::vector<std::string> ids;
    for (const NodeDescription* node : all) {
        const std::string name = node->id;
        ids.push_back(name);
        Check(!node->title.empty() && !node->category.empty() && !node->tooltip.empty(), name + " has a title, a category and a tooltip");
        bool execInput = false;
        bool execOutput = false;
        for (const PinDescription& pin : node->inputs) {
            execInput = execInput || pin.type == PinType::Exec;
            Check(pin.type == PinType::Exec || TypeOf(pin.defaultValue) == pin.type, name + ": the default of input '" + pin.name + "' has the type of the pin");
            Check(!pin.name.empty(), name + " has a name on every input");
        }
        for (const PinDescription& pin : node->outputs) {
            execOutput = execOutput || pin.type == PinType::Exec;
            Check(!pin.name.empty(), name + " has a name on every output");
        }
        if (node->kind == NodeKind::Pure) {
            Check(!execInput && !execOutput, name + " computes and so has no execution pins");
            Check(static_cast<bool>(node->run), name + " has code to run");
        }
        if (node->kind == NodeKind::Action) {
            Check(!node->inputs.empty() && node->inputs[0].type == PinType::Exec, name + " starts with an execution input");
            Check(execOutput, name + " has an execution output to go on from");
        }
        if (node->kind == NodeKind::Event) {
            Check(!execInput && execOutput, name + " has no way in and a way out");
        }
    }
    std::vector<std::string> sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    Check(ids == sorted && std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end(), "the ids are in order and none is twice");
    NodeDescription replacement;
    replacement.id = "math.add";
    replacement.title = "Changed";
    registry.Register(replacement);
    Check(registry.Find("math.add")->title == "Changed" && registry.All().size() == all.size(), "a kind registered again replaces the first");
    Check(registry.Find("no.such") == nullptr, "an unknown id finds nothing");
}

void EveryStandardPureNodeCanRunOnItsDefaults() {
    NodeRegistry registry;
    RegisterStandardNodes(registry);
    for (const NodeDescription* description : registry.All()) {
        if (description->kind != NodeKind::Pure || description->usesVariable) {
            continue;
        }
        // Print the first output of the node, with every input left as the pin says.
        Graph graph;
        const uint32_t start = graph.AddNode("event.start", glm::vec2(0.0f));
        const uint32_t print = graph.AddNode("debug.print", glm::vec2(0.0f));
        const uint32_t node = graph.AddNode(description->id, glm::vec2(0.0f));
        Check(graph.Connect(registry, {start, 0}, {print, 0}).empty(), "start to print");
        Check(graph.Connect(registry, {node, 0}, {print, 1}).empty(), description->id + " can be wired to a print");
        Interpreter interpreter(graph, registry);
        interpreter.Start();
        Check(interpreter.CanRun() && interpreter.RuntimeError().empty() && interpreter.Log().size() == 1, description->id + " runs on its defaults and prints one line");
    }
}

}  // namespace

int main() {
    struct Test {
        const char* name;
        std::function<void()> run;
    };
    const Test tests[] = {
        {"JsonWritesAndReadsEverything", JsonWritesAndReadsEverything},
        {"JsonRefusesWhatIsNotJsonAndSaysWhere", JsonRefusesWhatIsNotJsonAndSaysWhere},
        {"ValuesConvertWhereTheyShouldAndOnlyThere", ValuesConvertWhereTheyShouldAndOnlyThere},
        {"WiresMustFitAndCarryTheRightKindOfThing", WiresMustFitAndCarryTheRightKindOfThing},
        {"AnInputTakesOneWireAndAnExecutionOutputLeadsOneWay", AnInputTakesOneWireAndAnExecutionOutputLeadsOneWay},
        {"NoWireMayMakeALoop", NoWireMayMakeALoop},
        {"RemovingANodeRemovesItsWires", RemovingANodeRemovesItsWires},
        {"ArithmeticFollowsTheWiresAndTheTypedValues", ArithmeticFollowsTheWiresAndTheTypedValues},
        {"ABranchGoesOneWayAndASequenceInOrder", ABranchGoesOneWayAndASequenceInOrder},
        {"AForLoopRunsItsBodyOnceForEveryNumberAndThenCompletes", AForLoopRunsItsBodyOnceForEveryNumberAndThenCompletes},
        {"ARunThatWouldNeverEndIsStoppedAndSaysSo", ARunThatWouldNeverEndIsStoppedAndSaysSo},
        {"EveryFrameEventsGetTheTimeAndVariablesKeepTheirValuesBetweenFrames", EveryFrameEventsGetTheTimeAndVariablesKeepTheirValuesBetweenFrames},
        {"ASharedComputationIsWorkedOutOnceForEachStep", ASharedComputationIsWorkedOutOnceForEachStep},
        {"RandomNumbersRepeatForTheSameSeedAndDifferWithAnother", RandomNumbersRepeatForTheSameSeedAndDifferWithAnother},
        {"ALoopWithAnEmptyBodyAndALongChainOfComputationsAreStoppedToo", ALoopWithAnEmptyBodyAndALongChainOfComputationsAreStoppedToo},
        {"RandomNumbersRepeatWhenAGraphStartsAgain", RandomNumbersRepeatWhenAGraphStartsAgain},
        {"NumbersThatJsonCannotHoldAreSavedAsTheyWillBeRead", NumbersThatJsonCannotHoldAreSavedAsTheyWillBeRead},
        {"UnsafeMathGivesZeroNotACrash", UnsafeMathGivesZeroNotACrash},
        {"VariablesHaveTypesAndConvertWhatIsSetIntoThem", VariablesHaveTypesAndConvertWhatIsSetIntoThem},
        {"TheNodesThatRanAreRecordedForTheEditor", TheNodesThatRanAreRecordedForTheEditor},
        {"ValidationSaysWhatIsWrongWithWhichNode", ValidationSaysWhatIsWrongWithWhichNode},
        {"ValidationCatchesTheWiresThatAFileCanHoldButTheEditorCannotDraw", ValidationCatchesTheWiresThatAFileCanHoldButTheEditorCannotDraw},
        {"ASavedGraphLoadsBackAsTheSameGraphAndTheSameText", ASavedGraphLoadsBackAsTheSameGraphAndTheSameText},
        {"TheTextOfAGraphDoesNotDependOnTheOrderItWasBuiltIn", TheTextOfAGraphDoesNotDependOnTheOrderItWasBuiltIn},
        {"LoadingASpoiledFileKeepsTheGraphAndSaysWhy", LoadingASpoiledFileKeepsTheGraphAndSaysWhy},
        {"EveryStandardNodeIsWellFormed", EveryStandardNodeIsWellFormed},
        {"EveryStandardPureNodeCanRunOnItsDefaults", EveryStandardPureNodeCanRunOnItsDefaults},
    };
    for (const Test& test : tests) {
        const int failuresBefore = failureCount;
        try {
            test.run();
        } catch (const std::exception& error) {
            ++failureCount;
            std::cerr << "  threw: " << error.what() << "\n";
        }
        std::cout << (failureCount == failuresBefore ? "PASS  " : "FAIL  ") << test.name << "\n";
    }
    std::cout << (failureCount == 0 ? "all graph tests passed\n" : "graph tests failed\n");
    return failureCount == 0 ? 0 : 1;
}
