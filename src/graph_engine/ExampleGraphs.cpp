#include "graph_engine/ExampleGraphs.h"

#include <stdexcept>

#include "graph/Graph.h"
#include "graph/GraphFile.h"
#include "graph_engine/KeyNames.h"

namespace graph_engine {
namespace {

using graph::PinRef;

// Builds a graph node by node. A wire the registry refuses is a mistake in the example, so it throws.
class ExampleBuilder {
public:
    explicit ExampleBuilder(const graph::NodeRegistry& registry) : registry(registry) {}

    uint32_t Add(const std::string& type, float x, float y) { return graph.AddNode(type, {x, y}); }

    uint32_t AddKeyEvent(const std::string& key, float x, float y) {
        const uint32_t node = Add(kKeyPressedEvent, x, y);
        graph.FindNode(node)->properties[kKeyProperty] = key;
        return node;
    }

    uint32_t AddVariableNode(const std::string& type, const std::string& variable, float x, float y) {
        const uint32_t node = Add(type, x, y);
        graph.FindNode(node)->properties["variable"] = variable;
        return node;
    }

    void Type(uint32_t node, int input, graph::Value value) { graph.FindNode(node)->inputValues[input] = std::move(value); }

    void Wire(uint32_t from, int output, uint32_t to, int input) {
        const std::string why = graph.Connect(registry, {from, output}, {to, input});
        if (!why.empty()) {
            throw std::logic_error("Example graph: " + why);
        }
    }

    graph::Graph graph;

private:
    const graph::NodeRegistry& registry;
};

// The pins used below, by number, for the nodes that are wired many times.
constexpr int kExecIn = 0;
constexpr int kThen = 0;

// One key that runs one engine node that takes a crowd size: it sets the size, then says so on the screen.
void AddCrowdKey(ExampleBuilder& builder, const std::string& key, int people, const std::string& message, float y) {
    const uint32_t event = builder.AddKeyEvent(key, 40.0f, y);
    const uint32_t crowd = builder.Add("characters.set_crowd_size", 320.0f, y);
    builder.Type(crowd, 1, people);
    const uint32_t print = builder.Add("debug.print_on_screen", 620.0f, y);
    builder.Type(print, 1, message);
    builder.Wire(event, kThen, crowd, kExecIn);
    builder.Wire(crowd, kThen, print, kExecIn);
}

// The point six metres in front of the camera, for something to appear where the player looks.
uint32_t AddPointInFrontOfCamera(ExampleBuilder& builder, float x, float y) {
    const uint32_t position = builder.Add("camera.get_position", x, y);
    const uint32_t direction = builder.Add("camera.get_direction", x, y + 120.0f);
    const uint32_t reach = builder.Add("vec3.scale", x + 260.0f, y + 120.0f);
    builder.Type(reach, 1, 6.0f);
    const uint32_t sum = builder.Add("vec3.add", x + 500.0f, y + 60.0f);
    builder.Wire(direction, 0, reach, 0);
    builder.Wire(position, 0, sum, 0);
    builder.Wire(reach, 0, sum, 1);
    return sum;
}

}  // namespace

std::string DayNightCycleGraphText(const graph::NodeRegistry& registry) {
    ExampleBuilder builder(registry);
    constexpr float kSunriseHour = 6.0f;
    constexpr float kHoursPerSecond = 0.5f;  // a whole day in 48 seconds
    builder.graph.Variables().push_back({"Hour", graph::PinType::Float, kSunriseHour});

    const uint32_t start = builder.Add("event.start", 40.0f, 40.0f);
    const uint32_t hello = builder.Add("debug.print_on_screen", 320.0f, 40.0f);
    builder.Type(hello, 1, std::string("Day and night: R starts the day again"));
    builder.Wire(start, kThen, hello, kExecIn);

    const uint32_t tick = builder.Add("event.tick", 40.0f, 240.0f);
    const uint32_t hour = builder.AddVariableNode("variable.get", "Hour", 40.0f, 400.0f);
    const uint32_t advance = builder.Add("math.multiply", 40.0f, 480.0f);
    builder.Type(advance, 1, kHoursPerSecond);
    const uint32_t later = builder.Add("math.add", 320.0f, 400.0f);
    const uint32_t store = builder.AddVariableNode("variable.set", "Hour", 600.0f, 240.0f);
    const uint32_t timeOfDay = builder.Add("world.set_time_of_day", 880.0f, 240.0f);
    builder.Wire(tick, 1, advance, 0);  // the seconds since the last frame, times the hours a second
    builder.Wire(hour, 0, later, 0);
    builder.Wire(advance, 0, later, 1);
    builder.Wire(tick, kThen, store, kExecIn);
    builder.Wire(later, 0, store, 1);
    builder.Wire(store, kThen, timeOfDay, kExecIn);
    builder.Wire(store, 1, timeOfDay, 1);

    const uint32_t restart = builder.AddKeyEvent("R", 40.0f, 620.0f);
    const uint32_t reset = builder.AddVariableNode("variable.set", "Hour", 320.0f, 620.0f);
    builder.Type(reset, 1, kSunriseHour);
    builder.Wire(restart, kThen, reset, kExecIn);
    return graph::SaveGraph(builder.graph);
}

std::string SpawnCrowdOnKeyGraphText(const graph::NodeRegistry& registry) {
    ExampleBuilder builder(registry);
    AddCrowdKey(builder, "N", 15000, "A big crowd", 40.0f);
    AddCrowdKey(builder, "M", 2000, "A small crowd", 200.0f);

    const uint32_t crateKey = builder.AddKeyEvent("C", 40.0f, 360.0f);
    const uint32_t crate = builder.Add("physics.spawn_crate", 960.0f, 360.0f);
    const uint32_t crateSpot = AddPointInFrontOfCamera(builder, 300.0f, 360.0f);
    builder.Wire(crateKey, kThen, crate, kExecIn);
    builder.Wire(crateSpot, 0, crate, 1);

    const uint32_t personKey = builder.AddKeyEvent("B", 40.0f, 680.0f);
    const uint32_t seed = builder.Add("debug.random_seed", 300.0f, 680.0f);
    const uint32_t person = builder.Add("characters.spawn", 960.0f, 680.0f);
    const uint32_t personSpot = AddPointInFrontOfCamera(builder, 300.0f, 780.0f);
    builder.Wire(personKey, kThen, seed, kExecIn);
    builder.Wire(seed, kThen, person, kExecIn);
    builder.Wire(personSpot, 0, person, 1);
    builder.Wire(seed, 1, person, 7);
    return graph::SaveGraph(builder.graph);
}

}  // namespace graph_engine
