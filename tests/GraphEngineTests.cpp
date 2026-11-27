// Tests of the engine nodes and the example graphs (no window, no GPU): a graph runs against a host that writes down every call.
//
// What the nodes must do, and what these tests hold them to: every node is described completely, an action calls the host with
// what its inputs say, every argument is finite and inside its limits whatever the graph feeds in, no host means no effect and
// no crash, keys and timers start exactly the events they should, and every example graph that ships loads, is free of problems,
// is the very text the code makes, and does what it says on the mock host.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "graph/Graph.h"
#include "graph/GraphFile.h"
#include "graph/Interpreter.h"
#include "graph/NodeRegistry.h"
#include "graph_engine/EngineHost.h"
#include "graph_engine/EngineNodes.h"
#include "graph_engine/ExampleGraphs.h"
#include "graph_engine/KeyNames.h"
#include "graph_engine/MockEngineHost.h"

#ifndef XCOM_ASSET_DIR
#error "XCOM_ASSET_DIR must name the assets folder of the repository"
#endif

namespace {

using namespace graph_engine;
using graph::Value;

int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

bool Near(float a, float b, float tolerance = 1.0e-3f) {
    return std::abs(a - b) <= tolerance;
}

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInfinity = std::numeric_limits<float>::infinity();

// A registry with every kind of node, a graph with a Start event, and a mock host to run it on.
struct Fixture {
    graph::Graph graph;
    graph::NodeRegistry registry;
    MockEngineHost host;
    std::unique_ptr<graph::Interpreter> interpreter;
    uint32_t start = 0;
    uint32_t last = 0;  // the end of the chain of actions that Start leads to

    Fixture() {
        graph::RegisterStandardNodes(registry);
        RegisterEngineNodes(registry);
        start = graph.AddNode("event.start", {0.0f, 0.0f});
        last = start;
    }

    uint32_t Add(const std::string& type) { return graph.AddNode(type, {0.0f, 0.0f}); }

    void Wire(uint32_t from, int output, uint32_t to, int input) {
        const std::string why = graph.Connect(registry, {from, output}, {to, input});
        Check(why.empty(), "wiring was refused: " + why);
    }

    // Adds an action after the last one, typing values into its inputs (counted with the Exec pin as input 0).
    uint32_t Then(const std::string& type, const std::vector<std::pair<int, Value>>& inputs = {}) {
        const uint32_t node = Add(type);
        for (const auto& [input, value] : inputs) {
            graph.FindNode(node)->inputValues[input] = value;
        }
        Wire(last, 0, node, 0);
        last = node;
        return node;
    }

    // Makes the interpreter, hands it the host (or none), and runs Start.
    graph::Interpreter& Run(bool withHost = true) {
        interpreter = std::make_unique<graph::Interpreter>(graph, registry);
        if (withHost) {
            interpreter->SetHost(static_cast<EngineHost*>(&host));
        }
        interpreter->Start();
        return *interpreter;
    }
};

// The ids of the kinds that RegisterEngineNodes adds to the standard ones.
std::set<std::string> EngineNodeIds() {
    graph::NodeRegistry standard;
    graph::RegisterStandardNodes(standard);
    graph::NodeRegistry all;
    graph::RegisterStandardNodes(all);
    RegisterEngineNodes(all);
    std::set<std::string> ids;
    for (const graph::NodeDescription* description : all.All()) {
        if (standard.Find(description->id) == nullptr) {
            ids.insert(description->id);
        }
    }
    return ids;
}

// ----- The set of nodes -----------------------------------------------------------------------------------------------

void EveryEngineNodeIsWellFormed() {
    Fixture fixture;
    const std::set<std::string> ids = EngineNodeIds();
    Check(ids.size() >= 26, "the engine adds at least 26 kinds of node (" + std::to_string(ids.size()) + ")");

    const std::set<std::string> categories = {"World", "Camera", "Characters", "Physics", "Water", "Debug", "Events"};
    std::set<std::string> titles;
    std::set<std::string> seenCategories;
    for (const std::string& id : ids) {
        const graph::NodeDescription& node = *fixture.registry.Find(id);
        Check(!node.title.empty() && !node.tooltip.empty(), id + " has a title and a tooltip");
        Check(titles.insert(node.title).second, id + " has a title of its own (" + node.title + ")");
        Check(categories.count(node.category) == 1, id + " is in a known category (" + node.category + ")");
        seenCategories.insert(node.category);
        Check((node.kind == graph::NodeKind::Event) == (node.category == "Events"), id + " is an event exactly when it is in Events");

        const auto checkPins = [&](const std::vector<graph::PinDescription>& pins, const char* side) {
            std::set<std::string> names;
            for (const graph::PinDescription& pin : pins) {
                Check(!pin.name.empty() && names.insert(pin.name).second, id + " has a distinct name for each " + side + " pin");
                if (pin.type != graph::PinType::Exec) {
                    Check(graph::TypeOf(pin.defaultValue) == pin.type, id + " " + pin.name + " has a default of the type of its pin");
                    Check(!pin.tooltip.empty() || std::string(side) == "output" || node.kind == graph::NodeKind::Event, id + " " + pin.name + " says what it is for");
                }
            }
        };
        checkPins(node.inputs, "input");
        checkPins(node.outputs, "output");

        if (node.kind == graph::NodeKind::Action) {
            Check(!node.inputs.empty() && node.inputs.front().type == graph::PinType::Exec, id + " takes Exec first");
            Check(!node.outputs.empty() && node.outputs.front().type == graph::PinType::Exec, id + " goes on by Exec first");
            Check(static_cast<bool>(node.run), id + " has code to run");
        } else if (node.kind == graph::NodeKind::Pure) {
            Check(static_cast<bool>(node.run), id + " has code to run");
            Check(!node.outputs.empty(), id + " gives a value");
            for (const graph::PinDescription& pin : node.inputs) {
                Check(pin.type != graph::PinType::Exec, id + " has no Exec pin");
            }
        } else {
            Check(node.inputs.empty() && !node.outputs.empty() && node.outputs.front().type == graph::PinType::Exec, id + " is an event: no way in, Exec out");
        }
        for (const graph::PropertyDescription& property : node.properties) {
            Check(graph::TypeOf(property.defaultValue) == property.type && !property.tooltip.empty(), id + " setting " + property.name + " is described");
            if (!property.choices.empty()) {
                Check(std::find(property.choices.begin(), property.choices.end(), std::get<std::string>(property.defaultValue)) != property.choices.end(),
                      id + " default is among the choices of " + property.name);
            }
        }
        if (!node.timerSecondsProperty.empty()) {
            Check(node.kind == graph::NodeKind::Event && node.properties.size() == 1 && node.properties[0].name == node.timerSecondsProperty &&
                      node.properties[0].type == graph::PinType::Float,
                  id + " is a timer with a Float setting for its seconds");
        }
    }
    for (const char* category : kEngineCategories) {
        Check(seenCategories.count(category) == 1, std::string("some node is in ") + category);
    }
}

void EveryNodeThatChangesTheEngineIsAnAction() {
    // A Pure node may run any number of times, or not at all, in one step of a graph: it must only read.
    Fixture fixture;
    for (const std::string& id : EngineNodeIds()) {
        const graph::NodeDescription& node = *fixture.registry.Find(id);
        if (node.kind != graph::NodeKind::Pure) {
            continue;
        }
        Fixture run;
        const uint32_t reader = run.Add(id);
        const uint32_t print = run.Then("debug.print");
        run.Wire(reader, 0, print, 1);
        run.Run();
        Check(run.host.Calls().empty(), id + " is Pure and asks the host nothing that changes it");
        Check(run.interpreter->Log().size() == 1, id + " gives a value that can be printed");
    }
}

void EveryActionRunsOnItsDefaultsAndLeavesTheHostInRange() {
    for (const std::string& id : EngineNodeIds()) {
        Fixture fixture;
        if (fixture.registry.Find(id)->kind != graph::NodeKind::Action) {
            continue;
        }
        fixture.Then(id);
        graph::Interpreter& interpreter = fixture.Run();
        Check(interpreter.CanRun() && interpreter.RuntimeError().empty(), id + " runs on its defaults");
        for (const RecordedCall& call : fixture.host.Calls()) {
            for (float number : call.numbers) {
                Check(std::isfinite(number), id + " passes finite numbers to " + call.name);
            }
        }
    }
}

// ----- What the actions pass on -----------------------------------------------------------------------------------------

void ActionsCallTheHostWithWhatTheirInputsSay() {
    Fixture fixture;
    fixture.Then("world.set_sun_elevation", {{1, 40.0f}});
    fixture.Then("world.set_sun_azimuth", {{1, 100.0f}});
    fixture.Then("world.set_time_of_day", {{1, 18.5f}});
    fixture.Then("camera.set_position", {{1, glm::vec3(10.0f, 20.0f, 30.0f)}});
    fixture.Then("camera.set_direction", {{1, glm::vec3(0.0f, 3.0f, 0.0f)}});
    fixture.Then("camera.teleport", {{1, glm::vec3(-5.0f, 6.0f, 7.0f)}, {2, glm::vec3(2.0f, 0.0f, 0.0f)}});
    fixture.Then("characters.set_crowd_size", {{1, 123}});
    fixture.Then("water.set_level", {{1, 1.5f}});
    fixture.Then("water.add_wave", {{1, glm::vec3(1.0f, 2.0f, 3.0f)}, {2, 0.5f}, {3, 4.0f}});
    fixture.Then("debug.print_on_screen", {{1, std::string("hi there")}});
    const uint32_t crate = fixture.Then("physics.spawn_crate", {{1, glm::vec3(1.0f, 2.0f, 3.0f)}});
    fixture.Then("physics.spawn_barrel", {{1, glm::vec3(4.0f, 5.0f, 6.0f)}});
    const uint32_t ball = fixture.Then("physics.spawn_ball", {{1, glm::vec3(7.0f, 8.0f, 9.0f)}});
    fixture.Then("physics.apply_impulse", {{2, glm::vec3(0.0f, 30.0f, 0.0f)}});
    fixture.Then("physics.remove_prop");
    graph::Interpreter& interpreter = fixture.Run();
    const MockEngineHost& host = fixture.host;

    Check(interpreter.RuntimeError().empty(), "the run did not stop early");
    Check(host.LastOf("SetSunElevation") && Near(host.LastOf("SetSunElevation")->numbers[0], 40.0f), "sun elevation 40");
    Check(host.LastOf("SetSunAzimuth") && Near(host.LastOf("SetSunAzimuth")->numbers[0], 100.0f), "sun direction 100");
    Check(host.LastOf("SetTimeOfDay") && Near(host.LastOf("SetTimeOfDay")->numbers[0], 18.5f), "time of day 18.5");
    Check(host.LastOf("SetCameraPosition") && host.LastOf("SetCameraPosition")->numbers == std::vector<float>({10.0f, 20.0f, 30.0f}), "camera position");
    Check(host.LastOf("SetCameraDirection") && host.LastOf("SetCameraDirection")->numbers == std::vector<float>({0.0f, 1.0f, 0.0f}), "camera direction is made a unit vector");
    Check(host.LastOf("TeleportCamera") && host.LastOf("TeleportCamera")->numbers == std::vector<float>({-5.0f, 6.0f, 7.0f, 1.0f, 0.0f, 0.0f}), "teleport");
    Check(host.LastOf("SetCrowdSize") && host.LastOf("SetCrowdSize")->numbers[0] == 123.0f, "crowd size 123");
    Check(host.LastOf("SetRiverLevel") && Near(host.LastOf("SetRiverLevel")->numbers[0], 1.5f), "river level");
    Check(host.LastOf("AddRiverWave") && host.LastOf("AddRiverWave")->numbers == std::vector<float>({1.0f, 2.0f, 3.0f, 0.5f, 4.0f}), "a wave");
    Check(host.LastOf("PrintToScreen") && host.LastOf("PrintToScreen")->text == "hi there", "text on the screen");
    Check(!interpreter.Log().empty() && interpreter.Log().back() == "hi there", "the text is also in the log of the run");

    const std::vector<RecordedCall>& calls = host.Calls();
    const auto spawns = [&] {
        std::vector<const RecordedCall*> found;
        for (const RecordedCall& call : calls) {
            if (call.name == "SpawnProp") {
                found.push_back(&call);
            }
        }
        return found;
    }();
    Check(spawns.size() == 3 && spawns[0]->text == "crate" && spawns[1]->text == "barrel" && spawns[2]->text == "ball", "a crate, a barrel and a ball were spawned in turn");
    Check(spawns.size() == 3 && spawns[1]->numbers == std::vector<float>({static_cast<float>(PropKind::Barrel), 4.0f, 5.0f, 6.0f}), "the barrel is where it was asked to be");
    // The ids the host gave come out of the nodes on the Prop Id pin.
    const graph::Value* crateId = interpreter.LastOutput({crate, 1});
    const graph::Value* ballId = interpreter.LastOutput({ball, 1});
    Check(crateId != nullptr && std::get<int>(*crateId) == 1 && ballId != nullptr && std::get<int>(*ballId) == 3, "the Prop Id pins hold the ids the host gave");
    // Prop Id 0 (nothing wired) is no prop, so neither the impulse nor the removal reaches the host.
    Check(host.CountOf("ApplyImpulse") == 0 && host.CountOf("RemoveProp") == 0, "an id of 0 is not passed on");
}

void ImpulsesAndRemovalsUseTheIdOfTheSpawn() {
    Fixture fixture;
    const uint32_t spawn = fixture.Then("physics.spawn_ball", {{1, glm::vec3(0.0f, 5.0f, 0.0f)}});
    const uint32_t kick = fixture.Then("physics.apply_impulse", {{2, glm::vec3(0.0f, 30.0f, 0.0f)}});
    const uint32_t remove = fixture.Then("physics.remove_prop");
    fixture.Wire(spawn, 1, kick, 1);
    fixture.Wire(spawn, 1, remove, 1);
    graph::Interpreter& interpreter = fixture.Run();
    const RecordedCall* impulse = fixture.host.LastOf("ApplyImpulse");
    Check(impulse != nullptr && impulse->numbers == std::vector<float>({1.0f, 0.0f, 30.0f, 0.0f}), "the impulse goes to the prop that was spawned");
    const RecordedCall* removal = fixture.host.LastOf("RemoveProp");
    Check(removal != nullptr && removal->numbers[0] == 1.0f, "so does the removal");
    const graph::Value* removed = interpreter.LastOutput({remove, 1});
    Check(removed != nullptr && std::get<bool>(*removed), "Removed says true");
}

void ASpawnTheHostRefusesGivesIdZero() {
    Fixture fixture;
    fixture.host.refuseSpawns = true;
    const uint32_t crate = fixture.Then("physics.spawn_crate");
    const uint32_t person = fixture.Then("characters.spawn");
    graph::Interpreter& interpreter = fixture.Run();
    Check(std::get<int>(*interpreter.LastOutput({crate, 1})) == 0, "no crate, Prop Id 0");
    Check(std::get<int>(*interpreter.LastOutput({person, 1})) == 0, "no person, Character Id 0");
}

void ACharacterIsSpawnedWithTheNumbersOnItsPins() {
    Fixture fixture;
    fixture.Then("characters.spawn", {{1, glm::vec3(3.0f, 0.0f, 4.0f)}, {2, 1.8f}, {3, 80.0f}, {4, 45.0f}, {5, 1.0f}, {6, 0.7f}, {7, 99}, {8, std::string("suit")}});
    fixture.Run();
    const RecordedCall* call = fixture.host.LastOf("SpawnCharacter");
    Check(call != nullptr && call->numbers == std::vector<float>({1.8f, 80.0f, 45.0f, 1.0f, 0.7f, 99.0f, 3.0f, 0.0f, 4.0f}), "every number of the person");
    Check(call != nullptr && call->text == "suit", "the outfit");
}

// ----- Safety ---------------------------------------------------------------------------------------------------------

void ArgumentsAreMadeFiniteAndKeptInRange() {
    Fixture fixture;
    fixture.Then("world.set_sun_elevation", {{1, 1000.0f}});
    fixture.Then("world.set_sun_elevation", {{1, -1000.0f}});
    fixture.Then("world.set_sun_elevation", {{1, kNaN}});
    fixture.Then("world.set_sun_azimuth", {{1, 725.0f}});
    fixture.Then("world.set_sun_azimuth", {{1, -90.0f}});
    fixture.Then("world.set_sun_azimuth", {{1, kInfinity}});
    fixture.Then("world.set_time_of_day", {{1, 25.0f}});
    fixture.Then("world.set_time_of_day", {{1, -1.0f}});
    fixture.Then("world.set_time_of_day", {{1, kNaN}});
    fixture.Then("characters.set_crowd_size", {{1, 2000000000}});
    fixture.Then("characters.set_crowd_size", {{1, -5}});
    fixture.Then("water.set_level", {{1, 99.0f}});
    fixture.Then("water.set_level", {{1, -99.0f}});
    fixture.Then("water.add_wave", {{2, 99.0f}, {3, 0.0f}});
    fixture.Then("water.add_wave", {{2, -1.0f}, {3, 1000.0f}});
    fixture.Then("debug.print_on_screen", {{1, std::string(1000, 'x')}});
    fixture.Run();

    // The first number of every call, by method.
    std::map<std::string, std::vector<float>> firstNumbers;
    for (const RecordedCall& call : fixture.host.Calls()) {
        if (!call.numbers.empty()) {
            firstNumbers[call.name].push_back(call.numbers[0]);
        }
    }
    const std::vector<float>& elevations = firstNumbers["SetSunElevation"];
    const std::vector<float>& azimuths = firstNumbers["SetSunAzimuth"];
    const std::vector<float>& hours = firstNumbers["SetTimeOfDay"];
    const std::vector<float>& crowds = firstNumbers["SetCrowdSize"];
    const std::vector<float>& levels = firstNumbers["SetRiverLevel"];
    Check(elevations == std::vector<float>({kSunElevationMax, kSunElevationMin, 24.0f}), "the sun is kept between -5 and 85 degrees; no number gives 24");
    Check(azimuths.size() == 3 && Near(azimuths[0], 5.0f) && Near(azimuths[1], 270.0f) && Near(azimuths[2], 230.0f), "the direction of the sun wraps round the circle");
    Check(hours.size() == 3 && Near(hours[0], 1.0f) && Near(hours[1], 23.0f) && Near(hours[2], 12.0f), "the hour wraps round the day");
    Check(crowds == std::vector<float>({static_cast<float>(kMaxCrowdSize), 0.0f}), "the crowd is kept between 0 and its largest size");
    Check(levels == std::vector<float>({kRiverLevelRange, -kRiverLevelRange}), "the river is kept within 5 metres of still");

    const std::vector<RecordedCall>& calls = fixture.host.Calls();
    std::vector<const RecordedCall*> waves;
    for (const RecordedCall& call : calls) {
        if (call.name == "AddRiverWave") {
            waves.push_back(&call);
        }
    }
    Check(waves.size() == 2 && waves[0]->numbers[3] == kWaveAmplitudeMax && waves[0]->numbers[4] == kWaveRadiusMin, "a wave is kept to 2 metres high and at least 0.5 wide");
    Check(waves.size() == 2 && waves[1]->numbers[3] == 0.0f && waves[1]->numbers[4] == kWaveRadiusMax, "and at most 20 wide, and never below 0");
    Check(fixture.host.LastOf("PrintToScreen") != nullptr && fixture.host.LastOf("PrintToScreen")->text.size() == kMaxTextLength, "text is cut to 200 characters");
}

void PositionsDirectionsAndImpulsesAreKeptSafe() {
    Fixture fixture;
    fixture.Then("camera.set_position", {{1, glm::vec3(1.0e9f, -1.0e9f, kNaN)}});
    fixture.Then("camera.set_direction", {{1, glm::vec3(0.0f)}});
    fixture.Then("camera.set_direction", {{1, glm::vec3(kNaN, 1.0f, 0.0f)}});
    fixture.Then("camera.set_direction", {{1, glm::vec3(0.0f, 0.0f, 1.0e30f)}});
    fixture.Then("camera.teleport", {{1, glm::vec3(kInfinity, 0.0f, 0.0f)}, {2, glm::vec3(0.0f)}});
    const uint32_t spawn = fixture.Then("physics.spawn_ball", {{1, glm::vec3(0.0f, 5000.0f, 99999.0f)}});
    const uint32_t big = fixture.Then("physics.apply_impulse", {{2, glm::vec3(1000.0f, 0.0f, 0.0f)}});
    const uint32_t bad = fixture.Then("physics.apply_impulse", {{2, glm::vec3(kNaN, 0.0f, 0.0f)}});
    fixture.Wire(spawn, 1, big, 1);
    fixture.Wire(spawn, 1, bad, 1);
    fixture.Run();
    const MockEngineHost& host = fixture.host;

    const RecordedCall* position = host.LastOf("SetCameraPosition");
    Check(position != nullptr && position->numbers == std::vector<float>({kWorldHalfExtent, kWorldHeightMin, 3.0f}),
          "a huge position is kept in the world, and a number that is not one keeps the camera's own coordinate");
    Check(host.CountOf("SetCameraDirection") == 1, "a direction of zero, or not a number, changes nothing");
    const RecordedCall* direction = host.LastOf("SetCameraDirection");
    Check(direction != nullptr && direction->numbers == std::vector<float>({0.0f, 0.0f, 1.0f}), "a huge direction is made a unit vector");
    const RecordedCall* teleport = host.LastOf("TeleportCamera");
    Check(teleport != nullptr && teleport->numbers == std::vector<float>({1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}),
          "a teleport keeps the camera's own coordinate where a number is not one, and the way it looks where the direction is nothing");
    const RecordedCall* prop = host.LastOf("SpawnProp");
    Check(prop != nullptr && prop->numbers[2] == kWorldHeightMax &&prop->numbers[3] == kWorldHalfExtent, "a prop is spawned inside the world");
    int impulses = 0;
    for (const RecordedCall& call : host.Calls()) {
        if (call.name == "ApplyImpulse") {
            ++impulses;
            const glm::vec3 push(call.numbers[1], call.numbers[2], call.numbers[3]);
            Check(Near(glm::length(push), kMaxImpulse, 0.01f), "a strong impulse is limited to " + std::to_string(kMaxImpulse));
        }
    }
    Check(impulses == 1, "an impulse that is not a number is not sent");
}

void CharactersAreSpawnedWithinTheLimitsOfTheGenerator() {
    Fixture fixture;
    fixture.Then("characters.spawn", {{1, glm::vec3(0.0f)}, {2, 9.0f}, {3, 1.0f}, {4, 2.0f}, {5, 7.0f}, {6, -1.0f}, {8, std::string(100, 'o')}});
    fixture.Then("characters.spawn", {{2, kNaN}, {3, kInfinity}, {4, kNaN}, {5, kNaN}, {6, kNaN}});
    fixture.Run();
    const std::vector<RecordedCall>& calls = fixture.host.Calls();
    Check(calls.size() == 2, "two people were asked for");
    if (calls.size() == 2) {
        Check(calls[0].numbers[0] == kStatureMax && calls[0].numbers[1] == kMassMin && calls[0].numbers[2] == kAgeMin && calls[0].numbers[3] == 1.0f && calls[0].numbers[4] == 0.0f,
              "stature, mass, age, sex and muscularity are kept in range");
        Check(calls[0].text.size() == kMaxOutfitLength, "the outfit is cut short");
        Check(calls[1].numbers[0] == 1.70f && calls[1].numbers[1] == 70.0f && calls[1].numbers[2] == 30.0f && calls[1].numbers[3] == 0.5f && calls[1].numbers[4] == 0.4f,
              "numbers that are not numbers give the defaults");
    }
}

void WithoutAHostNothingHappensAndNothingFails() {
    for (const std::string& id : EngineNodeIds()) {
        Fixture fixture;
        if (fixture.registry.Find(id)->kind != graph::NodeKind::Action) {
            continue;
        }
        fixture.Then(id);
        graph::Interpreter& interpreter = fixture.Run(false);
        Check(interpreter.CanRun() && interpreter.RuntimeError().empty(), id + " runs without a host");
        Check(fixture.host.Calls().empty(), id + " does nothing to a host it was not given");
    }
    Fixture fixture;
    const uint32_t position = fixture.Add("camera.get_position");
    const uint32_t print = fixture.Then("debug.print");
    fixture.Wire(position, 0, print, 1);
    const uint32_t seed = fixture.Then("debug.random_seed");
    const uint32_t next = fixture.Then("debug.random_seed");
    graph::Interpreter& interpreter = fixture.Run(false);
    Check(interpreter.Log().size() == 1 && interpreter.Log()[0] == graph::ToText(glm::vec3(0.0f)), "a reader without a host gives the default of its pin");
    const int first = std::get<int>(*interpreter.LastOutput({seed, 1}));
    const int second = std::get<int>(*interpreter.LastOutput({next, 1}));
    Check(first >= 0 && second >= 0 && first != second, "with no host the seed comes from the graph's own generator, a new one each time");
}

void ReadersAnswerWithWhatTheHostSays() {
    Fixture fixture;
    fixture.host.cameraPosition = {7.0f, 8.0f, 9.0f};
    fixture.host.playerPosition = {1.0f, 0.0f, -1.0f};
    fixture.host.crowdSize = 4321;
    fixture.host.frameSeconds = 0.25f;
    fixture.host.framesPerSecond = 4.0f;
    const std::vector<std::pair<std::string, Value>> expected = {
        {"camera.get_position", glm::vec3(7.0f, 8.0f, 9.0f)},
        {"camera.get_direction", glm::vec3(0.0f, 0.0f, 1.0f)},
        {"characters.get_player_position", glm::vec3(1.0f, 0.0f, -1.0f)},
        {"characters.get_crowd_size", 4321},
        {"debug.get_frame_time", 0.25f},
        {"debug.get_fps", 4.0f},
    };
    for (const auto& [id, value] : expected) {
        const uint32_t reader = fixture.Add(id);
        const uint32_t print = fixture.Then("debug.print");
        fixture.Wire(reader, 0, print, 1);
        fixture.Run();
        Check(!fixture.interpreter->Log().empty() && fixture.interpreter->Log().back() == graph::ToText(value), id + " gives what the host says");
    }
}

// ----- Events ---------------------------------------------------------------------------------------------------------

// A graph with one key event per name, each printing its own name.
struct KeyGraph {
    Fixture fixture;

    explicit KeyGraph(const std::vector<std::string>& keys) {
        for (const std::string& key : keys) {
            const uint32_t event = fixture.Add(kKeyPressedEvent);
            fixture.graph.FindNode(event)->properties[kKeyProperty] = key;
            const uint32_t print = fixture.Add("debug.print");
            fixture.graph.FindNode(print)->inputValues[1] = key;
            fixture.Wire(event, 0, print, 0);
        }
        fixture.Run();
    }
};

void AKeyRunsOnlyTheEventsThatWaitForIt() {
    KeyGraph keys({"A", "B", "F5"});
    graph::Interpreter& interpreter = *keys.fixture.interpreter;
    interpreter.FireEvent(kKeyPressedEvent, KeyFilter("A"));
    Check(interpreter.Log() == std::vector<std::string>({"A"}), "A runs only the event for A");
    interpreter.FireEvent(kKeyPressedEvent, KeyFilter("f5"));
    Check(interpreter.Log() == std::vector<std::string>({"A", "F5"}), "the name of a key is not case sensitive");
    interpreter.FireEvent(kKeyPressedEvent, KeyFilter("Z"));
    Check(interpreter.Log().size() == 2, "a key nobody waits for runs nothing");
    interpreter.FireEvent(kKeyPressedEvent, {});
    Check(interpreter.Log().size() == 5, "no filter runs every event of the kind");
    Check(interpreter.ExecutedNodes().size() == 6, "the nodes that ran are recorded for the editor: three events and three prints");
}

void AKeyEventWithNoKeyWaitsForTheDefault() {
    Fixture fixture;
    const uint32_t event = fixture.Add(kKeyPressedEvent);  // as the palette adds it: no property
    const uint32_t print = fixture.Add("debug.print");
    fixture.Wire(event, 0, print, 0);
    graph::Interpreter& interpreter = fixture.Run();
    interpreter.FireEvent(kKeyPressedEvent, KeyFilter("A"));
    Check(interpreter.Log().empty(), "not A");
    interpreter.FireEvent(kKeyPressedEvent, KeyFilter(kDefaultKey));
    Check(interpreter.Log().size() == 1, std::string("the default key is ") + kDefaultKey);
}

// A graph with an Every N Seconds event (and nothing else) that prints.
std::unique_ptr<Fixture> TimerGraph(const std::string& type, float seconds) {
    auto fixture = std::make_unique<Fixture>();
    const uint32_t event = fixture->Add(type);
    fixture->graph.FindNode(event)->properties["Seconds"] = seconds;
    const uint32_t print = fixture->Add("debug.print");
    fixture->Wire(event, 0, print, 0);
    fixture->Run();
    return fixture;
}

void EveryNSecondsRunsOnceEachTimeThePeriodHasPassed() {
    const auto fixture = TimerGraph("event.every_n_seconds", 0.5f);
    graph::Interpreter& interpreter = *fixture->interpreter;
    for (int tick = 1; tick <= 3; ++tick) {
        interpreter.Tick(0.125f);
    }
    Check(interpreter.Log().empty(), "not yet after 0.375 seconds");
    interpreter.Tick(0.125f);
    Check(interpreter.Log().size() == 1, "once at 0.5 seconds");
    for (int tick = 1; tick <= 8; ++tick) {
        interpreter.Tick(0.125f);
    }
    Check(interpreter.Log().size() == 3, "and again each half second (3 runs in 1.5 seconds)");
}

void ATimerRunsAtMostOncePerTickWhateverTheFrameLength() {
    const auto fixture = TimerGraph("event.every_n_seconds", 0.5f);
    graph::Interpreter& interpreter = *fixture->interpreter;
    interpreter.Tick(100.0f);
    Check(interpreter.Log().size() == 1, "a frame of 100 seconds runs the event once, not 200 times");
    interpreter.Tick(0.0f);
    Check(interpreter.Log().size() == 1, "and the time it skipped is dropped");
    interpreter.Tick(kNaN);
    interpreter.Tick(-5.0f);
    Check(interpreter.Log().size() == 1, "a frame time that is not a time counts as none");
}

void ATimerCannotBeAskedForEveryFrame() {
    const auto fixture = TimerGraph("event.every_n_seconds", 0.0f);
    graph::Interpreter& interpreter = *fixture->interpreter;
    for (int tick = 0; tick < 40; ++tick) {
        interpreter.Tick(0.025f);
    }
    Check(interpreter.Log().size() == 20, "0 seconds is taken as 0.05: one run every two ticks of 0.025 (" + std::to_string(interpreter.Log().size()) + ")");
}

void AfterNSecondsRunsOnceAndStartAgainStartsTheClockAgain() {
    const auto fixture = TimerGraph("event.timer", 0.25f);
    graph::Interpreter& interpreter = *fixture->interpreter;
    interpreter.Tick(0.125f);
    Check(interpreter.Log().empty(), "not after 0.125 seconds");
    interpreter.Tick(0.125f);
    Check(interpreter.Log().size() == 1, "once at 0.25 seconds");
    for (int tick = 0; tick < 20; ++tick) {
        interpreter.Tick(0.125f);
    }
    Check(interpreter.Log().size() == 1, "never again");
    interpreter.Start();
    Check(interpreter.Log().empty(), "starting again clears the log");
    interpreter.Tick(0.125f);
    interpreter.Tick(0.125f);
    Check(interpreter.Log().size() == 1, "and the clock runs again from 0: once more at 0.25 seconds");
}

void ASettingThatDoesNotFitIsReportedAndAMissingOneUsesTheDefault() {
    Fixture fixture;
    const uint32_t key = fixture.Add(kKeyPressedEvent);
    fixture.graph.FindNode(key)->properties[kKeyProperty] = std::string("Banana");
    const uint32_t timer = fixture.Add("event.every_n_seconds");
    fixture.graph.FindNode(timer)->properties["Seconds"] = std::string("often");
    const graph::ValidationReport report = graph::Validate(fixture.graph, fixture.registry);
    int onKey = 0;
    int onTimer = 0;
    for (const graph::Issue& issue : report) {
        Check(!issue.isError, "a bad setting is a warning, not an error: " + issue.message);
        onKey += issue.node == key && issue.message.find("Banana") != std::string::npos;
        onTimer += issue.node == timer && issue.message.find("Seconds") != std::string::npos;
    }
    Check(onKey == 1, "'Banana' is not a key");
    Check(onTimer == 1, "'often' is not a number of seconds");
    Check(!graph::HasErrors(report), "the graph can still run");

    const graph::NodeDescription& description = *fixture.registry.Find("event.every_n_seconds");
    graph::Node bare;
    Check(std::get<float>(graph::PropertyValue(bare, description, "Seconds")) == 1.0f, "a node with no setting has the default");
    Check(std::get<float>(graph::PropertyValue(*fixture.graph.FindNode(timer), description, "Seconds")) == 1.0f, "so has one whose setting is of the wrong type");
}

// ----- Saving ---------------------------------------------------------------------------------------------------------

void AGraphThatUsesEngineNodesSavesAndLoadsBackTheSame() {
    Fixture fixture;
    // One node of every engine kind, with its settings changed so that a lost value shows.
    for (const std::string& id : EngineNodeIds()) {
        const uint32_t node = fixture.graph.AddNode(id, {static_cast<float>(id.size()), 1.0f});
        if (id == kKeyPressedEvent) {
            fixture.graph.FindNode(node)->properties[kKeyProperty] = std::string("F7");
        } else if (id == "event.every_n_seconds" || id == "event.timer") {
            fixture.graph.FindNode(node)->properties["Seconds"] = 2.5f;
        } else if (id == "characters.spawn") {
            fixture.graph.FindNode(node)->inputValues[8] = std::string("overcoat");
            fixture.graph.FindNode(node)->inputValues[2] = 1.9f;
        }
    }
    const std::string text = graph::SaveGraph(fixture.graph);
    graph::Graph loaded;
    const graph::LoadResult result = graph::LoadGraph(text, loaded);
    Check(result.ok && result.warnings.empty(), "the saved text loads without warnings");
    Check(graph::SaveGraph(loaded) == text, "saving the loaded graph gives the same text");
    Check(loaded.Nodes().size() == EngineNodeIds().size() + 1, "every node came back, and the Start event the fixture begins with");
    const auto key = std::find_if(loaded.Nodes().begin(), loaded.Nodes().end(), [](const graph::Node& node) { return node.type == kKeyPressedEvent; });
    Check(key != loaded.Nodes().end() && std::get<std::string>(key->properties.at(kKeyProperty)) == "F7", "the key kept its setting");
    const auto spawn = std::find_if(loaded.Nodes().begin(), loaded.Nodes().end(), [](const graph::Node& node) { return node.type == "characters.spawn"; });
    Check(spawn != loaded.Nodes().end() && std::get<std::string>(spawn->inputValues.at(8)) == "overcoat" && std::get<float>(spawn->inputValues.at(2)) == 1.9f,
          "a character kept the values typed into its pins");
    Check(!graph::HasErrors(graph::Validate(loaded, fixture.registry)), "the loaded graph has no errors");
}

// ----- The graphs that ship -------------------------------------------------------------------------------------------

std::string ReadAsset(const std::string& relativePath) {
    std::ifstream file(std::string(XCOM_ASSET_DIR) + "/" + relativePath, std::ios::binary);
    std::stringstream stream;
    stream << file.rdbuf();
    // A checkout that turns line ends into Windows ones must not change what the graph is.
    std::string text = stream.str();
    for (size_t at = text.find("\r\n"); at != std::string::npos; at = text.find("\r\n", at)) {
        text.erase(at, 1);
    }
    return text;
}

struct ShippedGraph {
    const char* file;
    std::string (*build)(const graph::NodeRegistry&);
};
const ShippedGraph kShippedGraphs[] = {
    {"graphs/day_night_cycle.json", DayNightCycleGraphText},
    {"graphs/spawn_crowd_on_key.json", SpawnCrowdOnKeyGraphText},
};

void EveryShippedGraphLoadsIsCleanAndIsWhatTheCodeMakes() {
    Fixture fixture;
    for (const ShippedGraph& shipped : kShippedGraphs) {
        const std::string text = ReadAsset(shipped.file);
        Check(!text.empty(), std::string(shipped.file) + " exists in the assets folder");
        graph::Graph loaded;
        const graph::LoadResult result = graph::LoadGraph(text, loaded);
        Check(result.ok && result.warnings.empty(), std::string(shipped.file) + " loads without warnings: " + result.error);
        const graph::ValidationReport report = graph::Validate(loaded, fixture.registry);
        Check(report.empty(), std::string(shipped.file) + " has no problems, not even warnings" + (report.empty() ? "" : ": " + report.front().message));
        Check(graph::SaveGraph(loaded) == text, std::string(shipped.file) + " is the canonical text of its graph");
        Check(shipped.build(fixture.registry) == text, std::string(shipped.file) + " is what ExampleGraphs.cpp makes (run graph_engine_tests --write-examples)");
    }
}

// Loads a shipped graph into a fixture, starts it on the mock host.
void LoadShipped(Fixture& fixture, const char* file) {
    fixture.graph = graph::Graph();
    const graph::LoadResult result = graph::LoadGraph(ReadAsset(file), fixture.graph);
    Check(result.ok, std::string("loading ") + file);
    fixture.Run();
}

void TheDayNightCycleAdvancesTheHourAndRestartsOnR() {
    Fixture fixture;
    LoadShipped(fixture, "graphs/day_night_cycle.json");
    graph::Interpreter& interpreter = *fixture.interpreter;
    Check(interpreter.CanRun() && fixture.host.CountOf("PrintToScreen") == 1, "starting says what the graph does");
    for (int frame = 0; frame < 20; ++frame) {
        interpreter.Tick(0.1f);
    }
    Check(interpreter.RuntimeError().empty(), "twenty frames run without a stop");
    Check(fixture.host.CountOf("SetTimeOfDay") == 20, "every frame sets the time of day");
    // Half an hour of the day goes by each second: twenty frames of a tenth of a second are one hour after sunrise.
    Check(Near(fixture.host.LastOf("SetTimeOfDay")->numbers[0], 7.0f, 0.01f), "two seconds later it is 7 o'clock");
    interpreter.FireEvent(kKeyPressedEvent, KeyFilter("R"));
    interpreter.Tick(0.1f);
    Check(Near(fixture.host.LastOf("SetTimeOfDay")->numbers[0], 6.05f, 0.01f), "R starts the day again at sunrise");
    interpreter.FireEvent(kKeyPressedEvent, KeyFilter("Q"));
    interpreter.Tick(0.1f);
    Check(Near(fixture.host.LastOf("SetTimeOfDay")->numbers[0], 6.1f, 0.01f), "another key does not");
}

void TheCrowdGraphActsOnItsKeys() {
    Fixture fixture;
    LoadShipped(fixture, "graphs/spawn_crowd_on_key.json");
    graph::Interpreter& interpreter = *fixture.interpreter;
    Check(fixture.host.Calls().empty(), "nothing happens until a key is pressed");
    interpreter.FireEvent(kKeyPressedEvent, KeyFilter("N"));
    Check(fixture.host.LastOf("SetCrowdSize") && fixture.host.LastOf("SetCrowdSize")->numbers[0] == 15000.0f, "N makes a big crowd");
    interpreter.FireEvent(kKeyPressedEvent, KeyFilter("M"));
    Check(fixture.host.LastOf("SetCrowdSize")->numbers[0] == 2000.0f, "M makes a small one");
    Check(fixture.host.CountOf("PrintToScreen") == 2, "and each says so");
    interpreter.FireEvent(kKeyPressedEvent, KeyFilter("C"));
    const RecordedCall* crate = fixture.host.LastOf("SpawnProp");
    Check(crate != nullptr && crate->text == "crate" && Near(crate->numbers[1], 1.0f) && Near(crate->numbers[2], 2.0f) && Near(crate->numbers[3], 9.0f),
          "C drops a crate six metres in front of the camera");
    interpreter.FireEvent(kKeyPressedEvent, KeyFilter("B"));
    const RecordedCall* person = fixture.host.LastOf("SpawnCharacter");
    Check(person != nullptr && Near(person->numbers[6], 1.0f) && Near(person->numbers[8], 9.0f), "B puts a person there");
    Check(person != nullptr && person->numbers[5] == 1000.0f, "with the seed the host gave");
    Check(interpreter.RuntimeError().empty(), "no run stopped");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--write-examples") {
        graph::NodeRegistry registry;
        graph::RegisterStandardNodes(registry);
        RegisterEngineNodes(registry);
        for (const ShippedGraph& shipped : kShippedGraphs) {
            const std::string path = std::string(argv[2]) + "/" + shipped.file;
            std::ofstream(path, std::ios::binary | std::ios::trunc) << shipped.build(registry);
            std::cout << "wrote " << path << "\n";
        }
        return 0;
    }

    struct Test {
        const char* name;
        std::function<void()> run;
    };
    const Test tests[] = {
        {"EveryEngineNodeIsWellFormed", EveryEngineNodeIsWellFormed},
        {"EveryNodeThatChangesTheEngineIsAnAction", EveryNodeThatChangesTheEngineIsAnAction},
        {"EveryActionRunsOnItsDefaultsAndLeavesTheHostInRange", EveryActionRunsOnItsDefaultsAndLeavesTheHostInRange},
        {"ActionsCallTheHostWithWhatTheirInputsSay", ActionsCallTheHostWithWhatTheirInputsSay},
        {"ImpulsesAndRemovalsUseTheIdOfTheSpawn", ImpulsesAndRemovalsUseTheIdOfTheSpawn},
        {"ASpawnTheHostRefusesGivesIdZero", ASpawnTheHostRefusesGivesIdZero},
        {"ACharacterIsSpawnedWithTheNumbersOnItsPins", ACharacterIsSpawnedWithTheNumbersOnItsPins},
        {"ArgumentsAreMadeFiniteAndKeptInRange", ArgumentsAreMadeFiniteAndKeptInRange},
        {"PositionsDirectionsAndImpulsesAreKeptSafe", PositionsDirectionsAndImpulsesAreKeptSafe},
        {"CharactersAreSpawnedWithinTheLimitsOfTheGenerator", CharactersAreSpawnedWithinTheLimitsOfTheGenerator},
        {"WithoutAHostNothingHappensAndNothingFails", WithoutAHostNothingHappensAndNothingFails},
        {"ReadersAnswerWithWhatTheHostSays", ReadersAnswerWithWhatTheHostSays},
        {"AKeyRunsOnlyTheEventsThatWaitForIt", AKeyRunsOnlyTheEventsThatWaitForIt},
        {"AKeyEventWithNoKeyWaitsForTheDefault", AKeyEventWithNoKeyWaitsForTheDefault},
        {"EveryNSecondsRunsOnceEachTimeThePeriodHasPassed", EveryNSecondsRunsOnceEachTimeThePeriodHasPassed},
        {"ATimerRunsAtMostOncePerTickWhateverTheFrameLength", ATimerRunsAtMostOncePerTickWhateverTheFrameLength},
        {"ATimerCannotBeAskedForEveryFrame", ATimerCannotBeAskedForEveryFrame},
        {"AfterNSecondsRunsOnceAndStartAgainStartsTheClockAgain", AfterNSecondsRunsOnceAndStartAgainStartsTheClockAgain},
        {"ASettingThatDoesNotFitIsReportedAndAMissingOneUsesTheDefault", ASettingThatDoesNotFitIsReportedAndAMissingOneUsesTheDefault},
        {"AGraphThatUsesEngineNodesSavesAndLoadsBackTheSame", AGraphThatUsesEngineNodesSavesAndLoadsBackTheSame},
        {"EveryShippedGraphLoadsIsCleanAndIsWhatTheCodeMakes", EveryShippedGraphLoadsIsCleanAndIsWhatTheCodeMakes},
        {"TheDayNightCycleAdvancesTheHourAndRestartsOnR", TheDayNightCycleAdvancesTheHourAndRestartsOnR},
        {"TheCrowdGraphActsOnItsKeys", TheCrowdGraphActsOnItsKeys},
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
    std::cout << (failureCount == 0 ? "all engine node tests passed\n" : "engine node tests failed\n");
    return failureCount == 0 ? 0 : 1;
}
