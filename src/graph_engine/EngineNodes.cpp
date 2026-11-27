#include "graph_engine/EngineNodes.h"

#include <cmath>
#include <functional>
#include <optional>

#include "graph/Interpreter.h"
#include "graph_engine/KeyNames.h"

namespace graph_engine {
namespace {

using graph::NodeContext;
using graph::NodeDescription;
using graph::NodeKind;
using graph::PinDescription;
using graph::PinType;
using graph::PropertyDescription;
using graph::Value;

constexpr float kDefaultSunElevation = 24.0f;
constexpr float kDefaultSunAzimuth = 230.0f;
constexpr float kHoursInDay = 24.0f;
constexpr float kDirectionLimit = 1.0e6f;  // keeps the length of a direction from overflowing

// ---------------------------------------------------------------------------------------------------------------------
// Building descriptions
// ---------------------------------------------------------------------------------------------------------------------

PinDescription Pin(const std::string& name, PinType type, Value value, const std::string& tooltip = {}) {
    return {name, type, std::move(value), tooltip};
}

PinDescription Exec(const std::string& name) {
    return {name, PinType::Exec, false, {}};
}

NodeDescription Describe(const std::string& id, const std::string& title, const std::string& category, const std::string& tooltip, NodeKind kind) {
    NodeDescription description;
    description.id = id;
    description.title = title;
    description.category = category;
    description.tooltip = tooltip;
    description.kind = kind;
    return description;
}

// The engine behind the interpreter, or null when the graph runs without one.
EngineHost* HostOf(const NodeContext& context) {
    return static_cast<EngineHost*>(context.Host());
}

// The data inputs of an action, counted from the first one after its Exec pin, so that the numbers here are the order of the
// pins as they are listed in the registration.
class Arguments {
public:
    explicit Arguments(const NodeContext& context) : context(context) {}
    float Float(int index) const { return context.Get<float>(index + 1); }
    int Int(int index) const { return context.Get<int>(index + 1); }
    glm::vec3 Vec3(int index) const { return context.Get<glm::vec3>(index + 1); }
    std::string Text(int index) const { return context.Get<std::string>(index + 1); }

private:
    const NodeContext& context;
};

using HostAction = std::function<void(NodeContext&, EngineHost&, const Arguments&)>;

// An action that does something to the engine: "In", the inputs, then "Then" and the outputs. It does nothing without a host.
void AddAction(graph::NodeRegistry& registry, const std::string& id, const std::string& title, const std::string& category,
               const std::string& tooltip, std::vector<PinDescription> inputs, std::vector<PinDescription> outputs, HostAction body) {
    NodeDescription description = Describe(id, title, category, tooltip, NodeKind::Action);
    description.inputs.push_back(Exec("In"));
    for (PinDescription& pin : inputs) {
        description.inputs.push_back(std::move(pin));
    }
    description.outputs.push_back(Exec("Then"));
    for (PinDescription& pin : outputs) {
        description.outputs.push_back(std::move(pin));
    }
    description.run = [body = std::move(body)](NodeContext& context) {
        if (EngineHost* host = HostOf(context)) {
            body(context, *host, Arguments(context));
        }
    };
    registry.Register(std::move(description));
}

// A node that only reads the engine, and gives `fallback` to its outputs when there is none.
void AddReader(graph::NodeRegistry& registry, const std::string& id, const std::string& title, const std::string& category,
               const std::string& tooltip, PinDescription output, std::function<Value(const EngineHost&)> read) {
    NodeDescription description = Describe(id, title, category, tooltip, NodeKind::Pure);
    const Value fallback = output.defaultValue;
    description.outputs.push_back(std::move(output));
    description.run = [read = std::move(read), fallback](NodeContext& context) {
        const EngineHost* host = HostOf(context);
        context.Set(0, host != nullptr ? read(*host) : fallback);
    };
    registry.Register(std::move(description));
}

// ---------------------------------------------------------------------------------------------------------------------
// Making arguments safe
// ---------------------------------------------------------------------------------------------------------------------

float WrapHours(float hours) {
    const float wrapped = std::fmod(FiniteOr(hours, 12.0f), kHoursInDay);
    return wrapped < 0.0f ? wrapped + kHoursInDay : wrapped;
}

// A direction of length one, or nothing when the vector has no direction (zero, or not a number).
std::optional<glm::vec3> UnitDirection(glm::vec3 direction) {
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(direction[axis])) {
            return std::nullopt;
        }
        direction[axis] = std::clamp(direction[axis], -kDirectionLimit, kDirectionLimit);
    }
    const float length = glm::length(direction);
    if (length < 1.0e-6f) {
        return std::nullopt;
    }
    return direction / length;
}

// An impulse with no more than kMaxImpulse of strength, or nothing when it is not a number.
std::optional<glm::vec3> LimitedImpulse(glm::vec3 impulse) {
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(impulse[axis])) {
            return std::nullopt;
        }
        impulse[axis] = std::clamp(impulse[axis], -kDirectionLimit, kDirectionLimit);
    }
    const float strength = glm::length(impulse);
    return strength > kMaxImpulse ? impulse * (kMaxImpulse / strength) : impulse;
}

uint32_t IdFromPin(int value) {
    return static_cast<uint32_t>(std::max(value, 0));
}

// ---------------------------------------------------------------------------------------------------------------------
// The nodes, by category
// ---------------------------------------------------------------------------------------------------------------------

void RegisterWorldNodes(graph::NodeRegistry& registry) {
    AddAction(registry, "world.set_sun_elevation", "Set Sun Elevation", "World",
              "Puts the sun at an angle above the horizon, from -5 to 85 degrees. Low angles give sunrise and sunset light.",
              {Pin("Degrees", PinType::Float, kDefaultSunElevation, "The angle above the horizon.")}, {},
              [](NodeContext&, EngineHost& host, const Arguments& in) {
                  host.SetSunElevation(ClampOr(in.Float(0), kSunElevationMin, kSunElevationMax, kDefaultSunElevation));
              });
    AddAction(registry, "world.set_sun_azimuth", "Set Sun Direction", "World",
              "Turns the sun round the sky. 0 degrees is towards +Z, 90 towards +X; any number wraps round the circle.",
              {Pin("Degrees", PinType::Float, kDefaultSunAzimuth, "The compass direction of the sun.")}, {},
              [](NodeContext&, EngineHost& host, const Arguments& in) { host.SetSunAzimuth(WrapDegrees(in.Float(0), kDefaultSunAzimuth)); });
    AddAction(registry, "world.set_time_of_day", "Set Time Of Day", "World",
              "Moves the sun to where it is at this hour of the day: 6 is sunrise, 12 noon and 18 sunset. Hours wrap round at 24.",
              {Pin("Hours", PinType::Float, 12.0f, "The hour of the day, from 0 to 24.")}, {},
              [](NodeContext&, EngineHost& host, const Arguments& in) { host.SetTimeOfDay(WrapHours(in.Float(0))); });
}

void RegisterCameraNodes(graph::NodeRegistry& registry) {
    AddReader(registry, "camera.get_position", "Get Camera Position", "Camera", "Where the camera is, in metres.",
              Pin("Position", PinType::Vec3, glm::vec3(0.0f)), [](const EngineHost& host) { return Value(host.CameraPosition()); });
    AddReader(registry, "camera.get_direction", "Get Camera Direction", "Camera", "The direction the camera looks along, as a vector of length 1.",
              Pin("Direction", PinType::Vec3, glm::vec3(0.0f, 0.0f, 1.0f)), [](const EngineHost& host) { return Value(host.CameraDirection()); });
    AddAction(registry, "camera.set_position", "Set Camera Position", "Camera",
              "Moves the camera to a place, keeping the way it looks. The place is kept within 5 km of the middle of the city.",
              {Pin("Position", PinType::Vec3, glm::vec3(0.0f, 70.0f, 0.0f), "Where to put the camera, in metres.")}, {},
              [](NodeContext&, EngineHost& host, const Arguments& in) { host.SetCameraPosition(ClampPosition(in.Vec3(0), host.CameraPosition())); });
    AddAction(registry, "camera.set_direction", "Set Camera Direction", "Camera",
              "Turns the camera to look along a vector, staying where it is. A vector with no direction (all zero) changes nothing.",
              {Pin("Direction", PinType::Vec3, glm::vec3(0.0f, 0.0f, 1.0f), "Where to look. Its length does not matter.")}, {},
              [](NodeContext&, EngineHost& host, const Arguments& in) {
                  if (const std::optional<glm::vec3> direction = UnitDirection(in.Vec3(0))) {
                      host.SetCameraDirection(*direction);
                  }
              });
    AddAction(registry, "camera.teleport", "Teleport Camera", "Camera",
              "Jumps the camera to a place and turns it to look along a vector. A vector with no direction keeps the way it looked.",
              {Pin("Position", PinType::Vec3, glm::vec3(0.0f, 70.0f, 0.0f), "Where to put the camera, in metres."),
               Pin("Direction", PinType::Vec3, glm::vec3(0.0f, 0.0f, 1.0f), "Where to look. Its length does not matter.")},
              {}, [](NodeContext&, EngineHost& host, const Arguments& in) {
                  const glm::vec3 direction = UnitDirection(in.Vec3(1)).value_or(host.CameraDirection());
                  host.TeleportCamera(ClampPosition(in.Vec3(0), host.CameraPosition()), direction);
              });
}

void RegisterCharacterNodes(graph::NodeRegistry& registry) {
    AddAction(registry, "characters.spawn", "Spawn Character", "Characters",
              "Makes a person with these measurements and puts them on the ground below a place, facing the camera. Out of range numbers "
              "are brought into range. Only a limited number of people can stand at once.",
              {Pin("Position", PinType::Vec3, glm::vec3(0.0f), "Where the person stands, in metres."),
               Pin("Stature", PinType::Float, 1.70f, "Height in metres, 1.45 to 2.05."),
               Pin("Mass", PinType::Float, 70.0f, "Body mass in kilograms, 30 to 200."),
               Pin("Age", PinType::Float, 30.0f, "Years, 18 to 80."),
               Pin("Sex", PinType::Float, 0.5f, "0 is a female body, 1 a male one, and between is a blend."),
               Pin("Muscularity", PinType::Float, 0.4f, "0 is slight, 1 is a trained athlete."),
               Pin("Seed", PinType::Int, 1, "The same seed gives the same person."),
               Pin("Outfit", PinType::String, std::string("casual"), "The name of the clothes.")},
              {Pin("Character Id", PinType::Int, 0, "A number that names the person, or 0 when none was made.")},
              [](NodeContext& context, EngineHost& host, const Arguments& in) {
                  CharacterSpec spec;
                  spec.statureMeters = in.Float(1);
                  spec.massKg = in.Float(2);
                  spec.ageYears = in.Float(3);
                  spec.masculinity = in.Float(4);
                  spec.muscularity = in.Float(5);
                  spec.seed = static_cast<uint32_t>(in.Int(6));
                  spec.outfit = in.Text(7);
                  const uint32_t id = host.SpawnCharacter(SanitisedCharacter(spec), ClampPosition(in.Vec3(0), host.PlayerPosition()));
                  context.Set(1, static_cast<int>(id));
              });
    AddAction(registry, "characters.set_crowd_size", "Set Crowd Size", "Characters",
              "Sets how many people walk the city, from 0 to 20000. Changing it makes the crowd again, which takes a moment.",
              {Pin("People", PinType::Int, 2000, "The number of people.")}, {},
              [](NodeContext&, EngineHost& host, const Arguments& in) { host.SetCrowdSize(std::clamp(in.Int(0), 0, kMaxCrowdSize)); });
    AddReader(registry, "characters.get_crowd_size", "Get Crowd Size", "Characters", "How many people walk the city.",
              Pin("People", PinType::Int, 0), [](const EngineHost& host) { return Value(host.CrowdSize()); });
    AddReader(registry, "characters.get_player_position", "Get Player Position", "Characters", "Where the player is, in metres.",
              Pin("Position", PinType::Vec3, glm::vec3(0.0f)), [](const EngineHost& host) { return Value(host.PlayerPosition()); });
}

// The three props differ only in what they are called.
void AddSpawnProp(graph::NodeRegistry& registry, const std::string& id, const std::string& title, PropKind kind) {
    AddAction(registry, id, title, "Physics",
              std::string("Drops a ") + PropKindName(kind) + " at a place. It falls and comes to rest under the physics. Only a limited number "
                                                              "of props can stand at once; when there is no room the node gives Prop Id 0.",
              {Pin("Position", PinType::Vec3, glm::vec3(0.0f), "Where the prop starts, in metres.")},
              {Pin("Prop Id", PinType::Int, 0, "A number that names the prop, or 0 when none was made.")},
              [kind](NodeContext& context, EngineHost& host, const Arguments& in) {
                  context.Set(1, static_cast<int>(host.SpawnProp(kind, ClampPosition(in.Vec3(0), host.PlayerPosition()))));
              });
}

void RegisterPhysicsNodes(graph::NodeRegistry& registry) {
    AddSpawnProp(registry, "physics.spawn_crate", "Spawn Crate", PropKind::Crate);
    AddSpawnProp(registry, "physics.spawn_barrel", "Spawn Barrel", PropKind::Barrel);
    AddSpawnProp(registry, "physics.spawn_ball", "Spawn Ball", PropKind::Ball);
    AddAction(registry, "physics.remove_prop", "Remove Prop", "Physics", "Takes a prop out of the world. Gives Removed true when there was such a prop.",
              {Pin("Prop Id", PinType::Int, 0, "The number a Spawn node gave.")}, {Pin("Removed", PinType::Bool, false)},
              [](NodeContext& context, EngineHost& host, const Arguments& in) {
                  const uint32_t id = IdFromPin(in.Int(0));
                  context.Set(1, id != 0 && host.RemoveProp(id));
              });
    AddAction(registry, "physics.apply_impulse", "Apply Impulse", "Physics",
              "Kicks a prop: changes its speed by the impulse divided by its mass. The strength is limited to 500 newton seconds.",
              {Pin("Prop Id", PinType::Int, 0, "The number a Spawn node gave."),
               Pin("Impulse", PinType::Vec3, glm::vec3(0.0f, 10.0f, 0.0f), "The push, in newton seconds.")},
              {}, [](NodeContext&, EngineHost& host, const Arguments& in) {
                  const uint32_t id = IdFromPin(in.Int(0));
                  if (const std::optional<glm::vec3> impulse = LimitedImpulse(in.Vec3(1)); impulse && id != 0) {
                      host.ApplyImpulse(id, *impulse);
                  }
              });
}

void RegisterWaterNodes(graph::NodeRegistry& registry) {
    AddAction(registry, "water.set_level", "Set River Level", "Water",
              "Raises or lowers the river from its still level, by up to 5 metres. Needs the river to be simulated (--river-sim).",
              {Pin("Level", PinType::Float, 0.0f, "Metres above the still level; below 0 is lower.")}, {},
              [](NodeContext&, EngineHost& host, const Arguments& in) {
                  host.SetRiverLevel(ClampOr(in.Float(0), -kRiverLevelRange, kRiverLevelRange, 0.0f));
              });
    AddAction(registry, "water.add_wave", "Add Wave", "Water",
              "Makes a ripple on the river at a place. Needs the river to be simulated (--river-sim).",
              {Pin("Position", PinType::Vec3, glm::vec3(0.0f), "Where the ripple starts, in metres."),
               Pin("Amplitude", PinType::Float, 0.3f, "How high the wave is, in metres, up to 2."),
               Pin("Radius", PinType::Float, 3.0f, "How wide it is, in metres, from 0.5 to 20.")},
              {}, [](NodeContext&, EngineHost& host, const Arguments& in) {
                  host.AddRiverWave(ClampPosition(in.Vec3(0), host.PlayerPosition()), ClampOr(in.Float(1), 0.0f, kWaveAmplitudeMax, 0.3f),
                                    ClampOr(in.Float(2), kWaveRadiusMin, kWaveRadiusMax, 3.0f));
              });
}

void RegisterDebugNodes(graph::NodeRegistry& registry) {
    // This one also writes to the log of the run, so that the editor shows the line even when the engine has no screen to put it on.
    NodeDescription print = Describe("debug.print_on_screen", "Print On Screen", "Debug",
                                     "Shows a line of text on the screen for a few seconds, and adds it to the log of the run.", NodeKind::Action);
    print.inputs = {Exec("In"), Pin("Text", PinType::String, std::string("Hello"), "What to show. Long text is cut after 200 characters.")};
    print.outputs = {Exec("Then")};
    print.run = [](NodeContext& context) {
        const std::string text = Truncated(context.Get<std::string>(1), kMaxTextLength);
        context.Log(text);
        if (EngineHost* host = HostOf(context)) {
            host->PrintToScreen(text);
        }
    };
    registry.Register(std::move(print));

    AddReader(registry, "debug.get_frame_time", "Get Frame Time", "Debug", "How long the last frame took, in seconds.",
              Pin("Seconds", PinType::Float, 0.0f), [](const EngineHost& host) { return Value(host.FrameSeconds()); });
    AddReader(registry, "debug.get_fps", "Get FPS", "Debug", "How many frames are drawn each second.",
              Pin("Frames Per Second", PinType::Float, 0.0f), [](const EngineHost& host) { return Value(host.FramesPerSecond()); });

    // Without a host the graph's own generator gives the number, so that a run is the same every time.
    NodeDescription seed = Describe("debug.random_seed", "Get Random Seed", "Debug",
                                    "Gives a whole number to seed a generator with, such as the Seed of Spawn Character. Each use gives a new one.",
                                    NodeKind::Action);
    seed.inputs = {Exec("In")};
    seed.outputs = {Exec("Then"), Pin("Seed", PinType::Int, 0)};
    seed.run = [](NodeContext& context) {
        EngineHost* host = HostOf(context);
        const uint32_t number = host != nullptr ? host->NextRandomSeed() : static_cast<uint32_t>(context.Random() * 16777216.0f);
        context.Set(1, static_cast<int>(number & 0x7FFFFFFFu));
    };
    registry.Register(std::move(seed));
}

void RegisterEvents(graph::NodeRegistry& registry) {
    NodeDescription key = Describe(kKeyPressedEvent, "On Key Pressed", "Events",
                                   "Runs when the chosen key goes down. The editor keeps the keys it is using for itself.", NodeKind::Event);
    key.outputs = {Exec("Pressed")};
    PropertyDescription keyProperty{kKeyProperty, PinType::String, std::string(kDefaultKey), "The key to wait for.", {}};
    for (std::string_view name : kKeyNames) {
        keyProperty.choices.emplace_back(name);
    }
    key.properties = {std::move(keyProperty)};
    registry.Register(std::move(key));

    NodeDescription every = Describe("event.every_n_seconds", "Every N Seconds", "Events",
                                     "Runs again and again, each time this many seconds have passed. At most once a frame, and never faster than 20 times a second.",
                                     NodeKind::Event);
    every.outputs = {Exec("Then")};
    every.properties = {{"Seconds", PinType::Float, 1.0f, "Seconds between runs (0.05 or more).", {}}};
    every.timerSecondsProperty = "Seconds";
    every.timerRepeats = true;
    registry.Register(std::move(every));

    NodeDescription once = Describe("event.timer", "After N Seconds", "Events", "Runs once, this many seconds after the graph started.", NodeKind::Event);
    once.outputs = {Exec("Then")};
    once.properties = {{"Seconds", PinType::Float, 5.0f, "Seconds after the start (0.05 or more).", {}}};
    once.timerSecondsProperty = "Seconds";
    once.timerRepeats = false;
    registry.Register(std::move(once));
}

}  // namespace

void RegisterEngineNodes(graph::NodeRegistry& registry) {
    RegisterEvents(registry);
    RegisterWorldNodes(registry);
    RegisterCameraNodes(registry);
    RegisterCharacterNodes(registry);
    RegisterPhysicsNodes(registry);
    RegisterWaterNodes(registry);
    RegisterDebugNodes(registry);
}

}  // namespace graph_engine
