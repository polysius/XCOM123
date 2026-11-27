#include "engine_host/LiveEngineHost.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>

#include "core/FlyCamera.h"
#include "fluid/RiverWater.h"
#include "physics/PropField.h"
#include "render/IRenderModule.h"
#include "world/CityGenerator.h"
#include "world/Crowd.h"

namespace engine_host {
namespace {

using graph_engine::CharacterSpec;
using graph_engine::PropKind;

constexpr size_t kMaxProps = 600;               // props that can stand at once
constexpr size_t kMaxCharacters = 64;           // stand-ins for spawned people
constexpr int kMaxSpawnsPerFrame = 20;          // props and people added in one frame, together
constexpr int kMaxPrintsPerFrame = 4;
constexpr int kMaxWavesPerFrame = 8;
constexpr float kMinCrowdRebuildSeconds = 3.0f; // making the crowd again costs a moment, so not more often than this
constexpr size_t kMaxMessages = 8;
constexpr float kMessageSeconds = 6.0f;
constexpr float kFpsSmoothing = 0.1f;           // how much of the newest frame goes into the frames per second shown
constexpr float kBaseStature = 1.70f;           // the height of a person of the crowd at height scale 1
constexpr float kPi = 3.14159265358979f;

physics::PropKind ToPhysics(PropKind kind) {
    switch (kind) {
    case PropKind::Crate: return physics::PropKind::Crate;
    case PropKind::Barrel: return physics::PropKind::Barrel;
    case PropKind::Ball: return physics::PropKind::Ball;
    }
    return physics::PropKind::Crate;
}

// A colour for an outfit name, the same for the same name, so that "suit" and "coat" are told apart on screen.
glm::vec3 OutfitColour(const std::string& outfit) {
    const size_t hash = std::hash<std::string>{}(outfit);
    return {0.2f + 0.6f * static_cast<float>(hash % 97) / 96.0f, 0.2f + 0.6f * static_cast<float>((hash / 97) % 89) / 88.0f,
            0.2f + 0.6f * static_cast<float>((hash / 8633) % 83) / 82.0f};
}

}  // namespace

LiveEngineHost::LiveEngineHost(const WorldAccess& worldAccess)
    : world(worldAccess), reservedPeople(worldAccess.crowd != nullptr ? worldAccess.crowd->AgentCount() : 0), seedState(worldAccess.worldSeed) {}

void LiveEngineHost::BeginFrame(float deltaSeconds) {
    frameSeconds = std::isfinite(deltaSeconds) ? std::max(deltaSeconds, 0.0f) : 0.0f;
    if (frameSeconds > 0.0f) {
        framesPerSecond += kFpsSmoothing * (1.0f / frameSeconds - framesPerSecond);
    }
    spawnsThisFrame = 0;
    printsThisFrame = 0;
    wavesThisFrame = 0;
    ApplyAskedRiverLevel();
    for (ScreenMessage& message : messages) {
        message.secondsLeft -= frameSeconds;
    }
    messages.erase(std::remove_if(messages.begin(), messages.end(), [](const ScreenMessage& message) { return message.secondsLeft <= 0.0f; }), messages.end());
    RebuildCrowdIfAsked(frameSeconds);
}

void LiveEngineHost::AppendCharacters(std::vector<NpcInstance>& people) const {
    for (const SpawnedCharacter& character : characters) {
        people.push_back(character.instance);
    }
}

// ----- World -----

void LiveEngineHost::SetSunElevation(float degrees) {
    world.sun->elevationDegrees = graph_engine::ClampOr(degrees, graph_engine::kSunElevationMin, graph_engine::kSunElevationMax, world.sun->elevationDegrees);
}

void LiveEngineHost::SetSunAzimuth(float degrees) {
    world.sun->azimuthDegrees = graph_engine::WrapDegrees(degrees, world.sun->azimuthDegrees);
}

void LiveEngineHost::SetTimeOfDay(float hours) {
    *world.sun = graph_engine::SunAtTimeOfDay(hours);
}

// ----- Camera -----

glm::vec3 LiveEngineHost::CameraPosition() const {
    return world.camera->Position();
}

glm::vec3 LiveEngineHost::CameraDirection() const {
    return world.camera->Forward();
}

void LiveEngineHost::SetCameraPosition(glm::vec3 position) {
    const glm::vec3 place = graph_engine::ClampPosition(position, world.camera->Position());
    world.camera->LookAt(place, place + world.camera->Forward());
}

void LiveEngineHost::SetCameraDirection(glm::vec3 direction) {
    if (glm::length(direction) < 1.0e-6f || !std::isfinite(glm::length(direction))) {
        return;
    }
    world.camera->LookAt(world.camera->Position(), world.camera->Position() + glm::normalize(direction));
}

void LiveEngineHost::TeleportCamera(glm::vec3 position, glm::vec3 direction) {
    const glm::vec3 place = graph_engine::ClampPosition(position, world.camera->Position());
    const bool hasDirection = std::isfinite(glm::length(direction)) && glm::length(direction) > 1.0e-6f;
    world.camera->LookAt(place, place + (hasDirection ? glm::normalize(direction) : world.camera->Forward()));
}

glm::vec3 LiveEngineHost::PlayerPosition() const {
    return world.camera->Position();  // the player is the free camera
}

// ----- Props -----

bool LiveEngineHost::MaySpawn(size_t alreadyThere, size_t limit) {
    if (alreadyThere >= limit || spawnsThisFrame >= kMaxSpawnsPerFrame) {
        SayOnce("Too many things at once: not spawning.", saidFull);
        return false;
    }
    ++spawnsThisFrame;
    return true;
}

bool LiveEngineHost::PrepareGroundForProps(glm::vec2 point) {
    physics::PropField& props = *world.props;
    if (props.HasGroundAt(point)) {
        return true;
    }
    if (props.Count() > 0) {
        return false;  // making the ground again would lose the props that are on it
    }
    // The first prop decides where the ground is: at the river when it is simulated, as for --props, and otherwise round the point.
    // The ground at the river is made once; a spawn elsewhere is refused after that, and does not make it again.
    physics::PropSettings settings{.count = 0, .seed = world.worldSeed};
    const bool overRiver = world.river != nullptr && world.river->IsBuilt();
    if (overRiver) {
        if (madeGroundAtRiver) {
            return false;
        }
        madeGroundAtRiver = true;
        settings.dropAt = world.river->Centre();
        settings.riverBed = world.river->RestLevel() - world.river->Depth();
    } else {
        settings.dropAt = point;
    }
    props.Spawn(*world.city, settings);
    if (overRiver) {
        props.SetWater(world.river);
    }
    return props.HasGroundAt(point);
}

uint32_t LiveEngineHost::SpawnProp(PropKind kind, glm::vec3 position) {
    physics::PropField& props = *world.props;
    // The ground is looked at before the budget is spent, so that a spawn that cannot happen costs nothing.
    if (!PrepareGroundForProps({position.x, position.z})) {
        SayOnce("No ground for props there: they can be added near the first one, or at the river when it is simulated.", saidNoGround);
        return 0;
    }
    if (!MaySpawn(props.Count(), kMaxProps)) {
        return 0;
    }
    return props.AddProp(ToPhysics(kind), position, NextRandomSeed());
}

bool LiveEngineHost::RemoveProp(uint32_t propId) {
    return world.props->RemoveProp(propId);
}

void LiveEngineHost::ApplyImpulse(uint32_t propId, glm::vec3 impulse) {
    world.props->ApplyImpulse(propId, impulse);
}

// ----- People -----

void LiveEngineHost::SetCrowdSize(int people) {
    askedCrowdSize = std::clamp(people, 0, graph_engine::kMaxCrowdSize);
}

int LiveEngineHost::CrowdSize() const {
    return askedCrowdSize.value_or(static_cast<int>(world.crowd->AgentCount()));
}

void LiveEngineHost::RebuildCrowdIfAsked(float deltaSeconds) {
    secondsSinceCrowdRebuild += deltaSeconds;
    if (!askedCrowdSize || secondsSinceCrowdRebuild < kMinCrowdRebuildSeconds) {
        return;
    }
    const size_t people = static_cast<size_t>(*askedCrowdSize);
    askedCrowdSize.reset();
    if (people == world.crowd->AgentCount()) {
        return;
    }
    world.crowd->Spawn(*world.city, {.agentCount = static_cast<uint32_t>(people), .seed = world.worldSeed * 7919u + 1u});
    if (people > reservedPeople) {
        world.renderer->ReserveNpcCapacity(people);  // outside a frame: it waits for the GPU
        reservedPeople = people;
    }
    secondsSinceCrowdRebuild = 0.0f;
}

uint32_t LiveEngineHost::SpawnCharacter(const CharacterSpec& spec, glm::vec3 position) {
    if (!MaySpawn(characters.size(), kMaxCharacters)) {
        return 0;
    }
    const CharacterSpec clean = graph_engine::SanitisedCharacter(spec);
    const glm::vec3 place = graph_engine::ClampPosition(position, world.camera->Position());

    SpawnedCharacter character;
    character.id = nextCharacterId++;
    // The person stands on the ground below the point, facing the camera.
    const glm::vec3 toCamera = world.camera->Position() - place;
    character.instance.position = glm::vec3(place.x, world.crowd->WalkableSpace().WalkHeightAt({place.x, place.z}), place.z);
    character.instance.heading = std::atan2(toCamera.x, toCamera.z);
    character.instance.clothingColor = OutfitColour(clean.outfit);
    character.instance.heightScale = clean.statureMeters / kBaseStature;
    character.instance.walkPhase = 0.0f;
    character.instance.stride = 0.0f;  // standing still
    characters.push_back(character);
    return character.id;
}

// ----- Water -----

void LiveEngineHost::SetRiverLevel(float metresAboveRest) {
    if (world.river == nullptr || !world.river->IsBuilt()) {
        SayOnce("The river is not simulated: start the engine with --river-sim.", saidNoRiver);
        return;
    }
    askedRiverLevel = metresAboveRest;  // applied once a frame, see BeginFrame
}

void LiveEngineHost::ApplyAskedRiverLevel() {
    if (!askedRiverLevel) {
        return;
    }
    if (askedRiverLevel != appliedRiverLevel) {
        world.river->SetLevelAboveRest(*askedRiverLevel);
        appliedRiverLevel = askedRiverLevel;
    }
    askedRiverLevel.reset();
}

void LiveEngineHost::AddRiverWave(glm::vec3 position, float amplitudeMeters, float radiusMeters) {
    if (world.river == nullptr || !world.river->IsBuilt()) {
        SayOnce("The river is not simulated: start the engine with --river-sim.", saidNoRiver);
        return;
    }
    if (wavesThisFrame >= kMaxWavesPerFrame) {
        return;
    }
    ++wavesThisFrame;
    // A bump of this height over a disc of this radius holds this much water.
    world.river->Splash({position.x, position.z}, radiusMeters, amplitudeMeters * kPi * radiusMeters * radiusMeters);
}

// ----- Debug -----

void LiveEngineHost::PrintToScreen(const std::string& text) {
    if (printsThisFrame >= kMaxPrintsPerFrame) {
        return;  // a graph that prints in a loop must not fill the console and the screen
    }
    ++printsThisFrame;
    std::cout << "[graph] " << text << "\n";
    messages.push_back({text, kMessageSeconds});
    if (messages.size() > kMaxMessages) {
        messages.erase(messages.begin());
    }
}

void LiveEngineHost::SayOnce(const std::string& text, bool& alreadySaid) {
    if (!alreadySaid) {
        alreadySaid = true;
        PrintToScreen(text);
    }
}

uint32_t LiveEngineHost::NextRandomSeed() {
    // SplitMix64: the same world seed gives the same numbers in the same order.
    seedState += 0x9E3779B97F4A7C15ull;
    uint64_t z = seedState;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z ^= z >> 31;
    return static_cast<uint32_t>(z >> 33);  // 31 bits, so that it fits an Int pin
}

}  // namespace engine_host
