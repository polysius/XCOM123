#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "graph_engine/EngineHost.h"
#include "graph_engine/SunPath.h"
#include "scene/SceneTypes.h"

class Crowd;
class FlyCamera;
class IRenderModule;
struct CityLayout;
namespace fluid {
class RiverWater;
}
namespace physics {
class PropField;
}

// The engine host of the running program: it turns what a graph asks for into changes of the camera, the sun, the crowd, the props
// and the river. The program owns all of these, and the host only points at them, so it is made after them and dropped before them.
//
// It also keeps the program safe from a graph that asks for too much: only so many props and people can stand at once, only a few
// may be added in one frame, and the crowd is only made again every few seconds, whatever the graph asks for.

namespace engine_host {

// What the host reaches in the program. Every pointer must stay valid for the life of the host.
struct WorldAccess {
    FlyCamera* camera = nullptr;
    graph_engine::SunSettings* sun = nullptr;
    const CityLayout* city = nullptr;
    Crowd* crowd = nullptr;
    physics::PropField* props = nullptr;
    fluid::RiverWater* river = nullptr;  // built only when the river is simulated
    IRenderModule* renderer = nullptr;
    uint32_t worldSeed = 1;
};

// A line of text shown on the screen for a few seconds.
struct ScreenMessage {
    std::string text;
    float secondsLeft = 0.0f;
};

class LiveEngineHost final : public graph_engine::EngineHost {
public:
    explicit LiveEngineHost(const WorldAccess& world);

    // Call once per frame, before the graph runs: takes the frame time, ages the messages, gives the frame its budget of spawns, and
    // makes the crowd again if a graph asked for a new size and the last change is long enough ago.
    void BeginFrame(float deltaSeconds);

    const std::vector<ScreenMessage>& ScreenMessages() const { return messages; }
    // The people that graphs have spawned, as stand-ins that the renderer draws as the people of the crowd: the character mesh
    // has no way to the screen yet. Appends them to the people of the frame.
    void AppendCharacters(std::vector<NpcInstance>& people) const;
    size_t CharacterCount() const { return characters.size(); }

    // ----- graph_engine::EngineHost -----
    void SetSunElevation(float degrees) override;
    void SetSunAzimuth(float degrees) override;
    void SetTimeOfDay(float hours) override;
    glm::vec3 CameraPosition() const override;
    glm::vec3 CameraDirection() const override;
    void SetCameraPosition(glm::vec3 position) override;
    void SetCameraDirection(glm::vec3 direction) override;
    void TeleportCamera(glm::vec3 position, glm::vec3 direction) override;
    glm::vec3 PlayerPosition() const override;
    uint32_t SpawnProp(graph_engine::PropKind kind, glm::vec3 position) override;
    bool RemoveProp(uint32_t propId) override;
    void ApplyImpulse(uint32_t propId, glm::vec3 impulse) override;
    void SetCrowdSize(int people) override;
    int CrowdSize() const override;
    uint32_t SpawnCharacter(const graph_engine::CharacterSpec& spec, glm::vec3 position) override;
    void SetRiverLevel(float metresAboveRest) override;
    void AddRiverWave(glm::vec3 position, float amplitudeMeters, float radiusMeters) override;
    void PrintToScreen(const std::string& text) override;
    float FrameSeconds() const override { return frameSeconds; }
    float FramesPerSecond() const override { return framesPerSecond; }
    uint32_t NextRandomSeed() override;

private:
    struct SpawnedCharacter {
        uint32_t id = 0;
        NpcInstance instance{};
    };

    // Whether a prop or a person may be added now: there is room, and the frame has not used up its spawns.
    bool MaySpawn(size_t alreadyThere, size_t limit);
    // Makes sure the props have ground at the point, which is only possible while there are no props (see PropField::HasGroundAt).
    bool PrepareGroundForProps(glm::vec2 point);
    void RebuildCrowdIfAsked(float deltaSeconds);
    void ApplyAskedRiverLevel();
    void SayOnce(const std::string& text, bool& alreadySaid);

    WorldAccess world;
    float frameSeconds = 0.0f;
    float framesPerSecond = 0.0f;
    int spawnsThisFrame = 0;
    std::vector<ScreenMessage> messages;

    // What a graph may do in one frame without being cut short: the lines it prints and the waves it makes. A river level is only
    // applied once a frame, and only when it changed, because moving the whole river costs a pass over its grid.
    int printsThisFrame = 0;
    int wavesThisFrame = 0;
    std::optional<float> askedRiverLevel;
    std::optional<float> appliedRiverLevel;
    bool madeGroundAtRiver = false;

    std::optional<int> askedCrowdSize;  // a size that a graph asked for and the crowd has not been made at yet
    float secondsSinceCrowdRebuild = 1.0e9f;
    size_t reservedPeople = 0;

    std::vector<SpawnedCharacter> characters;
    uint32_t nextCharacterId = 1;
    uint64_t seedState = 0;

    bool saidNoRiver = false;
    bool saidNoGround = false;
    bool saidFull = false;
};

}  // namespace engine_host
