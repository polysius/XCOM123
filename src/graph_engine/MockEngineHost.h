#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include "graph_engine/EngineHost.h"

// An engine host for tests: it does nothing but write down every call, with the numbers it was given, and answers queries with
// values the test sets. No window, no GPU, no world.

namespace graph_engine {

struct RecordedCall {
    std::string name;            // the method, such as "SetSunElevation"
    std::vector<float> numbers;  // its arguments in order; vectors are three numbers, a prop kind is its number in PropKind
    std::string text;            // the text argument, or the outfit of a character, or empty
};

class MockEngineHost final : public EngineHost {
public:
    // What the queries answer, and whether spawns are refused.
    glm::vec3 cameraPosition{1.0f, 2.0f, 3.0f};
    glm::vec3 cameraDirection{0.0f, 0.0f, 1.0f};
    glm::vec3 playerPosition{4.0f, 0.0f, 5.0f};
    int crowdSize = 1000;
    float frameSeconds = 0.016f;
    float framesPerSecond = 62.5f;
    bool refuseSpawns = false;

    const std::vector<RecordedCall>& Calls() const { return calls; }
    int CountOf(const std::string& name) const {
        return static_cast<int>(std::count_if(calls.begin(), calls.end(), [&](const RecordedCall& call) { return call.name == name; }));
    }
    // The most recent call of a method, or null.
    const RecordedCall* LastOf(const std::string& name) const {
        for (auto call = calls.rbegin(); call != calls.rend(); ++call) {
            if (call->name == name) {
                return &*call;
            }
        }
        return nullptr;
    }
    void ClearCalls() { calls.clear(); }

    void SetSunElevation(float degrees) override { Record("SetSunElevation", {degrees}); }
    void SetSunAzimuth(float degrees) override { Record("SetSunAzimuth", {degrees}); }
    void SetTimeOfDay(float hours) override { Record("SetTimeOfDay", {hours}); }

    glm::vec3 CameraPosition() const override { return cameraPosition; }
    glm::vec3 CameraDirection() const override { return cameraDirection; }
    void SetCameraPosition(glm::vec3 position) override { Record("SetCameraPosition", {position.x, position.y, position.z}); }
    void SetCameraDirection(glm::vec3 direction) override { Record("SetCameraDirection", {direction.x, direction.y, direction.z}); }
    void TeleportCamera(glm::vec3 position, glm::vec3 direction) override {
        Record("TeleportCamera", {position.x, position.y, position.z, direction.x, direction.y, direction.z});
    }
    glm::vec3 PlayerPosition() const override { return playerPosition; }

    uint32_t SpawnProp(PropKind kind, glm::vec3 position) override {
        Record("SpawnProp", {static_cast<float>(kind), position.x, position.y, position.z}, PropKindName(kind));
        return refuseSpawns ? 0u : nextId++;
    }
    bool RemoveProp(uint32_t propId) override {
        Record("RemoveProp", {static_cast<float>(propId)});
        return !refuseSpawns;
    }
    void ApplyImpulse(uint32_t propId, glm::vec3 impulse) override {
        Record("ApplyImpulse", {static_cast<float>(propId), impulse.x, impulse.y, impulse.z});
    }

    void SetCrowdSize(int people) override { Record("SetCrowdSize", {static_cast<float>(people)}); }
    int CrowdSize() const override { return crowdSize; }
    uint32_t SpawnCharacter(const CharacterSpec& spec, glm::vec3 position) override {
        Record("SpawnCharacter",
               {spec.statureMeters, spec.massKg, spec.ageYears, spec.masculinity, spec.muscularity, static_cast<float>(spec.seed), position.x, position.y,
                position.z},
               spec.outfit);
        return refuseSpawns ? 0u : nextId++;
    }

    void SetRiverLevel(float metresAboveRest) override { Record("SetRiverLevel", {metresAboveRest}); }
    void AddRiverWave(glm::vec3 position, float amplitudeMeters, float radiusMeters) override {
        Record("AddRiverWave", {position.x, position.y, position.z, amplitudeMeters, radiusMeters});
    }

    void PrintToScreen(const std::string& text) override { Record("PrintToScreen", {}, text); }
    float FrameSeconds() const override { return frameSeconds; }
    float FramesPerSecond() const override { return framesPerSecond; }
    uint32_t NextRandomSeed() override {
        Record("NextRandomSeed", {});
        return 1000u + seedsGiven++;
    }

private:
    void Record(std::string name, std::vector<float> numbers, std::string text = {}) {
        calls.push_back({std::move(name), std::move(numbers), std::move(text)});
    }

    std::vector<RecordedCall> calls;
    uint32_t nextId = 1;
    uint32_t seedsGiven = 0;
};

}  // namespace graph_engine
