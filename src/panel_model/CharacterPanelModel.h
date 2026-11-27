#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "character/CharacterGenerator.h"
#include "character/CharacterParameters.h"
#include "graph_engine/EngineHost.h"
#include "panel_model/BackgroundTask.h"

// The model behind the character generator panel: the numbers the sliders hold, what is wrong with them, how they become the
// parameters of the generator and the spec of a spawn, and the generator running in the background. The panel only draws this, so
// all of it can be tested without a window.

namespace panel_model {

// The numbers of the sliders. Mass is given as a body mass index, which stays sensible as the stature changes; the mass in
// kilograms follows from it.
struct CharacterPanelParameters {
    float statureMeters = 1.70f;
    float bodyMassIndex = 24.2f;  // kg per square metre of stature; 24.2 at 1.70 m is 70 kg
    float ageYears = 30.0f;
    float masculinity = 0.5f;     // 0 = a female body, 1 = a male one
    float muscularity = 0.4f;
    int seed = 1;
    std::string outfit = "casual";
};

// The ranges of the sliders, which are the ranges the generator accepts.
inline constexpr float kBodyMassIndexMin = 15.0f;
inline constexpr float kBodyMassIndexMax = 45.0f;
inline constexpr int kSeedMax = 99999;

// The same parameters with every number finite and inside its range (a number that is not a number becomes the default).
CharacterPanelParameters Clamped(const CharacterPanelParameters& parameters);

// What is wrong with the parameters as they are, one sentence each: the numbers that will be brought into range. Empty when all is well.
std::vector<std::string> Problems(const CharacterPanelParameters& parameters);

float MassKg(const CharacterPanelParameters& parameters);
character::CharacterParameters ToGeneratorParameters(const CharacterPanelParameters& parameters);
graph_engine::CharacterSpec ToSpawnSpec(const CharacterPanelParameters& parameters);
// A person drawn from the population, the same for the same seed (see character::RandomParameters), as slider values.
CharacterPanelParameters RandomPanelParameters(int seed);

// What the generator made, in numbers for the panel. The mesh itself is not kept.
struct CharacterReport {
    size_t vertexCount = 0;
    size_t triangleCount = 0;
    size_t pieceCount = 0;
    float massKg = 0.0f;
    float statureMeters = 0.0f;  // measured on the mesh
    double milliseconds = 0.0;   // how long making it took
};

// Makes the character and measures it, on the calling thread. `settings` says how fine the mesh is.
CharacterReport MakeCharacterReport(const CharacterPanelParameters& parameters, const character::MeshSettings& settings = {});

// The character generator running in the background, one person at a time.
class CharacterGeneratorModel {
public:
    CharacterPanelParameters parameters;

    // How fine the mesh is made. The panel leaves it at the default; tests make it coarse.
    character::MeshSettings meshSettings;

    // Starts making a person from the current parameters (brought into range). Returns false while the last one is still being made.
    bool Generate();
    bool IsRunning() const { return task.IsRunning(); }
    double SecondsRunning() const { return task.SecondsRunning(); }

    // What a finished run made: the spec to spawn the person with, and the numbers of the mesh. Gives each run once.
    struct Finished {
        graph_engine::CharacterSpec spec;
        CharacterReport report;
    };
    std::optional<Finished> Poll();

    // The numbers of the last person made, and the reason the last run failed, or empty.
    const std::optional<CharacterReport>& LastReport() const { return lastReport; }
    const std::string& LastError() const { return lastError; }

private:
    struct Made {
        graph_engine::CharacterSpec spec;
        CharacterReport report;
    };
    BackgroundTask<Made> task;
    std::optional<CharacterReport> lastReport;
    std::string lastError;
};

}  // namespace panel_model
