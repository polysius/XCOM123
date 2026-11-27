#include "panel_model/CharacterPanelModel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace panel_model {
namespace {

using graph_engine::ClampOr;

bool IsOutside(float value, float low, float high) {
    return !std::isfinite(value) || value < low || value > high;
}

std::string Shortest(float value) {
    char text[32];
    std::snprintf(text, sizeof(text), "%g", static_cast<double>(value));
    return text;
}

void ReportIfOutside(std::vector<std::string>& problems, const char* name, float value, float low, float high) {
    if (IsOutside(value, low, high)) {
        problems.push_back(std::string(name) + " is outside " + Shortest(low) + " to " + Shortest(high) + " and will be brought into range.");
    }
}

}  // namespace

CharacterPanelParameters Clamped(const CharacterPanelParameters& parameters) {
    const CharacterPanelParameters defaults;
    CharacterPanelParameters clean;
    clean.statureMeters = ClampOr(parameters.statureMeters, graph_engine::kStatureMin, graph_engine::kStatureMax, defaults.statureMeters);
    clean.bodyMassIndex = ClampOr(parameters.bodyMassIndex, kBodyMassIndexMin, kBodyMassIndexMax, defaults.bodyMassIndex);
    clean.ageYears = ClampOr(parameters.ageYears, graph_engine::kAgeMin, graph_engine::kAgeMax, defaults.ageYears);
    clean.masculinity = ClampOr(parameters.masculinity, 0.0f, 1.0f, defaults.masculinity);
    clean.muscularity = ClampOr(parameters.muscularity, 0.0f, 1.0f, defaults.muscularity);
    clean.seed = std::clamp(parameters.seed, 0, kSeedMax);
    clean.outfit = graph_engine::Truncated(parameters.outfit, graph_engine::kMaxOutfitLength);
    return clean;
}

std::vector<std::string> Problems(const CharacterPanelParameters& parameters) {
    std::vector<std::string> problems;
    ReportIfOutside(problems, "Stature", parameters.statureMeters, graph_engine::kStatureMin, graph_engine::kStatureMax);
    ReportIfOutside(problems, "Body mass index", parameters.bodyMassIndex, kBodyMassIndexMin, kBodyMassIndexMax);
    ReportIfOutside(problems, "Age", parameters.ageYears, graph_engine::kAgeMin, graph_engine::kAgeMax);
    ReportIfOutside(problems, "Sex", parameters.masculinity, 0.0f, 1.0f);
    ReportIfOutside(problems, "Muscularity", parameters.muscularity, 0.0f, 1.0f);
    if (parameters.seed < 0 || parameters.seed > kSeedMax) {
        problems.push_back("Seed is outside 0 to " + std::to_string(kSeedMax) + " and will be brought into range.");
    }
    if (parameters.outfit.size() > graph_engine::kMaxOutfitLength) {
        problems.push_back("The outfit name is longer than " + std::to_string(graph_engine::kMaxOutfitLength) + " characters and will be cut.");
    }
    return problems;
}

float MassKg(const CharacterPanelParameters& parameters) {
    const CharacterPanelParameters clean = Clamped(parameters);
    return clean.bodyMassIndex * clean.statureMeters * clean.statureMeters;
}

character::CharacterParameters ToGeneratorParameters(const CharacterPanelParameters& parameters) {
    const CharacterPanelParameters clean = Clamped(parameters);
    character::CharacterParameters generator;
    generator.masculinity = clean.masculinity;
    generator.ageYears = clean.ageYears;
    generator.statureMeters = clean.statureMeters;
    generator.massKg = MassKg(clean);
    generator.muscularity = clean.muscularity;
    generator.seed = static_cast<uint32_t>(clean.seed);
    return character::Sanitised(generator);
}

graph_engine::CharacterSpec ToSpawnSpec(const CharacterPanelParameters& parameters) {
    const character::CharacterParameters generator = ToGeneratorParameters(parameters);
    graph_engine::CharacterSpec spec;
    spec.statureMeters = generator.statureMeters;
    spec.massKg = generator.massKg;
    spec.ageYears = generator.ageYears;
    spec.masculinity = generator.masculinity;
    spec.muscularity = generator.muscularity;
    spec.seed = generator.seed;
    spec.outfit = Clamped(parameters).outfit;
    return graph_engine::SanitisedCharacter(spec);
}

CharacterPanelParameters RandomPanelParameters(int seed) {
    const character::CharacterParameters drawn = character::RandomParameters(static_cast<uint32_t>(std::max(seed, 0)));
    CharacterPanelParameters parameters;
    parameters.statureMeters = drawn.statureMeters;
    parameters.bodyMassIndex = character::BodyMassIndex(drawn);
    parameters.ageYears = drawn.ageYears;
    parameters.masculinity = drawn.masculinity;
    parameters.muscularity = drawn.muscularity;
    parameters.seed = static_cast<int>(drawn.seed % (kSeedMax + 1));
    return Clamped(parameters);
}

CharacterReport MakeCharacterReport(const CharacterPanelParameters& parameters, const character::MeshSettings& settings) {
    const auto begin = std::chrono::steady_clock::now();
    // One thread: the work is one job beside the frame, and must not queue its pieces in front of the crowd's.
    const character::CharacterMesh mesh = character::GenerateCharacter(ToGeneratorParameters(parameters), settings, nullptr);
    CharacterReport report;
    report.vertexCount = mesh.positions.size();
    report.triangleCount = mesh.indices.size() / 3;
    report.pieceCount = mesh.pieces.size();
    report.massKg = mesh.massKg;
    if (!mesh.positions.empty()) {
        const auto [lowest, highest] = std::minmax_element(mesh.positions.begin(), mesh.positions.end(),
                                                           [](const glm::vec3& a, const glm::vec3& b) { return a.y < b.y; });
        report.statureMeters = highest->y - lowest->y;
    }
    report.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    return report;
}

bool CharacterGeneratorModel::Generate() {
    // The parameters are copied now: the sliders go on moving while the person is made.
    const CharacterPanelParameters wanted = Clamped(parameters);
    const character::MeshSettings settings = meshSettings;
    return task.Start([wanted, settings]() { return Made{ToSpawnSpec(wanted), MakeCharacterReport(wanted, settings)}; });
}

std::optional<CharacterGeneratorModel::Finished> CharacterGeneratorModel::Poll() {
    std::optional<TaskOutcome<Made>> outcome = task.Poll();
    if (!outcome) {
        return std::nullopt;
    }
    if (!outcome->value) {
        lastError = outcome->error.empty() ? "The character could not be made." : outcome->error;
        return std::nullopt;
    }
    lastError.clear();
    lastReport = outcome->value->report;
    return Finished{outcome->value->spec, outcome->value->report};
}

}  // namespace panel_model
