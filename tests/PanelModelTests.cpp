// Tests of the models behind the generator panels (no window, no GPU): the numbers the sliders hold, how they are brought into range
// and turned into the parameters of the generators, and the generators running in the background beside the frame.
//
// What the models must do, and what these tests hold them to: any number a control can hold ends up inside the range of the generator,
// the same controls give the same result, a background run finishes and is collected once, a failing run is reported and not fatal, a
// second run cannot start over the first, and the preview and the library are made from the controls and not from anything else.
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "graph_engine/EngineHost.h"
#include "panel_model/BackgroundTask.h"
#include "panel_model/CharacterPanelModel.h"
#include "panel_model/MaterialPanelModel.h"

namespace {

using namespace panel_model;

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
constexpr auto kLongestWait = std::chrono::seconds(120);

// Calls `poll` until it says it is done or the wait is over; returns whether it finished.
bool WaitFor(const std::function<bool()>& poll) {
    const auto begin = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - begin < kLongestWait) {
        if (poll()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

// A coarse and quick mesh, so that tests of the background run do not wait for the full detail of a character.
character::MeshSettings CoarseMesh() {
    character::MeshSettings settings;
    settings.voxelMeters = 0.05f;
    settings.faceAndHands = false;
    return settings;
}

// ----- The background task -----------------------------------------------------------------------------------------------

void ATaskGivesItsResultOnceAndThenIsFree() {
    BackgroundTask<int> task;
    Check(!task.IsRunning() && !task.Poll(), "a task that was never started has nothing to give");
    Check(task.Start([] { return 41 + 1; }), "it starts");
    Check(task.IsRunning(), "and is running until its result is collected");
    std::optional<TaskOutcome<int>> outcome;
    Check(WaitFor([&] { outcome = task.Poll(); return outcome.has_value(); }), "it finishes");
    Check(outcome && outcome->value && *outcome->value == 42 && outcome->error.empty(), "with the result of the work");
    Check(!task.IsRunning() && !task.Poll(), "the result is given once");
    Check(task.Start([] { return 7; }), "and then the task can run again");
    Check(WaitFor([&] { outcome = task.Poll(); return outcome.has_value(); }) && *outcome->value == 7, "with a new result");
}

void ASecondRunCannotStartOverTheFirst() {
    BackgroundTask<int> task;
    Check(task.Start([] {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return 1;
    }),
          "the first starts");
    Check(!task.Start([] { return 2; }), "the second is refused while the first has not been collected");
    std::optional<TaskOutcome<int>> outcome;
    WaitFor([&] { outcome = task.Poll(); return outcome.has_value(); });
    Check(outcome && *outcome->value == 1, "the first run is the one that finished");
}

void WorkThatThrowsIsReportedAndDoesNotEndTheProgram() {
    BackgroundTask<int> task;
    task.Start([]() -> int { throw std::runtime_error("no luck"); });
    std::optional<TaskOutcome<int>> outcome;
    Check(WaitFor([&] { outcome = task.Poll(); return outcome.has_value(); }), "the task ends");
    Check(outcome && !outcome->value && outcome->error == "no luck", "with the reason and no value");
    task.Start([]() -> int { throw 5; });
    WaitFor([&] { outcome = task.Poll(); return outcome.has_value(); });
    Check(outcome && !outcome->value && !outcome->error.empty(), "also when what was thrown is not an exception");
}

void DroppingARunningTaskWaitsForItsWork() {
    bool finished = false;
    {
        BackgroundTask<int> task;
        task.Start([&finished] {
            std::this_thread::sleep_for(std::chrono::milliseconds(60));
            finished = true;
            return 0;
        });
    }
    Check(finished, "the destructor waits, so the work never touches what is gone");
}

// ----- The character panel -----------------------------------------------------------------------------------------------

void CharacterNumbersAreBroughtIntoRangeWhateverTheyAre() {
    CharacterPanelParameters wild;
    wild.statureMeters = 9.0f;
    wild.bodyMassIndex = -3.0f;
    wild.ageYears = kNaN;
    wild.masculinity = 5.0f;
    wild.muscularity = -1.0f;
    wild.seed = -50;
    wild.outfit = std::string(300, 'x');
    const CharacterPanelParameters clean = Clamped(wild);
    Check(clean.statureMeters == graph_engine::kStatureMax, "stature is limited to the tallest");
    Check(clean.bodyMassIndex == kBodyMassIndexMin, "the body mass index to the lowest");
    Check(clean.ageYears == CharacterPanelParameters{}.ageYears, "an age that is not a number is the default");
    Check(clean.masculinity == 1.0f && clean.muscularity == 0.0f, "sex and muscularity are kept between 0 and 1");
    Check(clean.seed == 0, "the seed is not negative");
    Check(clean.outfit.size() == graph_engine::kMaxOutfitLength, "the outfit name is cut");
    Check(Problems(clean).empty(), "what has been brought into range has no problems left");

    CharacterPanelParameters huge;
    huge.seed = 2000000000;
    Check(Clamped(huge).seed == kSeedMax, "a huge seed is the largest");
}

void ProblemsNameEveryNumberThatIsOutOfRangeAndNothingElse() {
    Check(Problems(CharacterPanelParameters{}).empty(), "the defaults are fine");
    CharacterPanelParameters wild;
    wild.statureMeters = 3.0f;
    wild.bodyMassIndex = 60.0f;
    wild.ageYears = 5.0f;
    wild.masculinity = 2.0f;
    wild.muscularity = kNaN;
    wild.seed = -1;
    wild.outfit = std::string(100, 'x');
    const std::vector<std::string> problems = Problems(wild);
    Check(problems.size() == 7, "one problem for each of the seven (" + std::to_string(problems.size()) + ")");
    const char* names[] = {"Stature", "Body mass index", "Age", "Sex", "Muscularity", "Seed", "outfit"};
    for (const char* name : names) {
        bool found = false;
        for (const std::string& problem : problems) {
            found = found || problem.find(name) != std::string::npos;
        }
        Check(found, std::string("a problem names ") + name);
    }
    CharacterPanelParameters edge;
    edge.statureMeters = graph_engine::kStatureMin;
    edge.ageYears = graph_engine::kAgeMax;
    edge.seed = kSeedMax;
    Check(Problems(edge).empty(), "the ends of the ranges are in range");
}

void TheBodyMassIndexGivesTheMassAtAnyStature() {
    CharacterPanelParameters parameters;
    parameters.statureMeters = 1.80f;
    parameters.bodyMassIndex = 25.0f;
    Check(Near(MassKg(parameters), 81.0f, 0.01f), "BMI 25 at 1.80 m is 81 kg");
    parameters.statureMeters = 1.50f;
    Check(Near(MassKg(parameters), 56.25f, 0.01f), "and 56.25 kg at 1.50 m");
    const character::CharacterParameters generator = ToGeneratorParameters(parameters);
    Check(Near(generator.statureMeters, 1.50f) && Near(generator.massKg, 56.25f, 0.01f), "the generator is given that stature and mass");
    Check(generator.seed == 1u && Near(generator.masculinity, 0.5f), "and the seed and the sex");
}

void ASpawnSpecCarriesTheSameNumbersAsTheGenerator() {
    CharacterPanelParameters parameters;
    parameters.statureMeters = 1.9f;
    parameters.bodyMassIndex = 27.0f;
    parameters.ageYears = 52.0f;
    parameters.masculinity = 0.8f;
    parameters.muscularity = 0.9f;
    parameters.seed = 314;
    parameters.outfit = "suit";
    const graph_engine::CharacterSpec spec = ToSpawnSpec(parameters);
    const character::CharacterParameters generator = ToGeneratorParameters(parameters);
    Check(spec.statureMeters == generator.statureMeters && spec.massKg == generator.massKg && spec.ageYears == generator.ageYears &&
              spec.masculinity == generator.masculinity && spec.muscularity == generator.muscularity && spec.seed == 314u && spec.outfit == "suit",
          "the spec is the generator's person, with the outfit");
    CharacterPanelParameters wild;
    wild.statureMeters = kNaN;
    const graph_engine::CharacterSpec fallback = ToSpawnSpec(wild);
    Check(Near(fallback.statureMeters, 1.70f), "bad numbers still give a spec that can be spawned");
}

void RandomPeopleRepeatForTheSameSeedAndStayInRange() {
    std::set<float> statures;
    for (int seed = 0; seed < 40; ++seed) {
        const CharacterPanelParameters a = RandomPanelParameters(seed);
        const CharacterPanelParameters b = RandomPanelParameters(seed);
        Check(a.statureMeters == b.statureMeters && a.bodyMassIndex == b.bodyMassIndex && a.seed == b.seed, "the same seed draws the same person");
        Check(Problems(a).empty(), "a person that is drawn needs nothing brought into range");
        statures.insert(a.statureMeters);
    }
    Check(statures.size() > 20, "different seeds draw different people");
}

void ACharacterIsMadeInTheBackgroundAndMeasured() {
    CharacterGeneratorModel model;
    model.meshSettings = CoarseMesh();
    model.parameters.statureMeters = 1.80f;
    model.parameters.bodyMassIndex = 24.0f;
    model.parameters.seed = 5;
    model.parameters.outfit = "coat";
    Check(model.Generate(), "making a person starts");
    Check(model.IsRunning() && !model.Generate(), "and a second cannot start over it");
    // The sliders move while the person is made; the person is the one that was asked for.
    model.parameters.statureMeters = 1.50f;
    std::optional<CharacterGeneratorModel::Finished> finished;
    Check(WaitFor([&] { finished = model.Poll(); return finished.has_value(); }), "the person is made");
    Check(!model.IsRunning(), "nothing runs afterwards");
    if (!finished) {
        return;
    }
    Check(Near(finished->spec.statureMeters, 1.80f) && finished->spec.seed == 5u && finished->spec.outfit == "coat", "it is made from the numbers at the start");
    const CharacterReport& report = finished->report;
    Check(report.vertexCount > 1000 && report.triangleCount > 1000 && report.pieceCount >= 1, "a mesh with vertices, triangles and pieces");
    Check(Near(report.statureMeters, 1.80f, 0.12f), "the mesh is as tall as asked (" + std::to_string(report.statureMeters) + ")");
    Check(Near(report.massKg, 1.80f * 1.80f * 24.0f, 6.0f), "and as heavy (" + std::to_string(report.massKg) + ")");
    Check(report.milliseconds > 0.0, "the time it took is measured");
    Check(model.LastReport() && model.LastReport()->vertexCount == report.vertexCount && model.LastError().empty(), "the model remembers the numbers");
    Check(!model.Poll(), "and gives the person once");
}

void TheSameControlsMakeTheSamePerson() {
    CharacterPanelParameters parameters;
    parameters.seed = 11;
    const CharacterReport first = MakeCharacterReport(parameters, CoarseMesh());
    const CharacterReport second = MakeCharacterReport(parameters, CoarseMesh());
    Check(first.vertexCount == second.vertexCount && first.triangleCount == second.triangleCount && first.massKg == second.massKg, "the mesh is the same");
    parameters.seed = 12;
    const CharacterReport other = MakeCharacterReport(parameters, CoarseMesh());
    Check(other.vertexCount != first.vertexCount || other.massKg != first.massKg || other.statureMeters != first.statureMeters, "another seed gives another person");
}

// ----- The material panel ------------------------------------------------------------------------------------------------

void MaterialNumbersAreBroughtIntoRange() {
    MaterialPanelParameters wild;
    wild.kind = static_cast<material::MaterialKind>(99);
    wild.seed = -4;
    wild.wear = 7.0f;
    wild.scanDetail = kNaN;
    const MaterialPanelParameters clean = Clamped(wild);
    Check(clean.kind == MaterialPanelParameters{}.kind, "a kind that does not exist is the default");
    Check(clean.seed == 0 && clean.wear == 1.0f && clean.scanDetail == MaterialPanelParameters{}.scanDetail, "seed, wear and detail are in range");
    MaterialPanelParameters huge;
    huge.seed = 2000000000;
    Check(Clamped(huge).seed == kMaterialSeedMax, "a huge seed is the largest");
}

void TheListOfKindsHasEveryKindOnceWithAName() {
    const std::vector<material::MaterialKind>& kinds = AllKinds();
    Check(kinds.size() == static_cast<size_t>(material::kMaterialKindCount), "every kind is in the list");
    std::set<std::string> names;
    for (material::MaterialKind kind : kinds) {
        const std::string name = DisplayName(kind);
        Check(!name.empty() && std::isupper(static_cast<unsigned char>(name[0])), "the name starts with a capital: " + name);
        names.insert(name);
        material::MaterialKind parsed = material::MaterialKind::Stone;
        std::string lower = name;
        lower[0] = static_cast<char>(std::tolower(static_cast<unsigned char>(lower[0])));
        Check(material::ParseKind(lower, parsed) && parsed == kind, "the name of " + name + " parses back to the kind");
    }
    Check(names.size() == kinds.size(), "no two kinds share a name");
}

void TheSettingsOfTheGeneratorsComeFromTheControls() {
    MaterialPanelParameters parameters;
    parameters.kind = material::MaterialKind::Brick;
    parameters.seed = 77;
    parameters.wear = 0.25f;
    parameters.scanDetail = 0.9f;
    const material::MaterialSettings settings = ToMaterialSettings(parameters, 128);
    Check(settings.kind == material::MaterialKind::Brick && settings.seed == 77u && settings.size == 128 && settings.wear == 0.25f && settings.scanDetail == 0.9f,
          "the material settings");
    const material::LibrarySettings library = ToLibrarySettings(parameters, 512);
    Check(library.seed == 77u && library.size == 512 && library.wear == 0.25f && library.scanDetail == 0.9f, "the library settings");
}

void APreviewIsAGridOfColoursThatFollowsTheKindAndTheSeed() {
    MaterialPanelParameters stone;
    stone.kind = material::MaterialKind::Stone;
    const MaterialPreview first = MakeMaterialPreview(stone);
    const MaterialPreview again = MakeMaterialPreview(stone);
    Check(first.colours.size() == static_cast<size_t>(kPreviewCells) * kPreviewCells, "a square grid of colours");
    bool inRange = true;
    for (const glm::vec3& colour : first.colours) {
        inRange = inRange && colour.r >= 0.0f && colour.r <= 1.0f && colour.g >= 0.0f && colour.g <= 1.0f && colour.b >= 0.0f && colour.b <= 1.0f;
    }
    Check(inRange, "every colour is a colour a screen can show");
    Check(first.colours == again.colours && first.averageColour == again.averageColour, "the same controls give the same preview");
    Check(first.tileMeters > 0.0f && first.reliefMillimeters > 0.0f && first.averageRoughness > 0.0f && first.milliseconds > 0.0, "with the numbers of the material");
    Check(first.parameters == stone, "and says what it was made from");

    MaterialPanelParameters wood = stone;
    wood.kind = material::MaterialKind::Wood;
    const MaterialPreview other = MakeMaterialPreview(wood);
    Check(glm::distance(first.averageColour, other.averageColour) > 0.02f, "another kind looks different");
    MaterialPanelParameters reseeded = stone;
    reseeded.seed = 999;
    Check(MakeMaterialPreview(reseeded).colours != first.colours, "another seed gives another pattern");

    MaterialPanelParameters metal = stone;
    metal.kind = material::MaterialKind::Metal;
    Check(MakeMaterialPreview(metal).averageMetallic > first.averageMetallic, "metal is more metallic than stone");
}

void ThePreviewFollowsTheControlsInTheBackground() {
    MaterialGeneratorModel model;
    Check(!model.Preview(), "there is no preview before the first update");
    model.Update(false);
    Check(!model.IsPreviewRunning(), "a hidden panel makes none");
    model.Update(true);
    Check(model.IsPreviewRunning(), "a shown one starts it");
    Check(WaitFor([&] { model.Update(false); return model.Preview().has_value(); }), "the preview arrives");
    Check(model.Preview()->parameters == Clamped(model.parameters), "it is of the controls as they were");
    model.Update(true);
    Check(!model.IsPreviewRunning(), "with the controls unchanged nothing new is started");
    model.parameters.kind = material::MaterialKind::Leather;
    model.Update(true);
    Check(model.IsPreviewRunning(), "a change of the controls starts a new preview");
    Check(WaitFor([&] { model.Update(false); return model.Preview()->parameters.kind == material::MaterialKind::Leather; }), "and it shows the new kind");
}

void ApplyBuildsTheLibraryOnceFromTheSeedWearAndDetail() {
    MaterialGeneratorModel model;
    model.parameters.seed = 3;
    model.parameters.wear = 0.2f;
    Check(!model.TakeLibrary(), "there is no library before Apply");
    Check(model.Apply() && model.IsApplying(), "Apply starts the build");
    Check(!model.Apply(), "and cannot start another over it");
    std::optional<material::MaterialLibrary> library;
    Check(WaitFor([&] {
              model.Update(false);
              library = model.TakeLibrary();
              return library.has_value();
          }),
          "the library is built");
    Check(!model.IsApplying() && !model.TakeLibrary(), "it is given once");
    if (library) {
        Check(library->size == kApplyLibraryPixels && library->layers == material::kMaterialKindCount, "a layer of every kind at the size the engine uses");
        const material::MaterialLibrary same = material::BuildMaterialLibrary(ToLibrarySettings(model.parameters, kApplyLibraryPixels), nullptr);
        Check(library->albedoRoughness == same.albedoRoughness, "it is what the engine's own builder makes from those settings");
        material::LibrarySettings other = ToLibrarySettings(model.parameters, kApplyLibraryPixels);
        other.seed = 4;
        Check(material::BuildMaterialLibrary(other, nullptr).albedoRoughness != library->albedoRoughness, "and another seed would have made another");
    }
    Check(model.LastLibraryMilliseconds() > 0.0 && model.LastError().empty(), "the time is measured");
}

}  // namespace

int main() {
    struct Test {
        const char* name;
        std::function<void()> run;
    };
    const Test tests[] = {
        {"ATaskGivesItsResultOnceAndThenIsFree", ATaskGivesItsResultOnceAndThenIsFree},
        {"ASecondRunCannotStartOverTheFirst", ASecondRunCannotStartOverTheFirst},
        {"WorkThatThrowsIsReportedAndDoesNotEndTheProgram", WorkThatThrowsIsReportedAndDoesNotEndTheProgram},
        {"DroppingARunningTaskWaitsForItsWork", DroppingARunningTaskWaitsForItsWork},
        {"CharacterNumbersAreBroughtIntoRangeWhateverTheyAre", CharacterNumbersAreBroughtIntoRangeWhateverTheyAre},
        {"ProblemsNameEveryNumberThatIsOutOfRangeAndNothingElse", ProblemsNameEveryNumberThatIsOutOfRangeAndNothingElse},
        {"TheBodyMassIndexGivesTheMassAtAnyStature", TheBodyMassIndexGivesTheMassAtAnyStature},
        {"ASpawnSpecCarriesTheSameNumbersAsTheGenerator", ASpawnSpecCarriesTheSameNumbersAsTheGenerator},
        {"RandomPeopleRepeatForTheSameSeedAndStayInRange", RandomPeopleRepeatForTheSameSeedAndStayInRange},
        {"ACharacterIsMadeInTheBackgroundAndMeasured", ACharacterIsMadeInTheBackgroundAndMeasured},
        {"TheSameControlsMakeTheSamePerson", TheSameControlsMakeTheSamePerson},
        {"MaterialNumbersAreBroughtIntoRange", MaterialNumbersAreBroughtIntoRange},
        {"TheListOfKindsHasEveryKindOnceWithAName", TheListOfKindsHasEveryKindOnceWithAName},
        {"TheSettingsOfTheGeneratorsComeFromTheControls", TheSettingsOfTheGeneratorsComeFromTheControls},
        {"APreviewIsAGridOfColoursThatFollowsTheKindAndTheSeed", APreviewIsAGridOfColoursThatFollowsTheKindAndTheSeed},
        {"ThePreviewFollowsTheControlsInTheBackground", ThePreviewFollowsTheControlsInTheBackground},
        {"ApplyBuildsTheLibraryOnceFromTheSeedWearAndDetail", ApplyBuildsTheLibraryOnceFromTheSeedWearAndDetail},
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
    std::cout << (failureCount == 0 ? "all panel model tests passed\n" : "panel model tests failed\n");
    return failureCount == 0 ? 0 : 1;
}
