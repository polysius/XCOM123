#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "material/MaterialGenerator.h"
#include "material/MaterialLibrary.h"
#include "panel_model/BackgroundTask.h"

// The model behind the material generator panel: the kind, seed and wear the controls hold, the small preview the CPU generator makes
// of them, and the whole library that Apply builds for the renderer. The preview is a grid of colours the panel draws as squares, so no
// GPU texture is needed. The panel only draws this, so all of it can be tested without a window.

namespace panel_model {

struct MaterialPanelParameters {
    material::MaterialKind kind = material::MaterialKind::Stone;
    int seed = 1;
    float wear = 0.5f;
    float scanDetail = 0.6f;

    bool operator==(const MaterialPanelParameters& other) const {
        return kind == other.kind && seed == other.seed && wear == other.wear && scanDetail == other.scanDetail;
    }
};

inline constexpr int kMaterialSeedMax = 99999;
inline constexpr int kPreviewPixels = 128;  // the generator's smallest size, which is fast: the preview follows the controls
inline constexpr int kPreviewCells = 32;    // squares along each side of the swatch
inline constexpr int kApplyLibraryPixels = 512;  // each layer of the library Apply builds, as the engine makes it by default

// The same parameters with every number finite and inside its range, and a kind that exists.
MaterialPanelParameters Clamped(const MaterialPanelParameters& parameters);

// The kinds in the order of a drop-down list, with the names to show.
const std::vector<material::MaterialKind>& AllKinds();
// The name of a kind with a capital: "Stone".
std::string DisplayName(material::MaterialKind kind);

material::MaterialSettings ToMaterialSettings(const MaterialPanelParameters& parameters, int pixels);
material::LibrarySettings ToLibrarySettings(const MaterialPanelParameters& parameters, int pixels);

// What the CPU generator made of the parameters, shrunk to a grid of colours to show.
struct MaterialPreview {
    MaterialPanelParameters parameters;      // what it was made from
    std::vector<glm::vec3> colours;          // kPreviewCells squared, rows from top to bottom, sRGB 0 to 1
    glm::vec3 averageColour{0.0f};           // sRGB
    float averageRoughness = 0.0f;
    float averageMetallic = 0.0f;
    float tileMeters = 0.0f;
    float reliefMillimeters = 0.0f;
    double milliseconds = 0.0;               // how long making it took
};

// Makes the preview on the calling thread.
MaterialPreview MakeMaterialPreview(const MaterialPanelParameters& parameters);

// The material generator panel's work: a preview that follows the controls, and the library for Apply, each in the background.
class MaterialGeneratorModel {
public:
    MaterialPanelParameters parameters;

    // Call once per frame. Collects what finished, and, when `wantPreview` is true, starts a new preview if the controls differ from the
    // last one made and none is running.
    void Update(bool wantPreview = true);
    const std::optional<MaterialPreview>& Preview() const { return preview; }
    bool IsPreviewRunning() const { return previewTask.IsRunning(); }

    // Starts building the whole library from the seed, wear and detail (every kind: the kind chosen only decides the preview). Returns
    // false while one is being built.
    bool Apply();
    bool IsApplying() const { return libraryTask.IsRunning(); }
    double SecondsApplying() const { return libraryTask.SecondsRunning(); }
    // The finished library, once, for the caller to give to the renderer; empty until a build has finished.
    std::optional<material::MaterialLibrary> TakeLibrary();
    // How long the last finished library took to build, in milliseconds, and the reason the last build failed, or empty.
    double LastLibraryMilliseconds() const { return lastLibraryMilliseconds; }
    const std::string& LastError() const { return lastError; }

private:
    BackgroundTask<MaterialPreview> previewTask;
    BackgroundTask<material::MaterialLibrary> libraryTask;
    std::optional<MaterialPreview> preview;
    std::optional<material::MaterialLibrary> finishedLibrary;
    double lastLibraryMilliseconds = 0.0;
    std::string lastError;
};

}  // namespace panel_model
