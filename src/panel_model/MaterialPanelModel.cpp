#include "panel_model/MaterialPanelModel.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>

#include "graph_engine/EngineHost.h"

namespace panel_model {
namespace {

float LinearToSrgb(float linear) {
    const float value = std::clamp(linear, 0.0f, 1.0f);
    return value <= 0.0031308f ? 12.92f * value : 1.055f * std::pow(value, 1.0f / 2.4f) - 0.055f;
}

glm::vec3 LinearToSrgb(glm::vec3 linear) {
    return {LinearToSrgb(linear.x), LinearToSrgb(linear.y), LinearToSrgb(linear.z)};
}

}  // namespace

MaterialPanelParameters Clamped(const MaterialPanelParameters& parameters) {
    const MaterialPanelParameters defaults;
    MaterialPanelParameters clean;
    clean.kind = static_cast<int>(parameters.kind) < material::kMaterialKindCount ? parameters.kind : defaults.kind;
    clean.seed = std::clamp(parameters.seed, 0, kMaterialSeedMax);
    clean.wear = graph_engine::ClampOr(parameters.wear, 0.0f, 1.0f, defaults.wear);
    clean.scanDetail = graph_engine::ClampOr(parameters.scanDetail, 0.0f, 1.0f, defaults.scanDetail);
    return clean;
}

const std::vector<material::MaterialKind>& AllKinds() {
    static const std::vector<material::MaterialKind> kinds = [] {
        std::vector<material::MaterialKind> all;
        for (int index = 0; index < material::kMaterialKindCount; ++index) {
            all.push_back(static_cast<material::MaterialKind>(index));
        }
        return all;
    }();
    return kinds;
}

std::string DisplayName(material::MaterialKind kind) {
    std::string name = material::KindName(kind);
    if (!name.empty()) {
        name[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(name[0])));
    }
    return name;
}

material::MaterialSettings ToMaterialSettings(const MaterialPanelParameters& parameters, int pixels) {
    const MaterialPanelParameters clean = Clamped(parameters);
    material::MaterialSettings settings;
    settings.kind = clean.kind;
    settings.seed = static_cast<uint32_t>(clean.seed);
    settings.size = pixels;
    settings.wear = clean.wear;
    settings.scanDetail = clean.scanDetail;
    return settings;
}

material::LibrarySettings ToLibrarySettings(const MaterialPanelParameters& parameters, int pixels) {
    const MaterialPanelParameters clean = Clamped(parameters);
    material::LibrarySettings settings;
    settings.seed = static_cast<uint32_t>(clean.seed);
    settings.size = pixels;
    settings.wear = clean.wear;
    settings.scanDetail = clean.scanDetail;
    return settings;
}

MaterialPreview MakeMaterialPreview(const MaterialPanelParameters& parameters) {
    const auto begin = std::chrono::steady_clock::now();
    MaterialPreview preview;
    preview.parameters = Clamped(parameters);
    const material::MaterialMaps maps = material::GenerateMaterial(ToMaterialSettings(preview.parameters, kPreviewPixels), nullptr);

    // Each square of the swatch is the average of the pixels under it, averaged in linear light as a camera would see them.
    const int pixelsPerCell = maps.size / kPreviewCells;
    preview.colours.assign(static_cast<size_t>(kPreviewCells) * kPreviewCells, glm::vec3(0.0f));
    glm::dvec3 albedoSum(0.0);
    double roughnessSum = 0.0;
    double metallicSum = 0.0;
    for (int cellY = 0; cellY < kPreviewCells; ++cellY) {
        for (int cellX = 0; cellX < kPreviewCells; ++cellX) {
            glm::vec3 sum(0.0f);
            for (int y = 0; y < pixelsPerCell; ++y) {
                for (int x = 0; x < pixelsPerCell; ++x) {
                    const size_t pixel = static_cast<size_t>(cellY * pixelsPerCell + y) * static_cast<size_t>(maps.size) + static_cast<size_t>(cellX * pixelsPerCell + x);
                    sum += maps.albedo[pixel];
                    roughnessSum += maps.roughness[pixel];
                    metallicSum += maps.metallic[pixel];
                }
            }
            const glm::vec3 average = sum / static_cast<float>(pixelsPerCell * pixelsPerCell);
            preview.colours[static_cast<size_t>(cellY) * kPreviewCells + static_cast<size_t>(cellX)] = LinearToSrgb(average);
            albedoSum += glm::dvec3(average);
        }
    }
    const double cells = static_cast<double>(kPreviewCells) * kPreviewCells;
    const double pixels = cells * pixelsPerCell * pixelsPerCell;
    preview.averageColour = LinearToSrgb(glm::vec3(albedoSum / cells));
    preview.averageRoughness = static_cast<float>(roughnessSum / pixels);
    preview.averageMetallic = static_cast<float>(metallicSum / pixels);
    preview.tileMeters = maps.tileMeters;
    preview.reliefMillimeters = maps.reliefMeters * 1000.0f;
    preview.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    return preview;
}

void MaterialGeneratorModel::Update(bool wantPreview) {
    if (std::optional<TaskOutcome<MaterialPreview>> done = previewTask.Poll()) {
        if (done->value) {
            preview = std::move(done->value);
        } else {
            lastError = done->error.empty() ? "The preview could not be made." : done->error;
        }
    }
    if (std::optional<TaskOutcome<material::MaterialLibrary>> done = libraryTask.Poll()) {
        if (done->value) {
            finishedLibrary = std::move(done->value);
            lastLibraryMilliseconds = done->milliseconds;
            lastError.clear();
        } else {
            lastError = done->error.empty() ? "The materials could not be made." : done->error;
        }
    }
    const MaterialPanelParameters wanted = Clamped(parameters);
    if (wantPreview && !previewTask.IsRunning() && (!preview || !(preview->parameters == wanted))) {
        previewTask.Start([wanted]() { return MakeMaterialPreview(wanted); });
    }
}

bool MaterialGeneratorModel::Apply() {
    const material::LibrarySettings settings = ToLibrarySettings(parameters, kApplyLibraryPixels);
    // One thread, as for the character: the build is one job beside the frame.
    return libraryTask.Start([settings]() { return material::BuildMaterialLibrary(settings, nullptr); });
}

std::optional<material::MaterialLibrary> MaterialGeneratorModel::TakeLibrary() {
    std::optional<material::MaterialLibrary> library = std::move(finishedLibrary);
    finishedLibrary.reset();
    return library;
}

}  // namespace panel_model
