// material_gen makes a procedural material and writes its maps as PNG files, with a lit preview of the tile repeated
// twice in each direction (so a seam would show). It needs no window and no GPU.
//
//   material_gen --kind stone --seed 3 --size 1024 --wear 0.5 --detail 0.6 --out materials
//   material_gen --kind all   (every kind, into the folder given by --out)
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "core/JobSystem.h"
#include "core/PngWriter.h"
#include "material/MaterialGenerator.h"

namespace {

using material::MaterialKind;
using material::MaterialMaps;
using material::MaterialSettings;

uint8_t ToByte(float value) {
    return static_cast<uint8_t>(std::lround(std::clamp(value, 0.0f, 1.0f) * 255.0f));
}

float ToSrgb(float linear) {
    linear = std::clamp(linear, 0.0f, 1.0f);
    return linear <= 0.0031308f ? 12.92f * linear : 1.055f * std::pow(linear, 1.0f / 2.4f) - 0.055f;
}

void PutPixel(std::vector<uint8_t>& rgba, size_t index, float r, float g, float b) {
    rgba[index * 4 + 0] = ToByte(r);
    rgba[index * 4 + 1] = ToByte(g);
    rgba[index * 4 + 2] = ToByte(b);
    rgba[index * 4 + 3] = 255;
}

bool WriteMap(const std::filesystem::path& path, const MaterialMaps& maps, const std::vector<uint8_t>& rgba) {
    const bool written = WritePng(path.string(), static_cast<uint32_t>(maps.size), static_cast<uint32_t>(maps.size), rgba);
    if (!written) {
        std::cerr << "could not write " << path << "\n";
    }
    return written;
}

// Writes albedo, normal, roughness, metallic, occlusion and height as separate images.
bool WriteMaps(const std::filesystem::path& folder, const std::string& name, const MaterialMaps& maps) {
    const size_t pixels = static_cast<size_t>(maps.size) * maps.size;
    std::vector<uint8_t> rgba(pixels * 4);
    bool ok = true;

    for (size_t i = 0; i < pixels; ++i) {
        PutPixel(rgba, i, ToSrgb(maps.albedo[i].r), ToSrgb(maps.albedo[i].g), ToSrgb(maps.albedo[i].b));
    }
    ok &= WriteMap(folder / (name + "_albedo.png"), maps, rgba);

    for (size_t i = 0; i < pixels; ++i) {
        PutPixel(rgba, i, 0.5f + 0.5f * maps.normal[i].x, 0.5f + 0.5f * maps.normal[i].y, 0.5f + 0.5f * maps.normal[i].z);
    }
    ok &= WriteMap(folder / (name + "_normal.png"), maps, rgba);

    const auto writeGrey = [&](const char* suffix, auto valueAt) {
        for (size_t i = 0; i < pixels; ++i) {
            const float value = valueAt(i);
            PutPixel(rgba, i, value, value, value);
        }
        ok &= WriteMap(folder / (name + suffix), maps, rgba);
    };
    writeGrey("_roughness.png", [&](size_t i) { return maps.roughness[i]; });
    writeGrey("_metallic.png", [&](size_t i) { return maps.metallic[i]; });
    writeGrey("_occlusion.png", [&](size_t i) { return maps.ambientOcclusion[i]; });
    writeGrey("_height.png", [&](size_t i) { return maps.height[i] / std::max(maps.reliefMeters, 1e-6f); });
    return ok;
}

// A preview of the tile, repeated twice each way, lit from the upper left with a sun and a sky.
bool WritePreview(const std::filesystem::path& path, const MaterialMaps& maps) {
    const int size = maps.size;
    const int side = size * 2;
    std::vector<uint8_t> rgba(static_cast<size_t>(side) * side * 4);
    const glm::vec3 toLight = glm::normalize(glm::vec3(-0.5f, 0.45f, 0.75f));
    const glm::vec3 toViewer(0.0f, 0.0f, 1.0f);
    const glm::vec3 halfway = glm::normalize(toLight + toViewer);

    for (int y = 0; y < side; ++y) {
        for (int x = 0; x < side; ++x) {
            const size_t source = static_cast<size_t>(y % size) * size + (x % size);
            const glm::vec3 normal = maps.normal[source];
            const float metallic = maps.metallic[source];
            const float roughness = maps.roughness[source];
            const float occlusion = maps.ambientOcclusion[source];

            const glm::vec3 albedo = maps.albedo[source];
            const glm::vec3 diffuseColour = albedo * (1.0f - metallic);
            const glm::vec3 specularColour = glm::mix(glm::vec3(0.04f), albedo, metallic);

            const float diffuseLight = std::max(0.0f, glm::dot(normal, toLight));
            const float shininess = 2.0f / std::max(roughness * roughness * roughness * roughness, 0.0004f) - 2.0f;
            const float highlight = std::pow(std::max(0.0f, glm::dot(normal, halfway)), shininess) * (shininess + 8.0f) / 25.0f;
            const glm::vec3 sky = glm::vec3(0.30f, 0.36f, 0.45f) * (0.5f + 0.5f * normal.y);

            const glm::vec3 light = diffuseColour * (glm::vec3(3.0f, 2.8f, 2.5f) * diffuseLight * (0.4f + 0.6f * occlusion) + sky * occlusion) +
                                    specularColour * (highlight * diffuseLight * 2.0f + sky * 1.6f * (1.0f - 0.6f * roughness)) * occlusion;
            const glm::vec3 mapped = light / (light + glm::vec3(1.0f));  // a plain tone curve
            PutPixel(rgba, static_cast<size_t>(y) * side + x, ToSrgb(mapped.r), ToSrgb(mapped.g), ToSrgb(mapped.b));
        }
    }
    return WritePng(path.string(), static_cast<uint32_t>(side), static_cast<uint32_t>(side), rgba);
}

// Reads a number for an option, or says what is wrong and ends the program with a usage error.
double NumberOrExit(const std::string& option, const std::string& text, double lowest, double highest) {
    double value = 0.0;
    size_t used = 0;
    try {
        value = std::stod(text, &used);
    } catch (const std::exception&) {
        used = 0;
    }
    if (used != text.size() || text.empty() || !(value >= lowest && value <= highest)) {
        std::cerr << option << " needs a number from " << lowest << " to " << highest << ", not \"" << text << "\"\n";
        std::exit(2);
    }
    return value;
}

}  // namespace

int main(int argc, char** argv) {
    MaterialSettings settings;
    settings.size = 512;
    std::string kindName = "stone";
    std::filesystem::path folder = "materials";

    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::cerr << argument << " needs a value\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (argument == "--kind") {
            kindName = next();
        } else if (argument == "--seed") {
            settings.seed = static_cast<uint32_t>(NumberOrExit(argument, next(), 0.0, 4294967295.0));
        } else if (argument == "--size") {
            settings.size = static_cast<int>(NumberOrExit(argument, next(), 128.0, 4096.0));
        } else if (argument == "--wear") {
            settings.wear = static_cast<float>(NumberOrExit(argument, next(), 0.0, 1.0));
        } else if (argument == "--detail") {
            settings.scanDetail = static_cast<float>(NumberOrExit(argument, next(), 0.0, 1.0));
        } else if (argument == "--out") {
            folder = next();
        } else {
            std::cerr << "unknown option " << argument << "\n";
            return 2;
        }
    }

    std::vector<MaterialKind> kinds;
    if (kindName == "all") {
        for (int i = 0; i < material::kMaterialKindCount; ++i) {
            kinds.push_back(static_cast<MaterialKind>(i));
        }
    } else {
        MaterialKind kind;
        if (!material::ParseKind(kindName, kind)) {
            std::cerr << "unknown kind '" << kindName << "'; use stone, brick, concrete, plaster, wood, metal, leather, fabric, asphalt, soil or all\n";
            return 2;
        }
        kinds.push_back(kind);
    }

    std::filesystem::create_directories(folder);
    bool ok = true;
    for (const MaterialKind kind : kinds) {
        settings.kind = kind;
        const MaterialMaps maps = material::GenerateMaterial(settings, &JobSystem::Shared());
        const std::string name = material::KindName(kind);
        ok &= WriteMaps(folder, name, maps);
        ok &= WritePreview(folder / (name + "_preview.png"), maps);
        std::cout << name << ": " << maps.size << " px, tile " << maps.tileMeters << " m, relief " << maps.reliefMeters * 1000.0f << " mm\n";
    }
    return ok ? 0 : 1;
}
