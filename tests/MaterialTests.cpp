// Tests of the procedural materials (no window, no GPU). Like the other test programs there is no test framework:
// each test is a function that calls Check, and main reports every failure it finds.
//
// What a laser scanned material must be, and what these tests hold the generator to: the same every time, with no
// seam where it repeats, normals that really are the slope of the height, values a surface can have, a surface that
// is rough at every scale from the whole tile down to the pixel, and a look that matches the kind it claims to be.
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include "core/JobSystem.h"
#include "material/MaterialGenerator.h"
#include "material/MaterialNoise.h"

namespace {

using material::MaterialKind;
using material::MaterialMaps;
using material::MaterialSettings;

int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

constexpr int kTestSize = 256;

MaterialMaps Make(MaterialKind kind, uint32_t seed = 1, float wear = 0.5f, float detail = 0.6f, int size = kTestSize) {
    MaterialSettings settings;
    settings.kind = kind;
    settings.seed = seed;
    settings.size = size;
    settings.wear = wear;
    settings.scanDetail = detail;
    return material::GenerateMaterial(settings);
}

std::string Name(MaterialKind kind) {
    return material::KindName(kind);
}

template <typename Body>
void ForEachKind(const Body& body) {
    for (int i = 0; i < material::kMaterialKindCount; ++i) {
        body(static_cast<MaterialKind>(i));
    }
}

float Luminance(glm::vec3 colour) {
    return 0.2126f * colour.r + 0.7152f * colour.g + 0.0722f * colour.b;
}

float At(const std::vector<float>& image, int size, int x, int y) {
    return image[static_cast<size_t>(((y % size) + size) % size) * size + ((x % size) + size) % size];
}

// The mean step between neighbouring values across the wrap-around edge, divided by the 99th percentile of the steps
// between neighbours inside the tile. Under 1 when the edge is like any other place in the tile, far over 1 when
// a seam shows, since a seam is a jump between unrelated values. The percentile, not the mean, is the yardstick
// because tiles with sharp narrow features (fabric threads, the gaps between planks) have a few big steps by nature.
float SeamRatio(const std::vector<float>& image, int size) {
    std::vector<float> inside;
    double edge = 0.0;
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size - 1; ++x) {
            inside.push_back(std::abs(At(image, size, x + 1, y) - At(image, size, x, y)));
            inside.push_back(std::abs(At(image, size, y, x + 1) - At(image, size, y, x)));
        }
        edge += std::abs(At(image, size, 0, y) - At(image, size, size - 1, y));
        edge += std::abs(At(image, size, y, 0) - At(image, size, y, size - 1));
    }
    edge /= 2.0 * size;
    std::sort(inside.begin(), inside.end());
    const float percentile = inside[static_cast<size_t>(0.99 * static_cast<double>(inside.size() - 1))];
    return static_cast<float>(edge / std::max(static_cast<double>(percentile), 1e-12));
}

std::vector<float> LuminanceOf(const MaterialMaps& maps) {
    std::vector<float> result;
    result.reserve(maps.albedo.size());
    for (const glm::vec3& colour : maps.albedo) {
        result.push_back(Luminance(colour));
    }
    return result;
}

glm::vec3 MeanAlbedo(const MaterialMaps& maps) {
    glm::dvec3 sum(0.0);
    for (const glm::vec3& colour : maps.albedo) {
        sum += glm::dvec3(colour);
    }
    return glm::vec3(sum / static_cast<double>(maps.albedo.size()));
}

float Mean(const std::vector<float>& values) {
    double sum = 0.0;
    for (const float value : values) {
        sum += value;
    }
    return static_cast<float>(sum / static_cast<double>(values.size()));
}

// How much the height bends from one pixel to the next, as a fraction of the relief: the fine scale grain. The second
// difference is used because it ignores slopes, however steep, and responds to what happens at the scale of a pixel.
float FineDetail(const MaterialMaps& maps) {
    double sum = 0.0;
    for (int y = 0; y < maps.size; ++y) {
        for (int x = 0; x < maps.size; ++x) {
            const float bend = At(maps.height, maps.size, x + 1, y) + At(maps.height, maps.size, x - 1, y) - 2.0f * At(maps.height, maps.size, x, y);
            sum += static_cast<double>(bend) * bend;
        }
    }
    return static_cast<float>(std::sqrt(sum / (static_cast<double>(maps.size) * maps.size)) / maps.reliefMeters);
}

// The spread of the heights as a fraction of the relief: the coarse scale shape.
float CoarseDetail(const MaterialMaps& maps) {
    const float mean = Mean(maps.height);
    double sum = 0.0;
    for (const float height : maps.height) {
        sum += static_cast<double>(height - mean) * (height - mean);
    }
    return static_cast<float>(std::sqrt(sum / static_cast<double>(maps.height.size())) / maps.reliefMeters);
}

// Noise primitives ----------------------------------------------------------------------------------

void NoiseRepeatsExactlyAtTheEdgeOfTheTile() {
    for (int i = 0; i < 40; ++i) {
        const float v = 0.0237f * static_cast<float>(i);
        Check(std::abs(material::GradientNoise(1.0f, v, 8, 5, 3) - material::GradientNoise(0.0f, v, 8, 5, 3)) < 1e-4f,
              "gradient noise is the same at u = 1 and u = 0");
        Check(std::abs(material::GradientNoise(v, 1.0f, 8, 5, 3) - material::GradientNoise(v, 0.0f, 8, 5, 3)) < 1e-4f,
              "and at v = 1 and v = 0");
        Check(std::abs(material::Fbm(1.0f, v, 4, 6, 4, 9) - material::Fbm(0.0f, v, 4, 6, 4, 9)) < 1e-4f, "fractal noise too");
        const material::Cellular outside = material::CellularNoise(1.0f + 0.001f, v, 7, 7, 5);
        const material::Cellular inside = material::CellularNoise(0.001f, v, 7, 7, 5);
        Check(std::abs(outside.nearest - inside.nearest) < 1e-3f && outside.cellHash == inside.cellHash, "and cellular noise");
    }
}

void NoiseHasTheRightSizeAndSpread() {
    double sum = 0.0;
    double sumSquares = 0.0;
    float lowest = 1.0f;
    float highest = -1.0f;
    constexpr int kSamples = 100;
    for (int y = 0; y < kSamples; ++y) {
        for (int x = 0; x < kSamples; ++x) {
            const float value = material::Fbm(static_cast<float>(x) / kSamples, static_cast<float>(y) / kSamples, 6, 6, 4, 17);
            sum += value;
            sumSquares += static_cast<double>(value) * value;
            lowest = std::min(lowest, value);
            highest = std::max(highest, value);
        }
    }
    const double mean = sum / (kSamples * kSamples);
    const double spread = std::sqrt(sumSquares / (kSamples * kSamples) - mean * mean);
    Check(std::abs(mean) < 0.05, "noise has no bias, mean " + std::to_string(mean));
    Check(lowest >= -1.2f && highest <= 1.2f, "and stays about between -1 and 1");
    Check(spread > 0.15 && spread < 0.6, "and uses its range, spread " + std::to_string(spread));
    const material::Cellular cell = material::CellularNoise(0.31f, 0.77f, 9, 9, 4);
    Check(cell.nearest <= cell.second && cell.nearest >= 0.0f && cell.second < 1.6f, "cellular distances are ordered and short");
}

// Materials ----------------------------------------------------------------------------------------

void AMaterialIsTheSameEveryTimeAndOnAnyNumberOfThreads() {
    const MaterialMaps first = Make(MaterialKind::Stone, 5);
    const MaterialMaps second = Make(MaterialKind::Stone, 5);
    Check(first.height == second.height && first.albedo == second.albedo && first.normal == second.normal, "the same settings give the same maps");

    JobSystem many(5);
    MaterialSettings settings;
    settings.kind = MaterialKind::Stone;
    settings.seed = 5;
    settings.size = kTestSize;
    const MaterialMaps threaded = material::GenerateMaterial(settings, &many);
    Check(first.height == threaded.height && first.albedo == threaded.albedo && first.roughness == threaded.roughness &&
              first.ambientOcclusion == threaded.ambientOcclusion,
          "on one thread and on six, to the last bit");

    const MaterialMaps other = Make(MaterialKind::Stone, 6);
    double difference = 0.0;
    for (size_t i = 0; i < first.height.size(); ++i) {
        difference += std::abs(first.height[i] - other.height[i]);
    }
    Check(difference / static_cast<double>(first.height.size()) > 0.1 * first.reliefMeters, "another seed gives another surface");
}

void EveryMaterialTilesWithoutASeam() {
    ForEachKind([](MaterialKind kind) {
        const MaterialMaps maps = Make(kind);
        Check(SeamRatio(maps.height, maps.size) < 1.0f, Name(kind) + ": the height has no seam, ratio " + std::to_string(SeamRatio(maps.height, maps.size)));
        Check(SeamRatio(LuminanceOf(maps), maps.size) < 1.0f, Name(kind) + ": nor has the colour, ratio " + std::to_string(SeamRatio(LuminanceOf(maps), maps.size)));
        std::vector<float> normalX;
        for (const glm::vec3& n : maps.normal) {
            normalX.push_back(n.x);
        }
        Check(SeamRatio(normalX, maps.size) < 1.0f, Name(kind) + ": nor has the normal map");
    });
}

void NormalsAreUnitVectorsThatPointOutAndFollowTheHeight() {
    ForEachKind([](MaterialKind kind) {
        const MaterialMaps maps = Make(kind);
        const float pixelMeters = maps.tileMeters / static_cast<float>(maps.size);
        bool unit = true;
        bool outward = true;
        glm::dvec3 sum(0.0);
        for (const glm::vec3& n : maps.normal) {
            unit = unit && std::abs(glm::length(n) - 1.0f) < 1e-3f;
            outward = outward && n.z > 0.0f;
            sum += glm::dvec3(n);
        }
        Check(unit, Name(kind) + ": normals have length 1");
        Check(outward, Name(kind) + ": and point out of the surface");
        const glm::dvec3 mean = sum / static_cast<double>(maps.normal.size());
        Check(std::abs(mean.x) < 0.03 && std::abs(mean.y) < 0.03, Name(kind) + ": and lean no way on average, as a flat tile must");

        // Spot checks against the slope of the height, taken independently here, including across the wrap-around edge.
        const int spots[5][2] = {{0, 0}, {maps.size - 1, 3}, {17, maps.size - 1}, {100, 100}, {maps.size - 1, maps.size - 1}};
        for (const auto& spot : spots) {
            const int x = spot[0];
            const int y = spot[1];
            const float slopeX = (At(maps.height, maps.size, x + 1, y) - At(maps.height, maps.size, x - 1, y)) / (2.0f * pixelMeters);
            const float slopeUp = (At(maps.height, maps.size, x, y - 1) - At(maps.height, maps.size, x, y + 1)) / (2.0f * pixelMeters);
            const glm::vec3 expected = glm::normalize(glm::vec3(-slopeX, -slopeUp, 1.0f));
            const glm::vec3 actual = maps.normal[static_cast<size_t>(y) * maps.size + x];
            Check(glm::distance(expected, actual) < 1e-3f, Name(kind) + ": a normal is the slope of the height at (" + std::to_string(x) + ", " + std::to_string(y) + ")");
        }
    });
}

void ValuesStayWhereASurfaceCanHaveThem() {
    ForEachKind([](MaterialKind kind) {
        for (const float wear : {0.0f, 1.0f}) {
            const MaterialMaps maps = Make(kind, 3, wear);
            bool albedoOk = true;
            bool roughnessOk = true;
            bool metallicOk = true;
            bool occlusionOk = true;
            bool heightOk = true;
            for (size_t i = 0; i < maps.height.size(); ++i) {
                const glm::vec3 a = maps.albedo[i];
                albedoOk = albedoOk && a.r >= 0.019f && a.g >= 0.019f && a.b >= 0.019f && a.r <= 0.951f && a.g <= 0.951f && a.b <= 0.951f;
                roughnessOk = roughnessOk && maps.roughness[i] >= 0.049f && maps.roughness[i] <= 1.001f;
                metallicOk = metallicOk && maps.metallic[i] >= 0.0f && maps.metallic[i] <= 1.0f;
                occlusionOk = occlusionOk && maps.ambientOcclusion[i] >= 0.099f && maps.ambientOcclusion[i] <= 1.001f;
                heightOk = heightOk && std::isfinite(maps.height[i]) && maps.height[i] > -0.01f * maps.reliefMeters && maps.height[i] < 1.01f * maps.reliefMeters;
            }
            const std::string label = Name(kind) + " at wear " + std::to_string(wear);
            Check(albedoOk, label + ": albedo is between 2% and 95%");
            Check(roughnessOk, label + ": roughness is between 0.05 and 1");
            Check(metallicOk, label + ": metallic is between 0 and 1");
            Check(occlusionOk, label + ": occlusion is between 0.1 and 1");
            Check(heightOk, label + ": heights lie within the relief");
        }
    });
}

void TheReliefIsUsedInFull() {
    ForEachKind([](MaterialKind kind) {
        const MaterialMaps maps = Make(kind);
        const float lowest = *std::min_element(maps.height.begin(), maps.height.end());
        const float highest = *std::max_element(maps.height.begin(), maps.height.end());
        Check(std::abs((highest - lowest) - maps.reliefMeters) < 0.05f * maps.reliefMeters, Name(kind) + ": the height spans the relief of the material");
    });
}

void ASurfaceIsRoughAtEveryScale() {
    ForEachKind([](MaterialKind kind) {
        const MaterialMaps maps = Make(kind);
        const float coarse = CoarseDetail(maps);
        const float fine = FineDetail(maps);
        Check(coarse > 0.03f, Name(kind) + ": the tile has shape at large scale, spread " + std::to_string(coarse));
        Check(fine > 0.002f, Name(kind) + ": and detail at the scale of a pixel, bend " + std::to_string(fine));
        // The pixel to pixel steps must not be all of it: a scan of a real surface has a spectrum that falls with scale.
        Check(fine < 0.6f, Name(kind) + ": the pixel bends are not just noise, " + std::to_string(fine));
    });
}

void MoreScanDetailMeansMoreFineGrain() {
    for (const MaterialKind kind : {MaterialKind::Stone, MaterialKind::Concrete, MaterialKind::Soil}) {
        const float smooth = FineDetail(Make(kind, 2, 0.5f, 0.0f));
        const float detailed = FineDetail(Make(kind, 2, 0.5f, 1.0f));
        Check(detailed > smooth, Name(kind) + ": a higher scan detail adds grain: " + std::to_string(smooth) + " to " + std::to_string(detailed));
    }
}

void EachKindLooksLikeItself() {
    const MaterialMaps metal = Make(MaterialKind::Metal, 1, 0.0f);
    Check(Mean(metal.metallic) > 0.8f, "clean metal is metallic: " + std::to_string(Mean(metal.metallic)));
    Check(Luminance(MeanAlbedo(metal)) > 0.3f, "and reflects strongly, as metal does");
    Check(Mean(metal.roughness) < 0.55f, "and is fairly smooth");
    ForEachKind([](MaterialKind kind) {
        if (kind == MaterialKind::Metal) {
            return;
        }
        const MaterialMaps maps = Make(kind);
        Check(Mean(maps.metallic) < 0.01f, Name(kind) + " is not metallic");
        const float brightness = Luminance(MeanAlbedo(maps));
        Check(brightness > 0.03f && brightness < 0.75f, Name(kind) + " reflects between 3% and 75% of the light: " + std::to_string(brightness));
        Check(Mean(maps.roughness) > 0.35f, Name(kind) + " is not shiny");
    });
    const glm::vec3 wood = MeanAlbedo(Make(MaterialKind::Wood));
    Check(wood.r > wood.g && wood.g > wood.b, "wood is brown: red above green above blue");
    const glm::vec3 brick = MeanAlbedo(Make(MaterialKind::Brick));
    Check(brick.r > brick.g && brick.g > brick.b, "brick is red");
    Check(Luminance(MeanAlbedo(Make(MaterialKind::Asphalt, 1, 0.0f))) < 0.15f, "new asphalt is dark");
    Check(Luminance(MeanAlbedo(Make(MaterialKind::Plaster))) > Luminance(MeanAlbedo(Make(MaterialKind::Asphalt))), "plaster is lighter than asphalt");
}

void WearAgesAMaterial() {
    const MaterialMaps newMetal = Make(MaterialKind::Metal, 4, 0.0f);
    const MaterialMaps oldMetal = Make(MaterialKind::Metal, 4, 1.0f);
    Check(Mean(oldMetal.metallic) < Mean(newMetal.metallic) - 0.15f, "old metal has rusted: " + std::to_string(Mean(newMetal.metallic)) + " to " + std::to_string(Mean(oldMetal.metallic)));
    Check(Mean(oldMetal.roughness) > Mean(newMetal.roughness), "and is rougher");

    const MaterialMaps newPlaster = Make(MaterialKind::Plaster, 4, 0.0f);
    const MaterialMaps oldPlaster = Make(MaterialKind::Plaster, 4, 1.0f);
    Check(Mean(oldPlaster.roughness) > Mean(newPlaster.roughness), "old plaster has lost paint, which is rougher");
    double changed = 0.0;
    for (size_t i = 0; i < newPlaster.albedo.size(); ++i) {
        changed += std::abs(Luminance(oldPlaster.albedo[i]) - Luminance(newPlaster.albedo[i]));
    }
    Check(changed / static_cast<double>(newPlaster.albedo.size()) > 0.003, "and the undercoat shows through in places, so its colour differs");

    const MaterialMaps newAsphalt = Make(MaterialKind::Asphalt, 4, 0.0f);
    const MaterialMaps oldAsphalt = Make(MaterialKind::Asphalt, 4, 1.0f);
    Check(Luminance(MeanAlbedo(oldAsphalt)) > Luminance(MeanAlbedo(newAsphalt)), "old asphalt has faded");
}

void PitsAreDarkerThanTheSurfaceAround() {
    // Occlusion is the cavity of the height field, so it must be lower where the surface is low.
    ForEachKind([](MaterialKind kind) {
        const MaterialMaps maps = Make(kind);
        Check(Mean(maps.ambientOcclusion) > 0.4f && Mean(maps.ambientOcclusion) < 0.995f, Name(kind) + ": some of the surface is occluded, but not all: " + std::to_string(Mean(maps.ambientOcclusion)));

        std::vector<size_t> order(maps.height.size());
        for (size_t i = 0; i < order.size(); ++i) {
            order[i] = i;
        }
        std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return maps.height[a] < maps.height[b]; });
        const size_t tenth = order.size() / 10;
        double low = 0.0;
        double high = 0.0;
        for (size_t i = 0; i < tenth; ++i) {
            low += maps.ambientOcclusion[order[i]];
            high += maps.ambientOcclusion[order[order.size() - 1 - i]];
        }
        Check(low < high, Name(kind) + ": the lowest tenth of the surface is more occluded than the highest tenth");
    });
}

void KindNamesRoundTripAndBadNamesAreRefused() {
    ForEachKind([](MaterialKind kind) {
        MaterialKind parsed = MaterialKind::Stone;
        Check(material::ParseKind(material::KindName(kind), parsed) && parsed == kind, Name(kind) + " can be named and found again");
    });
    MaterialKind kind = MaterialKind::Wood;
    Check(!material::ParseKind("granite", kind) && !material::ParseKind("", kind) && kind == MaterialKind::Wood, "a name that is no kind is refused and changes nothing");
}

void TheSizeIsKeptInRange() {
    MaterialSettings settings;
    settings.size = 3;
    Check(material::GenerateMaterial(settings).size == 32, "a size under 32 is raised to 32");
    settings.size = 64;
    const MaterialMaps maps = material::GenerateMaterial(settings);
    Check(maps.size == 64 && maps.height.size() == 64u * 64u && maps.albedo.size() == 64u * 64u && maps.normal.size() == 64u * 64u, "and every map has size * size values");
    settings.wear = 7.0f;
    settings.scanDetail = -3.0f;
    const MaterialMaps clamped = material::GenerateMaterial(settings);
    bool finite = true;
    for (size_t i = 0; i < clamped.height.size(); ++i) {
        finite = finite && std::isfinite(clamped.height[i]) && std::isfinite(clamped.albedo[i].r) && std::isfinite(clamped.normal[i].z);
    }
    Check(finite, "wear and detail outside 0 to 1 are clamped, not passed on");
}

void ASmallMaterialAlsoTiles() {
    // The finishing steps use blurs that scale with the size, so check a small one separately.
    ForEachKind([](MaterialKind kind) {
        const MaterialMaps maps = Make(kind, 8, 0.5f, 0.6f, 64);
        Check(SeamRatio(maps.height, maps.size) < 1.0f, Name(kind) + ": a 64 pixel tile has no seam, ratio " + std::to_string(SeamRatio(maps.height, maps.size)));
    });
}

}  // namespace

int main() {
    struct Test {
        const char* name;
        std::function<void()> run;
    };
    const Test tests[] = {
        {"NoiseRepeatsExactlyAtTheEdgeOfTheTile", NoiseRepeatsExactlyAtTheEdgeOfTheTile},
        {"NoiseHasTheRightSizeAndSpread", NoiseHasTheRightSizeAndSpread},
        {"AMaterialIsTheSameEveryTimeAndOnAnyNumberOfThreads", AMaterialIsTheSameEveryTimeAndOnAnyNumberOfThreads},
        {"EveryMaterialTilesWithoutASeam", EveryMaterialTilesWithoutASeam},
        {"NormalsAreUnitVectorsThatPointOutAndFollowTheHeight", NormalsAreUnitVectorsThatPointOutAndFollowTheHeight},
        {"ValuesStayWhereASurfaceCanHaveThem", ValuesStayWhereASurfaceCanHaveThem},
        {"TheReliefIsUsedInFull", TheReliefIsUsedInFull},
        {"ASurfaceIsRoughAtEveryScale", ASurfaceIsRoughAtEveryScale},
        {"MoreScanDetailMeansMoreFineGrain", MoreScanDetailMeansMoreFineGrain},
        {"EachKindLooksLikeItself", EachKindLooksLikeItself},
        {"WearAgesAMaterial", WearAgesAMaterial},
        {"PitsAreDarkerThanTheSurfaceAround", PitsAreDarkerThanTheSurfaceAround},
        {"KindNamesRoundTripAndBadNamesAreRefused", KindNamesRoundTripAndBadNamesAreRefused},
        {"TheSizeIsKeptInRange", TheSizeIsKeptInRange},
        {"ASmallMaterialAlsoTiles", ASmallMaterialAlsoTiles},
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
    std::cout << (failureCount == 0 ? "all material tests passed\n" : "material tests failed\n");
    return failureCount == 0 ? 0 : 1;
}
