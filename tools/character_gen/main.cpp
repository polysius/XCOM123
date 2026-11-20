// character_gen makes a character and writes a contact sheet of it as a PNG file: the body from the front, the side and
// three quarters, the skin weights with the skeleton, and the body in a pose from two sides, where linear blend skinning
// shows what it does to the joints. It also writes the mesh as a Wavefront OBJ file. No window and no GPU are needed.
//
//   character_gen --seed 7                       (a person drawn from the population)
//   character_gen --sex 0 --age 40 --height 1.65 --mass 62 --muscle 0.3 --out characters
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "character/CharacterGenerator.h"
#include "core/JobSystem.h"
#include "core/PngWriter.h"

namespace {

using character::CharacterMesh;
using character::CharacterParameters;

constexpr int kPanelWidth = 560;
constexpr int kPanelHeight = 760;

struct Image {
    int width;
    int height;
    std::vector<uint8_t> rgba;

    Image(int w, int h) : width(w), height(h), rgba(static_cast<size_t>(w) * h * 4, 255) {}

    void Put(int x, int y, glm::vec3 colour) {
        if (x < 0 || y < 0 || x >= width || y >= height) {
            return;
        }
        uint8_t* pixel = &rgba[(static_cast<size_t>(y) * width + x) * 4];
        for (int c = 0; c < 3; ++c) {
            pixel[c] = static_cast<uint8_t>(std::lround(std::clamp(colour[c], 0.0f, 1.0f) * 255.0f));
        }
    }
};

float ToSrgb(float linear) {
    return std::pow(std::clamp(linear, 0.0f, 1.0f), 1.0f / 2.2f);
}

// One orthographic view of a mesh into a panel of the sheet. `turn` is how far the camera moves round the figure.
class Panel {
public:
    Panel(Image& image, int left, int top, float turnDegrees, float stature)
        : image(image), left(left), top(top), depth(static_cast<size_t>(kPanelWidth) * kPanelHeight, -1e9f) {
        const float turn = turnDegrees * 0.01745329252f;
        cosTurn = std::cos(turn);
        sinTurn = std::sin(turn);
        scale = static_cast<float>(kPanelHeight) / (stature * 1.12f);
        // Light from the upper left in front, in the view's own axes.
        lightDirection = glm::normalize(glm::vec3(-0.45f, 0.55f, 0.7f));
        for (int y = 0; y < kPanelHeight; ++y) {
            for (int x = 0; x < kPanelWidth; ++x) {
                const float shade = 0.80f + 0.12f * static_cast<float>(y) / kPanelHeight;
                image.Put(left + x, top + y, glm::vec3(shade, shade, shade * 1.03f));
            }
        }
    }

    glm::vec3 ToView(glm::vec3 p) const {
        return {p.x * cosTurn + p.z * sinTurn, p.y, -p.x * sinTurn + p.z * cosTurn};
    }

    glm::vec2 ToPixel(glm::vec3 view) const {
        return {kPanelWidth * 0.5f + view.x * scale, kPanelHeight - (view.y + 0.04f) * scale};
    }

    // Fills the triangles with the colour of each vertex, lit by the normals.
    void Draw(const std::vector<glm::vec3>& positions, const std::vector<glm::vec3>& normals, const std::vector<uint32_t>& indices,
              const std::vector<glm::vec3>& colours) {
        for (size_t t = 0; t + 2 < indices.size(); t += 3) {
            const uint32_t id[3] = {indices[t], indices[t + 1], indices[t + 2]};
            glm::vec3 view[3];
            glm::vec2 pixel[3];
            glm::vec3 colour[3];
            for (int c = 0; c < 3; ++c) {
                view[c] = ToView(positions[id[c]]);
                pixel[c] = ToPixel(view[c]);
                const glm::vec3 normal = ToView(normals[id[c]]);
                const float light = 0.28f + 0.72f * std::max(glm::dot(normal, lightDirection), 0.0f) + 0.10f * (0.5f + 0.5f * normal.y);
                colour[c] = colours[id[c]] * light;
            }
            Fill(pixel, view, colour);
        }
    }

    void Line(glm::vec3 a, glm::vec3 b, glm::vec3 colour) {
        const glm::vec2 pa = ToPixel(ToView(a));
        const glm::vec2 pb = ToPixel(ToView(b));
        const int steps = static_cast<int>(std::max(std::abs(pb.x - pa.x), std::abs(pb.y - pa.y))) + 1;
        for (int i = 0; i <= steps; ++i) {
            const glm::vec2 p = pa + (pb - pa) * (static_cast<float>(i) / steps);
            Put(static_cast<int>(p.x), static_cast<int>(p.y), colour);
            Put(static_cast<int>(p.x) + 1, static_cast<int>(p.y), colour);
        }
    }

    void Dot(glm::vec3 a, glm::vec3 colour) {
        const glm::vec2 p = ToPixel(ToView(a));
        for (int dy = -3; dy <= 3; ++dy) {
            for (int dx = -3; dx <= 3; ++dx) {
                Put(static_cast<int>(p.x) + dx, static_cast<int>(p.y) + dy, colour);
            }
        }
    }

private:
    void Put(int x, int y, glm::vec3 colour) {
        if (x >= 0 && y >= 0 && x < kPanelWidth && y < kPanelHeight) {
            image.Put(left + x, top + y, colour);
        }
    }

    void Fill(const glm::vec2 pixel[3], const glm::vec3 view[3], const glm::vec3 colour[3]) {
        const float area = (pixel[1].x - pixel[0].x) * (pixel[2].y - pixel[0].y) - (pixel[2].x - pixel[0].x) * (pixel[1].y - pixel[0].y);
        if (std::abs(area) < 1e-6f) {
            return;
        }
        const int minX = std::max(0, static_cast<int>(std::floor(std::min({pixel[0].x, pixel[1].x, pixel[2].x}))));
        const int maxX = std::min(kPanelWidth - 1, static_cast<int>(std::ceil(std::max({pixel[0].x, pixel[1].x, pixel[2].x}))));
        const int minY = std::max(0, static_cast<int>(std::floor(std::min({pixel[0].y, pixel[1].y, pixel[2].y}))));
        const int maxY = std::min(kPanelHeight - 1, static_cast<int>(std::ceil(std::max({pixel[0].y, pixel[1].y, pixel[2].y}))));
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const glm::vec2 p(x + 0.5f, y + 0.5f);
                const float w0 = ((pixel[1].x - p.x) * (pixel[2].y - p.y) - (pixel[2].x - p.x) * (pixel[1].y - p.y)) / area;
                const float w1 = ((pixel[2].x - p.x) * (pixel[0].y - p.y) - (pixel[0].x - p.x) * (pixel[2].y - p.y)) / area;
                const float w2 = 1.0f - w0 - w1;
                if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) {
                    continue;
                }
                const float z = w0 * view[0].z + w1 * view[1].z + w2 * view[2].z;
                float& stored = depth[static_cast<size_t>(y) * kPanelWidth + x];
                if (z > stored) {
                    stored = z;
                    const glm::vec3 c = w0 * colour[0] + w1 * colour[1] + w2 * colour[2];
                    Put(x, y, glm::vec3(ToSrgb(c.r), ToSrgb(c.g), ToSrgb(c.b)));
                }
            }
        }
    }

    Image& image;
    int left;
    int top;
    std::vector<float> depth;
    float cosTurn = 1.0f;
    float sinTurn = 0.0f;
    float scale = 1.0f;
    glm::vec3 lightDirection{0.0f, 0.0f, 1.0f};
};

// A colour for each bone, for the weights view.
glm::vec3 BoneColour(int bone) {
    const float hue = std::fmod(static_cast<float>(bone) * 0.38197f, 1.0f);
    const glm::vec3 rgb(std::abs(hue * 6.0f - 3.0f) - 1.0f, 2.0f - std::abs(hue * 6.0f - 2.0f), 2.0f - std::abs(hue * 6.0f - 4.0f));
    return glm::mix(glm::vec3(0.7f), glm::clamp(rgb, 0.0f, 1.0f), 0.85f);
}

std::vector<glm::vec3> WeightColours(const CharacterMesh& mesh) {
    std::vector<glm::vec3> colours(mesh.positions.size());
    for (size_t v = 0; v < colours.size(); ++v) {
        glm::vec3 sum(0.0f);
        for (int slot = 0; slot < 4; ++slot) {
            sum += mesh.boneWeights[v][slot] * BoneColour(mesh.boneIndices[v][slot]);
        }
        colours[v] = sum * 0.9f;
    }
    return colours;
}

// Recomputes normals of a deformed mesh from its triangles.
std::vector<glm::vec3> NormalsOf(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& indices) {
    std::vector<glm::vec3> normals(positions.size(), glm::vec3(0.0f));
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        const glm::vec3 n = glm::cross(positions[indices[t + 1]] - positions[indices[t]], positions[indices[t + 2]] - positions[indices[t]]);
        for (int c = 0; c < 3; ++c) {
            normals[indices[t + c]] += n;
        }
    }
    for (glm::vec3& n : normals) {
        n = glm::length(n) > 1e-9f ? glm::normalize(n) : glm::vec3(0.0f, 1.0f, 0.0f);
    }
    return normals;
}

character::Pose DemoPose(const character::Skeleton& skeleton) {
    using character::FromDegrees;
    using character::HingeRotation;
    using character::Index;
    using character::Joint;
    character::Pose pose;
    pose.rotation[Index(Joint::UpperArmL)] = FromDegrees(0.0f, 0.0f, 50.0f);
    pose.rotation[Index(Joint::ForearmL)] = HingeRotation(skeleton, Joint::ForearmL, 90.0f);
    pose.rotation[Index(Joint::UpperArmR)] = FromDegrees(-70.0f, 0.0f, 0.0f);
    pose.rotation[Index(Joint::ForearmR)] = HingeRotation(skeleton, Joint::ForearmR, 90.0f);
    pose.rotation[Index(Joint::ThighL)] = FromDegrees(-70.0f, 0.0f, 0.0f);
    pose.rotation[Index(Joint::CalfL)] = FromDegrees(90.0f, 0.0f, 0.0f);
    pose.rotation[Index(Joint::CalfR)] = FromDegrees(35.0f, 0.0f, 0.0f);
    pose.rotation[Index(Joint::Spine2)] = FromDegrees(0.0f, 25.0f, 0.0f);
    pose.rotation[Index(Joint::Head)] = FromDegrees(0.0f, -20.0f, 0.0f);
    return pose;
}

bool WriteObj(const std::filesystem::path& path, const CharacterMesh& mesh) {
    std::ofstream file(path);
    if (!file) {
        return false;
    }
    for (const glm::vec3& p : mesh.positions) {
        file << "v " << p.x << " " << p.y << " " << p.z << "\n";
    }
    for (const glm::vec3& n : mesh.normals) {
        file << "vn " << n.x << " " << n.y << " " << n.z << "\n";
    }
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        file << "f";
        for (int c = 0; c < 3; ++c) {
            file << " " << mesh.indices[i + c] + 1 << "//" << mesh.indices[i + c] + 1;
        }
        file << "\n";
    }
    return true;
}

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
    uint32_t seed = 1;
    bool explicitPerson = false;
    CharacterParameters parameters;
    std::filesystem::path folder = "characters";
    float voxel = 0.024f;

    for (int i = 1; i < argc; ++i) {
        const std::string option = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::cerr << option << " needs a value\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (option == "--seed") {
            seed = static_cast<uint32_t>(NumberOrExit(option, next(), 0.0, 4294967295.0));
        } else if (option == "--sex") {
            parameters.masculinity = static_cast<float>(NumberOrExit(option, next(), 0.0, 1.0));
            explicitPerson = true;
        } else if (option == "--age") {
            parameters.ageYears = static_cast<float>(NumberOrExit(option, next(), 18.0, 80.0));
            explicitPerson = true;
        } else if (option == "--height") {
            parameters.statureMeters = static_cast<float>(NumberOrExit(option, next(), 1.45, 2.05));
            explicitPerson = true;
        } else if (option == "--mass") {
            parameters.massKg = static_cast<float>(NumberOrExit(option, next(), 30.0, 190.0));
            explicitPerson = true;
        } else if (option == "--muscle") {
            parameters.muscularity = static_cast<float>(NumberOrExit(option, next(), 0.0, 1.0));
            explicitPerson = true;
        } else if (option == "--voxel") {
            voxel = static_cast<float>(NumberOrExit(option, next(), 0.006, 0.04));
        } else if (option == "--out") {
            folder = next();
        } else {
            std::cerr << "unknown option " << option << "\n";
            return 2;
        }
    }
    if (explicitPerson) {
        parameters.seed = seed;
    } else {
        parameters = character::RandomParameters(seed);
    }

    character::MeshSettings settings;
    settings.voxelMeters = voxel;
    const auto start = std::chrono::steady_clock::now();
    const CharacterMesh mesh = character::GenerateCharacter(parameters, settings, &JobSystem::Shared());
    const double milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

    float top = 0.0f;
    for (const glm::vec3& p : mesh.positions) {
        top = std::max(top, p.y);
    }
    std::cout << "stature " << mesh.parameters.statureMeters << " m (mesh " << top << "), mass " << mesh.parameters.massKg << " kg (mesh " << mesh.massKg
              << "), age " << mesh.parameters.ageYears << ", masculinity " << mesh.parameters.masculinity << ", muscularity " << mesh.parameters.muscularity
              << "\n" << mesh.positions.size() << " vertices, " << mesh.indices.size() / 3 << " triangles, girth " << mesh.girth << ", made in "
              << static_cast<int>(milliseconds) << " ms\n";

    // The separate pieces of the surface, which should be one.
    {
        std::vector<int> parent(mesh.positions.size());
        for (size_t v = 0; v < parent.size(); ++v) {
            parent[v] = static_cast<int>(v);
        }
        const auto find = [&](int v) {
            while (parent[v] != v) {
                parent[v] = parent[parent[v]];
                v = parent[v];
            }
            return v;
        };
        for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            parent[find(static_cast<int>(mesh.indices[t]))] = find(static_cast<int>(mesh.indices[t + 1]));
            parent[find(static_cast<int>(mesh.indices[t + 1]))] = find(static_cast<int>(mesh.indices[t + 2]));
        }
        std::vector<int> triangles(mesh.positions.size(), 0);
        std::vector<glm::vec3> centre(mesh.positions.size(), glm::vec3(0.0f));
        for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            const int root = find(static_cast<int>(mesh.indices[t]));
            ++triangles[root];
            centre[root] += mesh.positions[mesh.indices[t]];
        }
        int pieces = 0;
        for (size_t v = 0; v < triangles.size(); ++v) {
            if (triangles[v] > 0) {
                ++pieces;
                if (triangles[v] < 2000) {
                    const glm::vec3 c = centre[v] / static_cast<float>(triangles[v]);
                    std::cout << "  small piece of " << triangles[v] << " triangles near (" << c.x << ", " << c.y << ", " << c.z << ")\n";
                }
            }
        }
        const character::MeshTopology topology = character::MeasureTopology(mesh);
        std::cout << "pieces: " << pieces << ", bad edges " << topology.badEdges << ", Euler characteristic " << topology.eulerCharacteristic
                  << (topology.IsClosedSphere() ? " (a closed sphere)" : " (NOT a closed sphere)") << "\n";
    }

    std::filesystem::create_directories(folder);
    const std::string name = "character_" + std::to_string(seed);

    // The contact sheet: two rows of three panels.
    Image sheet(kPanelWidth * 3, kPanelHeight * 2);
    const std::vector<glm::vec3> skin(mesh.positions.size(), glm::vec3(0.62f, 0.44f, 0.36f));
    const float turns[3] = {0.0f, 90.0f, 35.0f};  // front, side, three quarters
    for (int view = 0; view < 3; ++view) {
        Panel panel(sheet, view * kPanelWidth, 0, turns[view], mesh.skeleton.statureMeters);
        panel.Draw(mesh.positions, mesh.normals, mesh.indices, skin);
    }
    {
        Panel panel(sheet, 0, kPanelHeight, 0.0f, mesh.skeleton.statureMeters);
        panel.Draw(mesh.positions, mesh.normals, mesh.indices, WeightColours(mesh));
        for (int j = 0; j < character::kJointCount; ++j) {
            const auto& joint = mesh.skeleton.joints[j];
            if (joint.parent >= 0) {
                panel.Line(mesh.skeleton.joints[joint.parent].restPosition, joint.restPosition, glm::vec3(1.0f, 1.0f, 1.0f));
            }
        }
        for (const auto& joint : mesh.skeleton.joints) {
            panel.Dot(joint.restPosition, glm::vec3(0.9f, 0.1f, 0.1f));
        }
    }
    {
        const character::Pose pose = DemoPose(mesh.skeleton);
        const std::vector<glm::vec3> posed = character::SkinPositions(mesh, pose);
        const std::vector<glm::vec3> posedNormals = NormalsOf(posed, mesh.indices);
        Panel front(sheet, kPanelWidth, kPanelHeight, 0.0f, mesh.skeleton.statureMeters);
        front.Draw(posed, posedNormals, mesh.indices, skin);
        Panel threeQuarter(sheet, 2 * kPanelWidth, kPanelHeight, 40.0f, mesh.skeleton.statureMeters);
        threeQuarter.Draw(posed, posedNormals, mesh.indices, skin);
    }
    const bool sheetWritten = WritePng((folder / (name + "_sheet.png")).string(), static_cast<uint32_t>(sheet.width), static_cast<uint32_t>(sheet.height), sheet.rgba);
    const bool objWritten = WriteObj(folder / (name + ".obj"), mesh);
    if (!sheetWritten || !objWritten) {
        std::cerr << "could not write the files in " << folder << "\n";
        return 1;
    }
    return 0;
}
