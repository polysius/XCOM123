#include "AppearanceSheet.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

#include "core/PngWriter.h"

namespace appearance_sheet {
namespace {

using namespace character;

constexpr int kPanelWidth = 560;
constexpr int kPanelHeight = 760;
constexpr int kSupersample = 2;

struct Image {
    int width;
    int height;
    std::vector<uint8_t> rgba;

    Image(int w, int h) : width(w), height(h), rgba(static_cast<size_t>(w) * h * 4, 255) {}
};

glm::vec3 SrgbToLinear(glm::vec3 c) {
    return glm::pow(c, glm::vec3(2.2f));
}

// A bilinear lookup in an 8 bit image at uv (in 0 to 1, clamped), as values 0 to 1 per channel.
glm::vec4 Lookup(const ByteImage& image, glm::vec2 uv) {
    const float x = glm::clamp(uv.x, 0.0f, 1.0f) * static_cast<float>(image.width) - 0.5f;
    const float y = glm::clamp(uv.y, 0.0f, 1.0f) * static_cast<float>(image.height) - 0.5f;
    const int x0 = std::clamp(static_cast<int>(std::floor(x)), 0, image.width - 1);
    const int y0 = std::clamp(static_cast<int>(std::floor(y)), 0, image.height - 1);
    const int x1 = std::min(x0 + 1, image.width - 1);
    const int y1 = std::min(y0 + 1, image.height - 1);
    const float fx = glm::clamp(x - static_cast<float>(x0), 0.0f, 1.0f);
    const float fy = glm::clamp(y - static_cast<float>(y0), 0.0f, 1.0f);
    glm::vec4 result(0.0f, 0.0f, 0.0f, 1.0f);
    for (int c = 0; c < image.channels; ++c) {
        const float a = image.Pixel(x0, y0)[c];
        const float b = image.Pixel(x1, y0)[c];
        const float d = image.Pixel(x0, y1)[c];
        const float e = image.Pixel(x1, y1)[c];
        result[c] = (glm::mix(glm::mix(a, b, fx), glm::mix(d, e, fx), fy)) / 255.0f;
    }
    return result;
}

// What to draw and how it is lit.
struct Surface {
    const RenderMesh* mesh = nullptr;
    const std::vector<glm::vec3>* vertexColours = nullptr;  // linear; used when there is no albedo texture
    const ByteImage* albedo = nullptr;
    const ByteImage* normalMap = nullptr;
    const ByteImage* roughnessMap = nullptr;
    glm::vec3 tint{1.0f};
    float roughness = 0.5f;
    float subsurface = 0.0f;  // 0 = none; skin wraps red light round the terminator
    float specular = 1.0f;
    bool twoSided = false;
};

class View {
public:
    // `focus` is the point of the figure at the centre of the picture, `viewHeight` the metres of the figure over its height.
    View(float turnDegrees, glm::vec3 focus, float viewHeight)
        : width(kPanelWidth * kSupersample), height(kPanelHeight * kSupersample), colour(static_cast<size_t>(width) * height), depth(static_cast<size_t>(width) * height, -1e9f) {
        const float turn = turnDegrees * 0.01745329252f;
        cosTurn = std::cos(turn);
        sinTurn = std::sin(turn);
        scale = static_cast<float>(height) / viewHeight;
        focusInView = ToView(focus);
        for (int y = 0; y < height; ++y) {
            const float shade = 0.80f + 0.12f * static_cast<float>(y) / static_cast<float>(height);
            for (int x = 0; x < width; ++x) {
                colour[static_cast<size_t>(y) * width + x] = SrgbToLinear(glm::vec3(shade, shade, shade * 1.03f));
            }
        }
    }

    glm::vec3 ToView(glm::vec3 p) const { return {p.x * cosTurn + p.z * sinTurn, p.y, -p.x * sinTurn + p.z * cosTurn}; }

    void Draw(const Surface& surface) {
        const RenderMesh& mesh = *surface.mesh;
        for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            Triangle triangle;
            for (int c = 0; c < 3; ++c) {
                const uint32_t id = mesh.indices[t + c];
                triangle.view[c] = ToView(mesh.positions[id]);
                triangle.pixel[c] = glm::vec2(width * 0.5f + (triangle.view[c].x - focusInView.x) * scale, height * 0.5f - (triangle.view[c].y - focusInView.y) * scale);
                triangle.normal[c] = ToView(mesh.normals[id]);
                triangle.uv[c] = mesh.uvs.empty() ? glm::vec2(0.0f) : mesh.uvs[id];
                triangle.colour[c] = surface.vertexColours != nullptr ? (*surface.vertexColours)[id] : glm::vec3(1.0f);
            }
            ComputeTangents(triangle, mesh);
            Rasterise(surface, triangle);
        }
    }

    // The picture, averaged down to the panel size, in sRGB, copied into `target` at (left, top).
    void Resolve(Image& target, int left, int top) const {
        for (int y = 0; y < kPanelHeight; ++y) {
            for (int x = 0; x < kPanelWidth; ++x) {
                glm::vec3 sum(0.0f);
                for (int sy = 0; sy < kSupersample; ++sy) {
                    for (int sx = 0; sx < kSupersample; ++sx) {
                        sum += colour[static_cast<size_t>(y * kSupersample + sy) * width + x * kSupersample + sx];
                    }
                }
                sum /= static_cast<float>(kSupersample * kSupersample);
                // A soft shoulder to the tone, so that bright highlights do not clip.
                const glm::vec3 mapped = glm::vec3(1.0f) - glm::exp(-sum * 1.15f);
                const glm::vec3 srgb = glm::pow(glm::clamp(mapped, glm::vec3(0.0f), glm::vec3(1.0f)), glm::vec3(1.0f / 2.2f));
                uint8_t* pixel = &target.rgba[(static_cast<size_t>(top + y) * target.width + left + x) * 4];
                for (int c = 0; c < 3; ++c) {
                    pixel[c] = static_cast<uint8_t>(std::lround(srgb[c] * 255.0f));
                }
            }
        }
    }

private:
    struct Triangle {
        glm::vec3 view[3];
        glm::vec2 pixel[3];
        glm::vec3 normal[3];
        glm::vec2 uv[3];
        glm::vec3 colour[3];
        glm::vec3 tangent{1.0f, 0.0f, 0.0f};    // in view space: along +u
        glm::vec3 bitangent{0.0f, 1.0f, 0.0f};  // up the image
    };

    void ComputeTangents(Triangle& triangle, const RenderMesh& mesh) const {
        if (mesh.uvs.empty()) {
            return;
        }
        const glm::vec3 e1 = triangle.view[1] - triangle.view[0];
        const glm::vec3 e2 = triangle.view[2] - triangle.view[0];
        const glm::vec2 d1 = triangle.uv[1] - triangle.uv[0];
        const glm::vec2 d2 = triangle.uv[2] - triangle.uv[0];
        const float determinant = d1.x * d2.y - d2.x * d1.y;
        if (std::abs(determinant) < 1e-14f) {
            return;
        }
        triangle.tangent = glm::normalize((e1 * d2.y - e2 * d1.y) / determinant);
        // v runs down the image, the normal map's +Y runs up it.
        triangle.bitangent = -glm::normalize((e2 * d1.x - e1 * d2.x) / determinant);
    }

    void Rasterise(const Surface& surface, const Triangle& tri) {
        const glm::vec2* p = tri.pixel;
        const float area = (p[1].x - p[0].x) * (p[2].y - p[0].y) - (p[2].x - p[0].x) * (p[1].y - p[0].y);
        if (std::abs(area) < 1e-6f) {
            return;
        }
        const int minX = std::max(0, static_cast<int>(std::floor(std::min({p[0].x, p[1].x, p[2].x}))));
        const int maxX = std::min(width - 1, static_cast<int>(std::ceil(std::max({p[0].x, p[1].x, p[2].x}))));
        const int minY = std::max(0, static_cast<int>(std::floor(std::min({p[0].y, p[1].y, p[2].y}))));
        const int maxY = std::min(height - 1, static_cast<int>(std::ceil(std::max({p[0].y, p[1].y, p[2].y}))));
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const glm::vec2 q(static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f);
                const float w0 = ((p[1].x - q.x) * (p[2].y - q.y) - (p[2].x - q.x) * (p[1].y - q.y)) / area;
                const float w1 = ((p[2].x - q.x) * (p[0].y - q.y) - (p[0].x - q.x) * (p[2].y - q.y)) / area;
                const float w2 = 1.0f - w0 - w1;
                if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) {
                    continue;
                }
                const float z = w0 * tri.view[0].z + w1 * tri.view[1].z + w2 * tri.view[2].z;
                float& stored = depth[static_cast<size_t>(y) * width + x];
                if (z <= stored) {
                    continue;
                }
                stored = z;
                colour[static_cast<size_t>(y) * width + x] = Shade(surface, tri, w0, w1, w2);
            }
        }
    }

    glm::vec3 Shade(const Surface& surface, const Triangle& tri, float w0, float w1, float w2) const {
        const glm::vec2 uv = w0 * tri.uv[0] + w1 * tri.uv[1] + w2 * tri.uv[2];
        glm::vec3 albedo = w0 * tri.colour[0] + w1 * tri.colour[1] + w2 * tri.colour[2];
        float roughness = surface.roughness;
        if (surface.albedo != nullptr) {
            albedo = SrgbToLinear(glm::vec3(Lookup(*surface.albedo, uv)));
        }
        albedo *= surface.tint;
        if (surface.roughnessMap != nullptr) {
            roughness = Lookup(*surface.roughnessMap, uv).x;
        }
        glm::vec3 n = glm::normalize(w0 * tri.normal[0] + w1 * tri.normal[1] + w2 * tri.normal[2]);
        // A surface seen from behind (the inside of a collar, the back of a ribbon) is turned to face the eye, and is darker.
        float inside = 1.0f;
        if (surface.twoSided && n.z < 0.0f) {
            n = -n;
            inside = 0.55f;
        }
        if (surface.normalMap != nullptr) {
            const glm::vec4 texel = Lookup(*surface.normalMap, uv);
            const glm::vec3 tangentSpace(texel.x * 2.0f - 1.0f, texel.y * 2.0f - 1.0f, texel.z * 2.0f - 1.0f);
            const glm::vec3 t = glm::normalize(tri.tangent - n * glm::dot(tri.tangent, n));
            const glm::vec3 b = glm::cross(n, t);
            const float handedness = glm::dot(b, tri.bitangent) < 0.0f ? -1.0f : 1.0f;
            n = glm::normalize(t * tangentSpace.x + b * (handedness * tangentSpace.y) + n * tangentSpace.z);
        }

        // Two lights and a sky. Skin lets red light wrap round the terminator.
        const glm::vec3 keyDirection = glm::normalize(glm::vec3(-0.45f, 0.55f, 0.70f));
        const glm::vec3 fillDirection = glm::normalize(glm::vec3(0.65f, 0.10f, 0.45f));
        const glm::vec3 rimDirection = glm::normalize(glm::vec3(0.30f, 0.45f, -0.85f));
        const glm::vec3 wrap = surface.subsurface * glm::vec3(0.55f, 0.22f, 0.10f);
        const auto diffuse = [&](glm::vec3 direction) {
            const float d = glm::dot(n, direction);
            return glm::clamp((glm::vec3(d) + wrap) / (glm::vec3(1.0f) + wrap), glm::vec3(0.0f), glm::vec3(1.0f));
        };
        const glm::vec3 keyColour(2.4f, 2.25f, 2.05f);
        const glm::vec3 fillColour(0.45f, 0.55f, 0.75f);
        const glm::vec3 rimColour(0.55f, 0.60f, 0.70f);
        const glm::vec3 sky = glm::mix(glm::vec3(0.10f, 0.09f, 0.08f), glm::vec3(0.30f, 0.33f, 0.38f), 0.5f + 0.5f * n.y);
        glm::vec3 light = keyColour * diffuse(keyDirection) + fillColour * diffuse(fillDirection) + rimColour * 0.5f * diffuse(rimDirection) + sky;
        glm::vec3 result = albedo * light * inside;

        // Specular: a Blinn highlight whose width follows the roughness, and a weak one from the fill light.
        const glm::vec3 toEye(0.0f, 0.0f, 1.0f);
        const float alpha = std::max(roughness * roughness, 0.002f);
        const float shininess = std::clamp(2.0f / (alpha * alpha) - 2.0f, 4.0f, 6000.0f);
        float specular = 0.0f;
        for (const auto& [direction, strength] : {std::pair{keyDirection, 1.0f}, std::pair{fillDirection, 0.25f}}) {
            const glm::vec3 h = glm::normalize(direction + toEye);
            const float nh = std::max(glm::dot(n, h), 0.0f);
            const float facing = glm::dot(n, direction) > 0.0f ? 1.0f : 0.0f;
            specular += strength * facing * std::pow(nh, shininess) * (shininess + 8.0f) / 25.1f;
        }
        const float fresnel = 0.04f + 0.96f * std::pow(1.0f - std::max(glm::dot(n, toEye), 0.0f), 5.0f);
        result += glm::vec3(1.0f, 0.98f, 0.95f) * (specular * (0.04f + fresnel) * 0.6f * surface.specular * inside);
        return result;
    }

    int width;
    int height;
    std::vector<glm::vec3> colour;
    std::vector<float> depth;
    float cosTurn = 1.0f;
    float sinTurn = 0.0f;
    float scale = 1.0f;
    glm::vec3 focusInView{0.0f};
};

void DrawAppearance(View& view, const CharacterAppearance& appearance) {
    // The skin, with its textures.
    Surface skin;
    skin.mesh = &appearance.skin.mesh;
    skin.albedo = &appearance.skin.textures.albedo;
    skin.normalMap = &appearance.skin.textures.normal;
    skin.roughnessMap = &appearance.skin.textures.roughness;
    skin.subsurface = 1.0f;
    view.Draw(skin);

    for (const EyeParts& eye : appearance.eyes.eyes) {
        if (eye.eyeball.positions.empty()) {
            continue;
        }
        Surface eyeball;
        eyeball.mesh = &eye.eyeball;
        eyeball.albedo = &appearance.eyes.textures.albedo;
        eyeball.normalMap = &appearance.eyes.textures.normal;
        eyeball.roughnessMap = &appearance.eyes.textures.roughness;
        eyeball.specular = 2.5f;
        view.Draw(eyeball);
        for (const RenderMesh* lid : {&eye.upperLid, &eye.lowerLid}) {
            Surface lidSurface = skin;
            lidSurface.mesh = lid;
            view.Draw(lidSurface);
        }
    }

    // Hair, eyebrows, lashes and facial hair: ribbons, drawn on both sides.
    const auto drawRibbons = [&](const RibbonSet& set, float roughness) {
        if (set.mesh.positions.empty()) {
            return;
        }
        Surface surface;
        surface.mesh = &set.mesh;
        surface.vertexColours = &set.colours;
        surface.roughness = roughness;
        surface.twoSided = true;
        surface.specular = 0.7f;
        view.Draw(surface);
    };
    drawRibbons(appearance.hair.scalp, 0.45f);
    drawRibbons(appearance.hair.eyebrows, 0.5f);
    drawRibbons(appearance.hair.facial, 0.5f);
    for (const EyeParts& eye : appearance.eyes.eyes) {
        drawRibbons(eye.lashes, 0.4f);
    }

    for (const Garment& garment : appearance.outfit.garments) {
        Surface cloth;
        cloth.mesh = &garment.mesh;
        cloth.albedo = &garment.textures.albedo;
        cloth.normalMap = &garment.textures.normal;
        cloth.roughnessMap = &garment.textures.roughness;
        cloth.twoSided = true;
        cloth.specular = garment.material == GarmentMaterial::Leather ? 1.4f : 0.5f;
        view.Draw(cloth);
    }
}

bool SavePng(const std::filesystem::path& path, const Image& image) {
    return WritePng(path.string(), static_cast<uint32_t>(image.width), static_cast<uint32_t>(image.height), image.rgba);
}

// The images of a texture as RGBA, expanding a single channel to grey.
Image FromByteImage(const ByteImage& source) {
    Image image(source.width, source.height);
    for (int y = 0; y < source.height; ++y) {
        for (int x = 0; x < source.width; ++x) {
            const uint8_t* in = source.Pixel(x, y);
            uint8_t* out = &image.rgba[(static_cast<size_t>(y) * source.width + x) * 4];
            for (int c = 0; c < 3; ++c) {
                out[c] = in[source.channels == 1 ? 0 : c];
            }
            out[3] = 255;
        }
    }
    return image;
}

}  // namespace

bool WriteViews(const std::filesystem::path& folder, const std::string& name, const CharacterMesh& mesh, const CharacterAppearance& appearance) {
    const float stature = mesh.skeleton.statureMeters;
    bool ok = true;
    {
        Image sheet(kPanelWidth * 3, kPanelHeight);
        const float turns[3] = {0.0f, 90.0f, 35.0f};
        for (int i = 0; i < 3; ++i) {
            View view(turns[i], glm::vec3(0.0f, 0.56f * stature - 0.04f, 0.0f), stature * 1.12f);
            DrawAppearance(view, appearance);
            view.Resolve(sheet, i * kPanelWidth, 0);
        }
        ok = SavePng(folder / (name + "_dressed.png"), sheet) && ok;
    }
    {
        Image sheet(kPanelWidth * 3, kPanelHeight);
        const float turns[3] = {0.0f, 35.0f, 90.0f};
        for (int i = 0; i < 3; ++i) {
            View view(turns[i], glm::vec3(0.0f, 0.925f * stature, 0.02f * stature), 0.19f * stature);
            DrawAppearance(view, appearance);
            view.Resolve(sheet, i * kPanelWidth, 0);
        }
        ok = SavePng(folder / (name + "_face.png"), sheet) && ok;
    }
    {
        // The eyes, close enough to judge the lids, lashes and brows.
        Image sheet(kPanelWidth * 3, kPanelHeight);
        const float turns[3] = {0.0f, 30.0f, 70.0f};
        const float s = stature / 1.75f;
        for (int i = 0; i < 3; ++i) {
            View view(turns[i], glm::vec3(0.0f, 1.640f * s, 0.075f * s), 0.125f * s);
            DrawAppearance(view, appearance);
            view.Resolve(sheet, i * kPanelWidth, 0);
        }
        ok = SavePng(folder / (name + "_eyes.png"), sheet) && ok;
    }
    return ok;
}

bool WriteLook(const std::filesystem::path& folder, const std::string& name, const CharacterAppearance& appearance, glm::vec3 centre, float viewHeight, float turnDegrees) {
    Image picture(kPanelWidth, kPanelHeight);
    View view(turnDegrees, centre, viewHeight);
    DrawAppearance(view, appearance);
    view.Resolve(picture, 0, 0);
    return SavePng(folder / (name + "_look.png"), picture);
}

bool WriteTextures(const std::filesystem::path& folder, const std::string& name, const CharacterAppearance& appearance) {
    bool ok = true;
    const auto save = [&](const std::string& suffix, const ByteImage& image) { ok = SavePng(folder / (name + "_" + suffix + ".png"), FromByteImage(image)) && ok; };
    save("skin_albedo", appearance.skin.textures.albedo);
    save("skin_normal", appearance.skin.textures.normal);
    save("skin_roughness", appearance.skin.textures.roughness);
    save("eye_albedo", appearance.eyes.textures.albedo);
    for (const Garment& garment : appearance.outfit.garments) {
        const std::string label = std::string("garment_") + GarmentName(garment.kind);
        save(label + "_albedo", garment.textures.albedo);
        save(label + "_normal", garment.textures.normal);
    }
    return ok;
}

}  // namespace appearance_sheet
