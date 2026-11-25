#include "character/appearance/Clothing.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <map>

#include "character/Anthropometry.h"
#include "character/ParallelEach.h"
#include "character/appearance/Atlas.h"
#include "character/appearance/MeshTools.h"
#include "character/appearance/Noise3D.h"
#include "character/appearance/Skin.h"

namespace character {
namespace {

// Distances of the body that the cut of a garment follows, found from the skeleton.
struct BodyMeasures {
    float stature = 1.75f;
    float scale = 1.0f;  // stature / 1.75
    glm::vec3 shoulder[2];
    glm::vec3 armDirection[2];  // from the shoulder to the wrist
    float upperArmLength = 0.3f;
    float armLength = 0.5f;     // from the shoulder to the wrist
    float neckZ = 0.0f;         // where the axis of the neck runs, in front of or behind the middle of the body
};

BodyMeasures MeasureBody(const Skeleton& skeleton) {
    BodyMeasures m;
    m.stature = skeleton.statureMeters;
    m.scale = m.stature / 1.75f;
    const Joint shoulders[2] = {Joint::UpperArmL, Joint::UpperArmR};
    const Joint elbows[2] = {Joint::ForearmL, Joint::ForearmR};
    const Joint wrists[2] = {Joint::HandL, Joint::HandR};
    for (int i = 0; i < 2; ++i) {
        m.shoulder[i] = skeleton[shoulders[i]].restPosition;
        m.armDirection[i] = glm::normalize(skeleton[wrists[i]].restPosition - m.shoulder[i]);
    }
    m.upperArmLength = glm::distance(skeleton[shoulders[0]].restPosition, skeleton[elbows[0]].restPosition);
    m.armLength = glm::distance(skeleton[shoulders[0]].restPosition, skeleton[wrists[0]].restPosition);
    m.neckZ = skeleton[Joint::Neck].restPosition.z;
    return m;
}

// The share of a vertex that follows the bones of the arms, or of the legs.
float WeightOnLimbs(const MeshVertex& vertex, bool arms) {
    float sum = 0.0f;
    for (int slot = 0; slot < 4; ++slot) {
        const Joint joint = static_cast<Joint>(vertex.bones[slot]);
        const bool isArm = joint == Joint::UpperArmL || joint == Joint::UpperArmR || joint == Joint::ForearmL || joint == Joint::ForearmR || joint == Joint::HandL ||
                           joint == Joint::HandR;
        const bool isLeg = joint == Joint::ThighL || joint == Joint::ThighR || joint == Joint::CalfL || joint == Joint::CalfR || joint == Joint::FootL ||
                           joint == Joint::FootR || joint == Joint::ToesL || joint == Joint::ToesR;
        if (arms ? isArm : isLeg) {
            sum += vertex.weights[slot];
        }
    }
    return sum;
}

// How far along the arm, from the shoulder, a point is.
float AlongArm(const BodyMeasures& m, glm::vec3 p) {
    const int side = p.x >= 0.0f ? 0 : 1;
    return glm::dot(p - m.shoulder[side], m.armDirection[side]);
}

// Positive where a sleeve reaches, `sleeveEnd` metres along the arm from the shoulder. Away from the arms it is positive everywhere.
float SleeveField(const BodyMeasures& m, const MeshVertex& vertex, float sleeveEnd) {
    const float onArm = SmoothStep(0.25f, 0.60f, WeightOnLimbs(vertex, true));
    return glm::mix(1.0f, sleeveEnd - AlongArm(m, vertex.position), onArm);
}

// Positive inside the garment, in metres from its edge where that is known; zero on the edge.
struct GarmentSpec {
    GarmentKind kind = GarmentKind::TShirt;
    GarmentMaterial material = GarmentMaterial::Cotton;
    material::MaterialKind materialKind = material::MaterialKind::Fabric;
    glm::vec3 tint{1.0f};
    float thickness = 0.001f;
    std::function<float(const MeshVertex&)> field;  // where the garment is
    std::function<float(glm::vec3)> ease;           // how far it stands off the body at a point of the body
    float wear = 0.3f;
};

// The hem of a garment hangs looser than its body: the ease grows over the lowest `reach` metres above `hemY`.
std::function<float(glm::vec3)> EaseWithFlare(float base, float flare, float hemY, float reach) {
    return [=](glm::vec3 p) { return base + flare * SmoothStep(hemY + reach, hemY, p.y); };
}

glm::vec3 PickColour(const std::vector<glm::vec3>& palette, uint32_t seed, uint32_t salt) {
    return palette[material::Hash(seed, salt, 7u) % palette.size()];
}

glm::vec3 SrgbToLinear(glm::vec3 c) {
    return glm::pow(c, glm::vec3(2.2f));
}

GarmentSpec MakeSpec(GarmentKind kind, const BodyMeasures& m, uint32_t seed) {
    GarmentSpec spec;
    spec.kind = kind;
    const float stature = m.stature;
    const float s = m.scale;
    switch (kind) {
        case GarmentKind::TShirt: {
            const float hem = 0.54f * stature;
            spec.material = GarmentMaterial::Cotton;
            spec.materialKind = material::MaterialKind::Fabric;
            spec.tint = SrgbToLinear(PickColour({{0.85f, 0.85f, 0.82f}, {0.18f, 0.22f, 0.36f}, {0.62f, 0.14f, 0.14f}, {0.22f, 0.42f, 0.28f}, {0.78f, 0.62f, 0.20f}, {0.12f, 0.12f, 0.13f}, {0.45f, 0.47f, 0.50f}}, seed, 1u));
            spec.thickness = 0.0012f;
            spec.ease = EaseWithFlare(0.0050f, 0.0110f, hem, 0.06f);
            spec.field = [=](const MeshVertex& v) {
                // A crew neck is a hole round the neck, wider towards the front, that starts where the neck leaves the shoulders: the
                // shoulders of the body reach higher than a plane through the base of the neck would, so the cut follows the neck.
                const float front = SmoothStep(0.0f, 0.04f, v.position.z - m.neckZ);
                const float acrossHole = v.position.x / (0.080f * s);
                const float alongHole = (v.position.z - m.neckZ) / ((0.078f + 0.030f * front) * s);
                const float outsideHole = (std::sqrt(acrossHole * acrossHole + alongHole * alongHole) - 1.0f) * 0.04f;
                const float holeBottom = 0.846f * stature - 0.030f * front;
                const float collar = std::max(outsideHole, holeBottom - v.position.y);
                return std::min({v.position.y - hem, collar, SleeveField(m, v, 0.50f * m.upperArmLength)});
            };
            break;
        }
        case GarmentKind::Jacket: {
            const float hem = 0.52f * stature;
            spec.material = GarmentMaterial::Leather;
            spec.materialKind = material::MaterialKind::Leather;
            spec.tint = SrgbToLinear(PickColour({{0.10f, 0.09f, 0.09f}, {0.30f, 0.17f, 0.09f}, {0.20f, 0.12f, 0.08f}, {0.16f, 0.17f, 0.20f}}, seed, 2u));
            spec.thickness = 0.0028f;
            spec.ease = EaseWithFlare(0.0120f, 0.0140f, hem, 0.08f);
            spec.field = [=](const MeshVertex& v) {
                const float collar = 0.880f * stature - 0.012f * SmoothStep(0.0f, 0.05f, v.position.z);
                return std::min({v.position.y - hem, collar - v.position.y, SleeveField(m, v, m.armLength - 0.050f * s)});
            };
            break;
        }
        case GarmentKind::Trousers:
        case GarmentKind::Shorts: {
            const bool shorts = kind == GarmentKind::Shorts;
            const float waist = 0.56f * stature;
            const float cuff = shorts ? 0.37f * stature : anthropometry::kAnkleHeight * stature + 0.035f;
            spec.material = shorts ? GarmentMaterial::Cotton : GarmentMaterial::Denim;
            spec.materialKind = material::MaterialKind::Fabric;
            spec.tint = shorts ? SrgbToLinear(PickColour({{0.55f, 0.50f, 0.38f}, {0.20f, 0.24f, 0.34f}, {0.30f, 0.32f, 0.26f}}, seed, 3u))
                               : SrgbToLinear(PickColour({{0.12f, 0.17f, 0.30f}, {0.09f, 0.12f, 0.20f}, {0.10f, 0.10f, 0.11f}, {0.20f, 0.26f, 0.38f}}, seed, 4u));
            spec.thickness = shorts ? 0.0015f : 0.0020f;
            spec.ease = [](glm::vec3) { return 0.0060f; };
            spec.field = [=](const MeshVertex& v) { return std::min({waist - v.position.y, v.position.y - cuff, (0.5f - WeightOnLimbs(v, true)) * 0.1f}); };
            break;
        }
        case GarmentKind::Shoes:
        case GarmentKind::Boots: {
            const bool boots = kind == GarmentKind::Boots;
            const float top = (boots ? 0.115f : 0.056f) * stature;
            spec.material = GarmentMaterial::Leather;
            spec.materialKind = material::MaterialKind::Leather;
            spec.tint = SrgbToLinear(PickColour({{0.08f, 0.08f, 0.08f}, {0.25f, 0.14f, 0.07f}, {0.14f, 0.09f, 0.06f}, {0.55f, 0.55f, 0.52f}}, seed, 5u));
            spec.thickness = boots ? 0.0040f : 0.0030f;
            // Boots stand off further than the trousers that go into them; the outsole adds its own thickness under the foot.
            const float base = boots ? 0.0110f : 0.0050f;
            spec.ease = [=](glm::vec3 p) { return base + 0.0060f * SmoothStep(0.028f, 0.004f, p.y); };
            spec.field = [=](const MeshVertex& v) { return std::min(top - v.position.y, (WeightOnLimbs(v, false) - 0.5f) * 0.1f); };
            break;
        }
    }
    return spec;
}

// The move that brings a point to `minimum` away from the surface in `body`, seen from outside: zero if it is far enough. The move is
// away from the nearest point of the surface, so that where two parts of the body face each other across a narrow gap (the two thighs)
// the point settles between them instead of being driven into one by the other. A point inside the surface comes out through it.
glm::vec3 ClearancePush(const TriangleGrid& body, glm::vec3 point, float minimum) {
    const TriangleGrid::Nearest nearest = body.FindNearest(point, minimum);
    if (nearest.distance >= minimum) {
        return glm::vec3(0.0f);
    }
    const bool inside = glm::dot(point - nearest.point, nearest.faceNormal) < 0.0f;
    if (inside || nearest.distance < 1e-6f) {
        return nearest.faceNormal * (nearest.distance + minimum);
    }
    return (point - nearest.point) / nearest.distance * (minimum - nearest.distance);
}

// Moves the points of `inner`, the inner surface of a garment, until the triangles of `body` are at least `minimum` away from the
// points and also from the middles of the triangles and of their edges: a triangle can cut across a hollow of the body that its
// corners clear, as at the crotch or under an arm.
void EnforceClearance(std::vector<glm::vec3>& inner, const std::vector<uint32_t>& indices, const TriangleGrid& body, float minimum, JobSystem* jobs) {
    constexpr int kPasses = 150;
    const size_t triangleCount = indices.size() / 3;
    std::vector<glm::vec3> pushOfVertex(inner.size());
    std::vector<glm::vec3> pushOfTriangle(triangleCount);
    // The body does not move, so a point or a triangle that was far enough stays so until one of its corners moves: after the first
    // pass only what moved, and what touches it, is looked at again.
    std::vector<char> vertexActive(inner.size(), 1);
    std::vector<char> triangleActive(triangleCount, 1);
    const auto longer = [](glm::vec3 a, glm::vec3 b) { return glm::dot(a, a) >= glm::dot(b, b) ? a : b; };
    for (int pass = 0; pass < kPasses; ++pass) {
        ParallelEach(jobs, static_cast<int>(inner.size()), [&](int v) { pushOfVertex[v] = vertexActive[v] ? ClearancePush(body, inner[v], minimum) : glm::vec3(0.0f); });
        ParallelEach(jobs, static_cast<int>(triangleCount), [&](int t) {
            if (!triangleActive[t]) {
                pushOfTriangle[t] = glm::vec3(0.0f);
                return;
            }
            const glm::vec3 a = inner[indices[t * 3]];
            const glm::vec3 b = inner[indices[t * 3 + 1]];
            const glm::vec3 c = inner[indices[t * 3 + 2]];
            pushOfTriangle[t] = longer(longer(ClearancePush(body, (a + b + c) / 3.0f, minimum), ClearancePush(body, 0.5f * (a + b), minimum)),
                                       longer(ClearancePush(body, 0.5f * (b + c), minimum), ClearancePush(body, 0.5f * (c + a), minimum)));
        });
        // A triangle that is too close moves all three of its corners.
        for (size_t t = 0; t < triangleCount; ++t) {
            for (int corner = 0; corner < 3; ++corner) {
                glm::vec3& push = pushOfVertex[indices[t * 3 + corner]];
                push = longer(push, pushOfTriangle[t]);
            }
        }
        bool changed = false;
        for (size_t v = 0; v < inner.size(); ++v) {
            const float length = glm::length(pushOfVertex[v]);
            vertexActive[v] = length > 0.0f ? 1 : 0;
            if (length > 0.0f) {
                inner[v] += pushOfVertex[v] * (1.0f + 0.0002f / length);
                changed = true;
            }
        }
        if (!changed) {
            break;
        }
        for (size_t t = 0; t < triangleCount; ++t) {
            triangleActive[t] = vertexActive[indices[t * 3]] || vertexActive[indices[t * 3 + 1]] || vertexActive[indices[t * 3 + 2]];
        }
    }
}

// A closed shell from an open surface: the surface itself pushed out by `ease` (the inner side), a copy a thickness further, and a
// strip along every boundary edge to join the two.
TriangleMesh MakeShell(const TriangleMesh& surface, const std::function<float(glm::vec3)>& ease, float thickness, const TriangleGrid& layers, const TriangleGrid& bodyOnly, JobSystem* jobs) {
    const size_t count = surface.vertices.size();
    std::vector<glm::vec3> inner(count);
    std::vector<glm::vec3> normals(count);
    for (size_t v = 0; v < count; ++v) {
        normals[v] = surface.vertices[v].normal;
        inner[v] = surface.vertices[v].position + normals[v] * ease(surface.vertices[v].position);
    }
    // First clear everything below (the body and the garments under this one), then clear the body alone: where the layers are pinched
    // together (under an arm) the body has the last word, so that a garment never touches the body.
    EnforceClearance(inner, surface.indices, layers, kMinimumClearance + 0.0004f, jobs);
    EnforceClearance(inner, surface.indices, bodyOnly, kMinimumClearance + 0.0004f, jobs);
    const std::vector<glm::vec3> outwards = VertexNormals(inner, surface.indices);

    TriangleMesh shell;
    shell.vertices.resize(count * 2);
    for (size_t v = 0; v < count; ++v) {
        MeshVertex vertex = surface.vertices[v];
        vertex.position = inner[v];
        vertex.attribute.z = 0.0f;
        shell.vertices[v] = vertex;
        vertex.position = inner[v] + outwards[v] * thickness;
        vertex.attribute.z = 1.0f;
        shell.vertices[count + v] = vertex;
    }
    const uint32_t outerBase = static_cast<uint32_t>(count);
    for (size_t t = 0; t + 2 < surface.indices.size(); t += 3) {
        const uint32_t a = surface.indices[t];
        const uint32_t b = surface.indices[t + 1];
        const uint32_t c = surface.indices[t + 2];
        shell.indices.insert(shell.indices.end(), {outerBase + a, outerBase + b, outerBase + c});
        shell.indices.insert(shell.indices.end(), {a, c, b});
    }
    // Boundary edges: a directed edge of a triangle whose reverse belongs to no triangle.
    std::map<std::pair<uint32_t, uint32_t>, int> directed;
    for (size_t t = 0; t + 2 < surface.indices.size(); t += 3) {
        for (int c = 0; c < 3; ++c) {
            ++directed[{surface.indices[t + c], surface.indices[t + (c + 1) % 3]}];
        }
    }
    for (const auto& [edge, uses] : directed) {
        (void)uses;
        if (directed.count({edge.second, edge.first}) != 0) {
            continue;
        }
        const uint32_t p = edge.first;
        const uint32_t q = edge.second;
        shell.indices.insert(shell.indices.end(), {outerBase + q, outerBase + p, p});
        shell.indices.insert(shell.indices.end(), {outerBase + q, p, q});
    }
    std::vector<glm::vec3> positions(shell.vertices.size());
    for (size_t v = 0; v < positions.size(); ++v) {
        positions[v] = shell.vertices[v].position;
    }
    const std::vector<glm::vec3> shellNormals = VertexNormals(positions, shell.indices);
    for (size_t v = 0; v < shell.vertices.size(); ++v) {
        shell.vertices[v].normal = shellNormals[v];
    }
    return shell;
}

// The procedural material of a garment, sampled at a point of its surface by a flat projection along the dominant axis of the normal.
class ClothSampler {
public:
    explicit ClothSampler(const material::MaterialMaps& maps) : maps(maps) {
        double sum = 0.0;
        for (const glm::vec3& c : maps.albedo) {
            sum += Luma(c);
        }
        meanLuma = static_cast<float>(sum / std::max<size_t>(1, maps.albedo.size()));
    }

    struct Sample {
        float lumaRatio = 1.0f;  // the luminance of the material here, against its average: the pattern without the colour
        float roughness = 0.6f;
        float height = 0.0f;
    };

    Sample At(glm::vec3 p, glm::vec3 normal) const {
        const glm::vec3 a = glm::abs(normal);
        glm::vec2 plane;
        if (a.y > a.x && a.y > a.z) {
            plane = glm::vec2(p.x, p.z);
        } else if (a.x > a.z) {
            plane = glm::vec2(p.z, p.y);
        } else {
            plane = glm::vec2(p.x, p.y);
        }
        const float u = plane.x / maps.tileMeters;
        const float v = plane.y / maps.tileMeters;
        const float x = (u - std::floor(u)) * static_cast<float>(maps.size);
        const float y = (v - std::floor(v)) * static_cast<float>(maps.size);
        const int x0 = static_cast<int>(x) % maps.size;
        const int y0 = static_cast<int>(y) % maps.size;
        const size_t i = static_cast<size_t>(y0) * maps.size + x0;
        Sample sample;
        sample.lumaRatio = Luma(maps.albedo[i]) / std::max(meanLuma, 1e-4f);
        sample.roughness = maps.roughness[i];
        sample.height = maps.height[i] - 0.5f * maps.reliefMeters;
        return sample;
    }

private:
    static float Luma(glm::vec3 c) { return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b; }

    const material::MaterialMaps& maps;
    float meanLuma = 1.0f;
};

float Groove(float distance, float halfWidth) {
    const float x = distance / halfWidth;
    return std::exp(-x * x);
}

struct ClothPixel {
    glm::vec3 albedo{0.5f};
    float roughness = 0.7f;
    float height = 0.0f;
};

uint8_t Byte(float value) {
    return static_cast<uint8_t>(std::lround(glm::clamp(value, 0.0f, 1.0f) * 255.0f));
}

// The colour, roughness and relief of a garment at a texel: the weave, the stitching along its edges and the seams of its cut.
ClothPixel PaintCloth(const GarmentSpec& spec, const ClothSampler& cloth, const Texel& texel, float stature) {
    const glm::vec3 p = texel.position;
    const glm::vec3 n = texel.normal;
    const ClothSampler::Sample weave = cloth.At(p, n);
    ClothPixel pixel;
    // Only a trace of the weave goes into the albedo: its fine grain is smaller than a texel of this atlas, and the renderer adds it
    // from the material library at the right scale.
    pixel.albedo = spec.tint * (1.0f + 0.30f * (weave.lumaRatio - 1.0f));
    pixel.roughness = weave.roughness;
    pixel.height = 0.4f * weave.height;
    glm::vec3 thread = glm::mix(spec.tint, glm::vec3(0.55f, 0.40f, 0.12f), spec.kind == GarmentKind::Trousers ? 0.9f : 0.0f);
    thread = spec.kind == GarmentKind::Trousers ? thread : glm::min(spec.tint * 1.7f + glm::vec3(0.02f), glm::vec3(1.0f));

    float stitch = 0.0f;
    // Stitching along the hem, the cuffs and the collar: two rows, with the cloth folded over between them and the edge.
    const float edge = texel.attribute.x;
    stitch = std::max(stitch, Groove(edge - 0.0110f, 0.0007f));
    pixel.height += 0.0004f * SmoothStep(0.0125f, 0.0090f, edge) * SmoothStep(0.0f, 0.002f, edge);
    pixel.height -= 0.0005f * Groove(edge - 0.0110f, 0.0007f);

    // The seams down the sides of the body, where the front and back panels of the garment meet: the surface faces sideways there.
    const bool onTrunk = texel.tile == AtlasTile::Trunk;
    const bool onArm = texel.tile == AtlasTile::ArmLeft || texel.tile == AtlasTile::ArmRight;
    const float radius = (onTrunk ? 0.16f : (onArm ? 0.045f : 0.085f)) * stature / 1.75f;
    const float toSeam = std::abs(n.z) * radius;
    const float sidewaysFacing = SmoothStep(0.55f, 0.85f, std::abs(n.x));
    const float seam = Groove(toSeam, 0.0011f) * sidewaysFacing;
    pixel.height -= 0.0007f * seam;
    stitch = std::max(stitch, seam * (0.4f + 0.6f * SmoothStep(0.25f, 0.75f, std::abs(std::sin(p.y * 520.0f)))));

    if (spec.kind == GarmentKind::Jacket) {
        // The zip down the front: a groove with teeth.
        const float zip = Groove(std::abs(p.x), 0.0030f) * SmoothStep(0.2f, 0.7f, n.z);
        pixel.height -= 0.0008f * zip;
        pixel.height += 0.0004f * zip * SmoothStep(0.3f, 0.7f, std::sin(p.y * 1050.0f) * 0.5f + 0.5f);
        pixel.albedo = glm::mix(pixel.albedo, glm::vec3(0.30f), 0.75f * zip);
        pixel.roughness = glm::mix(pixel.roughness, 0.3f, zip);
    }
    if ((spec.kind == GarmentKind::Shoes || spec.kind == GarmentKind::Boots)) {
        // The outsole: rubber below the welt, with a groove where it joins the upper.
        const float sole = SmoothStep(0.017f, 0.013f, p.y);
        pixel.albedo = glm::mix(pixel.albedo, glm::vec3(0.025f), sole);
        pixel.roughness = glm::mix(pixel.roughness, 0.85f, sole);
        pixel.height -= 0.0006f * Groove(p.y - 0.0145f, 0.0008f);
        stitch = std::max(stitch, 0.7f * Groove(p.y - 0.0185f, 0.0006f));
    }
    pixel.albedo = glm::mix(pixel.albedo, thread, 0.8f * stitch);
    pixel.height -= 0.0003f * stitch;
    if (spec.kind == GarmentKind::Trousers) {
        // Denim fades where it is worn: at the hems and over the knees and thighs.
        const float worn = spec.wear * (0.5f * SmoothStep(0.05f, 0.0f, edge) + 0.5f * SmoothStep(0.3f, 0.7f, std::abs(n.z)) * Fbm3(p * 9.0f, 2, 17) * 0.5f + 0.1f);
        pixel.albedo = glm::mix(pixel.albedo, pixel.albedo * 1.5f + glm::vec3(0.02f), glm::clamp(worn, 0.0f, 0.6f));
    }
    pixel.albedo = glm::clamp(pixel.albedo, glm::vec3(0.0f), glm::vec3(1.0f));
    pixel.roughness = glm::clamp(pixel.roughness, 0.1f, 1.0f);
    return pixel;
}

GarmentTextures BakeGarmentTextures(const GarmentSpec& spec, const TriangleMesh& outerSurface, float stature, int tileSize, uint32_t seed, JobSystem* jobs) {
    // The material is made at a size that shows its weave, and no larger than the atlas tile asks for.
    material::MaterialSettings settings;
    settings.kind = spec.materialKind;
    settings.seed = seed + static_cast<uint32_t>(spec.kind) * 13u;
    settings.size = 256;
    settings.wear = spec.wear;
    settings.scanDetail = 0.5f;
    const material::MaterialMaps maps = material::GenerateMaterial(settings, jobs);
    const ClothSampler cloth(maps);

    const TexelField texels = BakeTexels(outerSurface, tileSize);
    std::vector<float> heights(static_cast<size_t>(texels.width) * texels.height, 0.0f);
    GarmentTextures textures;
    textures.tileSize = tileSize;
    textures.albedo = ByteImage(texels.width, texels.height, 4);
    textures.roughness = ByteImage(texels.width, texels.height, 1);
    ParallelEach(jobs, texels.height, [&](int y) {
        for (int x = 0; x < texels.width; ++x) {
            const Texel& texel = texels.At(x, y);
            uint8_t* albedo = textures.albedo.Pixel(x, y);
            ClothPixel pixel;
            if (texel.Used()) {
                pixel = PaintCloth(spec, cloth, texel, stature);
                heights[static_cast<size_t>(y) * texels.width + x] = pixel.height;
            } else {
                pixel.albedo = spec.tint;
            }
            const glm::vec3 srgb = glm::pow(pixel.albedo, glm::vec3(1.0f / 2.2f));
            albedo[0] = Byte(srgb.r);
            albedo[1] = Byte(srgb.g);
            albedo[2] = Byte(srgb.b);
            albedo[3] = 255;
            *textures.roughness.Pixel(x, y) = Byte(pixel.roughness);
        }
    });
    textures.normal = NormalMapFromHeights(texels, heights, tileSize, jobs);
    return textures;
}

Garment BuildGarment(const CharacterMesh& character, const TriangleMesh& body, const TriangleGrid& collision, const TriangleGrid& bodyOnly, const GarmentSpec& spec, int tileSize, JobSystem* jobs) {
    Garment garment;
    garment.kind = spec.kind;
    garment.material = spec.material;
    garment.materialKind = spec.materialKind;
    garment.tint = spec.tint;
    garment.thicknessMeters = spec.thickness;
    garment.clearanceMeters = spec.ease(glm::vec3(0.0f, character.skeleton.statureMeters * 0.6f, 0.0f));

    // Cut the part of the body that the garment covers; the field is also kept, as the distance from the edge.
    TriangleMesh covered = body;
    std::vector<float> field(covered.vertices.size());
    for (size_t v = 0; v < field.size(); ++v) {
        field[v] = spec.field(covered.vertices[v]);
        covered.vertices[v].attribute = glm::vec4(field[v], 0.0f, 0.0f, 0.0f);
    }
    const TriangleMesh surface = ClipByField(covered, field);
    if (surface.indices.empty()) {
        return garment;
    }
    TriangleMesh shell = MakeShell(surface, spec.ease, spec.thickness, collision, bodyOnly, jobs);
    // The shell before the unwrap: its vertices are shared by every triangle that meets there, so it is a watertight surface.
    garment.solid.positions.resize(shell.vertices.size());
    for (size_t v = 0; v < shell.vertices.size(); ++v) {
        garment.solid.positions[v] = shell.vertices[v].position;
    }
    garment.solid.indices = shell.indices;
    const TriangleMesh unwrapped = Unwrap(shell, character.skeleton, tileSize, -1);
    garment.mesh = ToRenderMesh(unwrapped);
    garment.vertexLayer.resize(unwrapped.vertices.size());
    for (size_t v = 0; v < unwrapped.vertices.size(); ++v) {
        garment.vertexLayer[v] = unwrapped.vertices[v].attribute.z > 0.5f ? 1 : 0;
    }
    garment.minimumClearance = MeasureClearance(character, garment);

    // The texture is baked from the outer surface alone: its triangles are the ones with no vertex on the inner one.
    TriangleMesh outer;
    outer.vertices = unwrapped.vertices;
    for (size_t t = 0; t + 2 < unwrapped.indices.size(); t += 3) {
        const uint32_t a = unwrapped.indices[t];
        const uint32_t b = unwrapped.indices[t + 1];
        const uint32_t c = unwrapped.indices[t + 2];
        if (garment.vertexLayer[a] == 1 && garment.vertexLayer[b] == 1 && garment.vertexLayer[c] == 1) {
            outer.indices.insert(outer.indices.end(), {a, b, c});
        }
    }
    garment.textures = BakeGarmentTextures(spec, outer, character.skeleton.statureMeters, tileSize, character.parameters.seed, jobs);
    return garment;
}

}  // namespace

const char* GarmentName(GarmentKind kind) {
    switch (kind) {
        case GarmentKind::TShirt: return "t-shirt";
        case GarmentKind::Jacket: return "jacket";
        case GarmentKind::Trousers: return "trousers";
        case GarmentKind::Shorts: return "shorts";
        case GarmentKind::Shoes: return "shoes";
        case GarmentKind::Boots: return "boots";
    }
    return "garment";
}

float MeasureClearance(const CharacterMesh& character, const Garment& garment) {
    const TriangleGrid grid(character.positions, character.indices, 0.03f);
    float least = 1e9f;
    for (size_t v = 0; v < garment.mesh.positions.size(); ++v) {
        if (garment.vertexLayer[v] == 0) {
            least = std::min(least, grid.FindNearest(garment.mesh.positions[v], 0.05f).distance);
        }
    }
    // The middles of the triangles of the inner surface too: a triangle may cut across a hollow that its corners clear.
    for (size_t t = 0; t + 2 < garment.mesh.indices.size(); t += 3) {
        const uint32_t id[3] = {garment.mesh.indices[t], garment.mesh.indices[t + 1], garment.mesh.indices[t + 2]};
        if (garment.vertexLayer[id[0]] == 0 && garment.vertexLayer[id[1]] == 0 && garment.vertexLayer[id[2]] == 0) {
            const glm::vec3 middle = (garment.mesh.positions[id[0]] + garment.mesh.positions[id[1]] + garment.mesh.positions[id[2]]) / 3.0f;
            least = std::min(least, grid.FindNearest(middle, 0.05f).distance);
        }
    }
    return least;
}

Outfit BuildOutfit(const CharacterMesh& character, const OutfitSettings& rawSettings, JobSystem* jobs) {
    OutfitSettings settings = rawSettings;
    settings.tileSize = std::clamp(settings.tileSize, 64, 1024);
    Outfit outfit;
    const BodyMeasures measures = MeasureBody(character.skeleton);
    const TriangleMesh body = ExtractPiece(character, 0);
    // Clearance is held against everything the character is made of (the body, the head and the hands) and against the garments
    // already made, so that each layer stands off the one below it.
    const TriangleGrid bodyOnly(character.positions, character.indices, 0.03f);
    std::vector<glm::vec3> obstaclePositions = character.positions;
    std::vector<uint32_t> obstacleIndices = character.indices;
    const uint32_t seed = character.parameters.seed;

    // From the inside out: what lies against the skin comes first, and each layer is built to stand off further than the one below.
    std::vector<GarmentKind> order;
    if (settings.legs == LegwearKind::Trousers) {
        order.push_back(GarmentKind::Trousers);
    } else if (settings.legs == LegwearKind::Shorts) {
        order.push_back(GarmentKind::Shorts);
    }
    if (settings.top == TopKind::TShirt || settings.top == TopKind::TShirtAndJacket) {
        order.push_back(GarmentKind::TShirt);
    }
    if (settings.top == TopKind::Jacket || settings.top == TopKind::TShirtAndJacket) {
        order.push_back(GarmentKind::Jacket);
    }
    if (settings.feet == FootwearKind::Shoes) {
        order.push_back(GarmentKind::Shoes);
    } else if (settings.feet == FootwearKind::Boots) {
        order.push_back(GarmentKind::Boots);
    }
    for (const GarmentKind kind : order) {
        Garment garment;
        {
            const TriangleGrid collision(obstaclePositions, obstacleIndices, 0.03f);
            garment = BuildGarment(character, body, collision, bodyOnly, MakeSpec(kind, measures, seed), settings.tileSize, jobs);
        }
        if (garment.mesh.indices.empty()) {
            continue;
        }
        const uint32_t firstVertex = static_cast<uint32_t>(obstaclePositions.size());
        obstaclePositions.insert(obstaclePositions.end(), garment.mesh.positions.begin(), garment.mesh.positions.end());
        for (const uint32_t index : garment.mesh.indices) {
            obstacleIndices.push_back(index + firstVertex);
        }
        if (kind == GarmentKind::Shoes || kind == GarmentKind::Boots) {
            for (const glm::vec3& p : garment.mesh.positions) {
                outfit.soleHeightMeters = std::max(outfit.soleHeightMeters, -p.y);
            }
        }
        outfit.garments.push_back(std::move(garment));
    }
    return outfit;
}

}  // namespace character
