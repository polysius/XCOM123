#include "character/appearance/Skin.h"

#include <algorithm>
#include <cmath>

#include "character/ParallelEach.h"
#include "character/appearance/Atlas.h"
#include "character/appearance/HeadRegions.h"
#include "character/appearance/MeshTools.h"
#include "character/appearance/Noise3D.h"

namespace character {
namespace {

float Gaussian(float distance, float width) {
    const float x = distance / width;
    return std::exp(-x * x);
}

glm::vec3 ToSrgb(glm::vec3 linear) {
    return glm::pow(glm::clamp(linear, glm::vec3(0.0f), glm::vec3(1.0f)), glm::vec3(1.0f / 2.2f));
}

uint8_t Byte(float value) {
    return static_cast<uint8_t>(std::lround(glm::clamp(value, 0.0f, 1.0f) * 255.0f));
}

// A groove of a line: 1 on the line and 0 a little way from it, where `phase` counts lines (a whole number is a line).
float LineGroove(float phase, float sharpness) {
    const float distanceToLine = std::abs(phase - std::round(phase));
    return std::exp(-sharpness * distanceToLine * distanceToLine);
}

struct Mole {
    glm::vec3 position{0.0f};
    float radius = 0.002f;
};

// Everything that decides how the skin looks at a point of the body. The same painter serves the vertices (the low
// frequency part: tone and redness) and the texels (all of it).
class SkinPainter {
public:
    SkinPainter(const CharacterMesh& character, const PersonTraits& traits, const SkinSettings& settings)
        : skeleton(character.skeleton),
          traits(traits),
          settings(settings),
          landmarks(ComputeHeadLandmarks(character.parameters)),
          seed(character.parameters.seed),
          scale(character.parameters.statureMeters / 1.75f) {
        PlaceMoles(character);
        for (int i = 0; i < 2; ++i) {
            const bool left = i == 0;
            const glm::vec3 wrist = skeleton[left ? Joint::HandL : Joint::HandR].restPosition;
            const glm::vec3 elbow = skeleton[left ? Joint::ForearmL : Joint::ForearmR].restPosition;
            handAlong[i] = glm::normalize(wrist - elbow);
            handBack[i] = -glm::normalize((left ? 1.0f : -1.0f) * glm::cross(handAlong[i], glm::vec3(0.0f, 0.0f, 1.0f)));
        }
    }

    // How red the skin is at a point: the cheeks, the nose, the ears, the knuckles, the elbows and the knees.
    float Redness(glm::vec3 p, glm::vec3 normal, AtlasTile tile) const {
        float redness = 0.18f + 0.10f * Fbm3(p * 9.0f, 2, seed + 1);
        if (tile == AtlasTile::Head) {
            const glm::vec3 q = p / scale;
            for (int i = 0; i < 2; ++i) {
                const float side = i == 0 ? 1.0f : -1.0f;
                redness += 0.55f * Gaussian(glm::distance(q, glm::vec3(side * 0.043f * landmarks.headWidth, 1.607f, 0.064f)), 0.020f);
                redness += 0.50f * Gaussian(glm::distance(q, landmarks.ears[i] / scale), 0.022f);
            }
            redness += 0.45f * Gaussian(glm::distance(q, landmarks.noseTip / scale + glm::vec3(0.0f, 0.0f, -0.006f)), 0.014f);
            redness += 0.20f * LipWeight(landmarks, p);
        } else if (tile == AtlasTile::HandLeft || tile == AtlasTile::HandRight) {
            const int i = tile == AtlasTile::HandLeft ? 0 : 1;
            const glm::vec3 wrist = skeleton[i == 0 ? Joint::HandL : Joint::HandR].restPosition;
            const float along = glm::dot(p - wrist, handAlong[i]) / scale;
            const float onBack = SmoothStep(0.2f, 0.7f, glm::dot(normal, handBack[i]));
            // The knuckles of the fingers, and the middle joints of the fingers a little further on.
            redness += 0.35f * onBack * (Gaussian(along - 0.087f, 0.010f) + 0.7f * Gaussian(along - 0.126f, 0.008f));
            redness += 0.10f;
        }
        for (const Joint joint : {Joint::ForearmL, Joint::ForearmR, Joint::CalfL, Joint::CalfR}) {
            // The point of an elbow is behind it, the point of a knee in front.
            const bool elbow = joint == Joint::ForearmL || joint == Joint::ForearmR;
            const float facing = SmoothStep(0.1f, 0.7f, elbow ? -normal.z : normal.z);
            redness += 0.30f * facing * Gaussian(glm::distance(p, skeleton[joint].restPosition), 0.045f * scale);
        }
        return glm::clamp(redness, 0.0f, 1.0f);
    }

    float Tone(glm::vec3 p) const {
        return 1.0f + 0.06f * Fbm3(p * 5.0f, 3, seed + 2);
    }

    struct Sample {
        glm::vec3 albedo{0.5f};
        float roughness = 0.5f;
        float height = 0.0f;  // in metres
    };

    Sample Paint(const Texel& texel) const {
        const glm::vec3 p = texel.position;
        const glm::vec3 q = p / scale;
        const bool onHead = texel.tile == AtlasTile::Head;
        const bool onHand = texel.tile == AtlasTile::HandLeft || texel.tile == AtlasTile::HandRight;
        const bool onArm = texel.tile == AtlasTile::ArmLeft || texel.tile == AtlasTile::ArmRight;
        Sample sample;
        float height = 0.0f;

        // Tone and blood.
        glm::vec3 albedo = traits.skinColour * texel.attribute.x;
        albedo *= 1.0f + 0.035f * Fbm3(p * 60.0f, 2, seed + 3);
        const float blood = glm::clamp(texel.attribute.y * traits.ruddiness * 1.25f, 0.0f, 1.0f);
        albedo *= glm::vec3(1.0f + 0.16f * blood, 1.0f - 0.34f * blood, 1.0f - 0.30f * blood);

        // Exposure to the sun: the face, the shoulders, the forearms and the hands.
        float sun = 0.15f;
        if (onHead) {
            const float cheeks = Gaussian(std::abs(q.x) - 0.040f, 0.030f) * Gaussian(q.y - 1.612f, 0.028f);
            const float noseBridge = Gaussian(q.x, 0.020f) * Gaussian(q.y - 1.628f, 0.022f);
            sun = std::max({cheeks, noseBridge, 0.25f * SmoothStep(1.66f, 1.70f, q.y)});
        } else if (onHand || onArm) {
            sun = 0.7f;
        }

        // Freckles: small round spots, one candidate to a cell, kept where the sun reaches.
        if (traits.freckles > 0.01f) {
            const Cellular3 cell = CellularNoise3(p / 0.0045f, seed + 11);
            const float chance = Random01(cell.cellHash, seed, 5u);
            const float radius = 0.20f + 0.25f * Random01(cell.cellHash, seed, 6u);
            if (chance < traits.freckles * 0.9f * sun) {
                const float spot = 1.0f - SmoothStep(radius * 0.55f, radius, cell.nearest);
                albedo *= glm::mix(glm::vec3(1.0f), glm::vec3(0.80f, 0.62f, 0.48f), 0.85f * spot * (1.0f - 0.6f * traits.melanin));
            }
        }

        // Age spots: blotches, on the parts the sun reaches.
        if (traits.ageSpots > 0.01f) {
            const float patch = SmoothStep(0.30f, 0.52f, Fbm3(p * 45.0f + glm::vec3(7.0f), 2, seed + 21));
            albedo *= glm::mix(glm::vec3(1.0f), glm::vec3(0.78f, 0.62f, 0.50f), 0.8f * patch * traits.ageSpots * sun);
        }

        // Moles: a few dark, slightly raised spots.
        for (const Mole& mole : moles) {
            const float distance = glm::distance(p, mole.position);
            if (distance < mole.radius) {
                const float spot = 1.0f - SmoothStep(mole.radius * 0.55f, mole.radius, distance);
                albedo = glm::mix(albedo, glm::vec3(0.07f, 0.04f, 0.03f), 0.85f * spot);
                height += 0.0004f * spot;
            }
        }

        float oiliness = 0.0f;
        if (onHead) {
            height += FaceCreases(q, p, albedo);
            oiliness = std::max(Gaussian(q.x, 0.030f) * SmoothStep(1.62f, 1.66f, q.y) * (1.0f - SmoothStep(1.695f, 1.72f, q.y)),
                                Gaussian(q.x, 0.016f) * Gaussian(q.y - 1.62f, 0.022f));
        }
        if (onHand || onArm) {
            height += HandAndArmCreases(texel, albedo);
        }
        if (texel.tile == AtlasTile::Trunk && q.y > 1.43f) {
            // The lines of the neck, running round it, above the collar bones.
            const float neck = SmoothStep(1.43f, 1.47f, q.y) * (1.0f - SmoothStep(1.52f, 1.55f, q.y));
            const float phase = q.y * 95.0f + 3.0f * Fbm3(p * 25.0f, 2, seed + 31);
            height -= 0.00045f * traits.wrinkles * neck * LineGroove(phase, 55.0f);
        }

        // Veins, bluish and a little raised, on the backs of the hands, the forearms and the temples.
        height += Veins(texel, albedo);

        // The lips.
        const float lip = onHead ? LipWeight(landmarks, p) : 0.0f;
        if (lip > 0.0f) {
            const glm::vec3 lipColour = glm::mix(traits.skinColour * glm::vec3(1.08f, 0.62f, 0.62f), glm::vec3(0.12f, 0.045f, 0.04f), 0.7f * traits.melanin);
            albedo = glm::mix(albedo, lipColour, 0.85f * lip);
            height -= 0.00015f * lip * LineGroove(p.x * 700.0f + 2.0f * Fbm3(p * 90.0f, 2, seed + 41), 40.0f) * (0.5f + traits.wrinkles);
        }

        // Pores: shallow pits on a loose grid, most on the nose and cheeks. The grid is never finer than the texels can show.
        const float poreSpacing = std::max(0.0011f, 2.3f * texel.metersPerTexel);
        const Cellular3 pores = CellularNoise3(p / poreSpacing, seed + 51);
        const float pit = 1.0f - SmoothStep(0.0f, 0.30f, pores.nearest);
        float poreAmount = onHead ? 0.8f + 0.9f * Gaussian(q.x, 0.03f) * Gaussian(q.y - 1.62f, 0.03f) : (onHand ? 0.5f : 0.35f);
        poreAmount *= 1.0f - lip;
        height -= 0.00010f * pit * poreAmount;

        // Fine lines of the skin, a net of shallow creases that deepens with age.
        height += 0.00010f * (0.4f + traits.wrinkles) * (Fbm3(p * 260.0f, 2, seed + 61));
        height += 0.00020f * Fbm3(p * 55.0f, 2, seed + 62);

        // Hair that is painted on: the dark of the scalp under the hair, and the shadow of stubble or a beard.
        float shine = 0.0f;
        if (onHead) {
            albedo = PaintHairOnSkin(p, albedo, shine);
        }

        sample.albedo = glm::clamp(albedo, glm::vec3(0.0f), glm::vec3(1.0f));
        const float roughnessNoise = 0.05f * Fbm3(p * 70.0f, 2, seed + 71);
        float roughness = 0.58f + roughnessNoise + 0.07f * pit * poreAmount - 0.16f * oiliness - 0.20f * lip;
        roughness = glm::mix(roughness, 0.75f, shine);
        sample.roughness = glm::clamp(roughness, 0.2f, 0.9f);
        sample.height = height;
        return sample;
    }

private:
    void PlaceMoles(const CharacterMesh& character) {
        const int count = 3 + static_cast<int>(std::lround(12.0f * traits.moles));
        const MeshPiece& body = character.pieces.front();
        for (int i = 0; i < count; ++i) {
            const uint32_t pick = material::Hash(seed, 900u + static_cast<uint32_t>(i), 1u) % body.vertexCount;
            Mole mole;
            mole.position = character.positions[body.firstVertex + pick];
            mole.radius = (0.0012f + 0.0022f * Random01(seed, 900u + static_cast<uint32_t>(i), 2u)) * scale;
            moles.push_back(mole);
        }
        // A mole or two on the face, where they are noticed.
        if (character.pieces.size() > 1) {
            const MeshPiece& head = character.pieces[1];
            for (int i = 0; i < 2; ++i) {
                const uint32_t pick = material::Hash(seed, 950u + static_cast<uint32_t>(i), 1u) % head.vertexCount;
                const glm::vec3 position = character.positions[head.firstVertex + pick];
                if (ScalpWeight(landmarks, traits, position) < 0.1f && Random01(seed, 950u + static_cast<uint32_t>(i), 3u) < 0.3f + 0.5f * traits.moles) {
                    moles.push_back({position, 0.0015f * scale});
                }
            }
        }
    }

    // Lines of the face that deepen with age: across the forehead, round the eyes (crow's feet), from the nose to the
    // mouth, and the dark hollow under the eyes. Returns the change of height, and darkens `albedo` where it must.
    float FaceCreases(glm::vec3 q, glm::vec3 p, glm::vec3& albedo) const {
        const float wrinkles = traits.wrinkles;
        float height = 0.0f;
        const float inFront = SmoothStep(0.03f, 0.06f, q.z);

        // Across the forehead: even lines that waver, over the brows and under the hairline.
        const float forehead = inFront * SmoothStep(1.676f, 1.686f, q.y) * (1.0f - SmoothStep(1.696f, 1.710f, q.y)) * (1.0f - SmoothStep(0.048f, 0.062f, std::abs(q.x)));
        const float foreheadPhase = q.y * 105.0f + 0.55f * Fbm3(p * 22.0f, 2, seed + 81);
        height -= 0.00075f * wrinkles * forehead * LineGroove(foreheadPhase, 70.0f);

        for (int i = 0; i < 2; ++i) {
            const float side = i == 0 ? 1.0f : -1.0f;
            const glm::vec3 eye = landmarks.eyeSockets[i].centre / scale;
            // Crow's feet: lines that fan out from the outer corner of the eye.
            const glm::vec2 fromCorner(side * (q.x - eye.x) - 0.019f * landmarks.headWidth, q.y - eye.y);
            const float distance = glm::length(fromCorner);
            const float angle = std::atan2(fromCorner.y, fromCorner.x);
            const float fan = Gaussian(angle, 0.9f) * SmoothStep(0.003f, 0.007f, distance) * (1.0f - SmoothStep(0.016f, 0.024f, distance));
            height -= 0.00060f * wrinkles * fan * LineGroove(angle * 3.3f + 0.4f * Fbm3(p * 40.0f, 1, seed + 83), 12.0f) * inFront * 0.5f;

            // The fold from the wing of the nose to the corner of the mouth.
            const glm::vec2 top(side * 0.0185f * landmarks.headWidth, 1.601f);
            const glm::vec2 bottom(side * 0.031f, 1.560f);
            const glm::vec2 point2(q.x, q.y);
            const glm::vec2 along = bottom - top;
            const float t = glm::clamp(glm::dot(point2 - top, along) / glm::dot(along, along), 0.0f, 1.0f);
            const float toLine = glm::length(point2 - (top + along * t));
            // The fold is wider and deeper with age, and starts at the lips' edge.
            height -= 0.00110f * (0.25f + wrinkles) * Gaussian(toLine, 0.0032f) * SmoothStep(0.05f, 0.075f, q.z) * (0.5f + 0.5f * t);
            // Both sides of the fold have the cheek pushed up above it.
            height += 0.00040f * wrinkles * Gaussian(toLine - 0.008f, 0.0035f) * SmoothStep(0.05f, 0.075f, q.z);

            // The hollow under the eye: a little darker and bluer, deeper with age.
            const float bag = Gaussian(q.x - eye.x, 0.016f) * Gaussian(q.y - (eye.y - 0.0150f), 0.0065f) * inFront;
            albedo *= glm::mix(glm::vec3(1.0f), glm::vec3(0.92f, 0.86f, 0.90f), bag * (0.45f + 0.55f * wrinkles));
            height -= 0.00035f * wrinkles * Gaussian(q.y - (eye.y - 0.0210f), 0.0020f) * Gaussian(q.x - eye.x, 0.017f) * inFront;
        }
        return height;
    }

    // Veins and the fine lines of the back of the hand.
    float Veins(const Texel& texel, glm::vec3& albedo) const {
        const glm::vec3 p = texel.position;
        float region = 0.0f;
        if (texel.tile == AtlasTile::HandLeft || texel.tile == AtlasTile::HandRight) {
            const int i = texel.tile == AtlasTile::HandLeft ? 0 : 1;
            region = SmoothStep(0.1f, 0.6f, glm::dot(texel.normal, handBack[i]));
        } else if (texel.tile == AtlasTile::ArmLeft || texel.tile == AtlasTile::ArmRight) {
            region = SmoothStep(0.45f, 0.85f, texel.tileUv.y) * 0.6f;  // the forearm, the lower half of the arm
        } else if (texel.tile == AtlasTile::Head) {
            const glm::vec3 q = p / scale;
            for (int i = 0; i < 2; ++i) {
                const float side = i == 0 ? 1.0f : -1.0f;
                region = std::max(region, Gaussian(glm::distance(q, glm::vec3(side * 0.060f * landmarks.headWidth, 1.660f, 0.030f)), 0.014f) * 0.7f);
            }
        }
        if (region <= 0.0f || traits.veins < 0.01f) {
            return 0.0f;
        }
        // A vein lies where the noise crosses zero: thin winding curves.
        const float field = Fbm3(p * 22.0f, 2, seed + 91);
        const float vein = std::exp(-field * field / 0.0036f) * region * traits.veins;
        albedo *= glm::mix(glm::vec3(1.0f), glm::vec3(0.76f, 0.88f, 1.0f), 0.65f * vein);
        return 0.00055f * vein;
    }

    float HandAndArmCreases(const Texel& texel, glm::vec3& albedo) const {
        // Loose, finely lined skin on the backs of old hands: lines that run across the hand.
        const glm::vec3 p = texel.position;
        const float lines = 1.0f - std::abs(Fbm3(p * glm::vec3(120.0f, 50.0f, 120.0f), 2, seed + 101));
        const float crease = SmoothStep(0.85f, 1.0f, lines);
        albedo *= 1.0f - 0.05f * crease * traits.wrinkles;
        return -0.00030f * crease * (0.3f + traits.wrinkles);
    }

    glm::vec3 PaintHairOnSkin(glm::vec3 p, glm::vec3 albedo, float& shine) const {
        const float grey = traits.greyFraction;
        const glm::vec3 hairColour = glm::mix(traits.hairColour, glm::vec3(0.42f), grey * 0.85f);
        if (settings.hairStyle != HairStyle::Bald) {
            // The scalp under the hair is dark with the roots; a thinning head shows some skin through.
            const float scalp = ScalpWeight(landmarks, traits, p) * glm::clamp(traits.hairDensity * 1.25f - 0.1f, 0.0f, 1.0f);
            albedo = glm::mix(albedo, hairColour * 0.75f, 0.9f * scalp);
            shine = std::max(shine, 0.9f * scalp);
        }
        if (settings.facialHair != FacialHair::None) {
            const float beard = BeardWeight(landmarks, p);
            // Stubble: a shadow of dots, the roots of hairs that have just come through.
            const Cellular3 roots = CellularNoise3(p / 0.0016f, seed + 111);
            const float dot = 1.0f - SmoothStep(0.10f, 0.38f, roots.nearest);
            const float cover = settings.facialHair == FacialHair::Beard ? 0.90f : 0.30f + 0.35f * dot;
            albedo = glm::mix(albedo, hairColour * 0.8f + glm::vec3(0.01f, 0.012f, 0.02f), cover * beard * 0.9f);
            shine = std::max(shine, cover * beard);
        }
        return albedo;
    }

    const Skeleton& skeleton;
    const PersonTraits& traits;
    const SkinSettings& settings;
    HeadLandmarks landmarks;
    uint32_t seed;
    float scale;
    std::vector<Mole> moles;
    glm::vec3 handAlong[2];
    glm::vec3 handBack[2];
};

// Thickness of the flesh under each vertex: the distance, along the inward normal, to the far side of the same piece.
// Curvature: how far the neighbours lie below the tangent plane.
void MeasureShape(const TriangleMesh& piece, JobSystem* jobs, std::vector<float>& thickness, std::vector<float>& curvature) {
    constexpr float kFullThickness = 0.06f;
    std::vector<glm::vec3> positions(piece.vertices.size());
    for (size_t v = 0; v < positions.size(); ++v) {
        positions[v] = piece.vertices[v].position;
    }
    const TriangleGrid grid(positions, piece.indices, 0.02f);
    std::vector<std::vector<uint32_t>> neighbours(positions.size());
    for (size_t t = 0; t + 2 < piece.indices.size(); t += 3) {
        for (int c = 0; c < 3; ++c) {
            neighbours[piece.indices[t + c]].push_back(piece.indices[t + (c + 1) % 3]);
        }
    }
    thickness.assign(positions.size(), 1.0f);
    curvature.assign(positions.size(), 0.5f);
    ParallelEach(jobs, static_cast<int>(positions.size()), [&](int v) {
        const glm::vec3 normal = piece.vertices[v].normal;
        const float hit = grid.Raycast(positions[v] - normal * 0.0005f, -normal, kFullThickness);
        thickness[v] = hit < 0.0f ? 1.0f : glm::clamp((hit + 0.0005f) / kFullThickness, 0.0f, 1.0f);

        float below = 0.0f;
        float spread = 0.0f;
        for (const uint32_t other : neighbours[v]) {
            const glm::vec3 offset = positions[other] - positions[v];
            below -= glm::dot(offset, normal);
            spread += glm::dot(offset, offset);
        }
        // Curvature in 1/m (about 2 / radius), mapped so that a convex fingertip (radius 8 mm) is near 1.
        const float k = spread > 1e-12f ? 2.0f * below / spread : 0.0f;
        curvature[v] = glm::clamp(0.5f + 0.5f * k / 120.0f, 0.0f, 1.0f);
    });
}

}  // namespace

RenderMesh ToRenderMesh(const TriangleMesh& mesh) {
    RenderMesh result;
    const size_t count = mesh.vertices.size();
    result.positions.resize(count);
    result.normals.resize(count);
    result.uvs.resize(count);
    result.boneIndices.resize(count);
    result.boneWeights.resize(count);
    for (size_t v = 0; v < count; ++v) {
        result.positions[v] = mesh.vertices[v].position;
        result.normals[v] = mesh.vertices[v].normal;
        result.uvs[v] = mesh.vertices[v].uv;
        result.boneIndices[v] = mesh.vertices[v].bones;
        result.boneWeights[v] = mesh.vertices[v].weights;
    }
    result.indices = mesh.indices;
    return result;
}

SkinResult BuildSkin(const CharacterMesh& character, const PersonTraits& traits, const SkinSettings& rawSettings, JobSystem* jobs) {
    SkinSettings settings = rawSettings;
    settings.tileSize = std::clamp(settings.tileSize, 64, 2048);
    const SkinPainter painter(character, traits, settings);

    // Unwrap each piece, with the numbers per vertex that follow the vertices through the unwrap in `attribute`.
    TriangleMesh unwrapped;
    SkinResult result;
    for (size_t piece = 0; piece < character.pieces.size(); ++piece) {
        TriangleMesh mesh = ExtractPiece(character, piece);
        const MeshPieceKind kind = character.pieces[piece].kind;
        const int forcedTile = kind == MeshPieceKind::Body ? -1 : static_cast<int>(TileForPiece(kind));

        std::vector<float> thickness;
        std::vector<float> curvature;
        MeasureShape(mesh, jobs, thickness, curvature);
        ParallelEach(jobs, static_cast<int>(mesh.vertices.size()), [&](int v) {
            MeshVertex& vertex = mesh.vertices[v];
            const AtlasTile tile = forcedTile >= 0 ? static_cast<AtlasTile>(forcedTile) : TileForBone(DominantBone(vertex));
            vertex.attribute = glm::vec4(painter.Tone(vertex.position), painter.Redness(vertex.position, vertex.normal, tile), thickness[v], curvature[v]);
        });

        const TriangleMesh part = Unwrap(mesh, character.skeleton, settings.tileSize, forcedTile);
        MeshPiece range;
        range.kind = kind;
        range.firstVertex = static_cast<uint32_t>(unwrapped.vertices.size());
        range.vertexCount = static_cast<uint32_t>(part.vertices.size());
        range.firstIndex = static_cast<uint32_t>(unwrapped.indices.size());
        range.indexCount = static_cast<uint32_t>(part.indices.size());
        unwrapped.vertices.insert(unwrapped.vertices.end(), part.vertices.begin(), part.vertices.end());
        for (const uint32_t index : part.indices) {
            unwrapped.indices.push_back(index + range.firstVertex);
        }
        result.pieces.push_back(range);
    }
    result.mesh = ToRenderMesh(unwrapped);
    result.vertexData.resize(unwrapped.vertices.size());
    for (size_t v = 0; v < unwrapped.vertices.size(); ++v) {
        const glm::vec4& a = unwrapped.vertices[v].attribute;
        result.vertexData[v] = {a.x, a.y, a.z, a.w};
    }

    // Bake: first the colour, roughness and height of every texel, then the normals from the heights.
    const TexelField texels = BakeTexels(unwrapped, settings.tileSize);
    const int width = texels.width;
    const int height = texels.height;
    std::vector<float> heights(static_cast<size_t>(width) * height, 0.0f);
    SkinTextures& textures = result.textures;
    textures.tileSize = settings.tileSize;
    textures.albedo = ByteImage(width, height, 4);
    textures.roughness = ByteImage(width, height, 1);
    ParallelEach(jobs, height, [&](int y) {
        for (int x = 0; x < width; ++x) {
            const Texel& texel = texels.At(x, y);
            uint8_t* albedoPixel = textures.albedo.Pixel(x, y);
            if (!texel.Used()) {
                // Texels that no triangle reaches show the average skin, so that nothing sampled near an edge is black.
                const glm::vec3 fill = ToSrgb(traits.skinColour);
                albedoPixel[0] = Byte(fill.r);
                albedoPixel[1] = Byte(fill.g);
                albedoPixel[2] = Byte(fill.b);
                albedoPixel[3] = 255;
                *textures.roughness.Pixel(x, y) = Byte(0.58f);
                continue;
            }
            const SkinPainter::Sample sample = painter.Paint(texel);
            const glm::vec3 colour = ToSrgb(sample.albedo);
            albedoPixel[0] = Byte(colour.r);
            albedoPixel[1] = Byte(colour.g);
            albedoPixel[2] = Byte(colour.b);
            albedoPixel[3] = 255;
            *textures.roughness.Pixel(x, y) = Byte(sample.roughness);
            heights[static_cast<size_t>(y) * width + x] = sample.height;
        }
    });
    textures.normal = NormalMapFromHeights(texels, heights, settings.tileSize, jobs);
    // The alpha of the normal map is the thickness of the flesh, for the subsurface hint.
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const Texel& texel = texels.At(x, y);
            textures.normal.Pixel(x, y)[3] = Byte(texel.Used() ? texel.attribute.z : 1.0f);
        }
    }
    return result;
}

}  // namespace character
