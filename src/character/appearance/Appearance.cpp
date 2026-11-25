#include "character/appearance/Appearance.h"

#include <algorithm>
#include <array>
#include <utility>

#include "character/appearance/Noise3D.h"

namespace character {
namespace {

enum Salt : uint32_t {
    kSaltTop = 301,
    kSaltLegs,
    kSaltFeet,
};

template <typename T, size_t N>
T Pick(const std::array<std::pair<float, T>, N>& table, float draw) {
    float total = 0.0f;
    for (const auto& entry : table) {
        total += entry.first;
    }
    float remaining = draw * total;
    for (const auto& entry : table) {
        remaining -= entry.first;
        if (remaining < 0.0f) {
            return entry.second;
        }
    }
    return table.back().second;
}

OutfitSettings ChooseOutfit(const CharacterParameters& parameters) {
    OutfitSettings outfit;
    const std::array<std::pair<float, TopKind>, 4> tops = {{{0.03f, TopKind::None}, {0.55f, TopKind::TShirt}, {0.20f, TopKind::Jacket}, {0.22f, TopKind::TShirtAndJacket}}};
    const std::array<std::pair<float, LegwearKind>, 3> legs = {{{0.05f, LegwearKind::None}, {0.65f, LegwearKind::Trousers}, {0.30f, LegwearKind::Shorts}}};
    const std::array<std::pair<float, FootwearKind>, 3> feet = {{{0.20f, FootwearKind::None}, {0.55f, FootwearKind::Shoes}, {0.25f, FootwearKind::Boots}}};
    outfit.top = Pick(tops, Random01(parameters.seed, kSaltTop, 1u));
    outfit.legs = Pick(legs, Random01(parameters.seed, kSaltLegs, 1u));
    outfit.feet = Pick(feet, Random01(parameters.seed, kSaltFeet, 1u));
    return outfit;
}

// Gives the vertices of a mesh the texture coordinates of the nearest point of one piece of the skin, so that something that is made of
// the same skin as the body (an eyelid) shows the same texture there.
void ProjectUvsOntoSkin(RenderMesh& target, const SkinResult& skin, MeshPieceKind kind) {
    for (const MeshPiece& piece : skin.pieces) {
        if (piece.kind != kind) {
            continue;
        }
        std::vector<glm::vec3> positions(skin.mesh.positions.begin() + piece.firstVertex, skin.mesh.positions.begin() + piece.firstVertex + piece.vertexCount);
        std::vector<uint32_t> indices(skin.mesh.indices.begin() + piece.firstIndex, skin.mesh.indices.begin() + piece.firstIndex + piece.indexCount);
        for (uint32_t& index : indices) {
            index -= piece.firstVertex;
        }
        const TriangleGrid grid(positions, indices, 0.02f);
        for (size_t v = 0; v < target.positions.size(); ++v) {
            const TriangleGrid::Nearest nearest = grid.FindNearest(target.positions[v], 0.04f);
            if (nearest.distance > 1e8f) {
                continue;
            }
            const uint32_t id[3] = {indices[nearest.triangle * 3], indices[nearest.triangle * 3 + 1], indices[nearest.triangle * 3 + 2]};
            // Barycentric weights of the nearest point in its triangle.
            const glm::vec3 a = positions[id[0]];
            const glm::vec3 edge1 = positions[id[1]] - a;
            const glm::vec3 edge2 = positions[id[2]] - a;
            const glm::vec3 offset = nearest.point - a;
            const float d11 = glm::dot(edge1, edge1);
            const float d12 = glm::dot(edge1, edge2);
            const float d22 = glm::dot(edge2, edge2);
            const float denominator = d11 * d22 - d12 * d12;
            if (std::abs(denominator) < 1e-18f) {
                continue;
            }
            const float w1 = (d22 * glm::dot(offset, edge1) - d12 * glm::dot(offset, edge2)) / denominator;
            const float w2 = (d11 * glm::dot(offset, edge2) - d12 * glm::dot(offset, edge1)) / denominator;
            const float w0 = 1.0f - w1 - w2;
            target.uvs[v] = w0 * skin.mesh.uvs[piece.firstVertex + id[0]] + w1 * skin.mesh.uvs[piece.firstVertex + id[1]] + w2 * skin.mesh.uvs[piece.firstVertex + id[2]];
        }
    }
}

}  // namespace

AppearanceSettings ChooseAppearance(const CharacterParameters& parameters) {
    AppearanceSettings settings;
    settings.hairStyle = ChooseHairStyle(parameters);
    settings.facialHair = ChooseFacialHair(parameters);
    settings.outfit = ChooseOutfit(Sanitised(parameters));
    return settings;
}

CharacterAppearance GenerateAppearance(const CharacterMesh& character, const AppearanceSettings& settings, JobSystem* jobs) {
    CharacterAppearance appearance;
    appearance.settings = settings;
    appearance.traits = DeriveTraits(character.parameters);

    SkinSettings skinSettings;
    skinSettings.tileSize = settings.skinTileSize;
    skinSettings.hairStyle = settings.hairStyle;
    skinSettings.facialHair = settings.facialHair;
    appearance.skin = BuildSkin(character, appearance.traits, skinSettings, jobs);

    bool hasHead = false;
    for (const MeshPiece& piece : character.pieces) {
        hasHead = hasHead || piece.kind == MeshPieceKind::Head;
    }
    if (hasHead) {
        EyeSettings eyeSettings;
        eyeSettings.textureSize = settings.eyeTextureSize;
        appearance.eyes = BuildEyes(character.parameters, appearance.traits, eyeSettings);
        for (EyeParts& eye : appearance.eyes.eyes) {
            ProjectUvsOntoSkin(eye.upperLid, appearance.skin, MeshPieceKind::Head);
            ProjectUvsOntoSkin(eye.lowerLid, appearance.skin, MeshPieceKind::Head);
        }
        appearance.hair = BuildHair(character, appearance.traits, {settings.hairStyle, settings.facialHair}, jobs);
    }
    appearance.outfit = BuildOutfit(character, settings.outfit, jobs);
    return appearance;
}

}  // namespace character
