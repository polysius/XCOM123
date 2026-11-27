#pragma once

#include <vector>

#include "character/CharacterGenerator.h"
#include "character/appearance/AppearanceTypes.h"
#include "character/appearance/MeshTools.h"
#include "character/appearance/Traits.h"

class JobSystem;

// The skin of a character: an unwrapped mesh of the body, the head and the hands with its texture coordinates (see Atlas.h
// for the unwrap), a few numbers per vertex for a vertex shader, and three textures baked from the surface: albedo, normal and
// roughness, which a renderer can upload as they are.
//
// Per vertex: the tone of the skin (a factor on the base colour), the redness of the blood near the surface (the cheeks,
// the nose, the ears, the knuckles, the elbows and the knees), the thickness of the flesh under the vertex and the curvature
// of the surface. Thin flesh (ears, nostrils, fingers) lets light through, and a subsurface scattering shader scales its blur
// with the thickness and adds light on the convex parts: those two are the hints for it.
//
// The textures hold what the eye needs of skin: tone, blood, freckles, moles, age spots, wrinkles, veins and pores, the
// colour of the scalp and of a beard under the hair, the lips, and an oiliness that makes the forehead and nose shinier.

namespace character {

struct SkinSettings {
    int tileSize = 512;  // texels along each tile of the atlas, 64 to 2048 (the atlas is 4 tiles by 2)
    HairStyle hairStyle = HairStyle::Short;
    FacialHair facialHair = FacialHair::None;
};

struct SkinVertexData {
    float tone = 1.0f;        // a factor on the base colour, about 0.9 to 1.1
    float redness = 0.0f;     // 0 to 1
    float thickness = 1.0f;   // 0 = paper thin, 1 = 6 cm or more of flesh
    float curvature = 0.5f;   // 0 = a deep crease, 0.5 = flat, 1 = a sharp convex edge
};

struct SkinTextures {
    ByteImage albedo;     // RGBA, sRGB colour, alpha 255
    ByteImage normal;     // RGBA, tangent space: x and y as 0.5 + 0.5 * value, z follows; alpha is the thickness of the flesh
    ByteImage roughness;  // one channel, 0 = mirror, 255 = chalk
    int tileSize = 0;
};

struct SkinResult {
    RenderMesh mesh;                          // the body, the head and the hands; their triangles are counted in `pieces`
    std::vector<MeshPiece> pieces;            // ranges of `mesh` for each piece of the character mesh, in the same order
    std::vector<SkinVertexData> vertexData;   // one for each vertex of `mesh`
    SkinTextures textures;
};

// Builds the skin. `jobs` may be null, which runs on the calling thread; the result is the same either way.
SkinResult BuildSkin(const CharacterMesh& character, const PersonTraits& traits, const SkinSettings& settings, JobSystem* jobs = nullptr);

// Converts a mesh of the unwrap stage to the arrays a renderer uses.
RenderMesh ToRenderMesh(const TriangleMesh& mesh);

}  // namespace character
