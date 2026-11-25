#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "character/CharacterGenerator.h"
#include "character/appearance/AppearanceTypes.h"
#include "character/appearance/MeshTools.h"

class JobSystem;

// The unwrap of a body into a texture atlas, and the baking of the atlas.
//
// The unwrap is cylindrical, one cylinder for each part of the body: a tile of the atlas for the trunk, one for each
// arm, one for each leg, one for the head and one for each hand (8 tiles, 4 across and 2 down). A cylinder runs along
// the axis of its part (down the limb, down the trunk and head), u goes round it with the front of the body at u = 0.5 and the
// seam at the back, where hair or a seam of clothes hides it, and v goes along it with the top at v = 0. A triangle that
// crosses the seam is cut in two, so every vertex has a u in [0, 1] and no triangle needs a texture sampler that wraps.
// A cylinder stretches at the poles (the crown, the shoulder), which hair and cloth cover.
//
// Baking a texture means finding, for each texel, the point of the surface that lies there. Everything that is painted on
// the skin or a garment is then worked out from that point in space, not from the uv, so a pattern runs on across the
// edges of the tiles without a seam, and the unwrap can change without the textures changing.

namespace character {

constexpr int kAtlasTileCount = 8;
constexpr int kAtlasColumns = 4;
constexpr int kAtlasRows = 2;

enum class AtlasTile : uint8_t {
    Trunk,
    ArmLeft,
    ArmRight,
    LegLeft,
    LegRight,
    Head,
    HandLeft,
    HandRight,
};

// The tile a bone's part of the body is drawn on.
AtlasTile TileForBone(int bone);
AtlasTile TileForPiece(MeshPieceKind kind);

// Unwraps a mesh: gives every vertex its atlas uv (in [0, 1] over the whole atlas) and cuts the triangles that cross a
// seam. `pieceTile` forces the tile of every triangle when it is not negative, as for the head and hands; otherwise a triangle goes
// to the tile of the bone that most of its vertices follow.
TriangleMesh Unwrap(const TriangleMesh& mesh, const Skeleton& skeleton, int tileSize, int pieceTile = -1);

// What one texel of the atlas shows.
struct Texel {
    glm::vec3 position{0.0f};  // on the surface, in the rest pose
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    glm::vec4 attribute{0.0f};  // the attribute of the vertices, interpolated
    float metersPerTexel = 0.001f;
    AtlasTile tile = AtlasTile::Trunk;
    glm::vec2 tileUv{0.0f};  // where in its tile, 0 to 1
    bool covered = false;    // the texel's centre lies inside a triangle
    bool gutter = false;     // the texel lies just outside the triangles and shows what the nearest one shows

    bool Used() const { return covered || gutter; }
};

struct TexelField {
    int width = 0;
    int height = 0;
    std::vector<Texel> texels;

    Texel& At(int x, int y) { return texels[static_cast<size_t>(y) * width + x]; }
    const Texel& At(int x, int y) const { return texels[static_cast<size_t>(y) * width + x]; }
};

// Rasterises the triangles of an unwrapped mesh into the atlas and then grows the covered area by a few texels, so that
// filtering and mip maps never reach into empty texels.
TexelField BakeTexels(const TriangleMesh& unwrapped, int tileSize);

// A tangent space normal map (RGBA, alpha 255) from a height in metres for every texel: the slope between a texel's neighbours,
// which run along the uv axes of its tile, +X to the right and +Y up the image like the normal maps of src/material. A texel at
// the edge of its tile sees a flat step beyond it.
ByteImage NormalMapFromHeights(const TexelField& texels, const std::vector<float>& heights, int tileSize, JobSystem* jobs);

}  // namespace character
