#pragma once

#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "character/CharacterGenerator.h"
#include "character/appearance/AppearanceTypes.h"
#include "character/appearance/MeshTools.h"
#include "material/MaterialGenerator.h"

class JobSystem;

// Clothes, made by wrapping the body: a garment is the part of the body surface that the garment covers (the triangles of
// the body mesh, cut along a smooth line where the garment ends: a hem, a cuff, a collar), moved out from the body by a clearance, with
// a second layer one thickness further out and a strip of cloth round the edge. So a garment is a closed shell with a thickness,
// and every vertex of it takes the skin weights of the body vertex it comes from: it moves with the skeleton as the body does.
//
// The clearance is the ease of the cloth: a t-shirt lies a few millimetres off the skin and a jacket a centimetre. Moving a vertex
// out along its normal is not enough where the body is hollow (under an arm, in the crook of a knee), so afterwards every vertex is
// checked against the triangles of the whole character (body, head and hands) and pushed out until it is at least `kMinimumClearance`
// away from them. That minimum is what the tests hold the garments to.
//
// A garment has its own texture atlas, laid out as the skin's is (see Atlas.h): the weave of the material (the procedural
// fabric or leather of src/material, tinted), seams and stitching as relief that goes into the normal map: a hem stitch, side seams, the
// zip of a jacket, the welt of a sole.
//
// Shoes have a sole that reaches below the floor the bare foot stands on; `soleHeightMeters` is how far, and a renderer lifts the
// character by that much when it shows shoes.

namespace character {

constexpr float kMinimumClearance = 0.002f;  // the least distance between a garment and the body, metres

enum class GarmentKind : uint8_t {
    TShirt,
    Jacket,
    Trousers,
    Shorts,
    Shoes,
    Boots,
};

const char* GarmentName(GarmentKind kind);

enum class TopKind : uint8_t { None, TShirt, Jacket, TShirtAndJacket };
enum class LegwearKind : uint8_t { None, Trousers, Shorts };
enum class FootwearKind : uint8_t { None, Shoes, Boots };

struct OutfitSettings {
    TopKind top = TopKind::TShirt;
    LegwearKind legs = LegwearKind::Trousers;
    FootwearKind feet = FootwearKind::Shoes;
    int tileSize = 512;  // texels along each tile of the garments' atlases, 64 to 1024
};

struct GarmentTextures {
    ByteImage albedo;     // RGBA, sRGB
    ByteImage normal;     // RGBA, tangent space, as the skin's
    ByteImage roughness;  // one channel
    int tileSize = 0;
};

struct Garment {
    GarmentKind kind = GarmentKind::TShirt;
    GarmentMaterial material = GarmentMaterial::Cotton;
    material::MaterialKind materialKind = material::MaterialKind::Fabric;  // the procedural material its textures are made from
    glm::vec3 tint{1.0f};                                                  // linear
    float thicknessMeters = 0.0f;
    float clearanceMeters = 0.0f;       // the ease the garment was built with
    float minimumClearance = 0.0f;      // the least distance of its inner surface from the character, as measured at the end
    RenderMesh mesh;                    // a closed shell, skinned like the body
    std::vector<uint8_t> vertexLayer;   // for each vertex of the mesh: 0 on the inner surface (towards the body), 1 on the outer one
    // The same shell before the unwrap, with no vertex split at a texture seam: a watertight, consistently oriented surface (no
    // edge in other than two triangles, every triangle facing outwards) for collision or physics. `mesh` has the same surface, cut
    // along the seams of the unwrap, where a triangle is split at a seam its neighbour in another tile is not.
    struct Solid {
        std::vector<glm::vec3> positions;
        std::vector<uint32_t> indices;
        uint32_t outerFirstVertex = 0;  // the vertices from this one on are on the outer surface, the ones before it on the inner
    } solid;
    GarmentTextures textures;
};

// The outer surface of a garment as a surface of its own, with the grid that finds the nearest point of it and which side a point is on.
// The things that are made after a garment and must stay outside it (the garments above it, the hair) are held clear of this.
struct OuterSurface {
    std::vector<glm::vec3> positions;
    std::vector<uint32_t> indices;
    std::unique_ptr<TriangleGrid> grid;

    explicit OuterSurface(const Garment& garment);
};

struct Outfit {
    std::vector<Garment> garments;  // from the inside out
    float soleHeightMeters = 0.0f;  // how far the shoes reach below the floor of the bare foot
};

// Dresses a character. Only the body piece is wrapped; the head and hands are left bare. `jobs` may be null; the result is the same
// either way.
Outfit BuildOutfit(const CharacterMesh& character, const OutfitSettings& settings, JobSystem* jobs = nullptr);

// The least distance from the vertices and the triangle middles of a garment's inner surface (layer 0) to the triangles of the character. The
// garment itself records this, and the tests measure it again independently with this function.
float MeasureClearance(const CharacterMesh& character, const Garment& garment);
float MeasureClearance(const TriangleGrid& character, const Garment& garment, JobSystem* jobs = nullptr);

}  // namespace character
