#pragma once

#include <glm/glm.hpp>

#include "character/CharacterParameters.h"
#include "character/appearance/AppearanceTypes.h"
#include "character/appearance/Ribbons.h"
#include "character/appearance/Traits.h"

class JobSystem;

// The eyes: two eyeballs, the lids around them and the lashes, set in the hollows that the head carves for them
// (see HeadLandmarks in DetailShapes.h).
//
// An eyeball is a sphere with the bulge of the cornea in front. It has the same texture on both sides: a view of the eye from the front,
// with the white of the eye and its veins, the iris with fibres that run out from the pupil, a darker ring at its edge and the
// pupil. Its uv is that view, a flat projection along the gaze, so only the front half of the sphere has a picture of its own. The
// roughness map is low over the cornea, so that a renderer shows a wet highlight there.
//
// The lids are two closed shells, thin curved slabs that wrap the front of the eyeball and spread out into the skin round the
// socket, where their edge is buried in the head. Their uv is that of the skin of the head, which the caller sets (see
// ProjectUvsOntoPiece), since they are made of the same skin. A lash is a ribbon.
//
// Everything follows the head bone. The skeleton has no eye bones, so the eyes look straight ahead and turn with the head.

namespace character {

struct EyeParts {
    glm::vec3 centre{0.0f};  // the centre of the eyeball, in the rest pose
    float radius = 0.012f;   // of the sphere of the eyeball
    RenderMesh eyeball;
    RenderMesh upperLid;     // a closed shell
    RenderMesh lowerLid;     // a closed shell
    RibbonSet lashes;
};

struct EyeTextures {
    ByteImage albedo;     // RGBA, sRGB; the view of the eye from the front
    ByteImage normal;     // RGBA, tangent space
    ByteImage roughness;  // one channel
};

struct EyeSet {
    EyeParts eyes[2];  // 0 is the character's left (+X)
    EyeTextures textures;
};

struct EyeSettings {
    int textureSize = 512;  // 64 to 2048
};

EyeSet BuildEyes(const CharacterParameters& parameters, const PersonTraits& traits, const EyeSettings& settings = {}, JobSystem* jobs = nullptr);

// The radius of the eyeball, cornea included, in the direction of a point from its centre: a point nearer to the centre than this is
// inside the eyeball.
float EyeballRadiusToward(const EyeParts& eye, glm::vec3 point);

}  // namespace character
