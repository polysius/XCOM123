#pragma once

#include "character/CharacterGenerator.h"
#include "character/appearance/AppearanceTypes.h"
#include "character/appearance/Clothing.h"
#include "character/appearance/Ribbons.h"
#include "character/appearance/Traits.h"

class JobSystem;

// Hair, made of ribbons (see Ribbons.h): each ribbon is a lock of a few millimetres' width that grows from a root on the
// scalp and follows the head. The choice of ribbons over single strands is the one a game makes: a head of hair is
// thousands of locks, not a hundred thousand strands, and a ribbon draws as one piece of geometry with a colour fading to
// the tip.
//
// The roots lie on the surface of the head piece, spread by area over the hairy part of the scalp (see HeadRegions.h) and thinner
// on a head with a lower hair density. A lock then grows step by step: its direction turns towards where the style wants it
// (outwards from the crown, then down under gravity, or towards the tie of a ponytail), and a lock that would sink into the head, or
// into the neck and shoulders, is pushed out along the surface. So long hair drapes over the head and the back instead of through
// them. Everything is chosen from the seed alone.
//
// Eyebrows are short locks on the brow ridge, laid along the arch of the brow. A beard grows on the jaw and the cheeks; stubble is
// a short one, and the skin under it is painted dark (see Skin.h).

namespace character {

struct HairSettings {
    HairStyle style = HairStyle::Short;
    FacialHair facialHair = FacialHair::None;
    float detail = 1.0f;  // the share of the usual number of cards, 0.25 to 1: fewer for a figure that is seen from far away
};

// The texture of a card of hair: strands side by side across it (u, repeating, so a card may start anywhere), each narrowing to a
// tip along it (v) and ending at its own length, so that the tip of a card is ragged. Alpha is the strand, so the card is drawn with an
// alpha test; the colour is a grey that lights and darkens strand by strand, to be multiplied by the colour of the hair; the normal
// map rounds each strand. Made from the seed alone, for exporting as an image.
struct HairTextures {
    ByteImage strands;  // RGBA: grey shade of the strand, alpha 255 on a strand and 0 between
    ByteImage normal;   // RGBA, tangent space
};

HairTextures MakeHairTextures(uint32_t seed, int width = 128, int height = 512);

struct HairSet {
    RibbonSet scalp;     // the hair of the head, as cards with the strand texture; empty for a bald one
    RibbonSet eyebrows;  // both, as thin plain ribbons
    RibbonSet facial;    // beard (cards) or stubble (thin plain ribbons)
    HairTextures textures;
};

// Builds the hair for a character made with a head piece. A character made without one (see MeshSettings::faceAndHands) gets an
// empty set. The hair lies outside the garments of `outfit` (a beard over the collar of a jacket), when there is one. `jobs` may be null;
// the result is the same either way.
HairSet BuildHair(const CharacterMesh& character, const PersonTraits& traits, const HairSettings& settings, const Outfit* outfit = nullptr, JobSystem* jobs = nullptr);

}  // namespace character
