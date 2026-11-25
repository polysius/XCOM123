#pragma once

#include "character/CharacterGenerator.h"
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
};

struct HairSet {
    RibbonSet scalp;     // the hair of the head; empty for a bald one
    RibbonSet eyebrows;  // both
    RibbonSet facial;    // beard or stubble
};

// Builds the hair for a character made with a head piece. A character made without one (see MeshSettings::faceAndHands) gets an
// empty set. `jobs` may be null; the result is the same either way.
HairSet BuildHair(const CharacterMesh& character, const PersonTraits& traits, const HairSettings& settings, JobSystem* jobs = nullptr);

}  // namespace character
