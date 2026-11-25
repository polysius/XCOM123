#pragma once

#include "character/BodyShape.h"
#include "character/CharacterParameters.h"
#include "character/Skeleton.h"

// The head and the hands as shapes of their own. The body is cut from a grid of 2.4 cm, which has no room for a nose or a
// finger, so these two are cut from finer grids (5 mm) and fastened to the bones of the head and the hands, and
// the body is built without them (see BodyShapeOptions). Both are made in the rest pose of the skeleton, at the size
// and with the sex, age and variation of the parameters.

namespace character {

// The skull, face, jaw, ears and the carved hollows for the eyes. It stops at the jaw: the neck of the body runs up into it.
BodyShape BuildHeadShape(const CharacterParameters& parameters);

// A hand at rest: the palm, four fingers a little apart and curled, and the thumb, from a cuff that overlaps the end of the
// forearm to the fingertips. The cuff is as wide as the forearm of a body of this `girth`.
BodyShape BuildHandShape(const Skeleton& skeleton, const CharacterParameters& parameters, bool left, float girth);

}  // namespace character
