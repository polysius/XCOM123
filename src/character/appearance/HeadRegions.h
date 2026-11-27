#pragma once

#include <glm/glm.hpp>

#include "character/DetailShapes.h"
#include "character/appearance/Traits.h"

// Where on the head hair grows and where the lips are, as weights from 0 to 1 at a point of the head in the rest pose. The
// skin painter (to colour the scalp and the beard) and the hair builder (to place roots) use the same regions, so the
// hair always grows from skin that is painted to match.

namespace character {

// The hairy part of the scalp: above a hairline that runs from the forehead back over the temples to the nape, and clear of the
// ears. The hairline withdraws from the temples as `traits.hairlineRecession` grows.
float ScalpWeight(const HeadLandmarks& landmarks, const PersonTraits& traits, glm::vec3 point);

// Where a beard grows: the jaw, the chin, the cheeks, the sideburns and above the upper lip, but not on the lips.
float BeardWeight(const HeadLandmarks& landmarks, glm::vec3 point);

// The lips.
float LipWeight(const HeadLandmarks& landmarks, glm::vec3 point);

}  // namespace character
