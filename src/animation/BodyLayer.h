#pragma once

#include <glm/glm.hpp>

#include "animation/Kinematics.h"

// A layer of motion that is added on top of a base motion before the legs are solved: breathing and weight shifts, the
// sway of a push. The rotations are small turns that are applied after the base rotation of each joint (so an unset
// joint is the identity). The pelvis rotation is a turn of the whole body in its own frame, and `pelvisOffset` moves the
// pelvis in the frame of the body (X to the left, Y up, Z forwards). The legs of a layer are ignored: the feet stay where the gait planted them, and the legs follow the pelvis.

namespace animation {

struct BodyLayer {
    Pose additive;
    glm::vec3 pelvisOffset{0.0f};
};

// A layer scaled to `weight` (0 gives no motion, 1 the whole layer): rotations are interpolated from the identity.
BodyLayer ScaledLayer(const BodyLayer& layer, float weight);

// Two layers added together: rotations are composed (`first` then `second`) and offsets summed.
BodyLayer CombinedLayers(const BodyLayer& first, const BodyLayer& second);

}  // namespace animation
