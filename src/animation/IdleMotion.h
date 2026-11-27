#pragma once

#include <cstdint>

#include "animation/BodyLayer.h"

// What a person does when they stand: breathing, a slow shift of weight from one leg to the other, and small movements of
// the head. All of it is smooth seeded noise and sine waves of the time, so the same seed always gives the same person
// the same fidgets, and the layer at a given time does not depend on how it was reached.

namespace animation {

class IdleMotion {
public:
    explicit IdleMotion(uint32_t seed = 1);

    // The layer at a time in seconds. The feet stay planted: the pelvis shifts and tilts and the legs follow.
    BodyLayer Evaluate(double timeSeconds) const;

private:
    uint32_t seed;
    float breathsPerSecond;
};

}  // namespace animation
