#include "character/appearance/HeadRegions.h"

#include <algorithm>
#include <cmath>

#include "character/appearance/Noise3D.h"

namespace character {
namespace {

// The regions are written for a head of 1.75 m stature, in metres; a point is divided by the scale of the head first.
glm::vec3 InHeadUnits(const HeadLandmarks& landmarks, glm::vec3 point) {
    return point / landmarks.scale;
}

float DistanceToNearestEar(const HeadLandmarks& landmarks, glm::vec3 q) {
    float nearest = 1e9f;
    for (const glm::vec3& ear : landmarks.ears) {
        nearest = std::min(nearest, glm::distance(q, ear / landmarks.scale));
    }
    return nearest;
}

}  // namespace

float ScalpWeight(const HeadLandmarks& landmarks, const PersonTraits& traits, glm::vec3 point) {
    const glm::vec3 q = InHeadUnits(landmarks, point);
    // The hairline climbs from the nape (1.60) over the ears to the forehead (1.705).
    float hairline = 1.600f + 0.105f * SmoothStep(-0.060f, 0.080f, q.z);
    // At the temples, and a little at the front, a receding hairline moves up and back.
    hairline += traits.hairlineRecession * (0.032f * SmoothStep(0.020f, 0.070f, q.z) * SmoothStep(0.015f, 0.045f, std::abs(q.x)) + 0.010f * SmoothStep(0.05f, 0.08f, q.z));
    const float aboveHairline = SmoothStep(hairline - 0.004f, hairline + 0.004f, q.y);
    const float clearOfEars = SmoothStep(0.026f, 0.036f, DistanceToNearestEar(landmarks, q));
    return aboveHairline * clearOfEars;
}

float BeardWeight(const HeadLandmarks& landmarks, glm::vec3 point) {
    const glm::vec3 q = InHeadUnits(landmarks, point);
    const float x = std::abs(q.x);
    // Below, the underside of the chin; above, a line that is low at the moustache, higher on the cheek and runs up the sideburn.
    const float lower = SmoothStep(1.516f, 1.530f, q.y);
    const float upper = 1.590f + 0.022f * SmoothStep(0.025f, 0.055f, x) + 0.045f * SmoothStep(0.062f, 0.076f, x);
    const float belowUpper = 1.0f - SmoothStep(upper - 0.004f, upper + 0.004f, q.y);
    const float inFront = SmoothStep(-0.012f, 0.012f, q.z);
    const float clearOfEars = SmoothStep(0.028f, 0.040f, DistanceToNearestEar(landmarks, q));
    // The lips are bare.
    const float lip = LipWeight(landmarks, point);
    return lower * belowUpper * inFront * clearOfEars * (1.0f - lip);
}

float LipWeight(const HeadLandmarks& landmarks, glm::vec3 point) {
    const glm::vec3 q = InHeadUnits(landmarks, point);
    const float across = q.x / 0.029f;
    const float up = (q.y - 1.5655f) / 0.0165f;
    const float inside = 1.0f - SmoothStep(0.80f, 1.05f, across * across + up * up);
    return inside * SmoothStep(0.060f, 0.075f, q.z);
}

}  // namespace character
