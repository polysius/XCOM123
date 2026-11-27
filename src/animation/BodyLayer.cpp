#include "animation/BodyLayer.h"

namespace animation {

BodyLayer ScaledLayer(const BodyLayer& layer, float weight) {
    const glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    BodyLayer scaled;
    for (int i = 0; i < kJointCount; ++i) {
        scaled.additive.rotation[i] = glm::slerp(identity, layer.additive.rotation[i], weight);
    }
    scaled.additive.rootTranslation = layer.additive.rootTranslation * weight;
    scaled.pelvisOffset = layer.pelvisOffset * weight;
    return scaled;
}

BodyLayer CombinedLayers(const BodyLayer& first, const BodyLayer& second) {
    BodyLayer combined;
    for (int i = 0; i < kJointCount; ++i) {
        combined.additive.rotation[i] = glm::normalize(first.additive.rotation[i] * second.additive.rotation[i]);
    }
    combined.additive.rootTranslation = first.additive.rootTranslation + second.additive.rootTranslation;
    combined.pelvisOffset = first.pelvisOffset + second.pelvisOffset;
    return combined;
}

}  // namespace animation
