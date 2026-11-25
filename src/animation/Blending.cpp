#include "animation/Blending.h"

#include <algorithm>
#include <cmath>

namespace animation {
namespace {

const glm::quat kIdentity(1.0f, 0.0f, 0.0f, 0.0f);

}  // namespace

Pose BlendPoses(const Pose& a, const Pose& b, float t) {
    const float blend = std::clamp(t, 0.0f, 1.0f);
    Pose result;
    for (int i = 0; i < kJointCount; ++i) {
        result.rotation[i] = glm::normalize(glm::slerp(a.rotation[i], b.rotation[i], blend));
    }
    result.rootTranslation = glm::mix(a.rootTranslation, b.rootTranslation, blend);
    return result;
}

Pose BlendWeighted(std::span<const WeightedPose> poses) {
    if (poses.empty()) {
        return Pose();
    }
    float total = 0.0f;
    for (const WeightedPose& entry : poses) {
        total += std::max(entry.weight, 0.0f);
    }
    if (total <= 0.0f) {
        return *poses.front().pose;
    }
    Pose result;
    result.rootTranslation = glm::vec3(0.0f);
    for (int i = 0; i < kJointCount; ++i) {
        const glm::quat& reference = poses.front().pose->rotation[i];
        glm::vec4 sum(0.0f);
        for (const WeightedPose& entry : poses) {
            glm::quat q = entry.pose->rotation[i];
            if (glm::dot(q, reference) < 0.0f) {
                q = -q;
            }
            sum += std::max(entry.weight, 0.0f) / total * glm::vec4(q.x, q.y, q.z, q.w);
        }
        result.rotation[i] = glm::normalize(glm::quat(sum.w, sum.x, sum.y, sum.z));
    }
    for (const WeightedPose& entry : poses) {
        result.rootTranslation += std::max(entry.weight, 0.0f) / total * entry.pose->rootTranslation;
    }
    return result;
}

Pose MakeAdditive(const Pose& pose, const Pose& reference) {
    Pose additive;
    for (int i = 0; i < kJointCount; ++i) {
        additive.rotation[i] = glm::normalize(glm::inverse(reference.rotation[i]) * pose.rotation[i]);
    }
    additive.rootTranslation = pose.rootTranslation - reference.rootTranslation;
    return additive;
}

Pose ApplyAdditive(const Pose& base, const Pose& additive, float weight) {
    Pose result;
    for (int i = 0; i < kJointCount; ++i) {
        result.rotation[i] = glm::normalize(base.rotation[i] * glm::slerp(kIdentity, additive.rotation[i], weight));
    }
    result.rootTranslation = base.rootTranslation + additive.rootTranslation * weight;
    return result;
}

void BlendTree::AddPose(const Pose& pose, float weight) {
    poses.push_back(pose);
    weights.push_back(weight);
}

void BlendTree::AddAdditive(const Pose& additive, float weight) {
    additives.push_back(additive);
    additiveWeights.push_back(weight);
}

void BlendTree::Clear() {
    poses.clear();
    weights.clear();
    additives.clear();
    additiveWeights.clear();
}

Pose BlendTree::Evaluate() const {
    std::vector<WeightedPose> entries;
    for (size_t i = 0; i < poses.size(); ++i) {
        entries.push_back({&poses[i], weights[i]});
    }
    Pose result = BlendWeighted(entries);
    for (size_t i = 0; i < additives.size(); ++i) {
        result = ApplyAdditive(result, additives[i], additiveWeights[i]);
    }
    return result;
}

void CrossFade::Begin(float durationSeconds) {
    duration = std::max(durationSeconds, 0.0f);
    elapsed = 0.0f;
}

void CrossFade::Advance(float dt) {
    elapsed = std::min(elapsed + std::max(dt, 0.0f), duration);
}

float CrossFade::Weight() const {
    if (duration <= 0.0f) {
        return 1.0f;
    }
    const float t = std::clamp(elapsed / duration, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

}  // namespace animation
