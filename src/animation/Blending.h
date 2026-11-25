#pragma once

#include <span>
#include <vector>

#include "animation/Kinematics.h"

// Combining poses: a blend of two, a weighted blend of many, additive layers, a blend tree that does both, and a cross fade
// between two states over time.
//
// The rotations of a pose are measured from the rest pose, so an additive pose is a pose of small rotations (the identity
// is no change), and applying it turns each joint a little further, after the rotation it already has.

namespace animation {

// From pose `a` at t = 0 to pose `b` at t = 1: each rotation by the shortest way round, the root translations by a straight line.
Pose BlendPoses(const Pose& a, const Pose& b, float t);

struct WeightedPose {
    const Pose* pose = nullptr;
    float weight = 0.0f;
};

// The blend of poses with weights. The weights are scaled to add up to 1, so only their ratios matter; negative weights count
// as 0, and if they are all 0 the result is the first pose. Each rotation is the normalised weighted sum of the rotations, with
// the signs of the quaternions matched to the first pose, which is exact for two poses and good for poses that are near each other.
Pose BlendWeighted(std::span<const WeightedPose> poses);

// A pose of the changes `pose` makes to `reference`: applying it to the reference gives `pose` back.
Pose MakeAdditive(const Pose& pose, const Pose& reference);

// `base` with `additive` applied at `weight` (0 is no change, 1 the whole of it).
Pose ApplyAdditive(const Pose& base, const Pose& additive, float weight);

// A blend tree of one level: poses that are blended by weight, with additive layers laid on top.
class BlendTree {
public:
    void AddPose(const Pose& pose, float weight);
    void AddAdditive(const Pose& additive, float weight);
    void Clear();

    // The blend of the poses, then the additive layers in the order they were added. With no pose, the rest pose.
    Pose Evaluate() const;

private:
    std::vector<Pose> poses;
    std::vector<float> weights;
    std::vector<Pose> additives;
    std::vector<float> additiveWeights;
};

// A cross fade from one state to another over a time. The caller runs both states and asks for the blend of their poses, so
// both keep moving through the fade.
class CrossFade {
public:
    // Starts a fade of `durationSeconds`; zero or less makes it instant.
    void Begin(float durationSeconds);
    void Advance(float dt);

    bool Active() const { return elapsed < duration; }
    // 0 at the start and 1 at the end, easing in and out so that the fade has no sudden start or stop.
    float Weight() const;
    Pose Blend(const Pose& from, const Pose& to) const { return BlendPoses(from, to, Weight()); }

private:
    float duration = 0.0f;
    float elapsed = 0.0f;
};

}  // namespace animation
