#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include <glm/glm.hpp>

#include "character/Skeleton.h"
#include "core/CommandLine.h"
#include "render/IRenderModule.h"
#include "scene/SceneTypes.h"
#include "world/CityGenerator.h"

namespace demo {

// Says how a character stands at a moment. Whatever animates the characters is plugged in here: it gets the skeleton (for the
// joint axes and limits), the time in seconds and the index of the character, and returns the pose. It must give the same pose
// for the same arguments, so that screenshots repeat.
using PoseProvider = std::function<character::Pose(const character::Skeleton&, float seconds, uint32_t characterIndex)>;

// A standing figure that sways, breathes, nods and bends an elbow, made from the rest pose and a few joint rotations. It is only
// here to show that skinning works until real animation is plugged in.
character::Pose IdleSwayPose(const character::Skeleton& skeleton, float seconds, uint32_t characterIndex);

// A few skinned characters standing in a grid in the largest square of the city, posed every frame by a PoseProvider.
//
// Making a body takes about a tenth of a second, so only a few distinct bodies are made and the characters share them; each
// character still has its own palette, so each can be in its own pose.
class CharacterShowcase {
public:
    // Makes the bodies, uploads them to the renderer and places `count` characters in the square.
    void Spawn(IRenderModule& renderer, const CityLayout& city, uint32_t count, uint32_t seed);

    void SetPoseProvider(PoseProvider provider) { poseProvider = std::move(provider); }

    // Poses every character for this time and puts them into the frame. What is put there stays valid until the next call.
    void Update(float worldSeconds, FrameInput& frame);

    uint32_t Count() const { return static_cast<uint32_t>(placements.size()); }
    // A camera that looks at the whole grid from the front.
    const std::optional<CameraPlacement>& ViewPoint() const { return viewPoint; }

private:
    struct Body {
        character::Skeleton skeleton;
        CharacterHandle handle;
        uint32_t jointCount = 0;
    };
    struct Placement {
        uint32_t body = 0;
        glm::mat4 world{1.0f};
        glm::vec3 tint{1.0f};
    };

    std::vector<Body> bodies;
    std::vector<Placement> placements;
    PoseProvider poseProvider = IdleSwayPose;
    std::vector<glm::mat4> palettes;      // joint matrices of every character, one palette after the other
    std::vector<SkinnedInstance> instances;
    std::optional<CameraPlacement> viewPoint;
};

}  // namespace demo
