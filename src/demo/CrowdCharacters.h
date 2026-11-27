#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "character/Skeleton.h"
#include "render/IRenderModule.h"
#include "scene/SceneTypes.h"

namespace demo {

// The people nearest to the camera, drawn as skinned, walking characters instead of the plain figures the rest of the crowd is
// drawn with. A person keeps walking where the crowd walks them; only the way they are drawn changes, and the renderer chooses
// a coarser mesh for each character as it gets further from the camera.
//
// Making a body takes about a tenth of a second, so only a few distinct bodies are made, and everyone uses one of them. Each
// person has a palette of their own, posed from their walk cycle (the phase and stride the crowd keeps for them).
class CrowdCharacters {
public:
    // Makes the bodies, with their coarser versions, and uploads them. Takes about a second.
    void MakeBodies(IRenderModule& renderer, uint32_t seed);

    bool IsReady() const { return !bodies.empty(); }

    // Cost of the last TakeNearest, for finding out what the characters cost a frame.
    double LastTakeMilliseconds() const { return lastTakeMilliseconds; }
    size_t LastCharacterCount() const { return instances.size(); }
    size_t LastPosedAlone() const { return lastPosedAlone; }

    // Takes up to `limit` people within `reach` metres of `camera` out of `people`, nearest first, and puts them into `frame` as
    // posed characters. What is put in `frame` stays valid until the next call.
    void TakeNearest(std::vector<NpcInstance>& people, glm::vec3 camera, uint32_t limit, float reach, FrameInput& frame);

private:
    struct Body {
        character::Skeleton skeleton;
        CharacterHandle handle;
    };

    static character::Pose WalkPose(const character::Skeleton& skeleton, float phase, float stride, float variation);
    // Fills the palette of a person: posed for them alone when they are near, and from a small set of shared poses beyond.
    void AppendPalette(const NpcInstance& person, float distance, size_t bodyIndex);
    // Moves the `count` people chosen to the front of `people`, nearest first, without keeping the order of the rest.
    static size_t ChooseNearest(std::vector<NpcInstance>& people, glm::vec3 camera, uint32_t limit, float reach);

    std::vector<Body> bodies;
    // Poses shared by the people further than kPosedAloneReach: for each body, a few strides and phases of the walk, made the first
    // time they are asked for and kept. Each is one palette of joint matrices.
    std::vector<std::array<glm::mat4, character::kJointCount>> sharedPoses;
    std::vector<uint8_t> sharedPoseMade;
    double lastTakeMilliseconds = 0.0;
    size_t lastPosedAlone = 0;
    std::vector<glm::mat4> palettes;  // every character's joint matrices, one palette after the other
    std::vector<SkinnedInstance> instances;
};

}  // namespace demo
