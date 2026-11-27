#pragma once

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

    // Takes up to `limit` people within `reach` metres of `camera` out of `people`, nearest first, and puts them into `frame` as
    // posed characters. What is put in `frame` stays valid until the next call.
    void TakeNearest(std::vector<NpcInstance>& people, glm::vec3 camera, uint32_t limit, float reach, FrameInput& frame);

private:
    struct Body {
        character::Skeleton skeleton;
        CharacterHandle handle;
    };

    static character::Pose WalkPose(float phase, float stride, float variation);
    // Moves the `count` people chosen to the front of `people`, nearest first, without keeping the order of the rest.
    static size_t ChooseNearest(std::vector<NpcInstance>& people, glm::vec3 camera, uint32_t limit, float reach);

    std::vector<Body> bodies;
    std::vector<glm::mat4> palettes;  // every character's joint matrices, one palette after the other
    std::vector<SkinnedInstance> instances;
};

}  // namespace demo
