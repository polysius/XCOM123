#include "animation/BodyModel.h"

#include <cmath>

namespace animation {

BodyModel MakeBodyModel(const character::CharacterParameters& parameters) {
    using character::Joint;
    BodyModel body;
    body.parameters = character::Sanitised(parameters);
    body.skeleton = character::BuildSkeleton(body.parameters);
    const character::Skeleton& skeleton = body.skeleton;

    body.legLength = skeleton[Joint::ThighL].restPosition.y;
    body.thighLength = glm::length(skeleton[Joint::CalfL].restPosition - skeleton[Joint::ThighL].restPosition);
    body.shankLength = glm::length(skeleton[Joint::FootL].restPosition - skeleton[Joint::CalfL].restPosition);
    body.ankleHeight = skeleton[Joint::FootL].restPosition.y;
    body.footHalfWidth = std::abs(skeleton[Joint::FootL].restPosition.x);

    const glm::vec3 ankle = skeleton[Joint::FootL].restPosition;
    body.heelOffset = skeleton.leftFoot.heel - ankle;
    body.ballOffset = skeleton.leftFoot.ball - ankle;
    body.toeOffset = skeleton.leftFoot.toe - ankle;
    body.ballFromHeel = body.ballOffset.z - body.heelOffset.z;
    body.hipOffsetLeft = skeleton[Joint::ThighL].restOffset;
    body.hipOffsetRight = skeleton[Joint::ThighR].restOffset;
    return body;
}

}  // namespace animation
