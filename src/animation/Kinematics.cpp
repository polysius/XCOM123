#include "animation/Kinematics.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/matrix_transform.hpp>

namespace animation {

WorldPose ForwardKinematics(const Skeleton& skeleton, const Pose& pose) {
    WorldPose world;
    for (int i = 0; i < kJointCount; ++i) {
        const character::JointInfo& joint = skeleton.joints[i];
        if (joint.parent < 0) {
            world.position[i] = pose.rootTranslation + joint.restOffset;
            world.rotation[i] = pose.rotation[i];
        } else {
            world.position[i] = world.position[joint.parent] + world.rotation[joint.parent] * joint.restOffset;
            world.rotation[i] = world.rotation[joint.parent] * pose.rotation[i];
        }
    }
    return world;
}

JointMatrices WorldMatrices(const Skeleton& skeleton, const Pose& pose) {
    const WorldPose world = ForwardKinematics(skeleton, pose);
    JointMatrices matrices;
    for (int i = 0; i < kJointCount; ++i) {
        glm::mat4 matrix = glm::mat4_cast(world.rotation[i]);
        matrix[3] = glm::vec4(world.position[i], 1.0f);
        matrices[i] = matrix;
    }
    return matrices;
}

JointMatrices SkinningMatrices(const Skeleton& skeleton, const Pose& pose) {
    return character::SkinMatrices(skeleton, pose);
}

glm::quat ParentWorldRotation(const Skeleton& skeleton, const WorldPose& world, Joint joint) {
    const int parent = skeleton[joint].parent;
    return parent < 0 ? glm::quat(1.0f, 0.0f, 0.0f, 0.0f) : world.rotation[parent];
}

void SetWorldRotation(const Skeleton& skeleton, Pose& pose, Joint joint, const glm::quat& worldRotation) {
    const WorldPose world = ForwardKinematics(skeleton, pose);
    const glm::quat parent = ParentWorldRotation(skeleton, world, joint);
    pose.rotation[character::Index(joint)] = glm::normalize(glm::inverse(parent) * worldRotation);
}

void SetPelvisPosition(const Skeleton& skeleton, Pose& pose, const glm::vec3& worldPosition) {
    pose.rootTranslation = worldPosition - skeleton[Joint::Pelvis].restOffset;
}

glm::quat RotationBetween(const glm::vec3& from, const glm::vec3& to) {
    const glm::vec3 a = glm::normalize(from);
    const glm::vec3 b = glm::normalize(to);
    const float cosine = glm::dot(a, b);
    if (cosine > 0.999999f) {
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }
    if (cosine < -0.999999f) {
        // Opposite directions: any axis perpendicular to `a` turns it round.
        glm::vec3 axis = glm::cross(a, glm::vec3(1.0f, 0.0f, 0.0f));
        if (glm::dot(axis, axis) < 1e-6f) {
            axis = glm::cross(a, glm::vec3(0.0f, 1.0f, 0.0f));
        }
        return glm::angleAxis(3.14159265f, glm::normalize(axis));
    }
    const glm::vec3 axis = glm::normalize(glm::cross(a, b));
    return glm::angleAxis(std::acos(std::clamp(cosine, -1.0f, 1.0f)), axis);
}

glm::quat RotationBetweenFrames(const glm::vec3& fromFirst, const glm::vec3& fromSecond, const glm::vec3& toFirst, const glm::vec3& toSecond) {
    const glm::mat3 from(fromFirst, fromSecond, glm::cross(fromFirst, fromSecond));
    const glm::mat3 to(toFirst, toSecond, glm::cross(toFirst, toSecond));
    return glm::normalize(glm::quat_cast(to * glm::transpose(from)));
}

glm::quat YawRotation(float radians) {
    return glm::angleAxis(radians, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::quat FootOrientation(float yaw, float pitch) {
    return YawRotation(yaw) * glm::angleAxis(-pitch, glm::vec3(1.0f, 0.0f, 0.0f));
}

float RotationAngleDegrees(const glm::quat& rotation) {
    const float w = std::clamp(std::abs(glm::normalize(rotation).w), 0.0f, 1.0f);
    return 2.0f * std::acos(w) * kRadiansToDegrees;
}

}  // namespace animation
