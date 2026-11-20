#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "character/CharacterParameters.h"
#include "character/Skeleton.h"

// The surface of a body as an implicit function: a negative number inside the body, a positive one outside, and about
// zero on the skin. The body is built from parts (tapered capsules and ellipsoids, each owned by a bone) that are
// merged with smooth unions where they grow into one another (a limb into its joint and into the trunk) and with plain
// unions where they must stay apart: the two legs are only ever joined through the body, never to each other.

namespace character {

// One lump of the body: a capsule from `start` to `end` whose cross section is an ellipse that changes from one end to
// the other, or an ellipsoid when the two ends are the same point. The cross section has `side` and `depth` axes.
struct ShapePart {
    glm::vec3 start{0.0f};
    glm::vec3 end{0.0f};
    float sideStart = 0.05f;   // radii of the cross section at each end, along the side axis and the depth axis
    float depthStart = 0.05f;
    float sideEnd = 0.05f;
    float depthEnd = 0.05f;
    float alongRadius = 0.0f;  // the radius of the rounded ends along the axis; 0 means the mean of the other two
    glm::vec3 side{1.0f, 0.0f, 0.0f};
    glm::vec3 depth{0.0f, 0.0f, 1.0f};
    Joint bone = Joint::Pelvis;
};

class BodyShape {
public:
    // The signed distance (approximately) from the surface at a point of the rest pose.
    float Distance(glm::vec3 point) const;

    // The same for one part on its own, which the skin weights use.
    float PartDistance(size_t part, glm::vec3 point) const;

    size_t PartCount() const { return parts.size(); }
    Joint PartBone(size_t part) const { return parts[part].bone; }
    glm::vec3 BoundsMin() const { return boundsMin; }
    glm::vec3 BoundsMax() const { return boundsMax; }

private:
    friend BodyShape BuildBodyShape(const Skeleton&, const CharacterParameters&, float, float);

    // A node of the tree that combines the parts: a part, or two nodes joined with smoothing of size k (0 for none).
    struct Node {
        int part = -1;  // the part, for a leaf
        int first = -1;
        int second = -1;
        float smoothing = 0.0f;
        glm::vec3 lower{0.0f};  // a box that holds every point where the node is closer than a margin to the surface
        glm::vec3 upper{0.0f};
    };

    float EvaluateNode(int node, glm::vec3 point) const;

    std::vector<ShapePart> parts;
    std::vector<Node> nodes;
    int root = -1;
    glm::vec3 boundsMin{0.0f};
    glm::vec3 boundsMax{0.0f};
};

// The body of a person: `girth` scales every radius of the trunk and limbs (1 is a body of average build for the
// parameters), and the generator solves it so that the mass comes out right. No radius is made smaller than
// `minimumRadius`: the surface is cut from a grid, and a part thinner than a cell would break up.
BodyShape BuildBodyShape(const Skeleton& skeleton, const CharacterParameters& parameters, float girth, float minimumRadius = 0.0f);

}  // namespace character
