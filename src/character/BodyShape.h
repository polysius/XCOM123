#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "character/CharacterParameters.h"
#include "character/Skeleton.h"
#include "character/SurfaceExtraction.h"

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

class BodyShape final : public ImplicitShape {
public:
    // The signed distance (approximately) from the surface at a point of the rest pose.
    float Distance(glm::vec3 point) const override;

    // The same for one part on its own, which the skin weights use.
    float PartDistance(size_t part, glm::vec3 point) const;

    size_t PartCount() const { return parts.size(); }
    Joint PartBone(size_t part) const { return parts[part].bone; }
    glm::vec3 BoundsMin() const override { return boundsMin; }
    glm::vec3 BoundsMax() const override { return boundsMax; }

private:
    friend class ShapeAssembler;

    // A node of the tree that combines the parts: a part, or two nodes joined with smoothing of size k (0 for none), or
    // a node with another cut out of it.
    struct Node {
        int part = -1;  // the part, for a leaf
        int first = -1;
        int second = -1;
        bool carve = false;  // the second node is cut out of the first, instead of being united with it
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
    float cullDistance = 0.10f;
};

// Builds a shape from parts: add the parts, combine them into a tree of unions and cuts, and take the finished shape.
// The body, the head and the hands are all made with it.
class ShapeAssembler {
public:
    // Adds a tapered capsule or an ellipsoid and returns its number. `depthDirection` is where the shallow axis of a
    // capsule's cross section points. An ellipsoid has radii along X, Y and Z.
    int Capsule(Joint bone, glm::vec3 start, glm::vec3 end, float sideStart, float depthStart, float sideEnd, float depthEnd,
                glm::vec3 depthDirection = glm::vec3(0.0f, 0.0f, 1.0f));
    int Ellipsoid(Joint bone, glm::vec3 centre, float radiusX, float radiusY, float radiusZ);

    // The nodes of the tree. A leaf is a part. A join unites two nodes, with the crease between them rounded off over
    // `smoothing` metres (0 for a plain union). A carve cuts the second node out of the first, with the edge rounded
    // over `smoothing`. A merge joins a list of parts one after another.
    int Leaf(int part);
    int Join(int first, int second, float smoothing);
    int Carve(int from, int cut, float smoothing);
    int Merge(const std::vector<int>& parts, float smoothing);

    // The finished shape, with `root` as the node that is the whole of it. Its box holds every part and `margin` more,
    // for what smoothing adds outside the parts. Far from a node's own box the shape is not evaluated; `cullDistance` is how
    // far that is, and it must be wider than any smoothing (the body needs 10 cm, the small shapes much less).
    BodyShape Finish(int root, float margin, float cullDistance = 0.10f);

private:
    BodyShape shape;
};

// What a body is built with. `girth` scales every radius of the trunk and limbs (1 is a body of average build for the
// parameters), and the generator solves it so that the mass comes out right. No radius is made smaller than
// `minimumRadius`: the surface is cut from a grid, and a part thinner than a cell would break up.
struct BodyShapeOptions {
    float girth = 1.0f;
    float minimumRadius = 0.0f;
    // The head and the hands are made as finer shapes of their own (see DetailShapes.h) when the body is cut without
    // them: the neck then runs up into the head, and each forearm ends at the wrist.
    bool includeHead = true;
    bool includeHands = true;
};

BodyShape BuildBodyShape(const Skeleton& skeleton, const CharacterParameters& parameters, const BodyShapeOptions& options = {});

}  // namespace character
