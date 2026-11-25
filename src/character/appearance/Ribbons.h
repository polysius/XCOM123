#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "character/Skeleton.h"
#include "character/appearance/AppearanceTypes.h"

// Hair and lashes are ribbons: a flat strip that follows a curve, narrowing towards the tip. A ribbon stands for a lock of
// hair (a few millimetres wide) or one lash; its two faces are the same, so a renderer draws strips with culling off. This
// is the usual "hair card" form of a game, without the alpha texture: here the shape of the strip is the shape of the lock.
//
// u runs across the strip (0 on one edge, 1 on the other) and v along it (0 at the root, 1 at the tip), which is what a
// strand texture or a fade of the colour towards the tip needs.

namespace character {

struct RibbonSet {
    RenderMesh mesh;
    std::vector<glm::vec3> colours;        // one for each vertex, linear
    std::vector<glm::vec3> roots;          // where each ribbon starts
    std::vector<uint32_t> firstVertex;     // the first vertex of each ribbon
    size_t RibbonCount() const { return roots.size(); }
};

// Adds a ribbon along `points` (at least two). The strip lies across `faceDirection` (the way the surface of the head faces
// there), `rootWidth` wide at the first point and `tipWidth` at the last, and the colour fades from `rootColour` to
// `tipColour`. Every vertex follows `bone` with full weight.
void AddRibbon(RibbonSet& set, const std::vector<glm::vec3>& points, glm::vec3 faceDirection, float rootWidth, float tipWidth, glm::vec3 rootColour, glm::vec3 tipColour,
               Joint bone);

}  // namespace character
