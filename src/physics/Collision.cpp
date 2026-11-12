#include "physics/Collision.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace physics {
namespace {

// A box in the world: its centre, its three unit axes (the columns of `axes`) and its half sizes.
struct Obb {
    glm::vec3 center;
    glm::mat3 axes;
    glm::vec3 half;
};

Obb ObbOf(const Body& body) {
    return {body.position, AxesOf(body), body.shape.halfExtents};
}

float SignOf(float value) {
    return value < 0.0f ? -1.0f : 1.0f;
}

// Spheres -------------------------------------------------------------------------------------

bool SphereSphere(const Body& a, const Body& b, Manifold& manifold) {
    const glm::vec3 between = b.position - a.position;
    const float distanceSquared = glm::dot(between, between);
    const float radii = a.shape.radius + b.shape.radius;
    const float reach = radii + kContactMargin;
    if (distanceSquared > reach * reach) {
        return false;
    }
    const float distance = std::sqrt(distanceSquared);
    const glm::vec3 normal = distance > 1e-6f ? between / distance : glm::vec3(0.0f, 1.0f, 0.0f);
    manifold.normal = normal;
    manifold.count = 1;
    // Halfway between the two surfaces along the line through the centres.
    manifold.points[0].position = a.position + normal * (0.5f * (a.shape.radius + distance - b.shape.radius));
    manifold.points[0].depth = radii - distance;
    return true;
}

// The contact between a sphere and a box. The normal points from the sphere to the box.
bool SphereBox(const Body& sphere, const Body& box, Manifold& manifold) {
    const Obb obb = ObbOf(box);
    const float radius = sphere.shape.radius;
    const glm::vec3 local = glm::transpose(obb.axes) * (sphere.position - obb.center);
    const glm::vec3 closest = glm::clamp(local, -obb.half, obb.half);
    const glm::vec3 delta = local - closest;  // from the closest point of the box to the sphere's centre
    const float distanceSquared = glm::dot(delta, delta);

    if (distanceSquared > 1e-12f) {
        const float reach = radius + kContactMargin;
        if (distanceSquared > reach * reach) {
            return false;
        }
        const float distance = std::sqrt(distanceSquared);
        manifold.normal = -(obb.axes * (delta / distance));
        manifold.points[0].position = obb.center + obb.axes * closest;
        manifold.points[0].depth = radius - distance;
    } else {
        // The centre is inside the box: leave through the nearest face.
        const glm::vec3 gaps = obb.half - glm::abs(local);
        int axis = 0;
        for (int i = 1; i < 3; ++i) {
            if (gaps[i] < gaps[axis]) {
                axis = i;
            }
        }
        glm::vec3 outwardLocal(0.0f);
        outwardLocal[axis] = SignOf(local[axis]);
        const glm::vec3 outward = obb.axes * outwardLocal;
        manifold.normal = -outward;
        manifold.points[0].position = sphere.position + outward * gaps[axis];
        manifold.points[0].depth = radius + gaps[axis];
    }
    manifold.count = 1;
    return true;
}

// Boxes ---------------------------------------------------------------------------------------

// How far apart two boxes are along the unit axis `axis`: negative when their shadows on it overlap.
float SeparationAlong(const Obb& a, const Obb& b, glm::vec3 centreOffset, glm::vec3 axis) {
    float shadowA = 0.0f;
    float shadowB = 0.0f;
    for (int i = 0; i < 3; ++i) {
        shadowA += a.half[i] * std::abs(glm::dot(a.axes[i], axis));
        shadowB += b.half[i] * std::abs(glm::dot(b.axes[i], axis));
    }
    return std::abs(glm::dot(centreOffset, axis)) - shadowA - shadowB;
}

struct Candidate {
    glm::vec3 position;
    float depth;
};

// Cuts a convex polygon down to the part with dot(point - origin, normal) <= limit.
std::vector<glm::vec3> ClipPolygon(const std::vector<glm::vec3>& polygon, glm::vec3 origin, glm::vec3 normal, float limit) {
    std::vector<glm::vec3> clipped;
    clipped.reserve(polygon.size() + 1);
    for (size_t i = 0; i < polygon.size(); ++i) {
        const glm::vec3 current = polygon[i];
        const glm::vec3 next = polygon[(i + 1) % polygon.size()];
        const float currentSide = glm::dot(current - origin, normal) - limit;
        const float nextSide = glm::dot(next - origin, normal) - limit;
        if (currentSide <= 0.0f) {
            clipped.push_back(current);
        }
        if ((currentSide < 0.0f && nextSide > 0.0f) || (currentSide > 0.0f && nextSide < 0.0f)) {
            clipped.push_back(current + (next - current) * (currentSide / (currentSide - nextSide)));
        }
    }
    return clipped;
}

// Keeps four of the candidates that cover the most area: the deepest one, the one farthest from it,
// and the two that stretch the quadrilateral furthest to either side.
void ReduceToFour(std::vector<Candidate>& candidates, glm::vec3 normal) {
    if (candidates.size() <= static_cast<size_t>(kMaxContactsPerPair)) {
        return;
    }
    std::array<size_t, 4> chosen{};
    size_t deepest = 0;
    for (size_t i = 1; i < candidates.size(); ++i) {
        if (candidates[i].depth > candidates[deepest].depth) {
            deepest = i;
        }
    }
    chosen[0] = deepest;
    size_t farthest = deepest == 0 ? 1 : 0;
    float farthestDistance = -1.0f;
    for (size_t i = 0; i < candidates.size(); ++i) {
        const float distance = glm::distance(candidates[i].position, candidates[deepest].position);
        if (i != deepest && distance > farthestDistance) {
            farthestDistance = distance;
            farthest = i;
        }
    }
    chosen[1] = farthest;

    const glm::vec3 base = candidates[deepest].position;
    const glm::vec3 edge = candidates[farthest].position - base;
    const auto signedArea = [&](size_t index) { return glm::dot(glm::cross(edge, candidates[index].position - base), normal); };
    size_t widest = chosen[0];
    float widestArea = -1.0f;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (i != chosen[0] && i != chosen[1] && std::abs(signedArea(i)) > widestArea) {
            widestArea = std::abs(signedArea(i));
            widest = i;
        }
    }
    chosen[2] = widest;
    const float side = SignOf(signedArea(widest));
    size_t opposite = chosen[0];
    float oppositeArea = std::numeric_limits<float>::max();
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (i != chosen[0] && i != chosen[1] && i != chosen[2] && signedArea(i) * side < oppositeArea) {
            oppositeArea = signedArea(i) * side;
            opposite = i;
        }
    }
    chosen[3] = opposite;

    std::vector<Candidate> kept;
    for (size_t index : chosen) {
        kept.push_back(candidates[index]);
    }
    candidates = kept;
}

// The contacts of a face of one box against the other box. `reference` owns the face, whose outward
// normal `referenceNormal` points towards `incident`.
void FaceContacts(const Obb& reference, int referenceAxis, glm::vec3 referenceNormal, const Obb& incident,
                  glm::vec3 normalFromAToB, Manifold& manifold) {
    // The face of the incident box that points most directly back at the reference face.
    int incidentAxis = 0;
    float mostFacing = 0.0f;
    for (int k = 0; k < 3; ++k) {
        const float facing = std::abs(glm::dot(incident.axes[k], referenceNormal));
        if (facing > mostFacing) {
            mostFacing = facing;
            incidentAxis = k;
        }
    }
    const float faceSign = glm::dot(incident.axes[incidentAxis], referenceNormal) > 0.0f ? -1.0f : 1.0f;
    const glm::vec3 faceCentre = incident.center + incident.axes[incidentAxis] * (faceSign * incident.half[incidentAxis]);
    const int first = (incidentAxis + 1) % 3;
    const int second = (incidentAxis + 2) % 3;
    const glm::vec3 alongFirst = incident.axes[first] * incident.half[first];
    const glm::vec3 alongSecond = incident.axes[second] * incident.half[second];
    std::vector<glm::vec3> polygon = {faceCentre + alongFirst + alongSecond, faceCentre - alongFirst + alongSecond,
                                      faceCentre - alongFirst - alongSecond, faceCentre + alongFirst - alongSecond};

    // Cut it to the sides of the reference face.
    const glm::vec3 referenceCentre = reference.center + referenceNormal * reference.half[referenceAxis];
    const int sideU = (referenceAxis + 1) % 3;
    const int sideV = (referenceAxis + 2) % 3;
    polygon = ClipPolygon(polygon, referenceCentre, reference.axes[sideU], reference.half[sideU]);
    polygon = ClipPolygon(polygon, referenceCentre, -reference.axes[sideU], reference.half[sideU]);
    polygon = ClipPolygon(polygon, referenceCentre, reference.axes[sideV], reference.half[sideV]);
    polygon = ClipPolygon(polygon, referenceCentre, -reference.axes[sideV], reference.half[sideV]);

    std::vector<Candidate> candidates;
    for (const glm::vec3& point : polygon) {
        const float depth = -glm::dot(point - referenceCentre, referenceNormal);
        if (depth >= -kContactMargin) {
            // Halfway between the incident corner and its shadow on the reference face.
            candidates.push_back({point + referenceNormal * (0.5f * depth), depth});
        }
    }
    ReduceToFour(candidates, referenceNormal);

    manifold.normal = normalFromAToB;
    manifold.count = static_cast<int>(candidates.size());
    for (int i = 0; i < manifold.count; ++i) {
        manifold.points[i].position = candidates[static_cast<size_t>(i)].position;
        manifold.points[i].depth = candidates[static_cast<size_t>(i)].depth;
    }
}

// The single contact of two crossing edges, edge `i` of `a` and edge `j` of `b`.
void EdgeContact(const Obb& a, int i, const Obb& b, int j, glm::vec3 normalFromAToB, float separation, Manifold& manifold) {
    // The edge of each box that lies furthest towards the other, along the contact normal.
    glm::vec3 pointA = a.center;
    glm::vec3 pointB = b.center;
    for (int k = 0; k < 3; ++k) {
        if (k != i) {
            pointA += a.axes[k] * (SignOf(glm::dot(a.axes[k], normalFromAToB)) * a.half[k]);
        }
        if (k != j) {
            pointB -= b.axes[k] * (SignOf(glm::dot(b.axes[k], normalFromAToB)) * b.half[k]);
        }
    }
    // The closest points of the two lines.
    const glm::vec3 directionA = a.axes[i];
    const glm::vec3 directionB = b.axes[j];
    const glm::vec3 offset = pointA - pointB;
    const float cross = glm::dot(directionA, directionB);
    const float onA = glm::dot(directionA, offset);
    const float onB = glm::dot(directionB, offset);
    const float denominator = 1.0f - cross * cross;
    const float s = denominator > 1e-6f ? (cross * onB - onA) / denominator : 0.0f;
    const float t = denominator > 1e-6f ? (onB - cross * onA) / denominator : 0.0f;
    manifold.normal = normalFromAToB;
    manifold.count = 1;
    manifold.points[0].position = 0.5f * ((pointA + directionA * s) + (pointB + directionB * t));
    manifold.points[0].depth = -separation;
}

bool BoxBox(const Body& bodyA, const Body& bodyB, Manifold& manifold) {
    const Obb a = ObbOf(bodyA);
    const Obb b = ObbOf(bodyB);
    const glm::vec3 offset = b.center - a.center;

    // The separating axis test: the boxes are apart exactly when some face normal of either box, or
    // some cross product of an edge of each, separates their shadows. The axis where they overlap
    // least gives the contact.
    float bestFaceSeparation = -std::numeric_limits<float>::max();
    int bestFaceAxis = 0;
    bool bestFaceIsA = true;
    for (int i = 0; i < 3; ++i) {
        const float separationA = SeparationAlong(a, b, offset, a.axes[i]);
        const float separationB = SeparationAlong(a, b, offset, b.axes[i]);
        if (separationA > kContactMargin || separationB > kContactMargin) {
            return false;
        }
        if (separationA > bestFaceSeparation) {
            bestFaceSeparation = separationA;
            bestFaceAxis = i;
            bestFaceIsA = true;
        }
        if (separationB > bestFaceSeparation) {
            bestFaceSeparation = separationB;
            bestFaceAxis = i;
            bestFaceIsA = false;
        }
    }

    float bestEdgeSeparation = -std::numeric_limits<float>::max();
    int edgeOfA = 0;
    int edgeOfB = 0;
    glm::vec3 edgeAxis(0.0f, 1.0f, 0.0f);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            glm::vec3 axis = glm::cross(a.axes[i], b.axes[j]);
            const float length = glm::length(axis);
            if (length < 1e-4f) {
                continue;  // parallel edges: the face axes already cover this direction
            }
            axis /= length;
            const float separation = SeparationAlong(a, b, offset, axis);
            if (separation > kContactMargin) {
                return false;
            }
            if (separation > bestEdgeSeparation) {
                bestEdgeSeparation = separation;
                edgeOfA = i;
                edgeOfB = j;
                edgeAxis = axis;
            }
        }
    }

    // A face wins unless an edge pair is clearly better: that keeps a box lying flat on another from
    // flickering between face and edge contacts.
    constexpr float kEdgePreference = 0.01f;
    if (bestEdgeSeparation > 0.95f * bestFaceSeparation + kEdgePreference) {
        const glm::vec3 normal = glm::dot(edgeAxis, offset) < 0.0f ? -edgeAxis : edgeAxis;
        EdgeContact(a, edgeOfA, b, edgeOfB, normal, bestEdgeSeparation, manifold);
        return true;
    }

    const Obb& reference = bestFaceIsA ? a : b;
    const Obb& incident = bestFaceIsA ? b : a;
    // The outward normal of the reference face points towards the other box.
    const glm::vec3 faceAxis = reference.axes[bestFaceAxis];
    const glm::vec3 towardsIncident = incident.center - reference.center;
    const glm::vec3 referenceNormal = glm::dot(faceAxis, towardsIncident) < 0.0f ? -faceAxis : faceAxis;
    const glm::vec3 normalFromAToB = bestFaceIsA ? referenceNormal : -referenceNormal;
    FaceContacts(reference, bestFaceAxis, referenceNormal, incident, normalFromAToB, manifold);
    return manifold.count > 0;
}

}  // namespace

bool Collide(const Body& a, const Body& b, Manifold& manifold) {
    manifold.count = 0;
    if (a.shape.type == ShapeType::Sphere && b.shape.type == ShapeType::Sphere) {
        return SphereSphere(a, b, manifold);
    }
    if (a.shape.type == ShapeType::Sphere) {
        return SphereBox(a, b, manifold);
    }
    if (b.shape.type == ShapeType::Sphere) {
        if (!SphereBox(b, a, manifold)) {
            return false;
        }
        manifold.normal = -manifold.normal;  // the normal must point from `a` to `b`
        return true;
    }
    return BoxBox(a, b, manifold);
}

}  // namespace physics
