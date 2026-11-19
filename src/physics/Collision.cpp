#include "physics/Collision.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
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

// One point where a round shape touches another shape. The normal points from the round shape to the other shape.
struct RoundContact {
    glm::vec3 normal;
    glm::vec3 position;
    float depth;
};

// The contact between two spheres, found from their centres. The normal points from `centreA` to `centreB`.
bool SpheresTouch(glm::vec3 centreA, float radiusA, glm::vec3 centreB, float radiusB, float margin, RoundContact& contact) {
    const glm::vec3 between = centreB - centreA;
    const float distanceSquared = glm::dot(between, between);
    const float radii = radiusA + radiusB;
    const float reach = radii + margin;
    if (distanceSquared > reach * reach) {
        return false;
    }
    const float distance = std::sqrt(distanceSquared);
    contact.normal = distance > 1e-6f ? between / distance : glm::vec3(0.0f, 1.0f, 0.0f);
    // Halfway between the two surfaces along the line through the centres.
    contact.position = centreA + contact.normal * (0.5f * (radiusA + distance - radiusB));
    contact.depth = radii - distance;
    return true;
}

// The contact between a sphere, given by its centre and radius, and a box.
bool SphereTouchesBox(glm::vec3 centre, float radius, const Obb& obb, float margin, RoundContact& contact) {
    const glm::vec3 local = glm::transpose(obb.axes) * (centre - obb.center);
    const glm::vec3 closest = glm::clamp(local, -obb.half, obb.half);
    const glm::vec3 delta = local - closest;  // from the closest point of the box to the sphere's centre
    const float distanceSquared = glm::dot(delta, delta);

    if (distanceSquared > 1e-12f) {
        const float reach = radius + margin;
        if (distanceSquared > reach * reach) {
            return false;
        }
        const float distance = std::sqrt(distanceSquared);
        contact.normal = -(obb.axes * (delta / distance));
        contact.position = obb.center + obb.axes * closest;
        contact.depth = radius - distance;
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
        contact.normal = -outward;
        contact.position = centre + outward * gaps[axis];
        contact.depth = radius + gaps[axis];
    }
    return true;
}

void SetSingleContact(const RoundContact& contact, Manifold& manifold) {
    manifold.normal = contact.normal;
    manifold.points[0].position = contact.position;
    manifold.points[0].depth = contact.depth;
    manifold.count = 1;
}

bool SphereSphere(const Body& a, const Body& b, Manifold& manifold, float margin) {
    RoundContact contact;
    if (!SpheresTouch(a.position, a.shape.radius, b.position, b.shape.radius, margin, contact)) {
        return false;
    }
    SetSingleContact(contact, manifold);
    return true;
}

// The contact between a sphere and a box. The normal points from the sphere to the box.
bool SphereBox(const Body& sphere, const Body& box, Manifold& manifold, float margin) {
    RoundContact contact;
    if (!SphereTouchesBox(sphere.position, sphere.shape.radius, ObbOf(box), margin, contact)) {
        return false;
    }
    SetSingleContact(contact, manifold);
    return true;
}

// Capsules ------------------------------------------------------------------------------------

// The straight part of a capsule in the world, from one end to the other.
struct Segment {
    glm::vec3 start;
    glm::vec3 end;
};

Segment SegmentOf(const Body& capsule) {
    const glm::vec3 along = AxesOf(capsule)[1] * capsule.shape.halfHeight;
    return {capsule.position - along, capsule.position + along};
}

glm::vec3 ClosestPointOnSegment(const Segment& segment, glm::vec3 point) {
    const glm::vec3 along = segment.end - segment.start;
    const float lengthSquared = glm::dot(along, along);
    if (lengthSquared < 1e-12f) {
        return segment.start;
    }
    const float t = glm::clamp(glm::dot(point - segment.start, along) / lengthSquared, 0.0f, 1.0f);
    return segment.start + along * t;
}

// The closest points of two segments, after Ericson, Real-Time Collision Detection, section 5.1.9.
void ClosestPointsOfSegments(const Segment& first, const Segment& second, glm::vec3& onFirst, glm::vec3& onSecond) {
    const glm::vec3 d1 = first.end - first.start;
    const glm::vec3 d2 = second.end - second.start;
    const glm::vec3 r = first.start - second.start;
    const float a = glm::dot(d1, d1);
    const float e = glm::dot(d2, d2);
    const float f = glm::dot(d2, r);
    float s = 0.0f;
    float t = 0.0f;
    if (a <= 1e-12f && e <= 1e-12f) {
        // Both are points.
    } else if (a <= 1e-12f) {
        t = glm::clamp(f / e, 0.0f, 1.0f);
    } else {
        const float c = glm::dot(d1, r);
        if (e <= 1e-12f) {
            s = glm::clamp(-c / a, 0.0f, 1.0f);
        } else {
            const float b = glm::dot(d1, d2);
            const float denominator = a * e - b * b;  // zero when the segments are parallel
            s = denominator > 1e-9f * a * e ? glm::clamp((b * f - c * e) / denominator, 0.0f, 1.0f) : 0.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) {
                t = 0.0f;
                s = glm::clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = glm::clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }
    onFirst = first.start + d1 * s;
    onSecond = second.start + d2 * t;
}

// A capsule is a segment with a radius, so every contact is a sphere centred on the segment touching something.
// One such sphere is not enough: a capsule that lies on a face touches it along its length and would rock on a
// single point. So the contact is tried at the closest point and at both ends, and the points that agree with the
// deepest one in direction are kept.
class ContactCollector {
public:
    explicit ContactCollector(Manifold& manifold) : manifold(manifold) {}

    void Add(const RoundContact& contact) {
        candidates[count++] = contact;
    }

    // Keeps the deepest contact, and the others that point the same way and lie apart from it and from each other.
    bool Finish() {
        if (count == 0) {
            return false;
        }
        int deepest = 0;
        for (int i = 1; i < count; ++i) {
            if (candidates[i].depth > candidates[deepest].depth) {
                deepest = i;
            }
        }
        manifold.normal = candidates[deepest].normal;
        manifold.count = 0;
        AddPoint(candidates[deepest]);
        for (int i = 0; i < count; ++i) {
            const bool sameDirection = glm::dot(candidates[i].normal, manifold.normal) > kSameDirection;
            if (i != deepest && sameDirection && IsApartFromKept(candidates[i].position)) {
                AddPoint(candidates[i]);
            }
        }
        return true;
    }

private:
    static constexpr float kSameDirection = 0.9f;
    static constexpr float kMinSpacing = 0.01f;  // contacts closer than 1 cm are one contact

    void AddPoint(const RoundContact& contact) {
        if (manifold.count < kMaxContactsPerPair) {
            manifold.points[manifold.count].position = contact.position;
            manifold.points[manifold.count].depth = contact.depth;
            ++manifold.count;
        }
    }

    bool IsApartFromKept(glm::vec3 position) const {
        for (int i = 0; i < manifold.count; ++i) {
            if (glm::distance(position, manifold.points[i].position) < kMinSpacing) {
                return false;
            }
        }
        return true;
    }

    Manifold& manifold;
    RoundContact candidates[8];
    int count = 0;
};

// The contact between a capsule and a sphere. The normal points from the capsule to the sphere.
bool CapsuleSphere(const Body& capsule, const Body& sphere, Manifold& manifold, float margin) {
    RoundContact contact;
    const glm::vec3 nearest = ClosestPointOnSegment(SegmentOf(capsule), sphere.position);
    if (!SpheresTouch(nearest, capsule.shape.radius, sphere.position, sphere.shape.radius, margin, contact)) {
        return false;
    }
    SetSingleContact(contact, manifold);
    return true;
}

// The contacts between two capsules. The normal points from `a` to `b`.
bool CapsuleCapsule(const Body& a, const Body& b, Manifold& manifold, float margin) {
    const Segment segmentA = SegmentOf(a);
    const Segment segmentB = SegmentOf(b);
    ContactCollector collector(manifold);
    const auto tryCentres = [&](glm::vec3 centreA, glm::vec3 centreB) {
        RoundContact contact;
        if (SpheresTouch(centreA, a.shape.radius, centreB, b.shape.radius, margin, contact)) {
            collector.Add(contact);
        }
    };
    glm::vec3 closestA;
    glm::vec3 closestB;
    ClosestPointsOfSegments(segmentA, segmentB, closestA, closestB);
    tryCentres(closestA, closestB);
    // Parallel capsules side by side touch along their overlap: the ends of each give the two corners of it.
    for (const glm::vec3 end : {segmentA.start, segmentA.end}) {
        tryCentres(end, ClosestPointOnSegment(segmentB, end));
    }
    for (const glm::vec3 end : {segmentB.start, segmentB.end}) {
        tryCentres(ClosestPointOnSegment(segmentA, end), end);
    }
    return collector.Finish();
}

// How far a point is from the box (zero inside), as a squared distance, with the point in the box's own axes.
float DistanceSquaredToBox(glm::vec3 localPoint, glm::vec3 half) {
    const glm::vec3 outside = localPoint - glm::clamp(localPoint, -half, half);
    return glm::dot(outside, outside);
}

// The point of the segment that comes closest to the box. The distance to a convex shape is convex along a line,
// so a ternary search finds it. Returns false when that point is inside the box, that is, the segment meets it.
bool PointOfSegmentNearestBox(const Segment& segment, const Obb& obb, glm::vec3& nearest) {
    const glm::mat3 toLocal = glm::transpose(obb.axes);
    const glm::vec3 start = toLocal * (segment.start - obb.center);
    const glm::vec3 end = toLocal * (segment.end - obb.center);
    const auto at = [&](float t) { return start + (end - start) * t; };
    constexpr int kIterations = 40;

    float low = 0.0f;
    float high = 1.0f;
    for (int i = 0; i < kIterations; ++i) {
        const float third = (high - low) / 3.0f;
        if (DistanceSquaredToBox(at(low + third), obb.half) < DistanceSquaredToBox(at(high - third), obb.half)) {
            high -= third;
        } else {
            low += third;
        }
    }
    const float best = 0.5f * (low + high);
    nearest = segment.start + (segment.end - segment.start) * best;
    return DistanceSquaredToBox(at(best), obb.half) > 1e-12f;
}

// Cuts the segment from `start` to `end` down to the part between the planes of the box on the axes that `useAxis`
// selects (in the box's own axes). Returns false when nothing is left; otherwise the part is the points at
// `start + (end - start) * t` for t from `from` to `to`.
bool ClipSegmentToSlabs(glm::vec3 start, glm::vec3 end, glm::vec3 half, const bool useAxis[3], float& from, float& to) {
    from = 0.0f;
    to = 1.0f;
    const glm::vec3 along = end - start;
    for (int axis = 0; axis < 3; ++axis) {
        if (!useAxis[axis]) {
            continue;
        }
        if (std::abs(along[axis]) < 1e-9f) {
            if (std::abs(start[axis]) > half[axis]) {
                return false;
            }
            continue;
        }
        float enter = (-half[axis] - start[axis]) / along[axis];
        float leave = (half[axis] - start[axis]) / along[axis];
        if (enter > leave) {
            std::swap(enter, leave);
        }
        from = std::max(from, enter);
        to = std::min(to, leave);
    }
    return from <= to;
}

// The contact of a capsule whose straight part meets the box. Choosing the face to leave through by the deepest
// point of the segment would be a guess (a capsule laid across a post has a whole stretch of equally deep points), so
// each of the six faces is tried: the capsule has to move far enough along its normal that even its most
// embedded end clears the face, and the face that asks for the least is the way out. Only the part of the segment
// that is inside the box counts: a long capsule that clips a corner is not embedded by its far ends.
RoundContact ExitFromBox(const Segment& segment, float radius, const Obb& obb) {
    const glm::mat3 toLocal = glm::transpose(obb.axes);
    glm::vec3 start = toLocal * (segment.start - obb.center);
    glm::vec3 end = toLocal * (segment.end - obb.center);
    const bool allAxes[3] = {true, true, true};
    float from;
    float to;
    if (ClipSegmentToSlabs(start, end, obb.half, allAxes, from, to)) {
        const glm::vec3 along = end - start;
        end = start + along * to;
        start = start + along * from;
    }
    float leastMove = std::numeric_limits<float>::max();
    int exitAxis = 0;
    float exitSign = 1.0f;
    glm::vec3 embeddedEnd = start;
    for (int axis = 0; axis < 3; ++axis) {
        for (const float sign : {-1.0f, 1.0f}) {
            // The capsule leaves through the face on the `sign` side, so its end furthest to the other side is the
            // one that has to travel the longest way.
            const bool startIsFurther = sign * start[axis] < sign * end[axis];
            const glm::vec3 furthest = startIsFurther ? start : end;
            const float move = obb.half[axis] - sign * furthest[axis] + radius;
            if (move < leastMove) {
                leastMove = move;
                exitAxis = axis;
                exitSign = sign;
                embeddedEnd = furthest;
            }
        }
    }
    glm::vec3 onFace = glm::clamp(embeddedEnd, -obb.half, obb.half);
    onFace[exitAxis] = exitSign * obb.half[exitAxis];
    RoundContact contact;
    contact.normal = -(obb.axes[exitAxis] * exitSign);
    contact.position = obb.center + obb.axes * onFace;
    contact.depth = leastMove;
    return contact;
}

// The contacts between a capsule and a box. The normal points from the capsule to the box.
bool CapsuleBox(const Body& capsule, const Body& box, Manifold& manifold, float margin) {
    const Obb obb = ObbOf(box);
    const Segment segment = SegmentOf(capsule);
    const float radius = capsule.shape.radius;
    ContactCollector collector(manifold);
    glm::vec3 nearest;
    if (!PointOfSegmentNearestBox(segment, obb, nearest)) {
        collector.Add(ExitFromBox(segment, radius, obb));
        return collector.Finish();
    }
    const auto tryCentre = [&](glm::vec3 centre) {
        RoundContact contact;
        if (SphereTouchesBox(centre, radius, obb, margin, contact)) {
            collector.Add(contact);
        }
    };
    tryCentre(nearest);
    // A capsule lying on a face touches it at both ends as well.
    tryCentre(segment.start);
    tryCentre(segment.end);
    // A capsule longer than the face it lies across has both ends in the air, so also try the two places where it
    // passes over the edges of the face. These are the ends of the stretch of the segment that is over the face.
    const glm::mat3 toLocal = glm::transpose(obb.axes);
    const glm::vec3 localStart = toLocal * (segment.start - obb.center);
    const glm::vec3 localEnd = toLocal * (segment.end - obb.center);
    const glm::vec3 localNearest = toLocal * (nearest - obb.center);
    const glm::vec3 outside = glm::abs(localNearest - glm::clamp(localNearest, -obb.half, obb.half));
    const int facingAxis = outside.x >= outside.y && outside.x >= outside.z ? 0 : (outside.y >= outside.z ? 1 : 2);
    bool overFace[3] = {true, true, true};
    overFace[facingAxis] = false;
    float from;
    float to;
    if (ClipSegmentToSlabs(localStart, localEnd, obb.half, overFace, from, to)) {
        tryCentre(segment.start + (segment.end - segment.start) * from);
        tryCentre(segment.start + (segment.end - segment.start) * to);
    }
    return collector.Finish();
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
                  glm::vec3 normalFromAToB, float margin, Manifold& manifold) {
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
        if (depth >= -margin) {
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

bool BoxBox(const Body& bodyA, const Body& bodyB, Manifold& manifold, float margin) {
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
        if (separationA > margin || separationB > margin) {
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
            if (separation > margin) {
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
    // flickering between face and edge contacts. "Better" is further apart, by a centimetre and a twentieth
    // of the separation, whether the boxes overlap (negative) or are still apart (positive).
    constexpr float kEdgePreference = 0.01f;
    if (bestEdgeSeparation > bestFaceSeparation + kEdgePreference + 0.05f * std::abs(bestFaceSeparation)) {
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
    FaceContacts(reference, bestFaceAxis, referenceNormal, incident, normalFromAToB, margin, manifold);
    return manifold.count > 0;
}

}  // namespace

bool Collide(const Body& a, const Body& b, Manifold& manifold, float margin) {
    manifold.count = 0;
    // Each pair is written once with its shapes in a fixed order, and the other order swaps the bodies and turns the
    // normal round. There is no default case: a new shape type must be handled here, not collided as a box.
    const auto swapped = [&](bool (*collide)(const Body&, const Body&, Manifold&, float)) {
        if (!collide(b, a, manifold, margin)) {
            return false;
        }
        manifold.normal = -manifold.normal;  // the normal must point from `a` to `b`
        return true;
    };
    switch (a.shape.type) {
    case ShapeType::Sphere:
        switch (b.shape.type) {
        case ShapeType::Sphere: return SphereSphere(a, b, manifold, margin);
        case ShapeType::Box: return SphereBox(a, b, manifold, margin);
        case ShapeType::Capsule: return swapped(CapsuleSphere);
        }
        break;
    case ShapeType::Box:
        switch (b.shape.type) {
        case ShapeType::Sphere: return swapped(SphereBox);
        case ShapeType::Box: return BoxBox(a, b, manifold, margin);
        case ShapeType::Capsule: return swapped(CapsuleBox);
        }
        break;
    case ShapeType::Capsule:
        switch (b.shape.type) {
        case ShapeType::Sphere: return CapsuleSphere(a, b, manifold, margin);
        case ShapeType::Box: return CapsuleBox(a, b, manifold, margin);
        case ShapeType::Capsule: return CapsuleCapsule(a, b, manifold, margin);
        }
        break;
    }
    return false;
}

}  // namespace physics
