#include "character/appearance/Clothing.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <map>

#include "character/Anthropometry.h"
#include "character/ParallelEach.h"
#include "character/appearance/Atlas.h"
#include "character/appearance/MeshTools.h"
#include "character/appearance/Noise3D.h"
#include "character/appearance/Skin.h"

namespace character {
namespace {

// Distances of the body that the cut of a garment follows, found from the skeleton.
struct BodyMeasures {
    float stature = 1.75f;
    float scale = 1.0f;  // stature / 1.75
    glm::vec3 shoulder[2];
    glm::vec3 armDirection[2];  // from the shoulder to the wrist
    float upperArmLength = 0.3f;
    float armLength = 0.5f;     // from the shoulder to the wrist
    float neckZ = 0.0f;         // where the axis of the neck runs, in front of or behind the middle of the body
    float ankleZ = 0.0f;        // where the ankle is, which the shape of a shoe is measured from
};

// How high up the leg a shoe or a boot reaches, as a fraction of stature.
constexpr float kShoeTop = 0.056f;
constexpr float kBootTop = 0.115f;
// Trousers end this far below the top of the footwear, so that no skin shows between them.
constexpr float kTrouserOverlap = 0.014f;

BodyMeasures MeasureBody(const Skeleton& skeleton) {
    BodyMeasures m;
    m.stature = skeleton.statureMeters;
    m.scale = m.stature / 1.75f;
    const Joint shoulders[2] = {Joint::UpperArmL, Joint::UpperArmR};
    const Joint elbows[2] = {Joint::ForearmL, Joint::ForearmR};
    const Joint wrists[2] = {Joint::HandL, Joint::HandR};
    for (int i = 0; i < 2; ++i) {
        m.shoulder[i] = skeleton[shoulders[i]].restPosition;
        m.armDirection[i] = glm::normalize(skeleton[wrists[i]].restPosition - m.shoulder[i]);
    }
    m.upperArmLength = glm::distance(skeleton[shoulders[0]].restPosition, skeleton[elbows[0]].restPosition);
    m.armLength = glm::distance(skeleton[shoulders[0]].restPosition, skeleton[wrists[0]].restPosition);
    m.neckZ = skeleton[Joint::Neck].restPosition.z;
    m.ankleZ = skeleton[Joint::FootL].restPosition.z;
    return m;
}

// The share of a vertex that follows the bones of the arms, or of the legs.
float WeightOnLimbs(const MeshVertex& vertex, bool arms) {
    float sum = 0.0f;
    for (int slot = 0; slot < 4; ++slot) {
        const Joint joint = static_cast<Joint>(vertex.bones[slot]);
        const bool isArm = joint == Joint::UpperArmL || joint == Joint::UpperArmR || joint == Joint::ForearmL || joint == Joint::ForearmR || joint == Joint::HandL ||
                           joint == Joint::HandR;
        const bool isLeg = joint == Joint::ThighL || joint == Joint::ThighR || joint == Joint::CalfL || joint == Joint::CalfR || joint == Joint::FootL ||
                           joint == Joint::FootR || joint == Joint::ToesL || joint == Joint::ToesR;
        if (arms ? isArm : isLeg) {
            sum += vertex.weights[slot];
        }
    }
    return sum;
}

// How far along the arm, from the shoulder, a point is.
float AlongArm(const BodyMeasures& m, glm::vec3 p) {
    const int side = p.x >= 0.0f ? 0 : 1;
    return glm::dot(p - m.shoulder[side], m.armDirection[side]);
}

// Positive where a sleeve reaches, `sleeveEnd` metres along the arm from the shoulder. Away from the arms it is positive everywhere.
float SleeveField(const BodyMeasures& m, const MeshVertex& vertex, float sleeveEnd) {
    const float onArm = SmoothStep(0.25f, 0.60f, WeightOnLimbs(vertex, true));
    return glm::mix(1.0f, sleeveEnd - AlongArm(m, vertex.position), onArm);
}

// Positive inside the garment, in metres from its edge where that is known; zero on the edge.
struct GarmentSpec {
    GarmentKind kind = GarmentKind::TShirt;
    GarmentMaterial material = GarmentMaterial::Cotton;
    material::MaterialKind materialKind = material::MaterialKind::Fabric;
    glm::vec3 tint{1.0f};
    float thickness = 0.001f;
    std::function<float(const MeshVertex&)> field;  // where the garment is
    std::function<float(glm::vec3)> ease;           // how far it stands off the body at a point of the body
    float wear = 0.3f;
    int smoothingPasses = 0;  // cloth hangs more smoothly than the body under it: the offset surface is relaxed this many times
};

// The hem of a garment hangs looser than its body: the ease grows over the lowest `reach` metres above `hemY`.
std::function<float(glm::vec3)> EaseWithFlare(float base, float flare, float hemY, float reach) {
    return [=](glm::vec3 p) { return base + flare * SmoothStep(hemY + reach, hemY, p.y); };
}

glm::vec3 PickColour(const std::vector<glm::vec3>& palette, uint32_t seed, uint32_t salt) {
    return palette[material::Hash(seed, salt, 7u) % palette.size()];
}

glm::vec3 SrgbToLinear(glm::vec3 c) {
    return glm::pow(c, glm::vec3(2.2f));
}

GarmentSpec MakeSpec(GarmentKind kind, const BodyMeasures& m, uint32_t seed, FootwearKind footwear) {
    GarmentSpec spec;
    spec.kind = kind;
    const float stature = m.stature;
    const float s = m.scale;
    switch (kind) {
        case GarmentKind::TShirt: {
            const float hem = 0.54f * stature;
            spec.material = GarmentMaterial::Cotton;
            spec.materialKind = material::MaterialKind::Fabric;
            spec.tint = SrgbToLinear(PickColour({{0.85f, 0.85f, 0.82f}, {0.18f, 0.22f, 0.36f}, {0.62f, 0.14f, 0.14f}, {0.22f, 0.42f, 0.28f}, {0.78f, 0.62f, 0.20f}, {0.12f, 0.12f, 0.13f}, {0.45f, 0.47f, 0.50f}}, seed, 1u));
            spec.thickness = 0.0012f;
            spec.ease = EaseWithFlare(0.0050f, 0.0110f, hem, 0.06f);
            spec.smoothingPasses = 3;
            spec.field = [=](const MeshVertex& v) {
                // A round neckline is the cut of an ellipsoid round the base of the neck: it reaches further down at the front than at
                // the back, and the shoulders of the body, which rise higher than a plane through the base of the neck, stay covered.
                const glm::vec3 centre(0.0f, 0.846f * stature, m.neckZ + 0.014f * s);
                const glm::vec3 radii(0.086f * s, 0.062f * s, 0.098f * s);
                const float outsideHole = (glm::length((v.position - centre) / radii) - 1.0f) * 0.03f;
                // The neck above the neckline is bare: nothing above the top of the hole.
                const float belowTop = centre.y + 0.028f * s - v.position.y;
                return std::min({v.position.y - hem, outsideHole, belowTop, SleeveField(m, v, 0.50f * m.upperArmLength)});
            };
            break;
        }
        case GarmentKind::Jacket: {
            const float hem = 0.52f * stature;
            spec.material = GarmentMaterial::Leather;
            spec.materialKind = material::MaterialKind::Leather;
            spec.tint = SrgbToLinear(PickColour({{0.10f, 0.09f, 0.09f}, {0.30f, 0.17f, 0.09f}, {0.20f, 0.12f, 0.08f}, {0.16f, 0.17f, 0.20f}}, seed, 2u));
            spec.thickness = 0.0028f;
            // A jacket hangs loose over the chest, the hips and the shoulders instead of following them like paint; the relaxation of the
            // surface takes out the bumps of the body, and the clearance check puts the cloth back out where the body is still higher.
            spec.ease = EaseWithFlare(0.0200f, 0.0160f, hem, 0.10f);
            spec.smoothingPasses = 10;
            spec.field = [=](const MeshVertex& v) {
                const float collar = 0.860f * stature - 0.012f * SmoothStep(0.0f, 0.05f, v.position.z);
                return std::min({v.position.y - hem, collar - v.position.y, SleeveField(m, v, m.armLength - 0.075f * s)});
            };
            break;
        }
        case GarmentKind::Trousers:
        case GarmentKind::Shorts: {
            const bool shorts = kind == GarmentKind::Shorts;
            const float waist = 0.56f * stature;
            // Trousers go down over the top of whatever is on the feet, with at least a centimetre of overlap, or to just above the ankle.
            const float footTop = footwear == FootwearKind::Boots ? kBootTop : (footwear == FootwearKind::Shoes ? kShoeTop : 0.0f);
            const float cuff = shorts ? 0.37f * stature : (footTop > 0.0f ? footTop * stature - kTrouserOverlap * s : anthropometry::kAnkleHeight * stature + 0.035f);
            spec.material = shorts ? GarmentMaterial::Cotton : GarmentMaterial::Denim;
            spec.materialKind = material::MaterialKind::Fabric;
            spec.tint = shorts ? SrgbToLinear(PickColour({{0.55f, 0.50f, 0.38f}, {0.20f, 0.24f, 0.34f}, {0.30f, 0.32f, 0.26f}}, seed, 3u))
                               : SrgbToLinear(PickColour({{0.12f, 0.17f, 0.30f}, {0.09f, 0.12f, 0.20f}, {0.10f, 0.10f, 0.11f}, {0.20f, 0.26f, 0.38f}}, seed, 4u));
            spec.thickness = shorts ? 0.0015f : 0.0020f;
            spec.ease = [](glm::vec3) { return 0.0080f; };
            spec.smoothingPasses = 4;
            spec.field = [=](const MeshVertex& v) { return std::min({waist - v.position.y, v.position.y - cuff, (0.5f - WeightOnLimbs(v, true)) * 0.1f}); };
            break;
        }
        case GarmentKind::Shoes:
        case GarmentKind::Boots: {
            const bool boots = kind == GarmentKind::Boots;
            const float top = (boots ? kBootTop : kShoeTop) * stature;
            spec.material = GarmentMaterial::Leather;
            spec.materialKind = material::MaterialKind::Leather;
            spec.tint = SrgbToLinear(PickColour({{0.08f, 0.08f, 0.08f}, {0.25f, 0.14f, 0.07f}, {0.14f, 0.09f, 0.06f}, {0.55f, 0.55f, 0.52f}}, seed, 5u));
            spec.thickness = boots ? 0.0040f : 0.0030f;
            // The footwear is the first layer, with the trousers outside it. The shape of a shoe is the shape of the foot with a stiff
            // rounded toe box (more ease over the toes), a heel counter behind, and the thickness of the outsole under the foot.
            const float base = boots ? 0.0070f : 0.0050f;
            const float ankleZ = m.ankleZ;
            spec.ease = [=](glm::vec3 p) {
                const float toeBox = SmoothStep(ankleZ + 0.06f * s, ankleZ + 0.12f * s, p.z);
                const float heel = SmoothStep(ankleZ - 0.01f * s, ankleZ - 0.05f * s, p.z);
                return base + 0.0070f * toeBox + 0.0040f * heel + 0.0060f * SmoothStep(0.028f, 0.004f, p.y);
            };
            spec.smoothingPasses = 3;
            spec.field = [=](const MeshVertex& v) { return std::min(top - v.position.y, (WeightOnLimbs(v, false) - 0.5f) * 0.1f); };
            break;
        }
    }
    return spec;
}

// The surfaces a garment must stay clear of: the character itself, and the outer surface of each garment below this one.
// `aligned` marks a surface that only counts for points that face the way it does: the outer surface of a garment below is a wall that a
// point must be outside of, and its far wall (the other side of a sleeve) must not push the point back in.
struct ObstacleRef {
    const TriangleGrid* grid;
    bool aligned;
};
using ObstacleList = std::vector<ObstacleRef>;

// The move that brings a point to `minimum` away from every surface in `obstacles`, on the outside: zero if it is far enough. A point
// is on the wrong side of a surface when it is inside it, however deep (within a few centimetres): it is carried out along the normal
// of the surface by its depth and the clearance. Where two parts of the body face each other across a narrow gap (the two thighs) each
// pushes along its own normal, so the point settles between them instead of being driven into one by the other.
glm::vec3 ClearancePush(const ObstacleList& obstacles, glm::vec3 point, glm::vec3 facing, float minimum, float searchRadius) {
    constexpr float kMostPush = 0.004f;      // one pass moves a point at most this far, so that no push can throw it across the body
    glm::vec3 longest(0.0f);
    float longestSquared = 0.0f;
    for (const ObstacleRef& obstacle : obstacles) {
        const TriangleGrid::Nearest nearest = obstacle.grid->FindNearest(point, searchRadius);
        if (nearest.distance > 1e8f || nearest.signedDistance >= minimum) {
            continue;
        }
        if (obstacle.aligned && glm::dot(nearest.surfaceNormal, facing) < 0.3f) {
            continue;
        }
        glm::vec3 push = nearest.surfaceNormal * (minimum - nearest.signedDistance);
        const float length = glm::length(push);
        if (length > kMostPush) {
            push *= kMostPush / length;
        }
        const float squared = glm::dot(push, push);
        if (squared > longestSquared) {
            longestSquared = squared;
            longest = push;
        }
    }
    return longest;
}

// Moves the points of a garment surface until every surface in `obstacles` is at least `minimum` away from the points and from the
// middles of the triangles and of their edges, outside it: a triangle can cut across a hollow of the body that its corners clear, as
// at the crotch or under an arm.
// `searchRadius` is how deep inside a surface a point is still found: the cloth is moved a few millimetres at a time and is never buried far, so
// the rounds look 12 mm deep, which is cheap, and the last pass of a garment looks 4 cm deep, which finds whatever got further in.
void EnforceClearance(std::vector<glm::vec3>& points, const std::vector<glm::vec3>& facing, const std::vector<uint32_t>& indices, const ObstacleList& obstacles, float minimum,
                      JobSystem* jobs, float searchRadius = 0.012f) {
    constexpr int kPasses = 150;
    const size_t triangleCount = indices.size() / 3;
    std::vector<glm::vec3> pushOfVertex(points.size());
    std::vector<glm::vec3> pushOfTriangle(triangleCount);
    // The obstacles do not move, so a point or a triangle that was far enough stays so until one of its corners moves: after the first
    // pass only what moved, and what touches it, is looked at again.
    std::vector<char> vertexActive(points.size(), 1);
    std::vector<char> triangleActive(triangleCount, 1);
    const auto longer = [](glm::vec3 a, glm::vec3 b) { return glm::dot(a, a) >= glm::dot(b, b) ? a : b; };
    for (int pass = 0; pass < kPasses; ++pass) {
        ParallelEach(jobs, static_cast<int>(points.size()), [&](int v) { pushOfVertex[v] = vertexActive[v] ? ClearancePush(obstacles, points[v], facing[v], minimum, searchRadius) : glm::vec3(0.0f); });
        ParallelEach(jobs, static_cast<int>(triangleCount), [&](int t) {
            if (!triangleActive[t]) {
                pushOfTriangle[t] = glm::vec3(0.0f);
                return;
            }
            const glm::vec3 a = points[indices[t * 3]];
            const glm::vec3 b = points[indices[t * 3 + 1]];
            const glm::vec3 c = points[indices[t * 3 + 2]];
            const glm::vec3 side = facing[indices[t * 3]] + facing[indices[t * 3 + 1]] + facing[indices[t * 3 + 2]];
            pushOfTriangle[t] = longer(longer(ClearancePush(obstacles, (a + b + c) / 3.0f, side, minimum, searchRadius), ClearancePush(obstacles, 0.5f * (a + b), side, minimum, searchRadius)),
                                       longer(ClearancePush(obstacles, 0.5f * (b + c), side, minimum, searchRadius), ClearancePush(obstacles, 0.5f * (c + a), side, minimum, searchRadius)));
        });
        // A triangle that is too close moves all three of its corners.
        for (size_t t = 0; t < triangleCount; ++t) {
            for (int corner = 0; corner < 3; ++corner) {
                glm::vec3& push = pushOfVertex[indices[t * 3 + corner]];
                push = longer(push, pushOfTriangle[t]);
            }
        }
        bool changed = false;
        for (size_t v = 0; v < points.size(); ++v) {
            const float length = glm::length(pushOfVertex[v]);
            vertexActive[v] = length > 0.0f ? 1 : 0;
            if (length > 0.0f) {
                points[v] += pushOfVertex[v] * (1.0f + 0.0002f / length);
                changed = true;
            }
        }
        if (!changed) {
            break;
        }
        for (size_t t = 0; t < triangleCount; ++t) {
            triangleActive[t] = vertexActive[indices[t * 3]] || vertexActive[indices[t * 3 + 1]] || vertexActive[indices[t * 3 + 2]];
        }
    }
}

// Relaxes the points of a surface towards the average of their neighbours, `passes` times; the points in `fixed` stay where they are.
void RelaxSurface(std::vector<glm::vec3>& points, const std::vector<uint32_t>& indices, const std::vector<char>& fixed, int passes) {
    std::vector<std::vector<uint32_t>> neighbours(points.size());
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        for (int c = 0; c < 3; ++c) {
            neighbours[indices[t + c]].push_back(indices[t + (c + 1) % 3]);
            neighbours[indices[t + c]].push_back(indices[t + (c + 2) % 3]);
        }
    }
    std::vector<glm::vec3> next = points;
    for (int pass = 0; pass < passes; ++pass) {
        for (size_t v = 0; v < points.size(); ++v) {
            if (fixed[v] || neighbours[v].empty()) {
                continue;
            }
            glm::vec3 sum(0.0f);
            for (const uint32_t other : neighbours[v]) {
                sum += points[other];
            }
            next[v] = glm::mix(points[v], sum / static_cast<float>(neighbours[v].size()), 0.5f);
        }
        points.swap(next);
        next = points;
    }
}

// A closed shell from an open surface: the surface itself pushed out by `ease` and relaxed (the inner side), a copy a thickness
// further, and a strip along every boundary edge to join the two. Both sides are held clear of the character and of the garments
// below. The character comes first, then the layers together with it.
TriangleMesh MakeShell(const TriangleMesh& surface, const GarmentSpec& spec, const TriangleGrid& bodyOnly, const ObstacleList& bodyAndLayers, JobSystem* jobs) {
    const size_t count = surface.vertices.size();
    constexpr float kEnforced = kMinimumClearance + 0.0004f;
    std::vector<glm::vec3> inner(count);
    for (size_t v = 0; v < count; ++v) {
        inner[v] = surface.vertices[v].position + surface.vertices[v].normal * spec.ease(surface.vertices[v].position);
    }
    // A boundary vertex is on an edge that belongs to one triangle only; the hem and the cuffs keep their shape when the cloth relaxes.
    std::vector<char> onBoundary(count, 0);
    {
        std::map<std::pair<uint32_t, uint32_t>, int> directed;
        for (size_t t = 0; t + 2 < surface.indices.size(); t += 3) {
            for (int c = 0; c < 3; ++c) {
                ++directed[{surface.indices[t + c], surface.indices[t + (c + 1) % 3]}];
            }
        }
        for (const auto& [edge, uses] : directed) {
            (void)uses;
            if (directed.count({edge.second, edge.first}) == 0) {
                onBoundary[edge.first] = 1;
                onBoundary[edge.second] = 1;
            }
        }
    }
    // Relax, then push out of whatever is in the way. Where the body or a layer below is in the way the push is uneven from one vertex
    // to the next and leaves ripples, so the two are repeated: each round relaxes the ripples a little and the clearance has the last word.
    std::vector<glm::vec3> bodyNormals(count);
    for (size_t v = 0; v < count; ++v) {
        bodyNormals[v] = surface.vertices[v].normal;
    }
    constexpr int kRounds = 2;
    for (int round = 0; round < kRounds; ++round) {
        RelaxSurface(inner, surface.indices, onBoundary, round == 0 ? spec.smoothingPasses : 4);
        EnforceClearance(inner, bodyNormals, surface.indices, ObstacleList{{&bodyOnly, false}}, kEnforced, jobs);
        EnforceClearance(inner, bodyNormals, surface.indices, bodyAndLayers, kEnforced, jobs);
    }
    EnforceClearance(inner, bodyNormals, surface.indices, ObstacleList{{&bodyOnly, false}}, kEnforced, jobs, 0.04f);
    EnforceClearance(inner, bodyNormals, surface.indices, bodyAndLayers, kEnforced, jobs, 0.04f);

    // The outer side follows the normals of the inner one (the way the body faces, where those turn inwards in a fold), and is
    // held clear of the same things: a fold of the cloth must not put it into the body either.
    const std::vector<glm::vec3> smoothNormals = VertexNormals(inner, surface.indices);
    std::vector<glm::vec3> outer(count);
    for (size_t v = 0; v < count; ++v) {
        const glm::vec3 bodyNormal = surface.vertices[v].normal;
        const glm::vec3 direction = glm::dot(smoothNormals[v], bodyNormal) > 0.2f ? smoothNormals[v] : bodyNormal;
        outer[v] = inner[v] + direction * spec.thickness;
    }
    EnforceClearance(outer, bodyNormals, surface.indices, bodyAndLayers, kEnforced, jobs);
    EnforceClearance(outer, bodyNormals, surface.indices, bodyAndLayers, kEnforced, jobs, 0.04f);

    TriangleMesh shell;
    shell.vertices.resize(count * 2);
    for (size_t v = 0; v < count; ++v) {
        MeshVertex vertex = surface.vertices[v];
        vertex.position = inner[v];
        vertex.attribute.z = 0.0f;
        shell.vertices[v] = vertex;
        vertex.position = outer[v];
        vertex.attribute.z = 1.0f;
        shell.vertices[count + v] = vertex;
    }
    const uint32_t outerBase = static_cast<uint32_t>(count);
    for (size_t t = 0; t + 2 < surface.indices.size(); t += 3) {
        const uint32_t a = surface.indices[t];
        const uint32_t b = surface.indices[t + 1];
        const uint32_t c = surface.indices[t + 2];
        shell.indices.insert(shell.indices.end(), {outerBase + a, outerBase + b, outerBase + c});
        shell.indices.insert(shell.indices.end(), {a, c, b});
    }
    // Boundary edges: a directed edge of a triangle whose reverse belongs to no triangle.
    std::map<std::pair<uint32_t, uint32_t>, int> directed;
    for (size_t t = 0; t + 2 < surface.indices.size(); t += 3) {
        for (int c = 0; c < 3; ++c) {
            ++directed[{surface.indices[t + c], surface.indices[t + (c + 1) % 3]}];
        }
    }
    for (const auto& [edge, uses] : directed) {
        (void)uses;
        if (directed.count({edge.second, edge.first}) != 0) {
            continue;
        }
        const uint32_t p = edge.first;
        const uint32_t q = edge.second;
        shell.indices.insert(shell.indices.end(), {outerBase + q, outerBase + p, p});
        shell.indices.insert(shell.indices.end(), {outerBase + q, p, q});
    }
    std::vector<glm::vec3> positions(shell.vertices.size());
    for (size_t v = 0; v < positions.size(); ++v) {
        positions[v] = shell.vertices[v].position;
    }
    const std::vector<glm::vec3> shellNormals = VertexNormals(positions, shell.indices);
    for (size_t v = 0; v < shell.vertices.size(); ++v) {
        shell.vertices[v].normal = shellNormals[v];
    }
    return shell;
}

// The procedural material of a garment, sampled at a point of its surface by a flat projection along the dominant axis of the normal.
class ClothSampler {
public:
    explicit ClothSampler(const material::MaterialMaps& maps) : maps(maps) {
        double sum = 0.0;
        for (const glm::vec3& c : maps.albedo) {
            sum += Luma(c);
        }
        meanLuma = static_cast<float>(sum / std::max<size_t>(1, maps.albedo.size()));
    }

    struct Sample {
        float lumaRatio = 1.0f;  // the luminance of the material here, against its average: the pattern without the colour
        float roughness = 0.6f;
        float height = 0.0f;
    };

    Sample At(glm::vec3 p, glm::vec3 normal) const {
        const glm::vec3 a = glm::abs(normal);
        glm::vec2 plane;
        if (a.y > a.x && a.y > a.z) {
            plane = glm::vec2(p.x, p.z);
        } else if (a.x > a.z) {
            plane = glm::vec2(p.z, p.y);
        } else {
            plane = glm::vec2(p.x, p.y);
        }
        const float u = plane.x / maps.tileMeters;
        const float v = plane.y / maps.tileMeters;
        const float x = (u - std::floor(u)) * static_cast<float>(maps.size);
        const float y = (v - std::floor(v)) * static_cast<float>(maps.size);
        const int x0 = static_cast<int>(x) % maps.size;
        const int y0 = static_cast<int>(y) % maps.size;
        const size_t i = static_cast<size_t>(y0) * maps.size + x0;
        Sample sample;
        sample.lumaRatio = Luma(maps.albedo[i]) / std::max(meanLuma, 1e-4f);
        sample.roughness = maps.roughness[i];
        sample.height = maps.height[i] - 0.5f * maps.reliefMeters;
        return sample;
    }

private:
    static float Luma(glm::vec3 c) { return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b; }

    const material::MaterialMaps& maps;
    float meanLuma = 1.0f;
};

float Groove(float distance, float halfWidth) {
    const float x = distance / halfWidth;
    return std::exp(-x * x);
}

struct ClothPixel {
    glm::vec3 albedo{0.5f};
    float roughness = 0.7f;
    float height = 0.0f;
};

uint8_t Byte(float value) {
    return static_cast<uint8_t>(std::lround(glm::clamp(value, 0.0f, 1.0f) * 255.0f));
}

// The colour, roughness and relief of a garment at a texel: the weave, the stitching along its edges and the seams of its cut.
ClothPixel PaintCloth(const GarmentSpec& spec, const ClothSampler& cloth, const Texel& texel, float stature) {
    const glm::vec3 p = texel.position;
    const glm::vec3 n = texel.normal;
    const ClothSampler::Sample weave = cloth.At(p, n);
    ClothPixel pixel;
    // Only a trace of the weave goes into the albedo: its fine grain is smaller than a texel of this atlas, and the renderer adds it
    // from the material library at the right scale.
    pixel.albedo = spec.tint * (1.0f + 0.30f * (weave.lumaRatio - 1.0f));
    pixel.roughness = weave.roughness;
    pixel.height = 0.4f * weave.height;
    if (spec.material == GarmentMaterial::Leather) {
        // Leather has a fine pebble grain, cells of a few millimetres and not the large cells of the library's hide at this scale: low
        // rounded bumps with a soft crease between them, and a little mottling of the dye.
        const Cellular3 grain = CellularNoise3(p / 0.0034f, 77);
        const float bump = SmoothStep(0.0f, 0.55f, grain.second - grain.nearest);
        pixel.height = 0.00022f * bump + 0.00008f * Fbm3(p * 40.0f, 2, 79);
        pixel.albedo = spec.tint * (0.92f + 0.10f * bump + 0.06f * Fbm3(p * 14.0f, 2, 81));
        pixel.roughness = 0.52f - 0.10f * bump;
    }
    glm::vec3 thread = glm::mix(spec.tint, glm::vec3(0.55f, 0.40f, 0.12f), spec.kind == GarmentKind::Trousers ? 0.9f : 0.0f);
    thread = spec.kind == GarmentKind::Trousers ? thread : glm::min(spec.tint * 1.7f + glm::vec3(0.02f), glm::vec3(1.0f));

    float stitch = 0.0f;
    // Stitching along the hem, the cuffs and the collar: two rows, with the cloth folded over between them and the edge.
    const float edge = texel.attribute.x;
    stitch = std::max(stitch, Groove(edge - 0.0110f, 0.0007f));
    pixel.height += 0.0004f * SmoothStep(0.0125f, 0.0090f, edge) * SmoothStep(0.0f, 0.002f, edge);
    pixel.height -= 0.0005f * Groove(edge - 0.0110f, 0.0007f);

    // The seams down the sides of the body, where the front and back panels of the garment meet: the surface faces sideways there.
    const bool onTrunk = texel.tile == AtlasTile::Trunk;
    const bool onArm = texel.tile == AtlasTile::ArmLeft || texel.tile == AtlasTile::ArmRight;
    const float radius = (onTrunk ? 0.16f : (onArm ? 0.045f : 0.085f)) * stature / 1.75f;
    const float toSeam = std::abs(n.z) * radius;
    const float sidewaysFacing = SmoothStep(0.55f, 0.85f, std::abs(n.x));
    const float seam = Groove(toSeam, 0.0011f) * sidewaysFacing;
    pixel.height -= 0.0007f * seam;
    stitch = std::max(stitch, seam * (0.4f + 0.6f * SmoothStep(0.25f, 0.75f, std::abs(std::sin(p.y * 520.0f)))));

    if (spec.kind == GarmentKind::Jacket) {
        // The zip down the front: a groove with teeth.
        const float zip = Groove(std::abs(p.x), 0.0030f) * SmoothStep(0.2f, 0.7f, n.z);
        pixel.height -= 0.0008f * zip;
        pixel.height += 0.0004f * zip * SmoothStep(0.3f, 0.7f, std::sin(p.y * 1050.0f) * 0.5f + 0.5f);
        pixel.albedo = glm::mix(pixel.albedo, glm::vec3(0.30f), 0.75f * zip);
        pixel.roughness = glm::mix(pixel.roughness, 0.3f, zip);
    }
    if ((spec.kind == GarmentKind::Shoes || spec.kind == GarmentKind::Boots)) {
        // The outsole: rubber below the welt, with a groove where it joins the upper.
        const float sole = SmoothStep(0.017f, 0.013f, p.y);
        pixel.albedo = glm::mix(pixel.albedo, glm::vec3(0.025f), sole);
        pixel.roughness = glm::mix(pixel.roughness, 0.85f, sole);
        pixel.height -= 0.0006f * Groove(p.y - 0.0145f, 0.0008f);
        stitch = std::max(stitch, 0.7f * Groove(p.y - 0.0185f, 0.0006f));
    }
    pixel.albedo = glm::mix(pixel.albedo, thread, 0.8f * stitch);
    pixel.height -= 0.0003f * stitch;
    if (spec.kind == GarmentKind::Trousers) {
        // Denim fades where it is worn: at the hems and over the knees and thighs.
        const float worn = spec.wear * (0.5f * SmoothStep(0.05f, 0.0f, edge) + 0.5f * SmoothStep(0.3f, 0.7f, std::abs(n.z)) * Fbm3(p * 9.0f, 2, 17) * 0.5f + 0.1f);
        pixel.albedo = glm::mix(pixel.albedo, pixel.albedo * 1.5f + glm::vec3(0.02f), glm::clamp(worn, 0.0f, 0.6f));
    }
    pixel.albedo = glm::clamp(pixel.albedo, glm::vec3(0.0f), glm::vec3(1.0f));
    pixel.roughness = glm::clamp(pixel.roughness, 0.1f, 1.0f);
    return pixel;
}

GarmentTextures BakeGarmentTextures(const GarmentSpec& spec, const TriangleMesh& outerSurface, float stature, int tileSize, uint32_t seed, JobSystem* jobs) {
    // The material is made at a size that shows its weave, and no larger than the atlas tile asks for.
    material::MaterialSettings settings;
    settings.kind = spec.materialKind;
    settings.seed = seed + static_cast<uint32_t>(spec.kind) * 13u;
    settings.size = 256;
    settings.wear = spec.wear;
    settings.scanDetail = 0.5f;
    const material::MaterialMaps maps = material::GenerateMaterial(settings, jobs);
    const ClothSampler cloth(maps);

    const TexelField texels = BakeTexels(outerSurface, tileSize);
    std::vector<float> heights(static_cast<size_t>(texels.width) * texels.height, 0.0f);
    GarmentTextures textures;
    textures.tileSize = tileSize;
    textures.albedo = ByteImage(texels.width, texels.height, 4);
    textures.roughness = ByteImage(texels.width, texels.height, 1);
    ParallelEach(jobs, texels.height, [&](int y) {
        for (int x = 0; x < texels.width; ++x) {
            const Texel& texel = texels.At(x, y);
            uint8_t* albedo = textures.albedo.Pixel(x, y);
            ClothPixel pixel;
            if (texel.Used()) {
                pixel = PaintCloth(spec, cloth, texel, stature);
                heights[static_cast<size_t>(y) * texels.width + x] = pixel.height;
            } else {
                pixel.albedo = spec.tint;
            }
            const glm::vec3 srgb = glm::pow(pixel.albedo, glm::vec3(1.0f / 2.2f));
            albedo[0] = Byte(srgb.r);
            albedo[1] = Byte(srgb.g);
            albedo[2] = Byte(srgb.b);
            albedo[3] = 255;
            *textures.roughness.Pixel(x, y) = Byte(pixel.roughness);
        }
    });
    textures.normal = NormalMapFromHeights(texels, heights, tileSize, jobs);
    return textures;
}

Garment BuildGarment(const CharacterMesh& character, const TriangleMesh& body, const TriangleGrid& bodyOnly, const ObstacleList& bodyAndLayers, const GarmentSpec& spec, int tileSize,
                     JobSystem* jobs) {
    Garment garment;
    garment.kind = spec.kind;
    garment.material = spec.material;
    garment.materialKind = spec.materialKind;
    garment.tint = spec.tint;
    garment.thicknessMeters = spec.thickness;
    garment.clearanceMeters = spec.ease(glm::vec3(0.0f, character.skeleton.statureMeters * 0.6f, 0.0f));

    // Cut the part of the body that the garment covers; the field is also kept, as the distance from the edge.
    TriangleMesh covered = body;
    std::vector<float> field(covered.vertices.size());
    for (size_t v = 0; v < field.size(); ++v) {
        field[v] = spec.field(covered.vertices[v]);
        covered.vertices[v].attribute = glm::vec4(field[v], 0.0f, 0.0f, 0.0f);
    }
    const TriangleMesh surface = ClipByField(covered, field);
    if (surface.indices.empty()) {
        return garment;
    }
    TriangleMesh shell = MakeShell(surface, spec, bodyOnly, bodyAndLayers, jobs);
    // The shell before the unwrap: its vertices are shared by every triangle that meets there, so it is a watertight surface.
    garment.solid.positions.resize(shell.vertices.size());
    for (size_t v = 0; v < shell.vertices.size(); ++v) {
        garment.solid.positions[v] = shell.vertices[v].position;
    }
    garment.solid.indices = shell.indices;
    garment.solid.outerFirstVertex = static_cast<uint32_t>(surface.vertices.size());
    const TriangleMesh unwrapped = Unwrap(shell, character.skeleton, tileSize, -1);
    garment.mesh = ToRenderMesh(unwrapped);
    garment.vertexLayer.resize(unwrapped.vertices.size());
    for (size_t v = 0; v < unwrapped.vertices.size(); ++v) {
        garment.vertexLayer[v] = unwrapped.vertices[v].attribute.z > 0.5f ? 1 : 0;
    }
    garment.minimumClearance = MeasureClearance(bodyOnly, garment);

    // The texture is baked from the outer surface alone: its triangles are the ones with no vertex on the inner one.
    TriangleMesh outer;
    outer.vertices = unwrapped.vertices;
    for (size_t t = 0; t + 2 < unwrapped.indices.size(); t += 3) {
        const uint32_t a = unwrapped.indices[t];
        const uint32_t b = unwrapped.indices[t + 1];
        const uint32_t c = unwrapped.indices[t + 2];
        if (garment.vertexLayer[a] == 1 && garment.vertexLayer[b] == 1 && garment.vertexLayer[c] == 1) {
            outer.indices.insert(outer.indices.end(), {a, b, c});
        }
    }
    garment.textures = BakeGarmentTextures(spec, outer, character.skeleton.statureMeters, tileSize, character.parameters.seed, jobs);
    return garment;
}

// The outer surface of a garment as a surface of its own, with the grid that finds the nearest point of it, for the things that are
// made after it and must stay outside it (the garments above, the hair).
}  // namespace

OuterSurface::OuterSurface(const Garment& garment) : positions(garment.solid.positions) {
    for (size_t t = 0; t + 2 < garment.solid.indices.size(); t += 3) {
        const uint32_t* id = &garment.solid.indices[t];
        if (id[0] >= garment.solid.outerFirstVertex && id[1] >= garment.solid.outerFirstVertex && id[2] >= garment.solid.outerFirstVertex) {
            indices.insert(indices.end(), {id[0], id[1], id[2]});
        }
    }
    grid = std::make_unique<TriangleGrid>(positions, indices, 0.03f);
}

const char* GarmentName(GarmentKind kind) {
    switch (kind) {
        case GarmentKind::TShirt: return "t-shirt";
        case GarmentKind::Jacket: return "jacket";
        case GarmentKind::Trousers: return "trousers";
        case GarmentKind::Shorts: return "shorts";
        case GarmentKind::Shoes: return "shoes";
        case GarmentKind::Boots: return "boots";
    }
    return "garment";
}

float MeasureClearance(const CharacterMesh& character, const Garment& garment) {
    // Signed: a vertex inside the body is a negative clearance. Every vertex counts, on the inner and the outer surface and along
    // the edge strip, and so do the middles of the triangles of both surfaces.
    const TriangleGrid grid(character.positions, character.indices, 0.03f);
    return MeasureClearance(grid, garment);
}

float MeasureClearance(const TriangleGrid& character, const Garment& garment) {
    float least = 1e9f;
    for (const glm::vec3& p : garment.mesh.positions) {
        least = std::min(least, character.FindNearest(p, 0.06f).signedDistance);
    }
    for (size_t t = 0; t + 2 < garment.mesh.indices.size(); t += 3) {
        const uint32_t id[3] = {garment.mesh.indices[t], garment.mesh.indices[t + 1], garment.mesh.indices[t + 2]};
        if (garment.vertexLayer[id[0]] != garment.vertexLayer[id[1]] || garment.vertexLayer[id[1]] != garment.vertexLayer[id[2]]) {
            continue;  // a triangle of the edge strip: its corners are counted
        }
        const glm::vec3 middle = (garment.mesh.positions[id[0]] + garment.mesh.positions[id[1]] + garment.mesh.positions[id[2]]) / 3.0f;
        least = std::min(least, character.FindNearest(middle, 0.06f).signedDistance);
    }
    return least;
}

Outfit BuildOutfit(const CharacterMesh& character, const OutfitSettings& rawSettings, JobSystem* jobs) {
    OutfitSettings settings = rawSettings;
    settings.tileSize = std::clamp(settings.tileSize, 64, 2048);
    Outfit outfit;
    const BodyMeasures measures = MeasureBody(character.skeleton);
    const TriangleMesh body = ExtractPiece(character, 0);
    // Clearance is held against everything the character is made of (the body, the head and the hands), which is built once for all
    // the garments, and against the outer surface of each garment already made, so that each layer stands off the one below it.
    const TriangleGrid bodyOnly(character.positions, character.indices, 0.03f);
    std::vector<std::unique_ptr<OuterSurface>> lower;
    const uint32_t seed = character.parameters.seed;

    // From the inside out: the footwear is the first layer on the foot, and the trousers go over it; then the shirt, then the jacket.
    std::vector<GarmentKind> order;
    if (settings.feet == FootwearKind::Shoes) {
        order.push_back(GarmentKind::Shoes);
    } else if (settings.feet == FootwearKind::Boots) {
        order.push_back(GarmentKind::Boots);
    }
    if (settings.legs == LegwearKind::Trousers) {
        order.push_back(GarmentKind::Trousers);
    } else if (settings.legs == LegwearKind::Shorts) {
        order.push_back(GarmentKind::Shorts);
    }
    if (settings.top == TopKind::TShirt || settings.top == TopKind::TShirtAndJacket) {
        order.push_back(GarmentKind::TShirt);
    }
    if (settings.top == TopKind::Jacket || settings.top == TopKind::TShirtAndJacket) {
        order.push_back(GarmentKind::Jacket);
    }
    for (const GarmentKind kind : order) {
        ObstacleList obstacles{{&bodyOnly, false}};
        for (const auto& surface : lower) {
            obstacles.push_back({surface->grid.get(), true});
        }
        Garment garment = BuildGarment(character, body, bodyOnly, obstacles, MakeSpec(kind, measures, seed, settings.feet), settings.tileSize, jobs);
        if (garment.mesh.indices.empty()) {
            continue;
        }
        lower.push_back(std::make_unique<OuterSurface>(garment));
        if (kind == GarmentKind::Shoes || kind == GarmentKind::Boots) {
            for (const glm::vec3& p : garment.mesh.positions) {
                outfit.soleHeightMeters = std::max(outfit.soleHeightMeters, -p.y);
            }
        }
        outfit.garments.push_back(std::move(garment));
    }
    return outfit;
}

}  // namespace character
