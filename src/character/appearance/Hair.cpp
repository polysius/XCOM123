#include "character/appearance/Hair.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "character/BodyShape.h"
#include "character/DetailShapes.h"
#include "character/ParallelEach.h"
#include "character/appearance/HeadRegions.h"
#include "character/appearance/MeshTools.h"
#include "character/appearance/Noise3D.h"

namespace character {
namespace {

constexpr glm::vec3 kDown(0.0f, -1.0f, 0.0f);

// A root on the scalp or the face, on the surface of the head piece.
struct Root {
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
};

// Draws roots over the triangles of the head that a weight function marks, with the chance of a triangle proportional to its
// area and its weight, so the hair is even over the scalp and thins out at the edge of its region.
class RootSampler {
public:
    template <typename Weight>
    RootSampler(const TriangleMesh& head, const Weight& weight, float minimumWeight) : head(head) {
        float total = 0.0f;
        for (size_t t = 0; t + 2 < head.indices.size(); t += 3) {
            const glm::vec3& a = head.vertices[head.indices[t]].position;
            const glm::vec3& b = head.vertices[head.indices[t + 1]].position;
            const glm::vec3& c = head.vertices[head.indices[t + 2]].position;
            const float w = weight((a + b + c) / 3.0f);
            if (w < minimumWeight) {
                continue;
            }
            total += w * 0.5f * glm::length(glm::cross(b - a, c - a));
            triangles.push_back(static_cast<uint32_t>(t));
            cumulative.push_back(total);
        }
    }

    bool Empty() const { return triangles.empty(); }

    Root Draw(uint32_t seed, uint32_t salt, uint32_t index) const {
        const float pick = Random01(seed, salt, index * 3u) * cumulative.back();
        const size_t slot = static_cast<size_t>(std::lower_bound(cumulative.begin(), cumulative.end(), pick) - cumulative.begin());
        const uint32_t t = triangles[std::min(slot, triangles.size() - 1)];
        // A point of the triangle, uniform over its area.
        const float r1 = std::sqrt(Random01(seed, salt, index * 3u + 1u));
        const float r2 = Random01(seed, salt, index * 3u + 2u);
        const float wa = 1.0f - r1;
        const float wb = r1 * (1.0f - r2);
        const float wc = r1 * r2;
        const MeshVertex& a = head.vertices[head.indices[t]];
        const MeshVertex& b = head.vertices[head.indices[t + 1]];
        const MeshVertex& c = head.vertices[head.indices[t + 2]];
        Root root;
        root.position = wa * a.position + wb * b.position + wc * c.position;
        root.normal = glm::normalize(wa * a.normal + wb * b.normal + wc * c.normal);
        return root;
    }

private:
    const TriangleMesh& head;
    std::vector<uint32_t> triangles;  // the first index of each triangle that may carry roots
    std::vector<float> cumulative;
};

// What a lock does as it grows.
struct GrowthPlan {
    Root root;
    glm::vec3 flow{0.0f, 0.0f, 1.0f};  // along the surface, away from the crown
    float length = 0.05f;
    int points = 6;
    float gravity = 0.0f;     // 0 = grows out along the surface, 1 = hangs straight down
    float stiffness = 0.5f;   // how fast the direction follows what the style wants
    float lift = 0.0f;        // how far above the skin it starts, metres
    bool tied = false;
    glm::vec3 tie{0.0f};
    float wave = 0.0f;        // sideways wander in metres
    uint32_t salt = 0;
};

// The surface that locks must not enter: the head and, for long hair, the neck and body.
class Obstacle {
public:
    Obstacle(const BodyShape& head, const BodyShape* body, std::vector<const TriangleGrid*> garments) : head(head), body(body), garments(std::move(garments)) {}

    // Negative inside the head, the body or a garment. A lock lies on the outside of the cloth: it is held outside the outer surface of
    // every garment, where a beard meets the collar of a jacket and long hair the shoulders of one.
    float Distance(glm::vec3 p) const {
        float d = head.Distance(p);
        if (body != nullptr) {
            d = std::min(d, body->Distance(p));
        }
        return std::min(d, ClothDistance(p));
    }

    bool HasCloth() const { return !garments.empty(); }

    // Negative inside a garment; large when no garment is near.
    float ClothDistance(glm::vec3 p) const {
        float d = 1e9f;
        for (const TriangleGrid* garment : garments) {
            const TriangleGrid::Nearest nearest = garment->FindNearest(p, 0.03f);
            if (nearest.distance < 1e8f) {
                d = std::min(d, nearest.signedDistance);
            }
        }
        return d;
    }

    glm::vec3 Gradient(glm::vec3 p) const {
        constexpr float e = 0.0015f;
        const glm::vec3 g(Distance(p + glm::vec3(e, 0, 0)) - Distance(p - glm::vec3(e, 0, 0)), Distance(p + glm::vec3(0, e, 0)) - Distance(p - glm::vec3(0, e, 0)),
                          Distance(p + glm::vec3(0, 0, e)) - Distance(p - glm::vec3(0, 0, e)));
        return glm::length(g) > 1e-9f ? glm::normalize(g) : glm::vec3(0.0f, 1.0f, 0.0f);
    }

private:
    const BodyShape& head;
    const BodyShape* body;
    std::vector<const TriangleGrid*> garments;
};

std::vector<glm::vec3> Grow(const GrowthPlan& plan, const Obstacle& obstacle) {
    std::vector<glm::vec3> points;
    // The root is on the skin itself; the lock lifts away from it over the first two steps.
    glm::vec3 position = plan.root.position;
    points.push_back(position);
    // Hair lies over the scalp more than it stands up from it: the longer it is, the more it follows the flow.
    glm::vec3 direction = glm::normalize(plan.root.normal * 0.30f + plan.flow * 0.70f);
    const float step = plan.length / static_cast<float>(plan.points - 1);
    bool pastTie = false;
    for (int k = 1; k < plan.points; ++k) {
        const float t = static_cast<float>(k) / static_cast<float>(plan.points - 1);
        glm::vec3 wanted;
        if (plan.tied) {
            // To the tie, then straight down with a little splay.
            const glm::vec3 toTie = plan.tie - position;
            pastTie = pastTie || glm::length(toTie) < step * 1.3f;
            wanted = pastTie ? glm::normalize(kDown * 0.92f + glm::vec3(0.0f, 0.0f, -0.12f) + plan.flow * 0.08f) : glm::normalize(toTie);
        } else {
            const float gravity = plan.gravity * SmoothStep(0.0f, 0.85f, t);
            const glm::vec3 outward = glm::normalize(plan.root.normal * 0.35f + plan.flow * 0.65f);
            wanted = glm::normalize(outward * (1.0f - gravity) + kDown * gravity + plan.flow * 0.1f * gravity);
        }
        if (plan.wave > 0.0f) {
            const glm::vec3 sideways = glm::normalize(glm::cross(direction, glm::vec3(0.0f, 1.0f, 0.0f)) + glm::vec3(1e-4f));
            wanted = glm::normalize(wanted + sideways * (plan.wave / step) * 0.35f * std::sin(t * 9.0f + static_cast<float>(plan.salt % 97)));
        }
        direction = glm::normalize(glm::mix(direction, wanted, plan.stiffness));
        // A step is made in pieces of at most 8 mm, each checked, so that a long step cannot jump across a thin wall of cloth unseen.
        const int pieces = std::max(1, static_cast<int>(std::ceil(step / 0.008f)));
        const float clearance = 0.0012f + plan.lift * std::min(1.0f, static_cast<float>(k) / 2.0f);
        bool tuckedIn = false;
        for (int piece = 0; piece < pieces && !tuckedIn; ++piece) {
            position += direction * (step / static_cast<float>(pieces));
            // A lock never enters the head or the body: push it out along the surface, a few strands' thickness above it.
            for (int pass = 0; pass < 3; ++pass) {
                const float distance = obstacle.Distance(position);
                if (distance >= clearance) {
                    break;
                }
                position += obstacle.Gradient(position) * (clearance - distance);
            }
            // A lock that is still inside a garment after being pushed (a beard that has reached the inside of a collar) is tucked in:
            // it ends where it meets the cloth.
            tuckedIn = obstacle.ClothDistance(position) < -0.0005f;
        }
        if (tuckedIn) {
            break;
        }
        points.push_back(position);
    }
    return points;
}

glm::vec3 StrandColour(const PersonTraits& traits, uint32_t seed, uint32_t salt, uint32_t index, float lightness = 1.0f) {
    const bool grey = Random01(seed, salt, index * 5u + 3u) < traits.greyFraction;
    const glm::vec3 base = grey ? glm::vec3(0.14f + 0.16f * Random01(seed, salt, index * 5u + 4u)) : traits.hairColour;
    const float variation = 0.75f + 0.5f * Random01(seed, salt, index * 5u + 2u);
    return base * variation * lightness;
}

// Grows a lock for every plan, in parallel, and adds the ribbons in order, so the result does not depend on the threads.
void GrowAndAdd(RibbonSet& set, const std::vector<GrowthPlan>& plans, const Obstacle& obstacle, const PersonTraits& traits, uint32_t seed, float rootWidth, float tipWidth,
                JobSystem* jobs) {
    std::vector<std::vector<glm::vec3>> locks(plans.size());
    ParallelEach(jobs, static_cast<int>(plans.size()), [&](int i) { locks[i] = Grow(plans[i], obstacle); });
    const size_t firstNewVertex = set.mesh.positions.size();
    const size_t firstNewRibbon = set.RibbonCount();
    for (size_t i = 0; i < plans.size(); ++i) {
        if (locks[i].size() < 2) {
            continue;  // a lock that started inside a garment, or ended at once, is not drawn
        }
        const glm::vec3 colour = StrandColour(traits, seed, plans[i].salt, static_cast<uint32_t>(i));
        // The face direction of a ribbon is the way the head faces where the lock lies, a third of the way along.
        // A short lock, and a lock of a beard, lies along the skin it grows from; the gradient near a collar or a jaw is not steady
        // enough to orient a card by.
        const glm::vec3 faceDirection = plans[i].length < 0.06f ? plans[i].root.normal : obstacle.Gradient(locks[i][locks[i].size() / 3]);
        // The strand texture repeats across the ribbon; each ribbon starts at another place in it.
        const float uOffset = Random01(seed, plans[i].salt + 7u, static_cast<uint32_t>(i));
        AddRibbon(set, locks[i], faceDirection, rootWidth, tipWidth, colour * 0.55f, colour * 1.1f, Joint::Head, uOffset);
    }
    // The edges of a ribbon that lies on a curved surface can dip into it; lift every vertex that is below the surface.
    constexpr float kEdgeClearance = 0.0006f;
    ParallelEach(jobs, static_cast<int>(set.mesh.positions.size() - firstNewVertex), [&](int i) {
        glm::vec3& p = set.mesh.positions[firstNewVertex + i];
        for (int pass = 0; pass < 2; ++pass) {
            const float distance = obstacle.Distance(p);
            if (distance >= kEdgeClearance) {
                break;
            }
            p += obstacle.Gradient(p) * (kEdgeClearance - distance);
        }
    });
    // A card that still has a vertex inside a garment (the push could not get it out of a fold of the cloth) is left out: hair that
    // goes through a jacket is worse than a little less hair.
    if (obstacle.HasCloth()) {
        std::vector<char> drop(set.RibbonCount(), 0);
        bool any = false;
        for (size_t r = firstNewRibbon; r < set.RibbonCount(); ++r) {
            const uint32_t first = set.firstVertex[r];
            const uint32_t end = r + 1 < set.RibbonCount() ? set.firstVertex[r + 1] : static_cast<uint32_t>(set.mesh.positions.size());
            for (uint32_t v = first; v < end && drop[r] == 0; ++v) {
                if (obstacle.ClothDistance(set.mesh.positions[v]) < -0.0010f) {
                    drop[r] = 1;
                    any = true;
                }
            }
        }
        if (any) {
            RemoveRibbons(set, drop);
        }
    }
}

struct StyleSetting {
    int count;
    float lengthMin;   // metres at 1.75 m of stature
    float lengthMax;
    int points;
    float gravity;
    float stiffness;
    float lift;
    float wave;
    float width;
};

StyleSetting SettingOf(HairStyle style) {
    switch (style) {
        // A ribbon is a card with a strand texture on it (see MakeHairTextures), so it is wider than a lock and there are fewer.
        case HairStyle::Short:
            return {6000, 0.008f, 0.024f, 4, 0.35f, 0.6f, 0.0008f, 0.0f, 0.0050f};
        case HairStyle::Medium:
            return {4200, 0.070f, 0.110f, 9, 0.80f, 0.45f, 0.0035f, 0.0020f, 0.0085f};
        case HairStyle::Long:
            return {4600, 0.300f, 0.380f, 16, 1.00f, 0.40f, 0.0040f, 0.0040f, 0.0090f};
        case HairStyle::Tied:
            return {4200, 0.250f, 0.300f, 16, 0.0f, 0.55f, 0.0030f, 0.0015f, 0.0085f};
        default:
            return {0, 0, 0, 2, 0, 0, 0, 0, 0};
    }
}

void GrowScalpHair(HairSet& set, const TriangleMesh& head, const HeadLandmarks& landmarks, const PersonTraits& traits, const HairSettings& settings, const Obstacle& obstacle,
                   uint32_t seed, JobSystem* jobs) {
    const StyleSetting style = SettingOf(settings.style);
    if (style.count == 0) {
        return;
    }
    const float scale = landmarks.scale;
    const RootSampler sampler(head, [&](glm::vec3 p) { return ScalpWeight(landmarks, traits, p); }, 0.05f);
    if (sampler.Empty()) {
        return;
    }
    // Cropped hair grows out from a whorl at the back of the crown. Longer hair is combed back from the forehead, and parted a
    // little off the middle, so that it falls behind and beside the face and never over it.
    const bool combedBack = settings.style != HairStyle::Short;
    const glm::vec3 whorl = (combedBack ? glm::vec3(0.0f, 1.690f, 0.095f) : glm::vec3(0.0f, 1.735f, -0.040f)) * scale;
    const float partX = 0.018f * scale;
    const glm::vec3 tie = glm::vec3(0.0f, 1.690f, -0.112f) * scale;
    const int count = static_cast<int>(style.count * traits.hairDensity * std::clamp(settings.detail, 0.25f, 1.0f));
    std::vector<GrowthPlan> plans(count);
    for (int i = 0; i < count; ++i) {
        GrowthPlan& plan = plans[i];
        plan.salt = 400u;
        plan.root = sampler.Draw(seed, plan.salt, static_cast<uint32_t>(i));
        // Roots thin out near the edge of the hairy region, so that the hairline is soft.
        const float weight = ScalpWeight(landmarks, traits, plan.root.position);
        if (weight < 0.25f || Random01(seed, 410u, static_cast<uint32_t>(i)) > weight * 1.1f) {
            plan.points = 0;
            continue;
        }
        glm::vec3 away = plan.root.position - whorl;
        if (combedBack) {
            const float front = SmoothStep(0.0f, 0.06f, plan.root.position.z / scale);
            away.x += (plan.root.position.x > partX ? 1.0f : -1.0f) * 0.045f * scale * front;
        }
        const glm::vec3 alongSurface = away - plan.root.normal * glm::dot(away, plan.root.normal);
        plan.flow = glm::length(alongSurface) > 1e-5f ? glm::normalize(alongSurface) : glm::vec3(0.0f, 0.0f, -1.0f);
        const float height = plan.root.position.y / scale;
        const float randomLength = Random01(seed, 420u, static_cast<uint32_t>(i));
        float length = glm::mix(style.lengthMin, style.lengthMax, randomLength);
        if (settings.style == HairStyle::Short) {
            // Cropped shorter at the sides and the nape than on top.
            length *= glm::mix(0.45f, 1.0f, SmoothStep(1.650f, 1.715f, height));
        }
        plan.length = length * scale;
        plan.points = style.points;
        plan.gravity = style.gravity;
        plan.stiffness = style.stiffness;
        plan.lift = style.lift * scale * Random01(seed, 430u, static_cast<uint32_t>(i)) + 0.0008f * scale;
        plan.wave = style.wave * scale * (Random01(seed, 440u, static_cast<uint32_t>(i)) - 0.5f) * 2.0f;
        if (settings.style == HairStyle::Tied) {
            plan.tied = true;
            plan.tie = tie + glm::vec3(Random01(seed, 450u, static_cast<uint32_t>(i)) - 0.5f, Random01(seed, 451u, static_cast<uint32_t>(i)) - 0.5f, 0.0f) * 0.012f * scale;
            plan.length = (glm::distance(plan.root.position, plan.tie) * 1.08f + length) ;
        }
    }
    plans.erase(std::remove_if(plans.begin(), plans.end(), [](const GrowthPlan& plan) { return plan.points < 2; }), plans.end());
    GrowAndAdd(set.scalp, plans, obstacle, traits, seed, style.width * scale, 0.0016f * scale, jobs);
}

// A raycast against the head from the front at (x, y): where the surface of the face is.
bool FrontOfHead(const TriangleGrid& grid, float x, float y, glm::vec3& hit) {
    const glm::vec3 origin(x, y, 0.25f);
    const float distance = grid.Raycast(origin, glm::vec3(0.0f, 0.0f, -1.0f), 0.30f);
    if (distance < 0.0f) {
        return false;
    }
    hit = origin + glm::vec3(0.0f, 0.0f, -distance);
    return true;
}

void GrowEyebrows(HairSet& set, const TriangleGrid& grid, const HeadLandmarks& landmarks, const CharacterParameters& parameters, const PersonTraits& traits) {
    const float scale = landmarks.scale;
    const int count = 150 + static_cast<int>(70.0f * parameters.masculinity);
    for (int i = 0; i < 2; ++i) {
        const float side = i == 0 ? 1.0f : -1.0f;
        for (int j = 0; j < count; ++j) {
            const uint32_t index = static_cast<uint32_t>(i * 1000 + j);
            const float t = std::pow(Random01(parameters.seed, 500u, index), 1.15f);
            // The arch: it rises from the inner end to a peak about halfway, then drops away to the tail.
            const float x = side * landmarks.headWidth * (0.0115f + 0.0400f * t) * scale;
            const float thickness = glm::mix(0.0100f, 0.0040f, t) * (0.7f + 0.5f * parameters.masculinity);
            const float y = (1.6535f + 0.0105f * std::sin(3.14159f * t * 0.85f + 0.10f) + (Random01(parameters.seed, 501u, index) - 0.5f) * thickness) * scale;
            glm::vec3 start;
            if (!FrontOfHead(grid, x, y, start)) {
                continue;
            }
            // The hairs of the inner end stand up; towards the tail they lie out and down.
            const glm::vec3 outward(side, 0.0f, 0.0f);
            const glm::vec3 heading = glm::normalize(outward * (0.55f + 0.45f * t) + glm::vec3(0.0f, 0.55f * (1.0f - t) - 0.15f * t, 0.0f));
            const float length = (0.0055f + 0.0030f * Random01(parameters.seed, 502u, index)) * scale;
            std::vector<glm::vec3> points{start + glm::vec3(0.0f, 0.0f, 0.0005f * scale)};
            for (int k = 1; k <= 3; ++k) {
                glm::vec3 next = points.back() + heading * (length / 3.0f);
                glm::vec3 surface;
                if (FrontOfHead(grid, next.x, next.y, surface)) {
                    next.z = surface.z + 0.0007f * scale;
                }
                points.push_back(next);
            }
            const glm::vec3 colour = StrandColour(traits, parameters.seed, 510u, index, 0.9f);
            AddRibbon(set.eyebrows, points, glm::vec3(0.0f, 0.0f, 1.0f), 0.00070f * scale, 0.00020f * scale, colour * 0.6f, colour, Joint::Head);
        }
    }
}

void GrowFacialHair(HairSet& set, const TriangleMesh& head, const HeadLandmarks& landmarks, const PersonTraits& traits, const HairSettings& settings, const Obstacle& obstacle,
                    uint32_t seed, JobSystem* jobs) {
    if (settings.facialHair == FacialHair::None) {
        return;
    }
    const bool beard = settings.facialHair == FacialHair::Beard;
    const float scale = landmarks.scale;
    const RootSampler sampler(head, [&](glm::vec3 p) { return BeardWeight(landmarks, p); }, 0.05f);
    if (sampler.Empty()) {
        return;
    }
    const int count = static_cast<int>((beard ? 9000 : 6000) * std::clamp(settings.detail, 0.25f, 1.0f));
    std::vector<GrowthPlan> plans;
    for (int i = 0; i < count; ++i) {
        GrowthPlan plan;
        plan.salt = 600u;
        plan.root = sampler.Draw(seed, plan.salt, static_cast<uint32_t>(i));
        const float weight = BeardWeight(landmarks, plan.root.position);
        if (weight < 0.40f || Random01(seed, 610u, static_cast<uint32_t>(i)) > weight * 1.1f) {
            continue;
        }
        const glm::vec3 q = plan.root.position / scale;
        // A beard is longest at the chin and shorter on the cheeks and the upper lip.
        const float chin = SmoothStep(1.600f, 1.545f, q.y);
        const float length = beard ? glm::mix(0.008f, 0.024f, chin) * (0.7f + 0.6f * Random01(seed, 620u, static_cast<uint32_t>(i))) : 0.0028f;
        plan.flow = glm::normalize(kDown + glm::vec3(0.0f, 0.0f, 0.4f) * (1.0f - chin));
        plan.length = length * scale;
        plan.points = beard ? 5 : 3;
        plan.gravity = beard ? 0.55f : 0.0f;
        plan.stiffness = 0.5f;
        plan.lift = 0.0006f * scale;
        plan.wave = beard ? 0.002f * scale * (Random01(seed, 630u, static_cast<uint32_t>(i)) - 0.5f) : 0.0f;
        plans.push_back(plan);
    }
    GrowAndAdd(set.facial, plans, obstacle, traits, seed, (beard ? 0.0046f : 0.0009f) * scale, (beard ? 0.0016f : 0.0004f) * scale, jobs);
}

}  // namespace

HairTextures MakeHairTextures(uint32_t seed, int width, int height) {
    constexpr int kStrands = 7;  // across the card
    HairTextures textures;
    textures.strands = ByteImage(width, height, 4);
    textures.normal = ByteImage(width, height, 4);
    struct Strand {
        float centre;
        float halfWidth;
        float length;  // how far along the card (0 root to 1 tip) the strand reaches
        float shade;
        float sway;
        float phase;
    };
    std::vector<Strand> strands(kStrands);
    for (int k = 0; k < kStrands; ++k) {
        const uint32_t i = static_cast<uint32_t>(k);
        strands[k].centre = (static_cast<float>(k) + 0.5f + 0.3f * (Random01(seed, 1100u, i) - 0.5f)) / kStrands;
        strands[k].halfWidth = (0.30f + 0.12f * Random01(seed, 1101u, i)) / kStrands;
        strands[k].length = 0.55f + 0.45f * Random01(seed, 1102u, i);
        strands[k].shade = 0.70f + 0.45f * Random01(seed, 1103u, i);
        strands[k].sway = (0.10f + 0.20f * Random01(seed, 1104u, i)) / kStrands;
        strands[k].phase = 6.28f * Random01(seed, 1105u, i);
    }
    for (int y = 0; y < height; ++y) {
        const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(height);
        for (int x = 0; x < width; ++x) {
            const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(width);
            float alpha = 0.0f;
            float shade = 0.0f;
            float slope = 0.0f;  // across the strand, -1 to 1, for the normal
            for (const Strand& strand : strands) {
                if (v > strand.length) {
                    continue;
                }
                // A strand wanders a little as it goes, and narrows to a point at its end.
                const float taper = 1.0f - 0.85f * std::pow(v / strand.length, 2.0f);
                const float halfWidth = strand.halfWidth * taper;
                float across = u - (strand.centre + strand.sway * std::sin(v * 7.0f + strand.phase));
                across -= std::round(across);  // the texture repeats across
                if (std::abs(across) < halfWidth) {
                    const float edge = std::abs(across) / halfWidth;
                    alpha = 1.0f;
                    // A lighter line down the middle of each strand, where it catches the light, and a little noise along it.
                    shade = strand.shade * (0.85f + 0.25f * (1.0f - edge)) * (0.92f + 0.16f * Random01(static_cast<uint32_t>(x), static_cast<uint32_t>(y / 8), seed));
                    slope = across / halfWidth;
                }
            }
            uint8_t* albedo = textures.strands.Pixel(x, y);
            const uint8_t grey = static_cast<uint8_t>(std::lround(glm::clamp(shade * 0.85f, 0.0f, 1.0f) * 255.0f));
            albedo[0] = albedo[1] = albedo[2] = grey;
            albedo[3] = alpha > 0.5f ? 255 : 0;
            const glm::vec3 normal = glm::normalize(glm::vec3(0.9f * slope, 0.0f, 1.0f));
            uint8_t* n = textures.normal.Pixel(x, y);
            n[0] = static_cast<uint8_t>(std::lround((0.5f + 0.5f * normal.x) * 255.0f));
            n[1] = 128;
            n[2] = static_cast<uint8_t>(std::lround((0.5f + 0.5f * normal.z) * 255.0f));
            n[3] = 255;
        }
    }
    return textures;
}

HairSet BuildHair(const CharacterMesh& character, const PersonTraits& traits, const HairSettings& settings, const Outfit* outfit, JobSystem* jobs) {
    HairSet set;
    set.scalp.strandTexture = true;
    set.facial.strandTexture = settings.facialHair == FacialHair::Beard;
    set.textures = MakeHairTextures(character.parameters.seed);
    size_t headPiece = character.pieces.size();
    for (size_t piece = 0; piece < character.pieces.size(); ++piece) {
        if (character.pieces[piece].kind == MeshPieceKind::Head) {
            headPiece = piece;
        }
    }
    if (headPiece == character.pieces.size()) {
        return set;
    }
    const CharacterParameters& parameters = character.parameters;
    const HeadLandmarks landmarks = ComputeHeadLandmarks(parameters);
    const TriangleMesh head = ExtractPiece(character, headPiece);
    std::vector<glm::vec3> headPositions(head.vertices.size());
    for (size_t v = 0; v < headPositions.size(); ++v) {
        headPositions[v] = head.vertices[v].position;
    }
    const TriangleGrid grid(headPositions, head.indices, 0.01f);

    const BodyShape headShape = BuildHeadShape(parameters);
    // Long hair also lies on the neck and the shoulders. The body is built as the generator built it, without head and hands.
    // The thinnest part the generator allowed is the same fraction of its grid as when it was cut.
    const BodyShape bodyShape = BuildBodyShape(character.skeleton, parameters, {.girth = character.girth, .minimumRadius = 0.55f * character.voxelMeters, .includeHead = false, .includeHands = false});
    const bool longHair = settings.style == HairStyle::Medium || settings.style == HairStyle::Long || settings.style == HairStyle::Tied;
    // Hair lies outside every garment: the outer surfaces are found by their own grids, built here once for all the strands.
    std::vector<std::unique_ptr<OuterSurface>> cloth;
    std::vector<const TriangleGrid*> clothGrids;
    if (outfit != nullptr) {
        for (const Garment& garment : outfit->garments) {
            cloth.push_back(std::make_unique<OuterSurface>(garment));
            clothGrids.push_back(cloth.back()->grid.get());
        }
    }
    const Obstacle obstacle(headShape, longHair ? &bodyShape : nullptr, clothGrids);
    const Obstacle faceObstacle(headShape, &bodyShape, clothGrids);

    GrowScalpHair(set, head, landmarks, traits, settings, obstacle, parameters.seed, jobs);
    GrowEyebrows(set, grid, landmarks, parameters, traits);
    GrowFacialHair(set, head, landmarks, traits, settings, faceObstacle, parameters.seed, jobs);
    return set;
}

}  // namespace character
