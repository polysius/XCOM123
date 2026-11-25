#include "character/BodyShape.h"

#include <algorithm>
#include <cmath>

#include "character/Anthropometry.h"

namespace character {
namespace {

// The smooth minimum of two distances: the union of two shapes with the crease between them rounded off over a width k.
float SmoothMin(float a, float b, float k) {
    const float h = std::max(k - std::abs(a - b), 0.0f) / k;
    return std::min(a, b) - h * h * k * 0.25f;
}

// The smooth maximum: the intersection of two shapes, rounded the same way. Cutting one shape out of another is the
// smooth maximum of the first and the inside-out second.
float SmoothMax(float a, float b, float k) {
    return -SmoothMin(-a, -b, k);
}

float DistanceToBox(glm::vec3 point, glm::vec3 lower, glm::vec3 upper) {
    const glm::vec3 outside = glm::max(glm::max(lower - point, point - upper), glm::vec3(0.0f));
    return glm::length(outside);
}

// Everything a recipe for the body needs, worked out from the parameters once.
struct Proportions {
    float scale;      // stature relative to 1.75 m
    float female;     // 1 - masculinity
    float male;
    float muscle;     // muscularity, less with age
    float fat;        // 0 to 1, from the body mass index and the age
    float girth;
};

Proportions ProportionsOf(const CharacterParameters& parameters, float girth) {
    Proportions p;
    p.scale = parameters.statureMeters / 1.75f;
    p.male = parameters.masculinity;
    p.female = 1.0f - parameters.masculinity;
    const float ageEffect = std::clamp((parameters.ageYears - 30.0f) / 50.0f, 0.0f, 1.0f);
    p.muscle = parameters.muscularity * (1.0f - 0.35f * ageEffect);
    p.fat = std::clamp((BodyMassIndex(parameters) - 18.0f) / 16.0f + 0.12f * ageEffect, 0.0f, 1.0f);
    p.girth = girth;
    return p;
}

float Lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

class Builder {
public:
    Builder(const Skeleton& skeleton, const CharacterParameters& parameters, float girth, float minimumRadius)
        : skeleton(skeleton), p(ProportionsOf(parameters, girth)), stature(parameters.statureMeters), minimumRadius(minimumRadius) {}

    ShapeAssembler assembler;

    // Radii at a stature of 1.75 m, made to fit the stature and the build: `girthPower` says how much of the girth
    // factor a part takes (1 for the trunk and limbs, less for hands and feet, none for the head).
    float R(float radius, float girthPower = 1.0f) const { return std::max(radius * p.scale * std::pow(p.girth, girthPower), minimumRadius); }

    glm::vec3 At(Joint joint) const { return skeleton[joint].restPosition; }

    int Capsule(Joint bone, glm::vec3 start, glm::vec3 end, float sideStart, float depthStart, float sideEnd, float depthEnd,
                glm::vec3 depthDirection = glm::vec3(0.0f, 0.0f, 1.0f)) {
        return assembler.Capsule(bone, start, end, sideStart, depthStart, sideEnd, depthEnd, depthDirection);
    }

    int Ellipsoid(Joint bone, glm::vec3 centre, float radiusX, float radiusY, float radiusZ) {
        return assembler.Ellipsoid(bone, centre, radiusX, radiusY, radiusZ);
    }

    const Skeleton& skeleton;
    Proportions p;
    float stature;
    float minimumRadius;
};

}  // namespace

float BodyShape::PartDistance(size_t part, glm::vec3 point) const {
    const ShapePart& s = parts[part];
    const glm::vec3 along = s.end - s.start;
    const float lengthSquared = glm::dot(along, along);
    const float t = lengthSquared > 1e-10f ? std::clamp(glm::dot(point - s.start, along) / lengthSquared, 0.0f, 1.0f) : 0.0f;
    const glm::vec3 offset = point - (s.start + along * t);
    const float sideRadius = s.sideStart + (s.sideEnd - s.sideStart) * t;
    const float depthRadius = s.depthStart + (s.depthEnd - s.depthStart) * t;
    const float alongRadius = s.alongRadius > 0.0f ? s.alongRadius : 0.5f * (sideRadius + depthRadius);
    const glm::vec3 axis = lengthSquared > 1e-10f ? along / std::sqrt(lengthSquared) : glm::cross(s.side, s.depth);

    const float x = glm::dot(offset, s.side);
    const float z = glm::dot(offset, s.depth);
    const float y = glm::dot(offset, axis);
    // The ellipsoid f = 0 on the skin; dividing by the length of its gradient turns f into a distance near the skin.
    const float f = (x * x) / (sideRadius * sideRadius) + (z * z) / (depthRadius * depthRadius) + (y * y) / (alongRadius * alongRadius) - 1.0f;
    const glm::vec3 gradient(x / (sideRadius * sideRadius), z / (depthRadius * depthRadius), y / (alongRadius * alongRadius));
    const float slope = std::max(2.0f * glm::length(gradient), 1e-4f);
    return std::max(f / slope, -std::min(std::min(sideRadius, depthRadius), alongRadius));
}

float BodyShape::EvaluateNode(int index, glm::vec3 point) const {
    const Node& node = nodes[index];
    const float boxDistance = DistanceToBox(point, node.lower, node.upper);
    if (boxDistance > cullDistance) {
        return boxDistance;
    }
    if (node.part >= 0) {
        return PartDistance(static_cast<size_t>(node.part), point);
    }
    const float a = EvaluateNode(node.first, point);
    const float b = EvaluateNode(node.second, point);
    if (node.carve) {
        return node.smoothing > 0.0f ? SmoothMax(a, -b, node.smoothing) : std::max(a, -b);
    }
    return node.smoothing > 0.0f ? SmoothMin(a, b, node.smoothing) : std::min(a, b);
}

float BodyShape::Distance(glm::vec3 point) const {
    return EvaluateNode(root, point);
}

int ShapeAssembler::Capsule(Joint bone, glm::vec3 start, glm::vec3 end, float sideStart, float depthStart, float sideEnd, float depthEnd,
                            glm::vec3 depthDirection) {
    ShapePart part;
    part.bone = bone;
    part.start = start;
    part.end = end;
    part.sideStart = sideStart;
    part.depthStart = depthStart;
    part.sideEnd = sideEnd;
    part.depthEnd = depthEnd;
    const glm::vec3 axis = glm::normalize(end - start);
    const glm::vec3 depth = glm::normalize(depthDirection - axis * glm::dot(depthDirection, axis));
    part.depth = depth;
    part.side = glm::normalize(glm::cross(depth, axis));
    shape.parts.push_back(part);
    return static_cast<int>(shape.parts.size()) - 1;
}

int ShapeAssembler::Ellipsoid(Joint bone, glm::vec3 centre, float radiusX, float radiusY, float radiusZ) {
    ShapePart part;
    part.bone = bone;
    part.start = centre;
    part.end = centre;
    part.sideStart = part.sideEnd = radiusX;
    part.depthStart = part.depthEnd = radiusZ;
    part.alongRadius = radiusY;
    shape.parts.push_back(part);
    return static_cast<int>(shape.parts.size()) - 1;
}

int ShapeAssembler::Leaf(int part) {
    const ShapePart& sp = shape.parts[part];
    BodyShape::Node node;
    node.part = part;
    const float reach = std::max({sp.sideStart, sp.depthStart, sp.sideEnd, sp.depthEnd, sp.alongRadius}) * 1.15f;
    node.lower = glm::min(sp.start, sp.end) - glm::vec3(reach);
    node.upper = glm::max(sp.start, sp.end) + glm::vec3(reach);
    shape.nodes.push_back(node);
    return static_cast<int>(shape.nodes.size()) - 1;
}

int ShapeAssembler::Join(int first, int second, float smoothing) {
    BodyShape::Node node;
    node.first = first;
    node.second = second;
    node.smoothing = smoothing;
    node.lower = glm::min(shape.nodes[first].lower, shape.nodes[second].lower) - glm::vec3(smoothing);
    node.upper = glm::max(shape.nodes[first].upper, shape.nodes[second].upper) + glm::vec3(smoothing);
    shape.nodes.push_back(node);
    return static_cast<int>(shape.nodes.size()) - 1;
}

int ShapeAssembler::Carve(int from, int cut, float smoothing) {
    BodyShape::Node node;
    node.first = from;
    node.second = cut;
    node.carve = true;
    node.smoothing = smoothing;
    node.lower = shape.nodes[from].lower;  // cutting only takes away
    node.upper = shape.nodes[from].upper;
    shape.nodes.push_back(node);
    return static_cast<int>(shape.nodes.size()) - 1;
}

int ShapeAssembler::Merge(const std::vector<int>& parts, float smoothing) {
    int result = Leaf(parts[0]);
    for (size_t i = 1; i < parts.size(); ++i) {
        result = Join(result, Leaf(parts[i]), smoothing);
    }
    return result;
}

BodyShape ShapeAssembler::Finish(int root, float margin, float cullDistance) {
    shape.root = root;
    shape.cullDistance = cullDistance;
    shape.boundsMin = glm::vec3(1e9f);
    shape.boundsMax = glm::vec3(-1e9f);
    for (const ShapePart& part : shape.parts) {
        const float reach = std::max({part.sideStart, part.depthStart, part.sideEnd, part.depthEnd, part.alongRadius});
        shape.boundsMin = glm::min(shape.boundsMin, glm::min(part.start, part.end) - glm::vec3(reach));
        shape.boundsMax = glm::max(shape.boundsMax, glm::max(part.start, part.end) + glm::vec3(reach));
    }
    shape.boundsMin -= glm::vec3(margin);
    shape.boundsMax += glm::vec3(margin);
    return shape;
}

BodyShape BuildBodyShape(const Skeleton& skeleton, const CharacterParameters& parameters, const BodyShapeOptions& options) {
    const CharacterParameters clean = Sanitised(parameters);
    Builder b(skeleton, clean, options.girth, options.minimumRadius);
    const Proportions& p = b.p;
    const float s = p.scale;
    const float height = clean.statureMeters;
    namespace a = anthropometry;

    using J = Joint;
    const glm::vec3 up(0.0f, 1.0f, 0.0f);

    // --- the trunk -----------------------------------------------------------------------------------------------
    const glm::vec3 pelvisJoint = b.At(J::Pelvis);
    const int pelvis = b.Ellipsoid(J::Pelvis, pelvisJoint + glm::vec3(0.0f, 0.010f * s, -0.004f * s), b.R(0.140f + 0.012f * p.female + 0.01f * p.fat),
                                   b.R(0.095f), b.R(0.104f + 0.016f * p.fat));
    const float waistSide = Lerp(0.128f, 0.145f, p.male);
    const float chestSide = Lerp(0.143f, 0.162f, p.male);
    const int abdomen = b.Capsule(J::Spine1, b.At(J::Spine1) - glm::vec3(0.0f, 0.035f * s, 0.0f), b.At(J::Spine2), b.R(waistSide + 0.012f * p.fat),
                                  b.R(0.098f + 0.022f * p.fat), b.R(chestSide - 0.006f), b.R(0.108f));
    const glm::vec3 upperChest = glm::vec3(0.0f, 0.790f * height, 0.0f);
    const int chest = b.Capsule(J::Spine2, b.At(J::Spine2), upperChest, b.R(chestSide - 0.006f), b.R(0.108f), b.R(chestSide + 0.004f), b.R(0.116f));
    const glm::vec3 neckBase = b.At(J::Neck);
    const int neck = b.Capsule(J::Neck, neckBase - glm::vec3(0.0f, 0.020f * s, 0.0f), b.At(J::Head) + glm::vec3(0.0f, 0.012f * s, 0.006f * s),
                               b.R(Lerp(0.050f, 0.058f, p.male), 0.5f), b.R(Lerp(0.050f, 0.058f, p.male), 0.5f), b.R(Lerp(0.045f, 0.052f, p.male), 0.5f),
                               b.R(Lerp(0.045f, 0.052f, p.male), 0.5f));

    // The head does not grow with the body mass. Its size follows the stature.
    int skull = -1;
    int jaw = -1;
    if (options.includeHead) {
        skull = b.Ellipsoid(J::Head, glm::vec3(0.0f, 0.938f * height, 0.004f * s), 0.077f * s, 0.062f * height, 0.095f * s);
        jaw = b.Ellipsoid(J::Head, glm::vec3(0.0f, 0.900f * height, 0.016f * s), 0.062f * s, 0.040f * height, 0.080f * s);
    }

    std::vector<int> torsoParts = {pelvis, abdomen, chest};
    const int glutL = b.Ellipsoid(J::Pelvis, glm::vec3(0.075f * s, 0.502f * height, -0.076f * s), b.R(0.078f + 0.008f * p.female + 0.02f * p.fat),
                                  b.R(0.090f), b.R(0.086f + 0.010f * p.fat));
    const int glutR = b.Ellipsoid(J::Pelvis, glm::vec3(-0.075f * s, 0.502f * height, -0.076f * s), b.R(0.078f + 0.008f * p.female + 0.02f * p.fat),
                                  b.R(0.090f), b.R(0.086f + 0.010f * p.fat));
    torsoParts.insert(torsoParts.end(), {glutL, glutR});
    const float breast = p.female * (0.55f + 0.9f * p.fat);
    if (breast > 0.02f) {
        for (const float side : {1.0f, -1.0f}) {
            torsoParts.push_back(b.Ellipsoid(J::Spine2, glm::vec3(side * 0.086f * s, b.At(J::Spine2).y + 0.040f * s, 0.088f * s),
                                             b.R(0.074f * breast, 0.7f), b.R(0.070f * breast, 0.7f), b.R(0.060f * breast, 0.7f)));
        }
    }
    const float belly = p.fat * (0.4f + 0.6f * p.male);
    if (belly > 0.03f) {
        torsoParts.push_back(b.Ellipsoid(J::Spine1, glm::vec3(0.0f, b.At(J::Spine1).y - 0.010f * s, 0.062f * s), b.R(0.118f * belly + 0.03f),
                                         b.R(0.105f * belly + 0.03f), b.R(0.080f * belly + 0.02f)));
    }
    // The pectorals stay near the middle: out at the shoulder they would touch the arm and close the armpit into a tunnel.
    const float pectoral = p.male * (0.3f + 0.9f * p.muscle);
    if (pectoral > 0.05f) {
        for (const float side : {1.0f, -1.0f}) {
            torsoParts.push_back(b.Ellipsoid(J::Spine2, glm::vec3(side * 0.070f * s, 0.752f * height, 0.070f * s), b.R(0.078f * pectoral + 0.015f, 0.5f),
                                             b.R(0.050f * pectoral + 0.02f, 0.5f), b.R(0.030f * pectoral + 0.010f, 0.5f)));  // broad and flat
        }
    }

    // --- limbs, one side at a time -------------------------------------------------------------------------------------
    struct Side {
        std::vector<int> shoulderGirdle, upperArm, forearm, hand, thigh, calf, foot, toes;
    };
    Side sides[2];
    for (int which = 0; which < 2; ++which) {
        const bool left = which == 0;
        const float sign = left ? 1.0f : -1.0f;
        Side& out = sides[which];
        const J clavicle = left ? J::ClavicleL : J::ClavicleR;
        const J upperArm = left ? J::UpperArmL : J::UpperArmR;
        const J forearm = left ? J::ForearmL : J::ForearmR;
        const J hand = left ? J::HandL : J::HandR;
        const J thigh = left ? J::ThighL : J::ThighR;
        const J calf = left ? J::CalfL : J::CalfR;
        const J foot = left ? J::FootL : J::FootR;
        const J toes = left ? J::ToesL : J::ToesR;

        const glm::vec3 shoulder = b.At(upperArm);
        const glm::vec3 elbow = b.At(forearm);
        const glm::vec3 wrist = b.At(hand);
        const glm::vec3 armDirection = glm::normalize(wrist - shoulder);
        // The palm faces the thigh: the direction across the arm, in the plane of the body, towards the middle.
        const glm::vec3 medial = glm::normalize(sign * glm::cross(armDirection, glm::vec3(0.0f, 0.0f, 1.0f)));

        // The shoulder girdle: the slope of the trapezius from the neck to the point of the shoulder.
        out.shoulderGirdle.push_back(b.Capsule(clavicle, neckBase + glm::vec3(sign * 0.020f * s, -0.012f * s, 0.0f), shoulder + glm::vec3(sign * 0.012f * s, 0.0f, 0.0f),
                                               b.R(0.050f, 0.6f), b.R(0.074f, 0.6f), b.R(0.048f, 0.6f), b.R(0.064f, 0.6f)));

        const float armBuild = Lerp(0.88f, 1.0f, p.male) * (0.92f + 0.3f * p.muscle + 0.1f * p.fat);
        out.upperArm.push_back(b.Capsule(upperArm, shoulder, elbow, b.R(0.052f * armBuild), b.R(0.050f * armBuild), b.R(0.040f * armBuild), b.R(0.039f * armBuild)));
        out.upperArm.push_back(b.Ellipsoid(upperArm, shoulder + glm::vec3(sign * 0.012f * s, -0.008f * s, 0.0f), b.R(0.058f * (0.85f + 0.3f * p.muscle), 0.8f),
                                           b.R(0.064f * (0.85f + 0.3f * p.muscle), 0.8f), b.R(0.056f * (0.85f + 0.3f * p.muscle), 0.8f)));
        const glm::vec3 bicepsCentre = shoulder + (elbow - shoulder) * 0.38f + glm::vec3(0.0f, 0.0f, 0.010f * s);
        out.upperArm.push_back(b.Ellipsoid(upperArm, bicepsCentre, b.R(0.045f * (0.5f + p.muscle), 0.8f), b.R(0.085f), b.R(0.044f * (0.5f + p.muscle), 0.8f)));
        out.forearm.push_back(b.Capsule(forearm, elbow, wrist, b.R(0.041f * armBuild), b.R(0.040f * armBuild), b.R(0.026f * (0.9f + 0.1f * p.fat)), b.R(0.024f)));

        // The hand: a flat palm, the fingers together as one, and the thumb. The cross section is thin along `medial`. When
        // the body is cut without its hands, the forearm simply ends at the wrist.
        const float handSize = 0.35f;  // girth counts for little in the hands and feet
        if (options.includeHands) {
            const float handLength = a::kHandLength * height;
            const glm::vec3 palmEnd = wrist + armDirection * (0.46f * handLength);
            const glm::vec3 fingerEnd = wrist + armDirection * handLength;
            // The fingers are kept thicker than real ones: a part thinner than the grid the surface is cut from would break up.
            out.hand.push_back(b.Capsule(hand, wrist, palmEnd, b.R(0.037f, handSize), b.R(0.019f, handSize), b.R(0.042f, handSize), b.R(0.017f, handSize), medial));
            out.hand.push_back(b.Capsule(hand, palmEnd, fingerEnd - armDirection * (0.012f * s), b.R(0.040f, handSize), b.R(0.016f, handSize), b.R(0.029f, handSize), b.R(0.013f, handSize), medial));
            const glm::vec3 thumbStart = wrist + armDirection * (0.10f * handLength) + glm::vec3(0.0f, 0.0f, 0.020f * s);
            out.hand.push_back(b.Capsule(hand, thumbStart, thumbStart + armDirection * (0.28f * handLength) + glm::vec3(0.0f, 0.0f, 0.030f * s), b.R(0.018f, handSize),
                                         b.R(0.017f, handSize), b.R(0.014f, handSize), b.R(0.013f, handSize), medial));
        }

        const glm::vec3 hip = b.At(thigh);
        const glm::vec3 knee = b.At(calf);
        const glm::vec3 ankle = b.At(foot);
        const float legBuild = 1.0f + 0.03f * p.female + 0.08f * p.fat * p.female;
        out.thigh.push_back(b.Capsule(thigh, hip, knee, b.R(0.110f * legBuild), b.R(0.106f * legBuild), b.R(0.057f), b.R(0.056f)));
        out.thigh.push_back(b.Ellipsoid(thigh, hip + (knee - hip) * 0.42f + glm::vec3(0.0f, 0.0f, 0.012f * s), b.R(0.070f * (0.4f + p.muscle) * Lerp(0.85f, 1.0f, p.male), 0.8f),
                                        b.R(0.105f), b.R(0.068f * (0.4f + p.muscle) * Lerp(0.85f, 1.0f, p.male), 0.8f)));
        out.calf.push_back(b.Capsule(calf, knee, ankle, b.R(0.055f), b.R(0.054f), b.R(0.036f), b.R(0.036f)));
        out.calf.push_back(b.Ellipsoid(calf, knee + (ankle - knee) * 0.27f + glm::vec3(0.0f, 0.0f, -0.022f * s), b.R(0.048f * (0.75f + 0.5f * p.muscle), 0.8f), b.R(0.100f),
                                       b.R(0.050f * (0.75f + 0.5f * p.muscle), 0.8f)));

        // The foot lies on the floor: the centre line of each section is as high above y = 0 as the section is deep.
        const glm::vec3 ball = b.At(toes);
        const float heelDepth = 0.034f * s;
        const float ballDepth = 0.022f * s;
        // The smooth union with the leg adds a few millimetres under the sole, so the foot is lifted by as much.
        const float sole = 0.004f * s;
        const glm::vec3 heel(ankle.x, heelDepth + sole, -a::kAnkleToHeel * height + 0.035f * s);
        const glm::vec3 footEnd(ankle.x, ballDepth + sole, ball.z);
        out.foot.push_back(b.Capsule(foot, heel, footEnd, b.R(0.036f, handSize), heelDepth, b.R(0.046f, handSize), ballDepth, up));
        const glm::vec3 toeStart(ankle.x, 0.022f * s + sole, ball.z);
        const glm::vec3 toeEnd(ankle.x, 0.017f * s + sole, a::kAnkleToToe * height - 0.012f * s);
        out.toes.push_back(b.Capsule(toes, toeStart, toeEnd, b.R(0.044f, handSize), 0.022f * s, b.R(0.034f, handSize), 0.017f * s, up));
    }

    // --- the tree of unions -----------------------------------------------------------------------------------------------
    ShapeAssembler& tree = b.assembler;
    int body = tree.Merge(torsoParts, 0.050f * s);
    int legs[2] = {-1, -1};
    for (const int girdlePart : {sides[0].shoulderGirdle[0], sides[1].shoulderGirdle[0]}) {
        body = tree.Join(body, tree.Leaf(girdlePart), 0.030f * s);
    }
    body = tree.Join(body, tree.Leaf(neck), 0.040f * s);
    if (options.includeHead) {
        const int head = tree.Join(tree.Leaf(skull), tree.Leaf(jaw), 0.045f * s);
        body = tree.Join(body, head, 0.030f * s);
    }

    for (int which = 0; which < 2; ++which) {
        const Side& side = sides[which];
        // An arm grows out of the shoulder, and a leg out of the hip.
        const int upper = tree.Merge(side.upperArm, 0.030f * s);
        const int fore = tree.Merge(side.forearm, 0.020f * s);
        int arm = tree.Join(upper, fore, 0.025f * s);
        if (!side.hand.empty()) {
            arm = tree.Join(arm, tree.Merge(side.hand, 0.012f * s), 0.014f * s);
        }
        body = tree.Join(body, arm, 0.035f * s);

        const int upperLeg = tree.Merge(side.thigh, 0.030f * s);
        const int lowerLeg = tree.Merge(side.calf, 0.030f * s);
        const int footAndToes = tree.Join(tree.Merge(side.foot, 0.010f * s), tree.Merge(side.toes, 0.010f * s), 0.016f * s);
        legs[which] = tree.Join(tree.Join(upperLeg, lowerLeg, 0.032f * s), footAndToes, 0.020f * s);
    }
    // Each leg is joined to the body on its own, and the two are then united without smoothing, so that one thigh
    // never grows into the other and the body does not depend on which leg was joined first.
    const int withLeftLeg = tree.Join(body, legs[0], 0.045f * s);
    const int withRightLeg = tree.Join(body, legs[1], 0.045f * s);
    return tree.Finish(tree.Join(withLeftLeg, withRightLeg, 0.0f), 0.03f * s);
}

}  // namespace character
