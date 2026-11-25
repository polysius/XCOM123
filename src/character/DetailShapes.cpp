#include "character/DetailShapes.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "character/Anthropometry.h"

namespace character {
namespace {

constexpr float kDegreesToRadians = 0.01745329252f;

// A deterministic factor close to 1 from the seed: 1 + amount * x with x between -1 and 1. Every feature has its own salt,
// so that a long nose and a full lip are not tied together.
float Variation(uint32_t seed, uint32_t salt, float amount) {
    uint32_t h = seed * 0x9E3779B1u ^ (salt + 0x7F4A7C15u) * 0x85EBCA6Bu;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    h *= 0x297A2D39u;
    h ^= h >> 15;
    return 1.0f + amount * (static_cast<float>(h >> 8) * (2.0f / 16777216.0f) - 1.0f);
}

}  // namespace

BodyShape BuildHeadShape(const CharacterParameters& parameters) {
    const CharacterParameters clean = Sanitised(parameters);
    const float s = clean.statureMeters / 1.75f;
    const float male = clean.masculinity;
    const float female = 1.0f - clean.masculinity;
    const float age = std::clamp((clean.ageYears - 30.0f) / 50.0f, 0.0f, 1.0f);
    const uint32_t seed = clean.seed;

    // The features differ with sex and with the person. Every number below is in metres for a person of 1.75 m, as
    // measured on a face: the points are given where they sit on the head, from the chin at 1.5225 m up to the crown at
    // 1.75 m, and from the middle of the head forwards.
    const float headWidth = (1.0f - 0.03f * female) * Variation(seed, 11, 0.04f);
    const float jawWidth = (0.92f + 0.14f * male) * Variation(seed, 12, 0.06f);
    const float chinSize = (0.90f + 0.18f * male) * Variation(seed, 13, 0.10f);
    const float browSize = (0.70f + 0.65f * male) * Variation(seed, 14, 0.12f);
    const float cheekSize = (1.10f - 0.15f * male) * Variation(seed, 15, 0.08f);
    const float noseSize = (0.96f + 0.08f * male) * Variation(seed, 16, 0.06f);
    const float noseLength = Variation(seed, 17, 0.08f) * (1.0f + 0.04f * age);
    const float lipSize = (0.85f + 0.35f * female) * Variation(seed, 18, 0.12f) * (1.0f - 0.25f * age);
    const float earSize = Variation(seed, 19, 0.08f) * (1.0f + 0.06f * age);

    const auto P = [&](float x, float y, float z) { return glm::vec3(x * s, y * s, z * s); };
    ShapeAssembler tree;
    constexpr Joint kHead = Joint::Head;
    const glm::vec3 across(1.0f, 0.0f, 0.0f);

    // The skull and the mass of the face: the top of the crown is at the stature.
    const int cranium = tree.Ellipsoid(kHead, P(0.0f, 1.655f, -0.010f), 0.077f * headWidth * s, 0.095f * s, 0.095f * headWidth * s);
    const int occiput = tree.Ellipsoid(kHead, P(0.0f, 1.640f, -0.040f), 0.064f * headWidth * s, 0.070f * s, 0.060f * headWidth * s);
    const int forehead = tree.Ellipsoid(kHead, P(0.0f, 1.676f, 0.052f), 0.060f * headWidth * s, 0.044f * s, 0.040f * headWidth * s);
    const int faceMass = tree.Ellipsoid(kHead, P(0.0f, 1.590f, 0.030f), 0.062f * headWidth * s, 0.072f * s, 0.060f * headWidth * s);
    int head = tree.Merge({cranium, occiput, forehead, faceMass}, 0.030f * s);

    // The jaw runs from below the ear to the chin on each side.
    std::vector<int> jawParts;
    for (const float side : {1.0f, -1.0f}) {
        jawParts.push_back(tree.Capsule(kHead, P(side * 0.056f * headWidth, 1.598f, -0.012f), P(side * 0.019f * jawWidth, 1.540f, 0.062f),
                                        0.0215f * jawWidth * s, 0.021f * s, 0.017f * jawWidth * s, 0.017f * s, across));
    }
    jawParts.push_back(tree.Ellipsoid(kHead, P(0.0f, 1.539f, 0.070f), 0.020f * chinSize * s, 0.016f * s, 0.020f * chinSize * s));
    head = tree.Join(head, tree.Merge(jawParts, 0.020f * s), 0.030f * s);

    // The brow ridge and the cheekbones.
    const int brow = tree.Capsule(kHead, P(-0.047f * headWidth, 1.667f, 0.078f), P(0.047f * headWidth, 1.667f, 0.078f), 0.0080f * browSize * s,
                                  0.0100f * browSize * s, 0.0080f * browSize * s, 0.0100f * browSize * s);
    head = tree.Join(head, tree.Leaf(brow), 0.020f * s);
    for (const float side : {1.0f, -1.0f}) {
        const int cheek = tree.Ellipsoid(kHead, P(side * 0.047f * headWidth, 1.620f, 0.060f), 0.017f * cheekSize * s, 0.014f * cheekSize * s, 0.016f * cheekSize * s);
        head = tree.Join(head, tree.Leaf(cheek), 0.012f * s);
    }

    // The nose: the bridge, the tip and the wings.
    const glm::vec3 bridgeTop = P(0.0f, 1.646f, 0.083f);
    const glm::vec3 tipCentre = P(0.0f, 1.646f - 0.046f * noseLength, 0.1020f * noseSize);
    const int bridge = tree.Capsule(kHead, bridgeTop, tipCentre, 0.0100f * noseSize * s, 0.0085f * noseSize * s, 0.0125f * noseSize * s,
                                    0.0105f * noseSize * s, across);
    const int tip = tree.Ellipsoid(kHead, tipCentre + P(0.0f, -0.0010f, 0.0f), 0.0100f * noseSize * s, 0.0092f * noseSize * s, 0.0100f * noseSize * s);
    head = tree.Join(head, tree.Merge({bridge, tip}, 0.008f * s), 0.008f * s);
    for (const float side : {1.0f, -1.0f}) {
        const int wing = tree.Ellipsoid(kHead, P(side * 0.0150f * noseSize, tipCentre.y / s - 0.0040f, 0.0945f * noseSize), 0.0092f * noseSize * s,
                                        0.0088f * noseSize * s, 0.0100f * noseSize * s);
        head = tree.Join(head, tree.Leaf(wing), 0.006f * s);
    }

    // The lips, with the groove above the upper lip.
    // Thin lips are kept at least a cell and a half thick, or the grid the head is cut from would tear them.
    const float upperThickness = std::max(0.0075f, 0.0065f * lipSize);
    const float lowerThickness = std::max(0.0085f, 0.0078f * lipSize);
    const int upperLip = tree.Ellipsoid(kHead, P(0.0f, 1.5725f, 0.0875f), 0.0235f * lipSize * s, upperThickness * s, 0.0095f * lipSize * s);
    const int lowerLip = tree.Ellipsoid(kHead, P(0.0f, 1.5595f, 0.0855f), 0.0215f * lipSize * s, lowerThickness * s, 0.0110f * lipSize * s);
    const int philtrum = tree.Ellipsoid(kHead, P(0.0f, 1.583f, 0.088f), 0.0120f * s, 0.0100f * s, 0.0090f * s);
    head = tree.Join(head, tree.Merge({upperLip, lowerLip, philtrum}, 0.004f * s), 0.006f * s);

    // The ears.
    for (const float side : {1.0f, -1.0f}) {
        const int ear = tree.Capsule(kHead, P(side * 0.0800f * headWidth, 1.647f, -0.009f), P(side * 0.0810f * headWidth, 1.598f, 0.000f),
                                     0.0135f * earSize * s, 0.0062f * earSize * s, 0.0100f * earSize * s, 0.0058f * earSize * s, glm::vec3(side, 0.0f, 0.0f));
        head = tree.Join(head, tree.Leaf(ear), 0.005f * s);
    }

    // The hollows of the eyes, cut last. An eyeball and its lids will sit in them.
    for (const float side : {1.0f, -1.0f}) {
        const int socket = tree.Ellipsoid(kHead, P(side * 0.031f * headWidth, 1.636f, 0.089f), 0.0165f * s, 0.0115f * s, 0.0125f * s);
        head = tree.Carve(head, tree.Leaf(socket), 0.007f * s);
    }
    return tree.Finish(head, 0.015f * s, 0.05f * s);
}

BodyShape BuildHandShape(const Skeleton& skeleton, const CharacterParameters& parameters, bool left, float girth) {
    const CharacterParameters clean = Sanitised(parameters);
    const float s = clean.statureMeters / 1.75f;
    const float sign = left ? 1.0f : -1.0f;
    const Joint elbowJoint = left ? Joint::ForearmL : Joint::ForearmR;
    const Joint wristJoint = left ? Joint::HandL : Joint::HandR;
    const Joint bone = wristJoint;

    // The frame of the hand: `along` runs down the fingers, `inward` from the back of the hand through the palm, and
    // `across` over the width of the palm towards the thumb (forwards, with the palm facing the thigh).
    const glm::vec3 wrist = skeleton[wristJoint].restPosition;
    const glm::vec3 along = glm::normalize(wrist - skeleton[elbowJoint].restPosition);
    const glm::vec3 inward = glm::normalize(sign * glm::cross(along, glm::vec3(0.0f, 0.0f, 1.0f)));
    glm::vec3 across = glm::normalize(glm::cross(inward, along));
    if (across.z < 0.0f) {
        across = -across;
    }
    const auto P = [&](float a, float b, float c) { return wrist + (along * a + across * b + inward * c) * s; };
    // A direction `bend` degrees from `along` towards the palm and `fan` degrees towards the thumb side.
    const auto Direction = [&](float bendDegrees, float fanDegrees) {
        const float bend = bendDegrees * kDegreesToRadians;
        const glm::vec3 curled = along * std::cos(bend) + inward * std::sin(bend);
        return glm::normalize(curled + across * std::tan(fanDegrees * kDegreesToRadians));
    };

    ShapeAssembler tree;
    // The palm, a cuff that overlaps the forearm, and the pad at the base of the thumb.
    // The palm starts a little beyond the wrist and narrow: its rounded end must not reach back over the cuff and swell it.
    const int palm = tree.Capsule(bone, P(0.012f, 0.0f, 0.0f), P(0.088f, 0.0f, 0.0f), 0.0320f * s, 0.0135f * s, 0.0425f * s, 0.0125f * s, inward);
    // The cuff is a little wider than the forearm it covers, by the same share at every build. The forearm has its section
    // turned the other way, with its depth across the arm (forwards) and its side along it, so the radii are swapped.
    const WristSection wristSection = ForearmWristSection(clean, girth);
    const int cuff = tree.Capsule(bone, P(-0.025f, 0.0f, 0.0f), P(0.004f, 0.0f, 0.0f), 1.04f * wristSection.depth, 1.04f * wristSection.side, 1.10f * wristSection.depth,
                                  1.10f * wristSection.side, inward);
    const int thenar = tree.Capsule(bone, P(0.020f, 0.030f, 0.004f), P(0.040f, 0.030f, 0.004f), 0.0160f * s, 0.0110f * s, 0.0140f * s, 0.0100f * s, inward);
    const int core = tree.Join(tree.Merge({palm, cuff}, 0.012f * s), tree.Leaf(thenar), 0.010f * s);

    // The fingers: three segments each, curled a little towards the palm, and fanned a little apart.
    struct Finger {
        float across;       // where its base lies across the palm
        float length;       // from base to tip
        float radius;
        float fan;          // degrees from the line of the hand
        float extraCurl;    // degrees more than the others
    };
    // The bases are far enough apart, and the fingers fan far enough, that the gap between two is at least a cell of the grid.
    const std::array<Finger, 4> fingers = {{{0.0335f, 0.071f, 0.0086f, 5.5f, 0.0f},
                                            {0.0112f, 0.078f, 0.0090f, 1.5f, 2.0f},
                                            {-0.0112f, 0.072f, 0.0086f, -2.0f, 4.0f},
                                            {-0.0335f, 0.058f, 0.0075f, -6.0f, 8.0f}}};
    const float segmentShare[3] = {0.43f, 0.32f, 0.25f};
    const float segmentBend[3] = {10.0f, 38.0f, 52.0f};  // the bend of each segment from the line of the hand
    int hand = -1;
    const auto unite = [&](int node) {
        hand = hand < 0 ? node : tree.Join(hand, node, 0.0f);  // the fingers are never smoothed into one another
    };
    for (const Finger& finger : fingers) {
        glm::vec3 start = P(0.084f, finger.across, 0.0f);
        float radius = finger.radius;
        std::vector<int> segments;
        for (int k = 0; k < 3; ++k) {
            const glm::vec3 direction = Direction(segmentBend[k] + finger.extraCurl, finger.fan);
            const glm::vec3 end = start + direction * (finger.length * segmentShare[k] * s);
            const float endRadius = radius * (k == 2 ? 0.84f : 0.95f);
            segments.push_back(tree.Capsule(bone, start, end, radius * s, 0.92f * radius * s, endRadius * s, 0.92f * endRadius * s, inward));
            start = end;
            radius = endRadius;
        }
        unite(tree.Join(core, tree.Merge(segments, 0.007f * s), 0.006f * s));
    }

    // The thumb lies alongside the index finger, turned out a little.
    {
        glm::vec3 start = P(0.010f, 0.034f, 0.004f);
        const float lengths[3] = {0.036f, 0.029f, 0.025f};
        const float bends[3] = {8.0f, 20.0f, 30.0f};
        float radius = 0.0125f;
        std::vector<int> segments;
        for (int k = 0; k < 3; ++k) {
            const glm::vec3 direction = Direction(bends[k], 26.0f);
            const glm::vec3 end = start + direction * (lengths[k] * s);
            const float endRadius = radius * (k == 2 ? 0.84f : 0.93f);
            segments.push_back(tree.Capsule(bone, start, end, radius * s, 0.94f * radius * s, endRadius * s, 0.94f * endRadius * s, inward));
            start = end;
            radius = endRadius;
        }
        unite(tree.Join(core, tree.Merge(segments, 0.007f * s), 0.008f * s));
    }
    return tree.Finish(hand, 0.012f * s, 0.03f * s);
}

}  // namespace character
