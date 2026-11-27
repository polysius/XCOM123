#include "character/appearance/Eyes.h"

#include <algorithm>
#include <cmath>

#include "character/BodyShape.h"
#include "character/CharacterGenerator.h"
#include "character/DetailShapes.h"
#include "character/ParallelEach.h"
#include "character/appearance/MeshTools.h"
#include "character/appearance/Noise3D.h"
#include "material/MaterialNoise.h"

namespace character {
namespace {

constexpr float kTwoPi = 6.28318530718f;
constexpr float kEyeRadius = 0.0108f;       // metres at a stature of 1.75 m; an adult eye is 23 to 25 mm across
constexpr float kCorneaBulge = 0.0017f;
constexpr float kIrisRadiusShare = 0.4875f;  // sin of the angle of the edge of the iris: 11.7 mm of iris across 24 mm of eye
constexpr float kPupilShare = 0.36f;         // of the radius of the iris
constexpr float kLidThickness = 0.0010f;
constexpr float kLidClearance = 0.0004f;     // between the eyeball and the inside of a lid

// The frame of an eye: where it is and which way it looks (+Z), rises (+Y) and turns outwards (lateral).
struct EyeFrame {
    glm::vec3 centre{0.0f};
    glm::vec3 lateral{1.0f, 0.0f, 0.0f};  // towards the outer corner of the eye
    glm::vec3 up{0.0f, 1.0f, 0.0f};
    glm::vec3 forward{0.0f, 0.0f, 1.0f};
    float scale = 1.0f;

    float Radius() const { return kEyeRadius * scale; }
    glm::vec3 Direction(float azimuth, float elevation) const {
        return lateral * (std::sin(azimuth) * std::cos(elevation)) + up * std::sin(elevation) + forward * (std::cos(azimuth) * std::cos(elevation));
    }
    // The radius of the eyeball in a direction: the sphere, with the cornea bulging out of the front.
    float BallRadius(glm::vec3 direction) const {
        const float angle = std::acos(glm::clamp(glm::dot(direction, forward), -1.0f, 1.0f));
        return (kEyeRadius + kCorneaBulge * (1.0f - SmoothStep(0.28f, 0.58f, angle))) * scale;
    }
};

// Turns a shell inside out when its triangles face inwards.
void FaceOutwards(RenderMesh& mesh) {
    if (MeshVolume(mesh.positions, mesh.indices) < 0.0f) {
        for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            std::swap(mesh.indices[t + 1], mesh.indices[t + 2]);
        }
    }
    mesh.normals = VertexNormals(mesh.positions, mesh.indices);
}

void FollowHead(RenderMesh& mesh) {
    mesh.boneIndices.assign(mesh.positions.size(), {static_cast<uint8_t>(Index(Joint::Head)), 0, 0, 0});
    mesh.boneWeights.assign(mesh.positions.size(), {1.0f, 0.0f, 0.0f, 0.0f});
}

RenderMesh BuildEyeball(const EyeFrame& frame) {
    constexpr int kRings = 56;
    constexpr int kSegments = 64;
    RenderMesh mesh;
    // Rings from the front pole (the cornea) to the back pole; each ring has its own vertices, and the poles are single vertices.
    const auto add = [&](glm::vec3 direction) {
        mesh.positions.push_back(frame.centre + direction * frame.BallRadius(direction));
        // The picture of the eye is looked at along the gaze: x to the right of the eye as seen from the front, y up.
        const float x = glm::dot(direction, frame.lateral) * (frame.lateral.x > 0.0f ? 1.0f : -1.0f);
        const float y = glm::dot(direction, frame.up);
        mesh.uvs.emplace_back(0.5f + 0.5f * x, 0.5f - 0.5f * y);
    };
    add(frame.forward);
    for (int ring = 1; ring < kRings; ++ring) {
        const float theta = 3.14159265359f * static_cast<float>(ring) / kRings;
        for (int segment = 0; segment < kSegments; ++segment) {
            const float psi = kTwoPi * static_cast<float>(segment) / kSegments;
            const glm::vec3 direction = frame.lateral * (std::sin(theta) * std::cos(psi)) + frame.up * (std::sin(theta) * std::sin(psi)) + frame.forward * std::cos(theta);
            add(direction);
        }
    }
    add(-frame.forward);
    const uint32_t back = static_cast<uint32_t>(mesh.positions.size() - 1);
    const auto at = [&](int ring, int segment) { return static_cast<uint32_t>(1 + (ring - 1) * kSegments + (segment % kSegments)); };
    for (int segment = 0; segment < kSegments; ++segment) {
        mesh.indices.insert(mesh.indices.end(), {0u, at(1, segment), at(1, segment + 1)});
    }
    for (int ring = 1; ring + 1 < kRings; ++ring) {
        for (int segment = 0; segment < kSegments; ++segment) {
            mesh.indices.insert(mesh.indices.end(), {at(ring, segment), at(ring + 1, segment), at(ring + 1, segment + 1)});
            mesh.indices.insert(mesh.indices.end(), {at(ring, segment), at(ring + 1, segment + 1), at(ring, segment + 1)});
        }
    }
    for (int segment = 0; segment < kSegments; ++segment) {
        mesh.indices.insert(mesh.indices.end(), {at(kRings - 1, segment), back, at(kRings - 1, segment + 1)});
    }
    FollowHead(mesh);
    FaceOutwards(mesh);
    return mesh;
}

// The shape of the opening between the lids as a function of the angle round the eye, from 0 at the corners to 1 where it is widest.
float OpeningShape(float azimuth) {
    const float x = (azimuth + 0.12f) / 0.95f;
    return std::abs(x) < 1.0f ? std::pow(1.0f - x * x, 0.6f) : 0.0f;
}
float UpperEdge(float azimuth) {
    return 0.34f * OpeningShape(azimuth);
}
float LowerEdge(float azimuth) {
    return -0.40f * OpeningShape(azimuth);
}

// The distance from the eye along a direction at which the ray leaves the hollow of the socket and enters the flesh of the
// head again, or a negative number if it does not (the ray leaves the head altogether, or starts in flesh).
float SocketWall(const BodyShape& head, const EyeFrame& frame, glm::vec3 direction, float from) {
    bool inAir = false;
    // The hollow is not deeper than a centimetre; a wall further out than that belongs to some other feature (the nose, the cheek).
    for (float r = from; r < from + 0.016f * frame.scale; r += 0.0004f * frame.scale) {
        const float distance = head.Distance(frame.centre + direction * r);
        if (distance > 0.0003f) {
            inAir = true;
        } else if (inAir && distance < -0.0006f) {
            return r;
        }
    }
    return -1.0f;
}

// A closed slab from two grids of points: the inner surface and the outer one, each (across + 1) by (along + 1).
RenderMesh BuildSlab(const std::vector<glm::vec3>& inner, const std::vector<glm::vec3>& outer, int across, int along) {
    RenderMesh mesh;
    const int columns = across + 1;
    const uint32_t outerBase = static_cast<uint32_t>(inner.size());
    mesh.positions = inner;
    mesh.positions.insert(mesh.positions.end(), outer.begin(), outer.end());
    mesh.uvs.resize(mesh.positions.size());
    for (int b = 0; b <= along; ++b) {
        for (int a = 0; a <= across; ++a) {
            const glm::vec2 uv(static_cast<float>(a) / across, static_cast<float>(b) / along);
            mesh.uvs[b * columns + a] = uv;
            mesh.uvs[outerBase + b * columns + a] = uv;
        }
    }
    const auto in = [&](int a, int b) { return static_cast<uint32_t>(b * columns + a); };
    const auto out = [&](int a, int b) { return outerBase + static_cast<uint32_t>(b * columns + a); };
    for (int b = 0; b < along; ++b) {
        for (int a = 0; a < across; ++a) {
            mesh.indices.insert(mesh.indices.end(), {out(a, b), out(a + 1, b), out(a, b + 1)});
            mesh.indices.insert(mesh.indices.end(), {out(a + 1, b), out(a + 1, b + 1), out(a, b + 1)});
            mesh.indices.insert(mesh.indices.end(), {in(a, b), in(a, b + 1), in(a + 1, b)});
            mesh.indices.insert(mesh.indices.end(), {in(a + 1, b), in(a, b + 1), in(a + 1, b + 1)});
        }
    }
    // The walls round the edge: the boundary of the grid, counter-clockwise seen from outside.
    std::vector<std::pair<int, int>> loop;
    for (int a = 0; a < across; ++a) loop.emplace_back(a, 0);
    for (int b = 0; b < along; ++b) loop.emplace_back(across, b);
    for (int a = across; a > 0; --a) loop.emplace_back(a, along);
    for (int b = along; b > 0; --b) loop.emplace_back(0, b);
    for (size_t i = 0; i < loop.size(); ++i) {
        const auto [pa, pb] = loop[i];
        const auto [qa, qb] = loop[(i + 1) % loop.size()];
        mesh.indices.insert(mesh.indices.end(), {out(qa, qb), out(pa, pb), in(pa, pb)});
        mesh.indices.insert(mesh.indices.end(), {out(qa, qb), in(pa, pb), in(qa, qb)});
    }
    FollowHead(mesh);
    FaceOutwards(mesh);
    return mesh;
}

// One lid: the upper lid when `upper` is true. It spreads from the edge of the opening out over the brow or the cheek.
RenderMesh BuildLid(const BodyShape& head, const EyeFrame& frame, bool upper, JobSystem* jobs) {
    constexpr int kAcross = 28;
    constexpr int kAlong = 10;
    constexpr float kAzimuthInner = -1.40f;
    constexpr float kAzimuthOuter = 1.20f;
    const float reach = upper ? 0.85f : 0.70f;
    const int columns = kAcross + 1;
    const int pointCount = columns * (kAlong + 1);
    std::vector<glm::vec3> directions(pointCount);
    std::vector<float> spheres(pointCount);  // the radius of the eyeball plus the clearance, in each direction
    std::vector<float> radii(pointCount);    // the radius of the inner surface of the lid
    // Looking for the wall of the socket along a ray is the cost of a lid, and every ray is its own: they run on the job system.
    ParallelEach(jobs, pointCount, [&](int i) {
        const int a = i % columns;
        const int b = i / columns;
        const float spread = static_cast<float>(b) / kAlong;
        const float azimuth = glm::mix(kAzimuthInner, kAzimuthOuter, static_cast<float>(a) / kAcross);
        const float edge = upper ? UpperEdge(azimuth) : LowerEdge(azimuth);
        const float elevation = edge + (upper ? reach : -reach) * spread;
        const glm::vec3 direction = frame.Direction(azimuth, elevation);
        const float sphere = frame.BallRadius(direction) + kLidClearance * frame.scale;
        // Far from the opening the lid is carried out to the wall of the socket, so that no gap shows between it and the skin.
        // The rim is buried in the skin: it lies below the surface of the face.
        const float wall = SocketWall(head, frame, direction, sphere);
        const float buried = wall > 0.0f ? wall - 0.0014f * frame.scale : sphere + 0.0015f * frame.scale;
        directions[i] = direction;
        spheres[i] = sphere;
        radii[i] = glm::mix(sphere, std::max(buried, sphere), SmoothStep(0.20f, 0.95f, spread));
    });
    // The wall of the socket is found ray by ray and is ragged where a ray just misses a feature; smooth the radii of the lid,
    // which leaves its edge on the eyeball alone.
    for (int pass = 0; pass < 4; ++pass) {
        std::vector<float> smoothed = radii;
        for (int b = 1; b <= kAlong; ++b) {
            for (int a = 0; a <= kAcross; ++a) {
                float sum = 0.0f;
                int count = 0;
                for (int db = -1; db <= 1; ++db) {
                    for (int da = -1; da <= 1; ++da) {
                        const int nb = b + db;
                        const int na = a + da;
                        if (nb >= 0 && nb <= kAlong && na >= 0 && na <= kAcross) {
                            sum += radii[nb * columns + na];
                            ++count;
                        }
                    }
                }
                smoothed[b * columns + a] = std::max(sum / static_cast<float>(count), spheres[b * columns + a]);
            }
        }
        radii.swap(smoothed);
    }
    std::vector<glm::vec3> inner;
    std::vector<glm::vec3> outer;
    for (int b = 0; b <= kAlong; ++b) {
        // The lid thins out towards its rim, so that the skin runs into it without a step.
        const float thickness = kLidThickness * frame.scale * (1.0f - 0.7f * SmoothStep(0.5f, 1.0f, static_cast<float>(b) / kAlong));
        for (int a = 0; a <= kAcross; ++a) {
            const size_t i = static_cast<size_t>(b * columns + a);
            inner.push_back(frame.centre + directions[i] * radii[i]);
            outer.push_back(frame.centre + directions[i] * (radii[i] + thickness));
        }
    }
    return BuildSlab(inner, outer, kAcross, kAlong);
}

// Lashes: ribbons that start on the edge of a lid and curl away from the eye.
void AddLashes(RibbonSet& lashes, const EyeFrame& frame, const PersonTraits& traits, uint32_t seed, bool upper) {
    const int count = upper ? 42 : 26;
    const glm::vec3 grey(0.35f);
    const glm::vec3 colour = glm::mix(traits.hairColour, grey, 0.4f * traits.greyFraction) * 0.7f;
    for (int i = 0; i < count; ++i) {
        const float jitter = Random01(seed, 700u + static_cast<uint32_t>(i) + (upper ? 0u : 100u), 1u);
        const float azimuth = glm::mix(-0.70f, 0.90f, (static_cast<float>(i) + 0.5f) / count) + 0.02f * (jitter - 0.5f);
        const float edge = upper ? UpperEdge(azimuth) : LowerEdge(azimuth);
        const glm::vec3 direction = frame.Direction(azimuth, edge);
        // The way the surface of the eye rises towards the lid, and so the way the lash points when it grows out of the edge.
        const glm::vec3 awayFromEye = glm::normalize(frame.Direction(azimuth, edge + (upper ? 0.01f : -0.01f)) - direction);
        const float length = (upper ? 0.0098f : 0.0055f) * (0.55f + 0.45f * OpeningShape(azimuth)) * (0.85f + 0.3f * jitter) * frame.scale;
        const glm::vec3 base = frame.centre + direction * (frame.BallRadius(direction) + (kLidClearance + 0.6f * kLidThickness) * frame.scale);
        // Starts out along the lid, then curls towards the front and away from the eye.
        glm::vec3 heading = glm::normalize(direction * 0.55f + awayFromEye * 0.85f);
        const glm::vec3 curlTo = glm::normalize(frame.forward * 0.9f + (upper ? 1.0f : -1.0f) * frame.up * 0.8f);
        std::vector<glm::vec3> points{base};
        constexpr int kSegments = 4;
        for (int k = 0; k < kSegments; ++k) {
            heading = glm::normalize(glm::mix(heading, curlTo, 0.28f));
            points.push_back(points.back() + heading * (length / kSegments));
        }
        AddRibbon(lashes, points, frame.lateral, 0.00055f * frame.scale, 0.00012f * frame.scale, colour, colour * 0.8f, Joint::Head);
    }
}

glm::vec3 ToSrgb(glm::vec3 linear) {
    return glm::pow(glm::clamp(linear, glm::vec3(0.0f), glm::vec3(1.0f)), glm::vec3(1.0f / 2.2f));
}
uint8_t Byte(float value) {
    return static_cast<uint8_t>(std::lround(glm::clamp(value, 0.0f, 1.0f) * 255.0f));
}

struct EyePixel {
    glm::vec3 albedo{1.0f};
    float roughness = 0.3f;
    float height = 0.0f;  // metres
};

// The picture of the eye from the front, at a point (u, v) of the texture.
EyePixel PaintEye(float u, float v, const PersonTraits& traits, uint32_t seed) {
    const float x = (u - 0.5f) * 2.0f;
    const float y = (0.5f - v) * 2.0f;
    const float radius = std::sqrt(x * x + y * y);  // the sine of the angle from the gaze
    const float angle = std::atan2(y, x) / kTwoPi + 0.5f;
    EyePixel pixel;

    // The white of the eye: slightly warm, with red veins that thicken towards the edge and with age.
    glm::vec3 sclera(0.80f, 0.77f, 0.72f);
    const float veinField = material::RidgedFbm(u * 1.0f, v * 1.0f, 6, 4, seed + 5u);
    const float veinMask = SmoothStep(0.55f, 0.95f, radius) * SmoothStep(0.80f, 0.97f, veinField);
    sclera = glm::mix(sclera, glm::vec3(0.62f, 0.22f, 0.20f), 0.35f * veinMask * (0.5f + 0.5f * traits.wrinkles));
    sclera *= 1.0f - 0.18f * SmoothStep(0.7f, 1.0f, radius);  // the shadow of the lids at the edge
    sclera *= 1.0f + 0.05f * material::Fbm(u, v, 12, 12, 3, seed + 7u);

    // The iris: fibres running out from the pupil, a lighter ring round it, a dark ring at the edge.
    const float irisRadius = kIrisRadiusShare;
    const float t = radius / irisRadius;  // 0 at the pupil's centre, 1 at the edge of the iris
    const float fibres = material::Fbm(angle, t, 56, 5, 4, seed + 9u, 0.6f);
    const float streaks = material::Fbm(angle, t, 18, 3, 3, seed + 10u);
    const float crypts = SmoothStep(0.45f, 0.75f, material::Fbm(angle, t, 22, 8, 3, seed + 11u));
    glm::vec3 iris = traits.irisColour * (1.0f + 0.65f * fibres + 0.35f * streaks);
    // Round the pupil the iris is lighter and more golden; this is where blue and green eyes show a brown ring.
    const glm::vec3 amber(0.30f, 0.17f, 0.05f);
    iris = glm::mix(iris, amber * (1.0f + 0.4f * fibres), (1.0f - SmoothStep(0.30f, 0.62f, t)) * 0.50f);
    iris *= 1.0f - 0.35f * crypts * SmoothStep(0.35f, 0.7f, t);
    iris *= 1.0f - 0.85f * SmoothStep(0.80f, 1.0f, t);  // the limbal ring
    const float pupil = 1.0f - SmoothStep(kPupilShare - 0.03f, kPupilShare + 0.03f, t);
    iris = glm::mix(iris, glm::vec3(0.004f), pupil);

    const float irisMask = 1.0f - SmoothStep(0.97f, 1.04f, t);
    pixel.albedo = glm::mix(sclera, iris, irisMask);
    // The cornea is wet and clear over the iris and a little beyond it.
    const float cornea = 1.0f - SmoothStep(0.55f, 0.66f, radius);
    pixel.roughness = glm::mix(0.28f, 0.04f, cornea);
    pixel.height = irisMask * 0.00018f * (fibres + 0.5f * streaks) * (1.0f - pupil) - 0.00025f * pupil * irisMask;
    return pixel;
}

EyeTextures BakeEyeTextures(const PersonTraits& traits, uint32_t seed, int size, float scale, JobSystem* jobs) {
    EyeTextures textures;
    textures.albedo = ByteImage(size, size, 4);
    textures.normal = ByteImage(size, size, 4);
    textures.roughness = ByteImage(size, size, 1);
    std::vector<float> heights(static_cast<size_t>(size) * size);
    ParallelEach(jobs, size, [&](int y) {
        for (int x = 0; x < size; ++x) {
            const EyePixel pixel = PaintEye((static_cast<float>(x) + 0.5f) / size, (static_cast<float>(y) + 0.5f) / size, traits, seed);
            const glm::vec3 colour = ToSrgb(pixel.albedo);
            uint8_t* albedo = textures.albedo.Pixel(x, y);
            albedo[0] = Byte(colour.r);
            albedo[1] = Byte(colour.g);
            albedo[2] = Byte(colour.b);
            albedo[3] = 255;
            *textures.roughness.Pixel(x, y) = Byte(pixel.roughness);
            heights[static_cast<size_t>(y) * size + x] = pixel.height;
        }
    });
    // One texel is a fraction of the diameter of the eye.
    const float metersPerTexel = 2.0f * kEyeRadius * scale / static_cast<float>(size);
    const auto heightAt = [&](int x, int y) { return heights[static_cast<size_t>(std::clamp(y, 0, size - 1)) * size + std::clamp(x, 0, size - 1)]; };
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            const float slopeX = (heightAt(x + 1, y) - heightAt(x - 1, y)) / (2.0f * metersPerTexel);
            const float slopeY = (heightAt(x, y + 1) - heightAt(x, y - 1)) / (2.0f * metersPerTexel);
            const glm::vec3 normal = glm::normalize(glm::vec3(-slopeX, slopeY, 1.0f));
            uint8_t* pixel = textures.normal.Pixel(x, y);
            pixel[0] = Byte(0.5f + 0.5f * normal.x);
            pixel[1] = Byte(0.5f + 0.5f * normal.y);
            pixel[2] = Byte(0.5f + 0.5f * normal.z);
            pixel[3] = 255;
        }
    }
    return textures;
}

}  // namespace

float EyeballRadiusToward(const EyeParts& eye, glm::vec3 point) {
    EyeFrame frame;
    frame.scale = eye.radius / kEyeRadius;
    const glm::vec3 offset = point - eye.centre;
    return glm::length(offset) > 1e-9f ? frame.BallRadius(glm::normalize(offset)) : frame.Radius();
}

EyeSet BuildEyes(const CharacterParameters& rawParameters, const PersonTraits& traits, const EyeSettings& settings, JobSystem* jobs) {
    const CharacterParameters parameters = Sanitised(rawParameters);
    const HeadLandmarks landmarks = ComputeHeadLandmarks(parameters);
    const BodyShape head = BuildHeadShape(parameters);
    const float scale = landmarks.scale;

    EyeSet set;
    for (int i = 0; i < 2; ++i) {
        const float side = i == 0 ? 1.0f : -1.0f;
        EyeFrame frame;
        // The eyeball sits in the hollow with its front a little behind the brow, and its back in the flesh behind the socket.
        frame.centre = landmarks.eyeSockets[i].centre + glm::vec3(0.0f, 0.0f, -0.0190f * scale);
        frame.lateral = glm::vec3(side, 0.0f, 0.0f);
        frame.scale = scale;
        EyeParts& eye = set.eyes[i];
        eye.centre = frame.centre;
        eye.radius = frame.Radius();
        eye.eyeball = BuildEyeball(frame);
        eye.upperLid = BuildLid(head, frame, true, jobs);
        eye.lowerLid = BuildLid(head, frame, false, jobs);
        AddLashes(eye.lashes, frame, traits, parameters.seed + static_cast<uint32_t>(i) * 31u, true);
        AddLashes(eye.lashes, frame, traits, parameters.seed + static_cast<uint32_t>(i) * 31u, false);
    }
    set.textures = BakeEyeTextures(traits, parameters.seed, std::clamp(settings.textureSize, 64, 2048), scale, jobs);
    return set;
}

}  // namespace character
