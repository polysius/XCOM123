// anim_gen draws a contact sheet of a walk or a run as stick figures, so that the motion can be looked at without a renderer.
//
//   anim_gen [--out sheet.png] [--speed 1.4] [--stature 1.75] [--masculinity 0.5] [--age 30] [--mass 70] [--seed 1]
//            [--slope 0] [--frames 8] [--turn 0]
//
// The sheet has three rows. The top row is the world: the figure at `frames` moments of one cycle at its place in the world,
// with the ground, the path of the pelvis and a dot where each planted heel and ball touched the ground; a foot that does
// not slide leaves one dot, a foot that slides leaves a streak. The middle row is the side view and the bottom row the front
// view of each moment, with the pelvis in the middle of the cell. The left leg and arm are blue and the right ones are orange.
// The numbers measured on the way (stride, foot slip, knee flexion, pelvis height) are printed.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "animation/Gait.h"
#include "animation/Kinematics.h"
#include "animation/JointLimits.h"
#include "core/PngWriter.h"

namespace {

using namespace animation;
using character::Joint;

struct Color {
    uint8_t r, g, b;
    float alpha;
};

class Canvas {
public:
    Canvas(int width_, int height_) : width(width_), height(height_), pixels(static_cast<size_t>(width_) * height_ * 4, 255) {}

    void Disc(float cx, float cy, float radius, Color color) {
        const int x0 = std::max(0, static_cast<int>(std::floor(cx - radius)));
        const int x1 = std::min(width - 1, static_cast<int>(std::ceil(cx + radius)));
        const int y0 = std::max(0, static_cast<int>(std::floor(cy - radius)));
        const int y1 = std::min(height - 1, static_cast<int>(std::ceil(cy + radius)));
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                const float dx = x + 0.5f - cx;
                const float dy = y + 0.5f - cy;
                if (dx * dx + dy * dy <= radius * radius) {
                    Blend(x, y, color);
                }
            }
        }
    }

    void Line(float x0, float y0, float x1, float y1, float thickness, Color color) {
        const float length = std::hypot(x1 - x0, y1 - y0);
        const int steps = std::max(1, static_cast<int>(length));
        for (int i = 0; i <= steps; ++i) {
            const float t = static_cast<float>(i) / steps;
            Disc(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, thickness * 0.5f, color);
        }
    }

    void Rectangle(int x, int y, int w, int h, Color color) {
        for (int j = y; j < y + h; ++j) {
            for (int i = x; i < x + w; ++i) {
                if (i >= 0 && j >= 0 && i < width && j < height) {
                    Blend(i, j, color);
                }
            }
        }
    }

    bool Save(const std::string& path) const { return WritePng(path, width, height, pixels); }

private:
    void Blend(int x, int y, Color color) {
        uint8_t* pixel = &pixels[(static_cast<size_t>(y) * width + x) * 4];
        const uint8_t source[3] = {color.r, color.g, color.b};
        for (int c = 0; c < 3; ++c) {
            pixel[c] = static_cast<uint8_t>(pixel[c] + (source[c] - pixel[c]) * color.alpha);
        }
    }

    int width, height;
    std::vector<uint8_t> pixels;
};

// A view of the world on a part of the canvas: which two world axes go across and up, and how many pixels are a metre.
struct View {
    bool front = false;  // front view: X across; otherwise side view: Z across
    float originX = 0.0f;  // pixel of the world origin
    float originY = 0.0f;  // pixel of y = 0
    float scale = 100.0f;

    float ScreenX(const glm::vec3& p) const { return originX + (front ? p.x : p.z) * scale; }
    float ScreenY(const glm::vec3& p) const { return originY - p.y * scale; }
};

const Color kLeft = {40, 90, 220, 1.0f};
const Color kRight = {230, 120, 20, 1.0f};
const Color kSpine = {40, 40, 40, 1.0f};
const Color kGround = {120, 120, 120, 1.0f};

Color Faded(Color color, float alpha) {
    color.alpha = alpha;
    return color;
}

struct Snapshot {
    Pose pose;
    float time = 0.0f;
};

glm::vec3 Transform(const glm::mat4& matrix, const glm::vec3& point) {
    return glm::vec3(matrix * glm::vec4(point, 1.0f));
}

void DrawFigure(Canvas& canvas, const character::Skeleton& skeleton, const Pose& pose, const View& view, float alpha, float thickness) {
    const std::array<glm::vec3, kJointCount> p = character::JointPositions(skeleton, pose);
    auto at = [&](Joint joint) { return p[character::Index(joint)]; };
    auto bone = [&](Joint a, Joint b, Color color) {
        canvas.Line(view.ScreenX(at(a)), view.ScreenY(at(a)), view.ScreenX(at(b)), view.ScreenY(at(b)), thickness, Faded(color, alpha));
    };

    bone(Joint::Pelvis, Joint::Spine1, kSpine);
    bone(Joint::Spine1, Joint::Spine2, kSpine);
    bone(Joint::Spine2, Joint::Neck, kSpine);
    bone(Joint::Neck, Joint::Head, kSpine);
    bone(Joint::ThighL, Joint::ThighR, kSpine);
    bone(Joint::ClavicleL, Joint::ClavicleR, kSpine);
    for (const bool left : {true, false}) {
        const Color color = left ? kLeft : kRight;
        bone(left ? Joint::ClavicleL : Joint::ClavicleR, left ? Joint::UpperArmL : Joint::UpperArmR, color);
        bone(left ? Joint::UpperArmL : Joint::UpperArmR, left ? Joint::ForearmL : Joint::ForearmR, color);
        bone(left ? Joint::ForearmL : Joint::ForearmR, left ? Joint::HandL : Joint::HandR, color);
        bone(left ? Joint::ThighL : Joint::ThighR, left ? Joint::CalfL : Joint::CalfR, color);
        bone(left ? Joint::CalfL : Joint::CalfR, left ? Joint::FootL : Joint::FootR, color);

        // The sole: heel, ball and toe, through the skinning of the foot joint.
        const Joint foot = left ? Joint::FootL : Joint::FootR;
        const character::FootContacts& contacts = left ? skeleton.leftFoot : skeleton.rightFoot;
        const glm::mat4 skin = character::SkinMatrices(skeleton, pose)[character::Index(foot)];
        const glm::vec3 heel = Transform(skin, contacts.heel);
        const glm::vec3 toe = Transform(skin, contacts.toe);
        const glm::vec3 ankle = at(foot);
        canvas.Line(view.ScreenX(ankle), view.ScreenY(ankle), view.ScreenX(heel), view.ScreenY(heel), thickness, Faded(color, alpha));
        canvas.Line(view.ScreenX(heel), view.ScreenY(heel), view.ScreenX(toe), view.ScreenY(toe), thickness, Faded(color, alpha));
        canvas.Line(view.ScreenX(ankle), view.ScreenY(ankle), view.ScreenX(toe), view.ScreenY(toe), thickness, Faded(color, alpha));
    }
    // The head as a disc above the skull joint.
    const glm::quat headWorld = ForwardKinematics(skeleton, pose).rotation[character::Index(Joint::Head)];
    const glm::vec3 head = at(Joint::Head) + headWorld * glm::vec3(0.0f, 0.065f * skeleton.statureMeters, 0.0f);
    canvas.Disc(view.ScreenX(head), view.ScreenY(head), 0.065f * skeleton.statureMeters * view.scale, Faded(kSpine, alpha * 0.5f));
    // The nose, so that the facing can be seen in the side view.
    const glm::vec3 nose = head + headWorld * glm::vec3(0.0f, 0.0f, 0.07f * skeleton.statureMeters);
    canvas.Line(view.ScreenX(head), view.ScreenY(head), view.ScreenX(nose), view.ScreenY(nose), thickness, Faded(kSpine, alpha));
}

struct Options {
    std::string out = "animation_walk.png";
    float speed = 1.4f;
    float stature = 1.75f;
    float masculinity = 0.5f;
    float age = 30.0f;
    float mass = 70.0f;
    uint32_t seed = 1;
    float slopeDegrees = 0.0f;
    int frames = 8;
    float turn = 0.0f;
    bool trace = false;  // print the height of the pelvis at every step of the cycle
};

bool ParseOptions(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string name = argv[i];
        if (name == "--trace") {
            options.trace = true;
            continue;
        }
        if (i + 1 >= argc) {
            std::cerr << "missing value for " << name << "\n";
            return false;
        }
        const std::string value = argv[++i];
        if (name == "--out") options.out = value;
        else if (name == "--speed") options.speed = std::strtof(value.c_str(), nullptr);
        else if (name == "--stature") options.stature = std::strtof(value.c_str(), nullptr);
        else if (name == "--masculinity") options.masculinity = std::strtof(value.c_str(), nullptr);
        else if (name == "--age") options.age = std::strtof(value.c_str(), nullptr);
        else if (name == "--mass") options.mass = std::strtof(value.c_str(), nullptr);
        else if (name == "--seed") options.seed = static_cast<uint32_t>(std::strtoul(value.c_str(), nullptr, 10));
        else if (name == "--slope") options.slopeDegrees = std::strtof(value.c_str(), nullptr);
        else if (name == "--frames") options.frames = std::max(2, std::atoi(value.c_str()));
        else if (name == "--turn") options.turn = std::strtof(value.c_str(), nullptr);
        else {
            std::cerr << "unknown option " << name << "\n";
            return false;
        }
    }
    return true;
}

// What the run measured: the numbers that say whether the gait is good.
struct Measurements {
    float worstStanceSlip = 0.0f;  // metres a second: fastest horizontal speed of a heel or ball that is planted
    float minimumKneeFlexion = 1000.0f;
    float lowestPelvis = 1000.0f;
    float highestPelvis = -1000.0f;
    float deepestFootBelowGround = 0.0f;
};

// The world positions of the heel, the ball and the toe of a foot in a pose.
std::array<glm::vec3, 3> FootContactPoints(const character::Skeleton& skeleton, const Pose& pose, bool left) {
    const character::FootContacts& contacts = left ? skeleton.leftFoot : skeleton.rightFoot;
    const glm::mat4 skin = character::SkinMatrices(skeleton, pose)[character::Index(left ? Joint::FootL : Joint::FootR)];
    return {Transform(skin, contacts.heel), Transform(skin, contacts.ball), Transform(skin, contacts.toe)};
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!ParseOptions(argc, argv, options)) {
        return 2;
    }
    character::CharacterParameters parameters;
    parameters.masculinity = options.masculinity;
    parameters.ageYears = options.age;
    parameters.statureMeters = options.stature;
    parameters.massKg = options.mass;
    parameters.seed = options.seed;

    const BodyModel body = MakeBodyModel(parameters);
    const Personality personality = PersonalityFrom(body.parameters);
    const float slope = std::tan(options.slopeDegrees * kDegreesToRadians);
    GroundFunction ground = [slope](float, float z) { return slope * z; };
    Walker walker(body, personality, ground);
    walker.Reset(glm::vec3(0.0f), 0.0f);

    const float dt = 1.0f / 120.0f;
    const LocomotionCommand command{options.speed, options.turn};
    for (float t = 0.0f; t < 4.0f; t += dt) {
        walker.Update(dt, command);
    }

    // One cycle, sampled every dt; `frames` of them are drawn.
    const GaitShape shape = walker.Shape();
    const int steps = static_cast<int>(std::round(shape.period / dt));
    std::vector<Snapshot> all;
    Measurements measured;
    glm::vec3 previousHeel[2], previousBall[2];
    bool wasPlanted[2] = {false, false};
    for (int i = 0; i <= steps; ++i) {
        walker.Update(dt, command);
        const Pose& pose = walker.CurrentPose();
        all.push_back({pose, walker.Time()});
        const std::array<glm::vec3, kJointCount> joints = character::JointPositions(body.skeleton, pose);
        if (options.trace) {
            std::cout << "trace " << i << " pelvisY " << joints[0].y - walker.Position().y << " x " << joints[0].x << " phase " << walker.Phase() << " reach "
                      << walker.LegReachesFoot(true) << walker.LegReachesFoot(false) << "\n";
        }
        measured.lowestPelvis = std::min(measured.lowestPelvis, joints[0].y - walker.Position().y);
        measured.highestPelvis = std::max(measured.highestPelvis, joints[0].y - walker.Position().y);
        for (const bool left : {true, false}) {
            const int index = left ? 0 : 1;
            const std::array<glm::vec3, 3> contact = FootContactPoints(body.skeleton, pose, left);
            for (const glm::vec3& point : contact) {
                measured.deepestFootBelowGround = std::min(measured.deepestFootBelowGround, point.y - ground(point.x, point.z));
            }
            const bool planted = walker.IsPlanted(left);
            if (planted && wasPlanted[index]) {
                // The pivot is the heel while it is on the ground and the ball once the heel has lifted: the lower of the two moves least.
                const float heelSpeed = glm::length(glm::vec2(contact[0].x - previousHeel[index].x, contact[0].z - previousHeel[index].z)) / dt;
                const float ballSpeed = glm::length(glm::vec2(contact[1].x - previousBall[index].x, contact[1].z - previousBall[index].z)) / dt;
                measured.worstStanceSlip = std::max(measured.worstStanceSlip, std::min(heelSpeed, ballSpeed));            }
            previousHeel[index] = contact[0];
            previousBall[index] = contact[1];
            wasPlanted[index] = planted;
            const Joint knee = left ? Joint::CalfL : Joint::CalfR;
            measured.minimumKneeFlexion = std::min(measured.minimumKneeFlexion, FlexionDegrees(body.skeleton, knee, pose.rotation[character::Index(knee)]));
        }
    }

    const int frames = options.frames;
    const int width = 1800;
    const int topHeight = 440;
    const int cellHeight = 300;
    const int height = topHeight + 2 * cellHeight;
    Canvas canvas(width, height);
    const float margin = 60.0f;

    // Top row: the world, from the first to the last snapshot. The span of the figures sets the scale.
    const float zStart = walker.Position().z - shape.strideLength * 1.15f;
    const float zEnd = walker.Position().z + shape.strideLength * 0.2f;
    View world;
    world.scale = std::min((width - 2.0f * margin) / std::max(zEnd - zStart + 1.0f, 2.0f), 330.0f / options.stature);
    world.originX = margin - (zStart - 0.5f) * world.scale;
    // The ground line sits near the bottom of the row, at the height of the ground under the first figure.
    const float baseGround = ground(0.0f, walker.Position().z - shape.strideLength);
    world.originY = topHeight - 70.0f + baseGround * world.scale;
    for (float z = zStart - 0.5f; z <= zEnd + 0.5f; z += 0.02f) {
        const glm::vec3 a(0.0f, ground(0.0f, z), z);
        const glm::vec3 b(0.0f, ground(0.0f, z + 0.02f), z + 0.02f);
        canvas.Line(world.ScreenX(a), world.ScreenY(a), world.ScreenX(b), world.ScreenY(b), 2.0f, kGround);
    }
    // Contact dots for every sample of the cycle, planted feet only.
    {
        Walker replay(body, personality, ground);
        replay.Reset(glm::vec3(0.0f), 0.0f);
        for (float t = 0.0f; t < 4.0f; t += dt) {
            replay.Update(dt, command);
        }
        for (int i = 0; i <= steps; ++i) {
            replay.Update(dt, command);
            for (const bool left : {true, false}) {
                if (!replay.IsPlanted(left)) {
                    continue;
                }
                const std::array<glm::vec3, 3> contact = FootContactPoints(body.skeleton, replay.CurrentPose(), left);
                const Color color = left ? kLeft : kRight;
                for (int c = 0; c < 2; ++c) {
                    canvas.Disc(world.ScreenX(contact[c]), world.ScreenY(contact[c]), 2.0f, Faded(color, 0.5f));
                }
            }
        }
    }
    for (int f = 0; f < frames; ++f) {
        const Snapshot& snapshot = all[static_cast<size_t>(f) * steps / frames];
        DrawFigure(canvas, body.skeleton, snapshot.pose, world, 0.35f + 0.65f * (static_cast<float>(f) / frames), 2.5f);
    }
    // The path of the pelvis.
    for (size_t i = 1; i < all.size(); ++i) {
        const glm::vec3 a = character::JointPositions(body.skeleton, all[i - 1].pose)[0];
        const glm::vec3 b = character::JointPositions(body.skeleton, all[i].pose)[0];
        canvas.Line(world.ScreenX(a), world.ScreenY(a), world.ScreenX(b), world.ScreenY(b), 1.5f, Color{200, 40, 40, 0.8f});
    }

    // The middle and bottom rows: the side and the front view of each moment, the pelvis in the middle of the cell.
    const float cellWidth = static_cast<float>(width) / frames;
    for (int row = 0; row < 2; ++row) {
        for (int f = 0; f < frames; ++f) {
            const Snapshot& snapshot = all[static_cast<size_t>(f) * steps / frames];
            const glm::vec3 pelvis = character::JointPositions(body.skeleton, snapshot.pose)[0];
            View view;
            view.front = row == 1;
            view.scale = 0.78f * cellHeight / options.stature;
            view.originX = (f + 0.5f) * cellWidth - (row == 1 ? pelvis.x : pelvis.z) * view.scale;
            // The ground under the figure, at the height of the path point.
            const float groundY = ground(pelvis.x, pelvis.z);
            view.originY = topHeight + (row + 1) * cellHeight - 20.0f + groundY * view.scale;
            canvas.Line(f * cellWidth + 4.0f, view.originY - groundY * view.scale, (f + 1) * cellWidth - 4.0f, view.originY - groundY * view.scale, 1.5f, kGround);
            DrawFigure(canvas, body.skeleton, snapshot.pose, view, 1.0f, 3.0f);
        }
    }
    canvas.Rectangle(0, topHeight - 2, width, 3, Color{0, 0, 0, 0.6f});
    canvas.Rectangle(0, topHeight + cellHeight - 1, width, 2, Color{0, 0, 0, 0.3f});

    if (!canvas.Save(options.out)) {
        std::cerr << "could not write " << options.out << "\n";
        return 1;
    }
    std::cout << "speed " << options.speed << " m/s, stature " << options.stature << " m\n"
              << "period " << shape.period << " s, cadence " << shape.cadence * 60.0f << " steps/min, stride " << shape.strideLength << " m, duty "
              << shape.dutyFactor << ", run blend " << shape.runBlend << "\n"
              << "worst planted-foot slip " << measured.worstStanceSlip * 1000.0f << " mm/s, lowest knee flexion " << measured.minimumKneeFlexion
              << " deg, deepest foot point below ground " << measured.deepestFootBelowGround * 1000.0f << " mm\n"
              << "pelvis height above the ground between " << measured.lowestPelvis << " and " << measured.highestPelvis << " m\n"
              << "wrote " << options.out << "\n";
    return 0;
}
