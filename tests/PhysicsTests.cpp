// Tests of the rigid body physics (no window, no GPU). Like unit_tests there is no test framework: each
// test is a function that calls Check, and main reports every failure it finds. Every test sets up a small
// world and checks what physics says must happen: bodies fall at the speed of gravity, rest on what
// carries them, stay in a stack, bounce as high as their restitution allows, slide as far as friction
// lets them, and give the same result every time.
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "core/JobSystem.h"
#include "physics/Collision.h"
#include "physics/PhysicsWorld.h"

namespace {

using physics::Body;
using physics::BodyDescription;
using physics::PhysicsSettings;
using physics::PhysicsWorld;
using physics::Shape;

int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

bool Near(float value, float expected, float tolerance) {
    return std::abs(value - expected) <= tolerance;
}

void Simulate(PhysicsWorld& world, float seconds) {
    const int steps = static_cast<int>(std::lround(seconds / world.Settings().fixedStep));
    for (int i = 0; i < steps; ++i) {
        world.Step();
    }
}

// A slab whose top face is the plane y = 0.
uint32_t AddGround(PhysicsWorld& world, float friction = 0.6f, float restitution = 0.0f) {
    BodyDescription ground;
    ground.shape = Shape::Box({200.0f, 0.5f, 200.0f});
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.isStatic = true;
    ground.material = {friction, restitution};
    return world.AddBody(ground);
}

uint32_t AddBox(PhysicsWorld& world, glm::vec3 position, glm::vec3 half = {0.5f, 0.5f, 0.5f}) {
    BodyDescription box;
    box.shape = Shape::Box(half);
    box.position = position;
    return world.AddBody(box);
}

uint32_t AddSphere(PhysicsWorld& world, glm::vec3 position, float radius = 0.5f) {
    BodyDescription sphere;
    sphere.shape = Shape::Sphere(radius);
    sphere.position = position;
    return world.AddBody(sphere);
}

uint32_t AddCapsule(PhysicsWorld& world, glm::vec3 position, float radius = 0.3f, float halfHeight = 0.5f,
                    glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
    BodyDescription capsule;
    capsule.shape = Shape::Capsule(radius, halfHeight);
    capsule.position = position;
    capsule.orientation = orientation;
    return world.AddBody(capsule);
}


// How far the closest of a box's three axes is from straight up, as a cosine: 1 when the box lies flat.
float Flatness(const Body& body) {
    const glm::mat3 axes = glm::mat3_cast(body.orientation);
    float best = 0.0f;
    for (int i = 0; i < 3; ++i) {
        best = std::max(best, std::abs(axes[i].y));
    }
    return best;
}

// Free fall and resting -----------------------------------------------------------------------

void ABodyInFreeFallFollowsGravity() {
    PhysicsWorld world;
    const uint32_t ball = AddSphere(world, {0.0f, 10.0f, 0.0f});
    Simulate(world, 1.0f);
    // One second of fall is 4.9 m, and the integration error of a 120 Hz step is a few centimetres.
    Check(Near(world.GetBody(ball).position.y, 10.0f - 4.905f, 0.06f), "a ball falls 4.9 m in a second");
    Check(Near(world.GetBody(ball).linearVelocity.y, -9.81f, 0.02f), "and reaches 9.8 metres per second");
    Check(Near(world.GetBody(ball).position.x, 0.0f, 1e-6f), "without drifting sideways");
}

void ASphereComesToRestOnTheGround() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t ball = AddSphere(world, {0.0f, 3.0f, 0.0f});
    Simulate(world, 3.0f);
    const Body& body = world.GetBody(ball);
    Check(Near(body.position.y, 0.5f, 0.01f), "the ball rests with its surface on the ground");
    Check(glm::length(body.linearVelocity) < 0.02f, "and is still");
    Check(body.isAsleep, "and asleep");
}

void ABoxLandsAndLiesFlat() {
    PhysicsWorld world;
    AddGround(world);
    BodyDescription description;
    description.shape = Shape::Box({0.5f, 0.5f, 0.5f});
    description.position = {0.0f, 2.0f, 0.0f};
    description.orientation = glm::angleAxis(0.3f, glm::normalize(glm::vec3(1.0f, 0.0f, 1.0f)));
    const uint32_t box = world.AddBody(description);
    Simulate(world, 4.0f);
    const Body& body = world.GetBody(box);
    Check(Near(body.position.y, 0.5f, 0.015f), "the box rests on a face, half its height up");
    Check(Flatness(body) > 0.9995f, "and lies flat");
    Check(body.isAsleep, "and sleeps");
}

void ATiltedBoxFallsOverOntoAFace() {
    PhysicsWorld world;
    AddGround(world);
    BodyDescription description;
    description.shape = Shape::Box({1.0f, 0.3f, 0.5f});
    description.position = {0.0f, 3.0f, 0.0f};
    description.orientation = glm::angleAxis(0.7f, glm::vec3(0.0f, 0.0f, 1.0f));
    const uint32_t box = world.AddBody(description);
    Simulate(world, 5.0f);
    const Body& body = world.GetBody(box);
    Check(Flatness(body) > 0.999f, "a box dropped on its edge ends up on a face");
    Check(Near(body.position.y, 0.3f, 0.02f), "on its widest face, the one that is stable");
}

void ASphereRestsOnABox() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t box = AddBox(world, {0.0f, 0.5f, 0.0f});
    const uint32_t ball = AddSphere(world, {0.1f, 3.0f, 0.1f}, 0.25f);
    Simulate(world, 4.0f);
    Check(Near(world.GetBody(box).position.y, 0.5f, 0.01f), "the box carries the ball");
    Check(Near(world.GetBody(ball).position.y, 1.25f, 0.02f), "and the ball sits on top of it");
}

void ASphereBalancesOnASphereOnlyIfCentred() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t lower = AddSphere(world, {0.0f, 0.5f, 0.0f});
    const uint32_t upper = AddSphere(world, {0.0f, 2.0f, 0.0f}, 0.25f);
    Simulate(world, 2.0f);
    Check(world.GetBody(upper).position.y > 0.0f && world.GetBody(lower).position.y > 0.0f, "nothing fell through the ground");
    Check(Near(world.GetBody(lower).position.y, 0.5f, 0.02f), "the lower ball rests on the ground");
}

// Stacks --------------------------------------------------------------------------------------

void AStackOfBoxesStaysStanding() {
    PhysicsWorld world;
    AddGround(world);
    std::vector<uint32_t> stack;
    for (int i = 0; i < 6; ++i) {
        stack.push_back(AddBox(world, {0.0f, 0.5f + static_cast<float>(i), 0.0f}));
    }
    Simulate(world, 8.0f);
    for (int i = 0; i < 6; ++i) {
        const Body& body = world.GetBody(stack[static_cast<size_t>(i)]);
        Check(Near(body.position.y, 0.5f + static_cast<float>(i), 0.04f), "box " + std::to_string(i) + " keeps its height");
        Check(Near(body.position.x, 0.0f, 0.05f) && Near(body.position.z, 0.0f, 0.05f), "and its place in the stack");
    }
    Check(world.AwakeBodyCount() == 0, "the whole stack falls asleep together");
}

void AWallOfBoxesStaysStanding() {
    PhysicsWorld world;
    AddGround(world);
    std::vector<uint32_t> wall;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            const float shift = row % 2 == 0 ? 0.0f : 0.5f;  // laid in a bond, as bricks are
            wall.push_back(AddBox(world, {static_cast<float>(column) + shift, 0.5f + static_cast<float>(row), 0.0f}));
        }
    }
    Simulate(world, 6.0f);
    float worstDrift = 0.0f;
    for (size_t i = 0; i < wall.size(); ++i) {
        const int row = static_cast<int>(i) / 4;
        const int column = static_cast<int>(i) % 4;
        const float shift = row % 2 == 0 ? 0.0f : 0.5f;
        const glm::vec3 expected(static_cast<float>(column) + shift, 0.5f + static_cast<float>(row), 0.0f);
        worstDrift = std::max(worstDrift, glm::distance(world.GetBody(wall[i]).position, expected));
    }
    Check(worstDrift < 0.08f, "a wall of sixteen boxes stands, worst drift " + std::to_string(worstDrift));
}

void ATallThinColumnOfBoxesDoesNotExplode() {
    PhysicsWorld world;
    AddGround(world);
    std::vector<uint32_t> column;
    for (int i = 0; i < 12; ++i) {
        column.push_back(AddBox(world, {0.0f, 0.25f + 0.5f * static_cast<float>(i), 0.0f}, {0.25f, 0.25f, 0.25f}));
    }
    Simulate(world, 6.0f);
    float highest = 0.0f;
    float widest = 0.0f;
    for (uint32_t id : column) {
        highest = std::max(highest, world.GetBody(id).position.y);
        widest = std::max(widest, std::abs(world.GetBody(id).position.x) + std::abs(world.GetBody(id).position.z));
    }
    Check(highest < 6.2f, "no box was thrown upwards");
    Check(widest < 0.5f, "and the column did not scatter");
}

// Bouncing, sliding, friction ---------------------------------------------------------------

void ABallBouncesAsHighAsItsRestitutionAllows() {
    PhysicsWorld world;
    AddGround(world, 0.6f, 0.8f);
    BodyDescription description;
    description.shape = Shape::Sphere(0.25f);
    description.position = {0.0f, 5.25f, 0.0f};
    description.material = {0.6f, 0.8f};
    const uint32_t ball = world.AddBody(description);
    float apex = 0.0f;
    bool bounced = false;
    for (int i = 0; i < 600; ++i) {
        world.Step();
        const Body& body = world.GetBody(ball);
        if (!bounced && body.linearVelocity.y > 0.5f) {
            bounced = true;
        }
        if (bounced) {
            apex = std::max(apex, body.position.y - 0.25f);
        }
    }
    // A restitution of 0.8 keeps 0.8 of the speed, so 0.64 of the height: 3.2 m from 5 m.
    Check(bounced, "the ball bounces");
    Check(Near(apex, 3.2f, 0.4f), "to about 64% of the height it fell from, got " + std::to_string(apex));
}

void ASlowBallDoesNotBounce() {
    PhysicsWorld world;
    AddGround(world, 0.6f, 0.8f);
    BodyDescription description;
    description.shape = Shape::Sphere(0.25f);
    description.position = {0.0f, 0.25f + 0.04f, 0.0f};  // lands at under a metre per second
    description.material = {0.6f, 0.8f};
    const uint32_t ball = world.AddBody(description);
    float highest = 0.0f;
    for (int i = 0; i < 240; ++i) {
        world.Step();
        if (i > 20) {
            highest = std::max(highest, world.GetBody(ball).position.y);
        }
    }
    Check(highest < 0.27f, "a ball landing gently stays down");
}

void ABoxSlidingOnTheGroundStopsWhereFrictionSaysItMust() {
    PhysicsWorld world;
    AddGround(world, 0.5f, 0.0f);
    BodyDescription description;
    description.shape = Shape::Box({0.5f, 0.5f, 0.5f});
    description.position = {0.0f, 0.5f, 0.0f};
    description.linearVelocity = {5.0f, 0.0f, 0.0f};
    description.material = {0.5f, 0.0f};
    const uint32_t box = world.AddBody(description);
    Simulate(world, 5.0f);
    // The friction coefficient of the pair is 0.5: the box stops after v squared over two mu g, 2.55 m.
    const Body& body = world.GetBody(box);
    Check(Near(body.position.x, 2.55f, 0.25f), "the box stops 2.55 m away, got " + std::to_string(body.position.x));
    Check(Near(body.position.z, 0.0f, 0.02f), "straight ahead");
    Check(glm::length(body.linearVelocity) < 0.02f, "and stays there");
}

// A big slab tilted by `angle` about Z, with a box of the same tilt resting on its upper face.
struct Incline {
    PhysicsWorld world;
    uint32_t box = 0;
    glm::vec3 downhill{0.0f};
    glm::vec3 start{0.0f};
};

void BuildIncline(Incline& incline, float angle, float friction) {
    const glm::quat tilt = glm::angleAxis(angle, glm::vec3(0.0f, 0.0f, 1.0f));
    BodyDescription slab;
    slab.shape = Shape::Box({30.0f, 0.5f, 30.0f});
    slab.orientation = tilt;
    slab.isStatic = true;
    slab.material = {friction, 0.0f};
    incline.world.AddBody(slab);

    const glm::vec3 up = tilt * glm::vec3(0.0f, 1.0f, 0.0f);
    BodyDescription box;
    box.shape = Shape::Box({0.5f, 0.5f, 0.5f});
    box.orientation = tilt;
    box.position = up * (0.5f + 0.5f + 0.001f);  // a little above the slab's face
    box.material = {friction, 0.0f};
    incline.box = incline.world.AddBody(box);
    incline.downhill = -(tilt * glm::vec3(1.0f, 0.0f, 0.0f));  // the slab's +X axis points uphill
    incline.start = box.position;
}

void AGripOnASlopeHoldsABoxStill() {
    Incline incline;
    BuildIncline(incline, glm::radians(20.0f), 0.6f);  // tan(20 degrees) is 0.36, so friction 0.6 holds
    Simulate(incline.world, 3.0f);
    const float slid = glm::dot(incline.world.GetBody(incline.box).position - incline.start, incline.downhill);
    Check(std::abs(slid) < 0.1f, "a box on a 20 degree slope with grip stays put, moved " + std::to_string(slid));
}

void AWeakGripLetsABoxSlideDownASlope() {
    Incline incline;
    BuildIncline(incline, glm::radians(20.0f), 0.1f);
    Simulate(incline.world, 2.0f);
    const float slid = glm::dot(incline.world.GetBody(incline.box).position - incline.start, incline.downhill);
    // It accelerates at g (sin 20 - mu cos 20) = 2.4 m/s^2, so it is 4.8 m down after two seconds.
    Check(Near(slid, 4.8f, 0.7f), "a slippery box slides about 4.8 m in two seconds, got " + std::to_string(slid));
}

// Turned static bodies ----------------------------------------------------------------------

void ABoxRestsOnATurnedPlatform() {
    PhysicsWorld world;
    AddGround(world);
    BodyDescription platform;
    platform.shape = Shape::Box({3.0f, 0.5f, 1.0f});
    platform.position = {0.0f, 0.5f, 0.0f};
    platform.orientation = glm::angleAxis(glm::radians(37.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    platform.isStatic = true;
    world.AddBody(platform);
    const uint32_t box = AddBox(world, {0.5f, 3.0f, 0.2f}, {0.4f, 0.4f, 0.4f});
    Simulate(world, 4.0f);
    Check(Near(world.GetBody(box).position.y, 1.4f, 0.02f), "a box lands on a turned platform and rests on it");
}

void AThinPlateStopsAFastBox() {
    PhysicsWorld world;
    AddGround(world);
    BodyDescription plate;
    plate.shape = Shape::Box({5.0f, 0.1f, 5.0f});
    plate.position = {0.0f, 5.0f, 0.0f};
    plate.isStatic = true;
    world.AddBody(plate);
    BodyDescription box;
    box.shape = Shape::Box({0.3f, 0.3f, 0.3f});
    box.position = {0.0f, 8.0f, 0.0f};
    box.linearVelocity = {0.0f, -25.0f, 0.0f};
    const uint32_t id = world.AddBody(box);
    Simulate(world, 2.0f);
    Check(world.GetBody(id).position.y > 5.0f, "a box at 25 metres per second does not pass through a 20 cm plate");
}

// Sleeping ------------------------------------------------------------------------------------

void AnImpulseWakesASleepingBody() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t box = AddBox(world, {0.0f, 0.5f, 0.0f});
    Simulate(world, 2.0f);
    Check(world.GetBody(box).isAsleep, "the box is asleep");
    world.ApplyImpulse(box, {3000.0f, 0.0f, 0.0f}, world.GetBody(box).position);
    Check(!world.GetBody(box).isAsleep, "an impulse wakes it");
    Simulate(world, 0.5f);
    Check(world.GetBody(box).position.x > 0.2f, "and it moves");
}

void ABallThatLandsOnASleepingBoxWakesIt() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t box = AddBox(world, {0.0f, 0.5f, 0.0f}, {1.0f, 0.5f, 1.0f});
    Simulate(world, 2.0f);
    Check(world.GetBody(box).isAsleep, "the box is asleep");
    BodyDescription ball;
    ball.shape = Shape::Sphere(0.4f);
    ball.density = 4000.0f;
    ball.position = {0.9f, 6.0f, 0.0f};
    const uint32_t ballId = world.AddBody(ball);
    bool woken = false;
    for (int i = 0; i < 360; ++i) {
        world.Step();
        woken = woken || !world.GetBody(box).isAsleep;
    }
    Check(woken, "a heavy ball dropped on the edge of the box wakes it");
    Check(world.GetBody(ballId).position.y > 0.0f, "and does not fall through");
}

// Determinism and energy -------------------------------------------------------------------

// A pile of boxes and balls dropped from pseudo random places, the same every time.
void BuildPile(PhysicsWorld& world, int count) {
    AddGround(world);
    uint32_t state = 12345u;
    const auto next = [&state]() {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) / 16777216.0f;
    };
    for (int i = 0; i < count; ++i) {
        const glm::vec3 position(next() * 6.0f - 3.0f, 1.0f + static_cast<float>(i) * 0.7f, next() * 6.0f - 3.0f);
        if (i % 3 == 0) {
            AddSphere(world, position, 0.3f + 0.2f * next());
        } else {
            BodyDescription box;
            box.shape = Shape::Box({0.3f + 0.3f * next(), 0.2f + 0.3f * next(), 0.3f + 0.3f * next()});
            box.position = position;
            box.orientation = glm::angleAxis(next() * 3.0f, glm::normalize(glm::vec3(next() + 0.1f, next(), next())));
            world.AddBody(box);
        }
    }
}

void TheSameWorldAlwaysGivesTheSameResult() {
    PhysicsWorld first;
    PhysicsWorld second;
    BuildPile(first, 40);
    BuildPile(second, 40);
    Simulate(first, 5.0f);
    Simulate(second, 5.0f);
    bool identical = true;
    for (uint32_t i = 0; i < first.BodyCount(); ++i) {
        identical = identical && first.GetBody(i).position == second.GetBody(i).position &&
                    first.GetBody(i).orientation == second.GetBody(i).orientation;
    }
    Check(identical, "two runs of the same pile are identical to the last bit");
}

void TheResultDoesNotDependOnTheNumberOfThreads() {
    // Big enough that the broadphase and the narrowphase are split into several chunks, and long enough that bodies fall
    // asleep and a late ball wakes some of them.
    JobSystem alone(0);
    JobSystem many(6);
    PhysicsWorld serial(PhysicsSettings{}, &alone);
    PhysicsWorld parallel(PhysicsSettings{}, &many);
    BuildPile(serial, 400);
    BuildPile(parallel, 400);
    for (PhysicsWorld* world : {&serial, &parallel}) {
        for (int i = 0; i < 30; ++i) {
            const float fi = static_cast<float>(i);
            AddCapsule(*world, {std::sin(fi) * 2.0f, 3.0f + fi * 0.9f, std::cos(fi) * 2.0f}, 0.2f, 0.3f,
                       glm::angleAxis(fi, glm::normalize(glm::vec3(1.0f, 0.5f * fi, 0.3f))));
        }
    }
    for (PhysicsWorld* world : {&serial, &parallel}) {
        Simulate(*world, 6.0f);
        const uint32_t ball = AddSphere(*world, {0.0f, 6.0f, 0.0f}, 0.4f);
        world->SetVelocity(ball, {0.0f, -8.0f, 0.0f}, glm::vec3(0.0f));
        Simulate(*world, 3.0f);
    }
    bool identical = serial.BodyCount() == parallel.BodyCount();
    size_t asleep = 0;
    for (uint32_t i = 0; identical && i < serial.BodyCount(); ++i) {
        identical = serial.GetBody(i).position == parallel.GetBody(i).position &&
                    serial.GetBody(i).orientation == parallel.GetBody(i).orientation &&
                    serial.GetBody(i).isAsleep == parallel.GetBody(i).isAsleep;
        asleep += serial.GetBody(i).isAsleep ? 1 : 0;
    }
    Check(identical, "a pile of 400 comes out the same on one thread and on seven, to the last bit");
    Check(asleep > 0, "and part of it was asleep, so that the waking was part of what was compared");
}

void APileSettlesDownAndNothingFallsThroughTheGround() {
    PhysicsWorld world;
    BuildPile(world, 40);
    Simulate(world, 12.0f);
    float lowest = 1e9f;
    float highest = -1e9f;
    for (uint32_t i = 1; i < world.BodyCount(); ++i) {
        lowest = std::min(lowest, world.GetBody(i).position.y);
        highest = std::max(highest, world.GetBody(i).position.y);
    }
    Check(lowest > 0.1f, "nothing sank into the ground, lowest centre " + std::to_string(lowest));
    Check(highest < 6.0f, "nothing was thrown out of the pile");
    Check(world.AwakeBodyCount() < 6, "and the pile has come to rest, awake " + std::to_string(world.AwakeBodyCount()));
}

void ThePileNeverGainsEnergy() {
    PhysicsWorld world;
    BuildPile(world, 40);
    const auto potential = [&world]() {
        float energy = 0.0f;
        for (uint32_t i = 1; i < world.BodyCount(); ++i) {
            const Body& body = world.GetBody(i);
            energy += 9.81f * body.position.y / body.inverseMass;
        }
        return energy;
    };
    const float initial = potential() + world.KineticEnergy();
    float worst = 0.0f;
    for (int i = 0; i < 1200; ++i) {
        world.Step();
        worst = std::max(worst, potential() + world.KineticEnergy() - initial);
    }
    // Pushing bodies out of overlaps adds a little energy; it must stay a tiny share of what the pile holds.
    Check(worst < 0.02f * initial, "the pile's energy never rises above its start, worst excess " + std::to_string(worst));
}

// Contacts ----------------------------------------------------------------------------------

void ABoxOnABoxMakesFourContactsWithAnUpwardNormal() {
    Body lower;
    lower.shape = Shape::Box({1.0f, 0.5f, 1.0f});
    lower.position = {0.0f, 0.5f, 0.0f};
    Body upper;
    upper.shape = Shape::Box({0.5f, 0.5f, 0.5f});
    upper.position = {0.2f, 1.49f, 0.1f};  // overlapping by one centimetre
    physics::Manifold manifold;
    Check(physics::Collide(lower, upper, manifold), "the boxes touch");
    Check(manifold.count == 4, "face against face makes four contacts, got " + std::to_string(manifold.count));
    Check(Near(manifold.normal.y, 1.0f, 1e-4f), "the normal points from the lower box to the upper one");
    for (int i = 0; i < manifold.count; ++i) {
        Check(Near(manifold.points[i].depth, 0.01f, 0.002f), "each contact is one centimetre deep");
    }
    physics::Manifold swapped;
    Check(physics::Collide(upper, lower, swapped) && Near(swapped.normal.y, -1.0f, 1e-4f), "swapping the bodies flips the normal");
}

void BoxesThatAreApartMakeNoContacts() {
    Body a;
    a.shape = Shape::Box({0.5f, 0.5f, 0.5f});
    Body b = a;
    b.position = {1.2f, 0.0f, 0.0f};
    physics::Manifold manifold;
    Check(!physics::Collide(a, b, manifold), "boxes 20 cm apart do not touch");
    b.position = {1.01f, 0.0f, 0.0f};
    Check(physics::Collide(a, b, manifold) && manifold.points[0].depth < 0.0f, "boxes 1 cm apart count as close and have negative depth");
}

void TwoEdgesCrossingMakeOneContact() {
    Body a;
    a.shape = Shape::Box({1.0f, 0.1f, 0.1f});
    a.position = {0.0f, 0.0f, 0.0f};
    Body b;
    b.shape = Shape::Box({0.1f, 0.1f, 1.0f});
    b.position = {0.0f, 0.19f, 0.0f};  // a crossed rod lying on the first, overlapping by a centimetre
    physics::Manifold manifold;
    Check(physics::Collide(a, b, manifold), "the rods touch");
    Check(Near(manifold.normal.y, 1.0f, 1e-3f), "and push apart vertically");
}

void ASphereInsideABoxIsPushedOutThroughTheNearestFace() {
    Body ball;
    ball.shape = Shape::Sphere(0.2f);
    ball.position = {0.9f, 0.0f, 0.0f};
    Body box;
    box.shape = Shape::Box({1.0f, 1.0f, 1.0f});
    physics::Manifold manifold;
    Check(physics::Collide(ball, box, manifold), "the ball touches the box");
    // The ball is 0.1 m inside the +X face, so the box is pushed away along -X from the ball.
    Check(Near(manifold.normal.x, -1.0f, 1e-4f), "the normal points from the ball into the box");
    Check(Near(manifold.points[0].depth, 0.3f, 1e-3f), "the overlap is the radius plus the distance to the face");
}

// Robustness, and the cases an independent review found -------------------------------------

bool IsFinite(const Body& body) {
    const auto finite = [](glm::vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); };
    return finite(body.position) && finite(body.linearVelocity) && finite(body.angularVelocity) &&
           std::isfinite(body.orientation.w);
}

// Capsules ----------------------------------------------------------------------------------

Body CapsuleBody(glm::vec3 position, glm::quat orientation, float radius = 0.3f, float halfHeight = 0.5f) {
    Body body;
    body.shape = Shape::Capsule(radius, halfHeight);
    body.position = position;
    body.orientation = orientation;
    return body;
}

// A capsule is stood along local Y. This turns it to lie along X or along Z.
const glm::quat kLieAlongX = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
const glm::quat kLieAlongZ = glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

glm::vec3 AxisOf(const Body& body) {
    return glm::mat3_cast(body.orientation)[1];
}

// Adds up the mass and the second moments of a capsule by testing a fine grid of points, as an independent
// check of the formulas in AddBody.
void IntegrateCapsule(float radius, float halfHeight, float density, double& mass, double& acrossAxis, double& alongAxis) {
    constexpr int kCells = 90;
    const double reachY = halfHeight + radius;
    const double cellVolume = (2.0 * radius / kCells) * (2.0 * radius / kCells) * (2.0 * reachY / kCells);
    mass = acrossAxis = alongAxis = 0.0;
    for (int ix = 0; ix < kCells; ++ix) {
        for (int iy = 0; iy < kCells; ++iy) {
            for (int iz = 0; iz < kCells; ++iz) {
                const double x = (ix + 0.5) / kCells * 2.0 * radius - radius;
                const double y = (iy + 0.5) / kCells * 2.0 * reachY - reachY;
                const double z = (iz + 0.5) / kCells * 2.0 * radius - radius;
                const double beyondEnd = std::max(std::abs(y) - halfHeight, 0.0);
                if (x * x + z * z + beyondEnd * beyondEnd > radius * radius) {
                    continue;
                }
                const double dm = density * cellVolume;
                mass += dm;
                acrossAxis += dm * (y * y + z * z);
                alongAxis += dm * (x * x + z * z);
            }
        }
    }
}

void ACapsuleHasTheMassAndInertiaOfItsVolume() {
    const float radius = 0.3f;
    const float halfHeight = 0.5f;
    PhysicsWorld world;
    const Body& body = world.GetBody(AddCapsule(world, glm::vec3(0.0f), radius, halfHeight));
    double mass;
    double across;
    double along;
    IntegrateCapsule(radius, halfHeight, 1000.0f, mass, across, along);
    Check(Near(1.0f / body.inverseMass, static_cast<float>(mass), 0.015f * static_cast<float>(mass)), "its mass is that of the cylinder and the two half spheres");
    Check(Near(1.0f / body.inverseInertiaLocal.x, static_cast<float>(across), 0.02f * static_cast<float>(across)), "its inertia across the axis matches the integral");
    Check(Near(1.0f / body.inverseInertiaLocal.y, static_cast<float>(along), 0.02f * static_cast<float>(along)), "and along the axis");
    Check(body.inverseInertiaLocal.x == body.inverseInertiaLocal.z, "and the two directions across the axis are the same");
}

void ACapsuleWithNoStraightPartIsASphere() {
    PhysicsWorld world;
    const Body& capsule = world.GetBody(AddCapsule(world, glm::vec3(0.0f), 0.4f, 0.0f));
    const Body& sphere = world.GetBody(AddSphere(world, glm::vec3(5.0f, 0.0f, 0.0f), 0.4f));
    Check(Near(capsule.inverseMass, sphere.inverseMass, 1e-4f * sphere.inverseMass), "it weighs what the sphere weighs");
    Check(Near(capsule.inverseInertiaLocal.x, sphere.inverseInertiaLocal.x, 1e-4f * sphere.inverseInertiaLocal.x) &&
              Near(capsule.inverseInertiaLocal.y, sphere.inverseInertiaLocal.y, 1e-4f * sphere.inverseInertiaLocal.y),
          "and has its inertia");
}

void TheBoundsOfACapsuleFollowItsAxis() {
    PhysicsWorld world;
    const Body& standing = world.GetBody(AddCapsule(world, {1.0f, 2.0f, 3.0f}, 0.3f, 0.5f));
    Check(Near(standing.boundsMax.y - standing.boundsMin.y, 1.6f, 1e-4f) && Near(standing.boundsMax.x - standing.boundsMin.x, 0.6f, 1e-4f),
          "a standing capsule is as tall as its straight part and both ends");
    const Body& lying = world.GetBody(AddCapsule(world, {1.0f, 2.0f, 3.0f}, 0.3f, 0.5f, kLieAlongX));
    Check(Near(lying.boundsMax.x - lying.boundsMin.x, 1.6f, 1e-4f) && Near(lying.boundsMax.y - lying.boundsMin.y, 0.6f, 1e-4f),
          "a capsule lying along X is long in X");
}

void BadCapsuleSizesAreCleanedUp() {
    PhysicsWorld world;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const uint32_t notANumber = AddCapsule(world, {0.0f, 1.0f, 0.0f}, 0.3f, nan);
    const uint32_t negative = AddCapsule(world, {3.0f, 1.0f, 0.0f}, 0.3f, -2.0f);
    AddGround(world);
    Simulate(world, 2.0f);
    Check(IsFinite(world.GetBody(notANumber)), "a half height that is not a number is replaced, and the capsule falls like a body");
    Check(world.GetBody(negative).shape.halfHeight == 0.0f, "a negative half height is zero, which makes a ball");
    Check(IsFinite(world.GetBody(negative)) && world.GetBody(negative).position.y > 0.2f, "and that capsule rests on the ground");
}

// What touches a capsule, and where -------------------------------------------------------------

void ACapsuleLyingOnABoxTouchesItAtBothEnds() {
    Body ground;
    ground.shape = Shape::Box({5.0f, 0.5f, 5.0f});
    ground.position = {0.0f, -0.5f, 0.0f};
    const Body lying = CapsuleBody({0.0f, 0.29f, 0.0f}, kLieAlongX);
    physics::Manifold manifold;
    Check(physics::Collide(lying, ground, manifold), "a capsule lying on the ground touches it");
    Check(manifold.count >= 2, "at two or more points, so it cannot rock on one: " + std::to_string(manifold.count));
    Check(Near(manifold.normal.y, -1.0f, 1e-3f), "the normal points from the capsule down into the ground");
    float widest = 0.0f;
    for (int i = 0; i < manifold.count; ++i) {
        widest = std::max(widest, std::abs(manifold.points[i].position.x));
        Check(Near(manifold.points[i].depth, 0.01f, 2e-3f), "each point is 1 cm deep");
    }
    Check(widest > 0.4f, "and the points are spread along the length of the capsule");
}

void AStandingCapsuleTouchesTheGroundAtOnePoint() {
    Body ground;
    ground.shape = Shape::Box({5.0f, 0.5f, 5.0f});
    ground.position = {0.0f, -0.5f, 0.0f};
    const Body standing = CapsuleBody({0.0f, 0.5f + 0.3f - 0.01f, 0.0f}, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    physics::Manifold manifold;
    Check(physics::Collide(standing, ground, manifold), "a capsule on its end touches the ground");
    Check(manifold.count == 1, "at one point: " + std::to_string(manifold.count));
    Check(Near(manifold.points[0].depth, 0.01f, 2e-3f), "1 cm deep");
}

void ACapsuleTiltedOnABoxTouchesAtTheLowEndOnly() {
    Body ground;
    ground.shape = Shape::Box({5.0f, 0.5f, 5.0f});
    ground.position = {0.0f, -0.5f, 0.0f};
    const glm::quat tilt = glm::angleAxis(glm::radians(40.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    Body capsule = CapsuleBody({0.0f, 1.0f, 0.0f}, tilt);
    const glm::vec3 axis = AxisOf(capsule);
    capsule.position.y = 0.5f * std::abs(axis.y) + 0.3f - 0.005f;
    physics::Manifold manifold;
    Check(physics::Collide(capsule, ground, manifold), "a tilted capsule touches the ground");
    Check(manifold.count == 1, "at its low end only: " + std::to_string(manifold.count));
}

void ParallelCapsulesTouchAlongTheirLengthAndCrossedOnesAtOnePoint() {
    const Body below = CapsuleBody({0.0f, 0.0f, 0.0f}, kLieAlongX);
    const Body above = CapsuleBody({0.0f, 0.59f, 0.0f}, kLieAlongX);
    physics::Manifold manifold;
    Check(physics::Collide(below, above, manifold), "two capsules lying one on the other touch");
    Check(manifold.count >= 2, "along their length, at two points or more: " + std::to_string(manifold.count));
    Check(Near(manifold.normal.y, 1.0f, 1e-3f), "with the normal from the lower to the upper");

    const Body across = CapsuleBody({0.0f, 0.59f, 0.0f}, kLieAlongZ);
    Check(physics::Collide(below, across, manifold), "a capsule laid across another touches it");
    Check(manifold.count == 1, "at one point: " + std::to_string(manifold.count));
    Check(Near(manifold.points[0].depth, 0.01f, 2e-3f), "1 cm deep");
}

void AnyOrderOfTwoShapesGivesTheSameContactTurnedRound() {
    Body ground;
    ground.shape = Shape::Box({5.0f, 0.5f, 5.0f});
    ground.position = {0.0f, -0.5f, 0.0f};
    Body ball;
    ball.shape = Shape::Sphere(0.3f);
    ball.position = {0.2f, 0.29f, 0.1f};
    const Body capsule = CapsuleBody({0.1f, 0.29f, 0.0f}, kLieAlongX);
    const Body otherCapsule = CapsuleBody({0.4f, 0.55f, 0.1f}, kLieAlongZ);
    const Body* shapes[] = {&ground, &ball, &capsule, &otherCapsule};
    for (const Body* first : shapes) {
        for (const Body* second : shapes) {
            if (first == second) {
                continue;
            }
            physics::Manifold forward;
            physics::Manifold backward;
            const bool touching = physics::Collide(*first, *second, forward);
            Check(touching == physics::Collide(*second, *first, backward), "the same pair touches whichever is named first");
            if (touching) {
                Check(forward.count == backward.count, "with the same number of points");
                Check(glm::dot(forward.normal, backward.normal) < -0.999f, "and opposite normals");
                Check(Near(forward.points[0].depth, backward.points[0].depth, 1e-4f), "and the same depth");
            }
        }
    }
}

void ACapsuleAcrossAPostLeavesTheWayItCameIn() {
    // The straight part passes through the post, so no end of the capsule is inside it.
    Body post;
    post.shape = Shape::Box({0.2f, 1.0f, 0.2f});
    const Body capsule = CapsuleBody({0.0f, 0.0f, 0.1f}, kLieAlongX, 0.1f, 1.0f);
    physics::Manifold manifold;
    Check(physics::Collide(capsule, post, manifold), "a capsule through a post touches it");
    Check(manifold.count >= 1 && manifold.points[0].depth > 0.1f, "and is deep in it");
    Check(Near(glm::length(manifold.normal), 1.0f, 1e-4f), "with a normal of unit length");
    // The nearest face is the +Z one: the capsule has to move towards +Z to get out, so the post moves towards -Z.
    Check(manifold.normal.z < -0.9f, "pointing the short way out: " + std::to_string(manifold.normal.x) + " " + std::to_string(manifold.normal.y) + " " + std::to_string(manifold.normal.z) + " count " + std::to_string(manifold.count));
}

void CapsulesMeetingSpheresAndBoxesAtTheirRoundEnds() {
    const Body capsule = CapsuleBody({0.0f, 0.0f, 0.0f}, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    Body ball;
    ball.shape = Shape::Sphere(0.2f);
    ball.position = {0.0f, 0.5f + 0.3f + 0.2f - 0.01f, 0.0f};
    physics::Manifold manifold;
    Check(physics::Collide(capsule, ball, manifold), "a ball sitting on the top of a capsule touches it");
    Check(Near(manifold.normal.y, 1.0f, 1e-3f) && Near(manifold.points[0].depth, 0.01f, 2e-3f), "straight above, 1 cm deep");
    ball.position = {0.3f + 0.2f - 0.01f, 0.2f, 0.0f};
    Check(physics::Collide(capsule, ball, manifold), "a ball beside the straight part touches it");
    Check(Near(manifold.normal.x, 1.0f, 1e-3f), "from the side");
    ball.position = {0.3f + 0.2f + 0.1f, 0.2f, 0.0f};
    Check(!physics::Collide(capsule, ball, manifold), "and a ball 10 cm further away is not");
}

// What capsules do ------------------------------------------------------------------------------

void ACapsuleThatIsNotQuiteUprightFallsOverAndLiesDown() {
    PhysicsWorld world;
    AddGround(world);
    const glm::quat lean = glm::angleAxis(glm::radians(4.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    const uint32_t capsule = AddCapsule(world, {0.0f, 0.8f + 0.02f, 0.0f}, 0.3f, 0.5f, lean);
    Simulate(world, 8.0f);
    const Body& body = world.GetBody(capsule);
    Check(std::abs(AxisOf(body).y) < 0.15f, "it ends up lying: axis height " + std::to_string(AxisOf(body).y));
    Check(Near(body.position.y, 0.3f, 0.03f), "with its axis one radius above the ground: " + std::to_string(body.position.y));
    Check(body.isAsleep, "and then falls asleep");
}

void ACapsuleLyingOnASlopeRollsDownIt() {
    PhysicsWorld world;
    BodyDescription slope;
    slope.shape = Shape::Box({12.0f, 0.5f, 6.0f});
    slope.orientation = glm::angleAxis(glm::radians(-15.0f), glm::vec3(0.0f, 0.0f, 1.0f));  // +X is downhill
    slope.isStatic = true;
    world.AddBody(slope);
    const uint32_t capsule = AddCapsule(world, {0.0f, 1.0f, 0.0f}, 0.3f, 0.5f, kLieAlongZ);
    Simulate(world, 2.5f);
    const Body& body = world.GetBody(capsule);
    Check(body.position.x > 1.0f, "it rolls downhill: x = " + std::to_string(body.position.x));
    Check(std::abs(body.position.z) < 0.3f, "straight down, not sideways: z = " + std::to_string(body.position.z));
    Check(std::abs(AxisOf(body).y) < 0.15f, "and stays lying across the slope");
}

void ACapsuleTurnedOnTheSpotOnTheGroundStopsAndSleeps() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t capsule = AddCapsule(world, {0.0f, 0.3f, 0.0f}, 0.3f, 0.5f, kLieAlongX);
    world.SetVelocity(capsule, glm::vec3(0.0f), {0.0f, 6.0f, 0.0f});
    Simulate(world, 10.0f);
    const Body& body = world.GetBody(capsule);
    Check(body.isAsleep, "a spinning capsule comes to rest");
    Check(glm::length(body.position - glm::vec3(0.0f, 0.3f, 0.0f)) < 0.3f, "close to where it was: " + std::to_string(body.position.x) + " " + std::to_string(body.position.y) + " " + std::to_string(body.position.z));
}

void ACapsuleRollsToAStopAcrossTheFloor() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t capsule = AddCapsule(world, {0.0f, 0.3f, 0.0f}, 0.3f, 0.5f, kLieAlongZ);
    world.SetVelocity(capsule, {4.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -4.0f / 0.3f});  // rolling without slipping
    Simulate(world, 20.0f);
    const Body& body = world.GetBody(capsule);
    Check(body.isAsleep, "it stops");
    Check(body.position.x > 1.0f && body.position.x < 25.0f, "after rolling a while: " + std::to_string(body.position.x));
}

void CapsulesDroppedOnEachOtherAndOnABallSettleAndSleep() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t first = AddCapsule(world, {0.0f, 0.5f, 0.0f}, 0.3f, 0.5f, kLieAlongX);
    const uint32_t second = AddCapsule(world, {0.1f, 1.5f, 0.0f}, 0.3f, 0.5f, kLieAlongZ);
    const uint32_t third = AddCapsule(world, {-0.2f, 2.5f, 0.1f}, 0.3f, 0.5f);
    const uint32_t ball = AddSphere(world, {0.0f, 4.0f, 0.0f}, 0.3f);
    Simulate(world, 15.0f);
    for (const uint32_t index : {first, second, third, ball}) {
        const Body& body = world.GetBody(index);
        Check(IsFinite(body) && body.position.y > 0.25f, "nothing fell through the ground");
        Check(body.isAsleep, "and everything came to rest");
    }
}

float EnergyAboveTheFloor(const Body& body, float restingHeight) {
    const float mass = 1.0f / body.inverseMass;
    const glm::mat3 inertiaWorld = glm::inverse(body.inverseInertiaWorld);
    return mass * 9.81f * (body.position.y - restingHeight) + 0.5f * mass * glm::dot(body.linearVelocity, body.linearVelocity) +
           0.5f * glm::dot(body.angularVelocity, inertiaWorld * body.angularVelocity);
}

void AnElasticCapsuleDroppedAtAnyTiltNeverGainsEnergy() {
    for (const float degrees : {0.0f, 20.0f, 45.0f, 70.0f, 90.0f}) {
        PhysicsWorld world;
        AddGround(world, 0.6f, 1.0f);
        BodyDescription description;
        description.shape = Shape::Capsule(0.3f, 0.5f);
        description.position = {0.0f, 2.0f, 0.0f};
        description.orientation = glm::angleAxis(glm::radians(degrees), glm::vec3(0.0f, 0.0f, 1.0f));
        description.material = {0.6f, 1.0f};
        const uint32_t capsule = world.AddBody(description);
        const float start = EnergyAboveTheFloor(world.GetBody(capsule), 0.3f);
        float most = start;
        for (int i = 0; i < 900; ++i) {
            world.Step();
            most = std::max(most, EnergyAboveTheFloor(world.GetBody(capsule), 0.3f));
        }
        Check(most < start * 1.05f, "a capsule dropped at " + std::to_string(degrees) + " degrees keeps its energy at most: " +
                                       std::to_string(most / start));
    }
}

void AFastSmallCapsuleDoesNotPassThroughAThinPlate() {
    int passedThrough = 0;
    for (const float degrees : {0.0f, 30.0f, 90.0f}) {
        for (int phase = 0; phase < 12; ++phase) {
            PhysicsWorld world;
            BodyDescription plate;
            plate.shape = Shape::Box({5.0f, 0.05f, 5.0f});
            plate.position = {0.0f, 5.0f, 0.0f};
            plate.isStatic = true;
            world.AddBody(plate);
            BodyDescription description;
            description.shape = Shape::Capsule(0.05f, 0.1f);
            description.position = {0.0f, 6.0f + 0.01f * static_cast<float>(phase), 0.0f};
            description.orientation = glm::angleAxis(glm::radians(degrees), glm::vec3(0.0f, 0.0f, 1.0f));
            description.linearVelocity = {0.0f, -40.0f, 0.0f};
            const uint32_t capsule = world.AddBody(description);
            Simulate(world, 0.5f);
            passedThrough += world.GetBody(capsule).position.y < 5.0f ? 1 : 0;
        }
    }
    Check(passedThrough == 0, "a small capsule at 40 m/s stays above a 10 cm plate at every tilt and phase, passed " +
                                  std::to_string(passedThrough));
}

void ACapsuleThrownOntoAHeapOfBoxesLeavesItAsleep() {
    PhysicsWorld world;
    AddGround(world);
    for (int i = 0; i < 6; ++i) {
        AddBox(world, {0.1f * static_cast<float>(i % 2), 0.5f + static_cast<float>(i), 0.0f});
    }
    const uint32_t capsule = AddCapsule(world, {0.3f, 9.0f, 0.0f}, 0.3f, 0.5f, kLieAlongZ);
    world.SetVelocity(capsule, {0.0f, -3.0f, 0.0f}, glm::vec3(0.0f));
    Simulate(world, 15.0f);
    bool allAsleep = true;
    for (uint32_t i = 0; i < world.BodyCount(); ++i) {
        if (world.GetBody(i).isStatic) {
            continue;
        }
        allAsleep = allAsleep && world.GetBody(i).isAsleep;
        Check(IsFinite(world.GetBody(i)) && world.GetBody(i).position.y > 0.2f, "nothing fell through the ground");
    }
    Check(allAsleep, "the stack and the capsule come to rest");
}

void BadInputDoesNotPoisonTheWorld() {
    PhysicsWorld world;
    const uint32_t ground = AddGround(world);
    const uint32_t good = AddBox(world, {8.0f, 3.0f, 0.0f});
    AddBox(world, {0.5f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f});   // a box with no size
    AddBox(world, {-0.5f, 1.0f, 0.0f}, {0.0f, 0.5f, 0.5f});  // a box that is flat
    AddSphere(world, {1.0f, 1.0f, 0.0f}, 0.0f);              // a ball with no radius
    BodyDescription broken;
    broken.shape = Shape::Box({0.5f, 0.5f, 0.5f});
    broken.position = {std::nanf(""), 2.0f, 0.0f};
    broken.linearVelocity = {std::numeric_limits<float>::infinity(), 0.0f, 0.0f};
    world.AddBody(broken);
    Simulate(world, 2.0f);
    bool allFinite = true;
    for (uint32_t i = 0; i < world.BodyCount(); ++i) {
        allFinite = allFinite && IsFinite(world.GetBody(i));
    }
    Check(allFinite, "no body ends up with a number that is not a number");
    Check(world.GetBody(ground).position == glm::vec3(0.0f, -0.5f, 0.0f), "and the ground has not moved");
    Check(Near(world.GetBody(good).position.y, 0.5f, 0.03f), "a good box far from the bad ones still lands on the ground");
}

void ABodyStartingBadlyDoesNotSpreadToOthers() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t neighbour = AddBox(world, {3.0f, 0.5f, 0.0f});
    AddSphere(world, {0.0f, 0.5f, 0.0f}, 0.0f);
    Simulate(world, 2.0f);
    Check(IsFinite(world.GetBody(neighbour)) && Near(world.GetBody(neighbour).position.y, 0.5f, 0.02f),
          "a box on the same ground is unaffected");
}

void AHeavyBallLandingOnASleepingBoxDoesNotPushItIntoTheGround() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t box = AddBox(world, {0.0f, 0.5f, 0.0f}, {1.0f, 0.5f, 1.0f});
    Simulate(world, 2.0f);
    Check(world.GetBody(box).isAsleep, "the box is asleep");
    BodyDescription ball;
    ball.shape = Shape::Sphere(0.4f);
    ball.density = 20000.0f;
    ball.position = {0.9f, 6.0f, 0.0f};
    ball.linearVelocity = {0.0f, -30.0f, 0.0f};
    world.AddBody(ball);
    float lowest = 1e9f;
    for (int i = 0; i < 240; ++i) {
        world.Step();
        lowest = std::min(lowest, world.GetBody(box).position.y);
    }
    Check(lowest > 0.47f, "the box is not driven into the ground when it is woken, lowest centre " + std::to_string(lowest));
}

void ASleepingStackWakesAsAWhole() {
    PhysicsWorld world;
    AddGround(world);
    std::vector<uint32_t> stack;
    for (int i = 0; i < 4; ++i) {
        stack.push_back(AddBox(world, {0.0f, 0.5f + static_cast<float>(i), 0.0f}));
    }
    Simulate(world, 2.0f);
    Check(world.AwakeBodyCount() == 0, "the stack sleeps");
    BodyDescription ball;
    ball.shape = Shape::Sphere(0.3f);
    ball.density = 3000.0f;
    ball.position = {0.0f, 8.0f, 0.0f};
    world.AddBody(ball);
    bool lowerWoke = false;
    for (int i = 0; i < 240; ++i) {  // the ball takes about 1.3 seconds to arrive
        world.Step();
        lowerWoke = lowerWoke || !world.GetBody(stack[0]).isAsleep;
    }
    Check(lowerWoke, "a ball hitting the top of the stack wakes the bottom box too");
}

void APerfectlyElasticBounceKeepsItsHeightWhateverTheFramePhase() {
    float worst = 1.0f;
    float best = 0.0f;
    // The ball starts at heights spread over the distance it falls in one step at the moment of impact.
    for (int phase = 0; phase < 20; ++phase) {
        const float drop = 3.0f + 0.0033f * static_cast<float>(phase);  // 6.5 cm in all: a ball falls that far in a step
        PhysicsWorld world;
        AddGround(world, 0.6f, 1.0f);
        BodyDescription description;
        description.shape = Shape::Sphere(0.25f);
        description.position = {0.0f, 0.25f + drop, 0.0f};
        description.material = {0.6f, 1.0f, 0.0f};
        const uint32_t ball = world.AddBody(description);
        bool bounced = false;
        float apex = 0.0f;
        for (int i = 0; i < 400; ++i) {
            world.Step();
            const Body& body = world.GetBody(ball);
            bounced = bounced || body.linearVelocity.y > 1.0f;
            if (bounced) {
                apex = std::max(apex, body.position.y - 0.25f);
            }
        }
        worst = std::min(worst, apex / drop);
        best = std::max(best, apex / drop);
    }
    Check(worst > 0.9f, "a perfectly elastic ball always comes back up, worst ratio " + std::to_string(worst));
    Check(best < 1.03f, "and never higher than it fell from, best ratio " + std::to_string(best));
}

void BouncinessIsTheLargerOfTheTwoMaterials() {
    PhysicsWorld world;
    AddGround(world, 0.6f, 0.2f);
    BodyDescription description;
    description.shape = Shape::Sphere(0.25f);
    description.position = {0.0f, 5.25f, 0.0f};
    description.material = {0.6f, 0.8f, 0.0f};
    const uint32_t ball = world.AddBody(description);
    float apex = 0.0f;
    bool bounced = false;
    for (int i = 0; i < 400; ++i) {
        world.Step();
        const Body& body = world.GetBody(ball);
        bounced = bounced || body.linearVelocity.y > 1.0f;
        if (bounced) {
            apex = std::max(apex, body.position.y - 0.25f);
        }
    }
    // The pair bounces as the livelier surface does: 0.8, so 64% of the height.
    Check(apex > 2.5f, "a lively ball on a dead floor still bounces, apex " + std::to_string(apex));
}

void SmallFastBodiesDoNotTunnelThroughAThinPlate() {
    int passedThrough = 0;
    for (int phase = 0; phase < 12; ++phase) {
        PhysicsWorld world;
        BodyDescription plate;
        plate.shape = Shape::Box({5.0f, 0.05f, 5.0f});
        plate.position = {0.0f, 5.0f, 0.0f};
        plate.isStatic = true;
        world.AddBody(plate);
        BodyDescription ball;
        ball.shape = Shape::Sphere(0.05f);
        ball.position = {0.0f, 6.0f + 0.01f * static_cast<float>(phase), 0.0f};
        ball.linearVelocity = {0.0f, -40.0f, 0.0f};
        const uint32_t id = world.AddBody(ball);
        Simulate(world, 0.5f);
        passedThrough += world.GetBody(id).position.y < 5.0f ? 1 : 0;
    }
    Check(passedThrough == 0, "a 10 cm ball at 40 m/s stays above a 10 cm plate at every phase, passed " +
                                  std::to_string(passedThrough));
}

void BoxesMeetingHeadOnStopOrSwapSpeedsAsRestitutionSays() {
    for (float restitution : {0.0f, 1.0f}) {
        PhysicsSettings settings;
        settings.gravity = glm::vec3(0.0f);
        PhysicsWorld world(settings);
        BodyDescription left;
        left.shape = Shape::Box({0.5f, 0.5f, 0.5f});
        left.position = {-3.0f, 0.0f, 0.0f};
        left.linearVelocity = {10.0f, 0.0f, 0.0f};
        left.material = {0.0f, restitution, 0.0f};
        BodyDescription right = left;
        right.position = {3.0f, 0.0f, 0.0f};
        right.linearVelocity = {-10.0f, 0.0f, 0.0f};
        const uint32_t a = world.AddBody(left);
        const uint32_t b = world.AddBody(right);
        Simulate(world, 1.0f);
        const float speedA = world.GetBody(a).linearVelocity.x;
        const float speedB = world.GetBody(b).linearVelocity.x;
        const std::string speeds = " got " + std::to_string(speedA) + " and " + std::to_string(speedB);
        if (restitution == 0.0f) {
            Check(std::abs(speedA) < 0.3f && std::abs(speedB) < 0.3f, "equal boxes with no bounce stop dead," + speeds);
        } else {
            Check(Near(speedA, -10.0f, 0.8f) && Near(speedB, 10.0f, 0.8f), "equal elastic boxes swap speeds," + speeds);
        }
        Check(Near(speedA + speedB, 0.0f, 0.05f), "and momentum is conserved");
    }
}

void ABoxStartingDeepInTheGroundRisesOutGently() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t box = AddBox(world, {0.0f, 0.0f, 0.0f});  // half of it below the surface
    float fastest = 0.0f;
    float highest = 0.0f;
    for (int i = 0; i < 480; ++i) {
        world.Step();
        fastest = std::max(fastest, glm::length(world.GetBody(box).linearVelocity));
        highest = std::max(highest, world.GetBody(box).position.y);
    }
    Check(fastest < 1.0f, "pushing a box out of an overlap does not fling it, fastest " + std::to_string(fastest));
    Check(highest < 0.55f, "it does not shoot above where it rests, highest " + std::to_string(highest));
    Check(Near(world.GetBody(box).position.y, 0.5f, 0.015f), "and it ends up resting on the ground");
}

void ABoxHangingPastItsSupportTipsOverAndOneOverTheEdgeStays() {
    for (float overhang : {0.45f, 0.53f}) {
        PhysicsWorld world;
        AddGround(world);
        AddBox(world, {0.0f, 0.5f, 0.0f});
        const uint32_t upper = AddBox(world, {overhang, 1.5f, 0.0f});
        Simulate(world, 3.0f);
        if (overhang < 0.5f) {
            Check(Near(world.GetBody(upper).position.y, 1.5f, 0.03f), "a cube with its centre over the edge stays up");
        } else {
            Check(world.GetBody(upper).position.y < 1.2f, "a cube with its centre past the edge falls off");
        }
    }
}

void ASpinningBodyKeepsItsOrientationNormal() {
    PhysicsSettings settings;
    settings.gravity = glm::vec3(0.0f);
    PhysicsWorld world(settings);
    BodyDescription box;
    box.shape = Shape::Box({1.0f, 0.5f, 0.25f});
    box.angularVelocity = {7.0f, 5.0f, 3.0f};
    const uint32_t id = world.AddBody(box);
    Simulate(world, 10.0f);
    Check(Near(glm::length(world.GetBody(id).orientation), 1.0f, 1e-4f),
          "the orientation is still a unit quaternion after ten seconds of spin");
}

void CrossedEdgesAreFoundAsAnEdgeContact() {
    // Two cubes standing on an edge, one turned about Z and one about X, the second lying on the first.
    Body lower;
    lower.shape = Shape::Box({0.5f, 0.5f, 0.5f});
    lower.orientation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    Body upper;
    upper.shape = Shape::Box({0.5f, 0.5f, 0.5f});
    upper.orientation = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    const float edge = 0.5f * std::sqrt(2.0f);
    upper.position = {0.0f, 2.0f * edge - 0.01f, 0.0f};  // the edges overlap by a centimetre
    physics::Manifold manifold;
    Check(physics::Collide(lower, upper, manifold), "the cubes touch");
    Check(manifold.count == 1, "crossed edges make a single contact, got " + std::to_string(manifold.count));
    Check(manifold.normal.y > 0.99f, "and push apart straight up");
    Check(Near(manifold.points[0].depth, 0.01f, 0.003f), "by one centimetre");
}

void ContactsOfAnOverhangingBoxStayOnTheFaceBelow() {
    Body lower;
    lower.shape = Shape::Box({1.0f, 0.5f, 1.0f});
    lower.position = {0.0f, 0.5f, 0.0f};
    Body upper;
    upper.shape = Shape::Box({0.5f, 0.5f, 0.5f});
    upper.position = {1.2f, 1.49f, 0.0f};  // half of it hangs over the edge
    physics::Manifold manifold;
    Check(physics::Collide(lower, upper, manifold), "the boxes touch");
    float furthest = -1e9f;
    for (int i = 0; i < manifold.count; ++i) {
        furthest = std::max(furthest, manifold.points[i].position.x);
    }
    Check(furthest <= 1.0f + 1e-3f, "no contact lies beyond the edge of the box below, furthest " + std::to_string(furthest));
}

void AHeavyBoxOnLightOnesDoesNotScatterThem() {
    PhysicsWorld world;
    AddGround(world);
    std::vector<uint32_t> light;
    for (int i = 0; i < 3; ++i) {
        BodyDescription box;
        box.shape = Shape::Box({0.5f, 0.5f, 0.5f});
        box.position = {0.0f, 0.5f + static_cast<float>(i), 0.0f};
        box.density = 100.0f;
        light.push_back(world.AddBody(box));
    }
    BodyDescription heavy;
    heavy.shape = Shape::Box({0.5f, 0.5f, 0.5f});
    heavy.position = {0.0f, 3.5f, 0.0f};
    heavy.density = 1000.0f;  // ten times the others
    const uint32_t top = world.AddBody(heavy);
    Simulate(world, 6.0f);
    float drift = 0.0f;
    for (uint32_t id : light) {
        drift = std::max(drift, std::abs(world.GetBody(id).position.x) + std::abs(world.GetBody(id).position.z));
    }
    drift = std::max(drift, std::abs(world.GetBody(top).position.x) + std::abs(world.GetBody(top).position.z));
    Check(drift < 0.15f, "a box ten times heavier on top of three light ones keeps the stack, drift " + std::to_string(drift));
}

// What the second review found -------------------------------------------------------------------

void ATenHighStackFallsAsleep() {
    PhysicsWorld world;
    AddGround(world);
    std::vector<uint32_t> stack;
    for (int i = 0; i < 10; ++i) {
        stack.push_back(AddBox(world, {0.0f, 0.5f + static_cast<float>(i), 0.0f}));
    }
    Simulate(world, 20.0f);
    Check(world.AwakeBodyCount() == 0, "a stack of ten boxes comes to rest, awake " + std::to_string(world.AwakeBodyCount()));
    Check(Near(world.GetBody(stack.back()).position.y, 9.5f, 0.1f), "and stands at its height");
}

void ADenseHeapOfThreeHundredBodiesSettles() {
    PhysicsWorld world;
    AddGround(world);
    uint32_t state = 777u;
    const auto next = [&state]() {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) / 16777216.0f;
    };
    for (int i = 0; i < 300; ++i) {
        const glm::vec3 position(static_cast<float>(i % 6) * 1.1f - 2.75f + next() * 0.2f,
                                 0.6f + static_cast<float>(i / 36) * 1.1f, static_cast<float>((i / 6) % 6) * 1.1f - 2.75f + next() * 0.2f);
        if (i % 4 == 0) {
            AddSphere(world, position, 0.35f);
        } else {
            BodyDescription box;
            box.shape = Shape::Box({0.4f, 0.3f, 0.4f});
            box.position = position;
            box.orientation = glm::angleAxis(next() * 3.0f, glm::normalize(glm::vec3(next() + 0.1f, next(), next())));
            world.AddBody(box);
        }
    }
    Simulate(world, 25.0f);
    Check(world.AwakeBodyCount() < 30, "a heap of three hundred bodies mostly comes to rest, awake " + std::to_string(world.AwakeBodyCount()));
}

void AnElasticBoxDroppedAtAnyTiltNeverGainsEnergy() {
    struct Drop {
        float degrees;
        glm::vec3 axis;
    };
    const Drop drops[] = {{0.0f, {1.0f, 0.0f, 0.0f}},  {20.0f, {1.0f, 0.0f, 0.5f}}, {40.0f, {1.0f, 0.0f, 0.5f}},
                          {25.0f, {1.0f, 0.0f, 0.0f}}, {55.0f, {1.0f, 0.0f, 1.0f}}, {35.0f, {0.0f, 0.0f, 1.0f}}};
    for (const Drop& drop : drops) {
        const float degrees = drop.degrees;
        PhysicsWorld world;
        AddGround(world, 0.6f, 1.0f);
        BodyDescription description;
        description.shape = Shape::Box({0.5f, 0.5f, 0.5f});
        description.position = {0.0f, 3.0f, 0.0f};
        description.orientation = glm::angleAxis(glm::radians(degrees), glm::normalize(drop.axis));
        description.material = {0.6f, 1.0f, 0.0f};
        const uint32_t box = world.AddBody(description);
        const float mass = 1.0f / world.GetBody(box).inverseMass;
        const float initial = 9.81f * mass * 3.0f;
        float worst = 0.0f;
        for (int i = 0; i < 600; ++i) {
            world.Step();
            worst = std::max(worst, 9.81f * mass * world.GetBody(box).position.y + world.KineticEnergy() - initial);
        }
        // Several points bounce at once, and a sequential impulse solver satisfies their targets only
        // approximately: a tilted perfectly elastic box may come back up to a few percent too high.
        Check(worst < 0.05f * initial, "an elastic box dropped at " + std::to_string(degrees) +
                                           " degrees keeps its energy, excess " + std::to_string(worst / initial));
    }
}

void ABoxWellAboveAPlateStillMakesAFaceContact() {
    Body plate;
    plate.shape = Shape::Box({5.0f, 0.1f, 5.0f});
    Body box;
    box.shape = Shape::Box({0.1f, 0.1f, 0.1f});
    box.position = {0.1f, 0.2f + 0.25f, 0.1f};  // a quarter of a metre above the plate
    physics::Manifold manifold;
    Check(physics::Collide(plate, box, manifold, 0.4f), "the box is within the margin");
    Check(manifold.count == 4 && manifold.normal.y > 0.99f,
          "and the contact is the face below it, not an edge, got " + std::to_string(manifold.count) + " contacts");
    for (int i = 0; i < manifold.count; ++i) {
        Check(std::abs(manifold.points[i].position.z) < 1.0f, "with every point under the box");
    }
}

void TwoFastBallsMeetingHeadOnDoNotPassThroughEachOther() {
    int passed = 0;
    for (int phase = 0; phase < 24; ++phase) {
        PhysicsSettings settings;
        settings.gravity = glm::vec3(0.0f);
        PhysicsWorld world(settings);
        BodyDescription left;
        left.shape = Shape::Sphere(0.05f);
        left.position = {-2.0f + 0.01f * static_cast<float>(phase), 0.0f, 0.0f};
        left.linearVelocity = {60.0f, 0.0f, 0.0f};
        left.material = {0.0f, 1.0f, 0.0f};
        BodyDescription right = left;
        right.position = {2.0f, 0.0f, 0.0f};
        right.linearVelocity = {-60.0f, 0.0f, 0.0f};
        const uint32_t a = world.AddBody(left);
        const uint32_t b = world.AddBody(right);
        Simulate(world, 0.2f);
        passed += world.GetBody(a).position.x > world.GetBody(b).position.x ? 1 : 0;
    }
    Check(passed == 0, "balls meeting at 120 m/s together never swap sides, swapped " + std::to_string(passed));
}

void AnImpossiblyHeavyFastBodyDoesNotCorruptTheGround() {
    PhysicsWorld world;
    const uint32_t ground = AddGround(world);
    const uint32_t far = AddBox(world, {20.0f, 0.5f, 0.0f});
    BodyDescription ball;
    ball.shape = Shape::Sphere(0.5f);
    ball.density = 1e30f;
    ball.position = {0.0f, 0.5f, 0.0f};
    ball.linearVelocity = {0.0f, -3e37f, 0.0f};
    world.AddBody(ball);
    Simulate(world, 1.0f);
    const Body& body = world.GetBody(ground);
    Check(IsFinite(body) && body.linearVelocity == glm::vec3(0.0f), "the ground is never touched by the solver");
    Check(IsFinite(world.GetBody(far)) && Near(world.GetBody(far).position.y, 0.5f, 0.02f), "and neither is a box far away");
}

void ALongUpdateStepIsNotSlowMotion() {
    PhysicsWorld world;
    const uint32_t ball = AddSphere(world, {0.0f, 10.0f, 0.0f});
    for (int i = 0; i < 10; ++i) {
        world.Update(0.1f);  // a slow frame: more than a handful of fixed steps each
    }
    Check(Near(world.GetBody(ball).position.y, 10.0f - 4.905f, 0.1f),
          "a ball falls 4.9 m in a second of long updates, got " + std::to_string(10.0f - world.GetBody(ball).position.y));
}

void ADisabledBodyStaysWhereItIsAndCostsNothing() {
    PhysicsWorld world;
    const uint32_t ball = AddSphere(world, {0.0f, 10.0f, 0.0f});
    Simulate(world, 0.5f);
    world.Disable(ball);
    const glm::vec3 where = world.GetBody(ball).position;
    Simulate(world, 2.0f);
    Check(world.GetBody(ball).position == where, "a disabled body does not move");
    Check(world.AwakeBodyCount() == 0, "and is not awake");
}

// Forces from outside ----------------------------------------------------------------------------

void AForceAcceleratesABodyAsNewtonSays() {
    PhysicsSettings settings;
    settings.gravity = glm::vec3(0.0f);
    PhysicsWorld world(settings);
    const uint32_t box = AddBox(world, {0.0f, 0.0f, 0.0f});  // a tonne at 1000 kg per cubic metre and 1 m across
    const float mass = 1.0f / world.GetBody(box).inverseMass;
    for (int i = 0; i < 120; ++i) {
        world.AddForce(box, {2000.0f, 0.0f, 0.0f}, world.GetBody(box).position);  // every step, for a second
        world.Step();
    }
    Check(Near(world.GetBody(box).linearVelocity.x, 2000.0f / mass, 0.02f), "two thousand newtons on a tonne give two metres a second in a second");
    Check(glm::length(world.GetBody(box).angularVelocity) < 1e-5f, "and no spin, since it pushes through the middle");
}

void AForceOffCentreSpinsTheBody() {
    PhysicsSettings settings;
    settings.gravity = glm::vec3(0.0f);
    PhysicsWorld world(settings);
    const uint32_t box = AddBox(world, {0.0f, 0.0f, 0.0f});
    for (int i = 0; i < 60; ++i) {
        world.AddForce(box, {0.0f, 0.0f, 1000.0f}, {0.5f, 0.0f, 0.0f});  // pushing the +X side towards +Z turns it about Y
        world.Step();
    }
    Check(world.GetBody(box).angularVelocity.y < -0.1f || world.GetBody(box).angularVelocity.y > 0.1f, "a push at the side of a body turns it");
    Check(world.GetBody(box).linearVelocity.z > 0.1f, "as well as moving it");
}

void AForceLastsOneStepOnly() {
    PhysicsSettings settings;
    settings.gravity = glm::vec3(0.0f);
    PhysicsWorld world(settings);
    const uint32_t box = AddBox(world, {0.0f, 0.0f, 0.0f});
    world.AddForce(box, {60000.0f, 0.0f, 0.0f}, world.GetBody(box).position);  // enough to move it faster than it falls asleep at
    world.Step();
    const float afterOne = world.GetBody(box).linearVelocity.x;
    for (int i = 0; i < 60; ++i) {
        world.Step();
    }
    Check(afterOne > 0.0f && Near(world.GetBody(box).linearVelocity.x, afterOne, 1e-5f), "a force given once acts once, and the body goes on at the speed it was given");
}

void ForcesOnBodiesThatCannotMoveOrAreNotNumbersAreIgnored() {
    PhysicsSettings settings;
    settings.gravity = glm::vec3(0.0f);
    PhysicsWorld world(settings);
    const uint32_t ground = AddGround(world);
    const uint32_t box = AddBox(world, {0.0f, 5.0f, 0.0f});
    world.AddForce(ground, {1e6f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
    world.AddForce(box, {std::nanf(""), 0.0f, 0.0f}, {0.0f, 5.0f, 0.0f});
    world.AddForce(box, {1.0f, 0.0f, 0.0f}, {std::numeric_limits<float>::infinity(), 0.0f, 0.0f});
    Simulate(world, 1.0f);
    Check(world.GetBody(ground).position == glm::vec3(0.0f, -0.5f, 0.0f), "the ground is not pushed");
    Check(IsFinite(world.GetBody(box)) && glm::length(world.GetBody(box).linearVelocity) < 1e-6f, "and a force that is not a number does nothing");
}

}  // namespace

// Kinematic bodies -----------------------------------------------------------------------------

// A person as the crowd gives one to the physics: a box 50 cm wide and 1.8 m tall, standing on y = 0 at `feet`.
uint32_t AddPerson(PhysicsWorld& world, glm::vec3 feet) {
    BodyDescription person;
    person.shape = Shape::Box({0.25f, 0.9f, 0.25f});
    person.position = feet + glm::vec3(0.0f, 0.9f, 0.0f);
    person.isKinematic = true;
    return world.AddBody(person);
}

// Walks the person along +X at `speed` for `seconds`, one step at a time, as the crowd does every frame.
void WalkPerson(PhysicsWorld& world, uint32_t person, float speed, float seconds) {
    const float step = world.Settings().fixedStep;
    const int steps = static_cast<int>(std::lround(seconds / step));
    for (int i = 0; i < steps; ++i) {
        world.SetKinematicTarget(person, world.GetBody(person).position + glm::vec3(speed * step, 0.0f, 0.0f), step);
        world.Step();
    }
}

void AWalkingPersonPushesABallAheadAndIsNotSlowedByIt() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t ball = AddSphere(world, {2.0f, 0.3f, 0.0f}, 0.3f);
    const uint32_t person = AddPerson(world, {0.0f, 0.0f, 0.0f});
    WalkPerson(world, person, 1.5f, 3.0f);
    Check(Near(world.GetBody(person).position.x, 4.5f, 0.02f), "the person walks the same way with or without the ball");
    Check(Near(world.GetBody(person).position.y, 0.9f, 1e-4f), "and does not rise or sink");
    Check(world.GetBody(ball).position.x > world.GetBody(person).position.x, "the ball is ahead of the person");
    Check(world.GetBody(ball).position.x < 7.0f, "and not thrown far ahead, it is at " + std::to_string(world.GetBody(ball).position.x));
    Check(Near(world.GetBody(ball).position.z, 0.0f, 0.4f), "pushed in the way the person walks");
}

void AFastPersonDoesNotRunThroughABall() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t ball = AddSphere(world, {0.7f, 0.3f, 0.0f}, 0.3f);
    const uint32_t person = AddPerson(world, {0.0f, 0.0f, 0.0f});
    WalkPerson(world, person, 6.0f, 1.0f);  // a runner, faster than the crowd goes
    Check(world.GetBody(ball).position.x >= world.GetBody(person).position.x, "a ball is not run through by a fast person");
}

void KinematicBodiesTouchingNothingThatMovesStayWhereTheyAre() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t first = AddPerson(world, {0.0f, 0.0f, 0.0f});
    const uint32_t second = AddPerson(world, {0.1f, 0.0f, 0.0f});  // overlapping the first
    Simulate(world, 2.0f);
    for (uint32_t person : {first, second}) {
        const glm::vec3 at = world.GetBody(person).position;
        Check(std::isfinite(at.x) && std::isfinite(at.y) && std::isfinite(at.z), "a person on the ground has a position");
        Check(Near(at.y, 0.9f, 1e-5f), "and has not sunk into it");
    }
    Check(Near(world.GetBody(first).position.x, 0.0f, 1e-5f) && Near(world.GetBody(second).position.x, 0.1f, 1e-5f),
          "two people who overlap do not push each other");
    Check(std::isfinite(world.KineticEnergy()), "energy of a world with people in it is a number");
    Check(world.ContactPairCount() == 0, "nothing needs solving between people and the ground");
}

void NothingPushesAKinematicBody() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t person = AddPerson(world, {0.0f, 0.0f, 0.0f});
    const uint32_t crate = AddBox(world, {0.0f, 3.0f, 0.0f}, {0.4f, 0.4f, 0.4f});  // falls on the person's head
    world.ApplyImpulse(person, {1000.0f, 0.0f, 0.0f}, {0.0f, 0.9f, 0.0f});
    world.AddForce(person, {1000.0f, 0.0f, 0.0f}, {0.0f, 0.9f, 0.0f});
    Simulate(world, 2.0f);
    Check(Near(world.GetBody(person).position.x, 0.0f, 1e-5f) && Near(world.GetBody(person).position.y, 0.9f, 1e-5f),
          "an impulse, a force and a falling crate do not move a person");
    Check(glm::length(world.GetBody(person).linearVelocity) < 1e-5f, "or give them speed");
    Check(world.GetBody(crate).position.y > 0.0f, "and the crate has not gone through anything");
}

void ACrateThatFallsOnAPersonLandsOnTheGroundBesideThemOrOnTheirHead() {
    PhysicsWorld world;
    AddGround(world);
    AddPerson(world, {0.0f, 0.0f, 0.0f});
    const uint32_t crate = AddBox(world, {0.3f, 3.0f, 0.0f}, {0.4f, 0.4f, 0.4f});
    Simulate(world, 4.0f);
    const Body& body = world.GetBody(crate);
    const bool onTheGround = Near(body.position.y, 0.4f, 0.05f);
    const bool onTheHead = Near(body.position.y, 1.8f + 0.4f, 0.06f);
    Check(onTheGround || onTheHead, "the crate rests on the ground or on the head, at height " + std::to_string(body.position.y));
    Check(body.isAsleep, "and sleeps");
}

void AHeapSleepsBesideAStillPersonWakesForAWalkingOneAndSleepsAgain() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t ball = AddSphere(world, {0.0f, 0.3f, 0.0f}, 0.3f);
    Simulate(world, 2.0f);
    Check(world.GetBody(ball).isAsleep, "the ball is asleep");

    const uint32_t person = AddPerson(world, {1.5f, 0.0f, 0.0f});
    Simulate(world, 1.0f);
    Check(world.GetBody(ball).isAsleep, "a person standing still some way off does not wake it");

    // The person walks to the ball, which they push.
    const float step = world.Settings().fixedStep;
    bool woken = false;
    for (int i = 0; i < 120; ++i) {
        world.SetKinematicTarget(person, world.GetBody(person).position - glm::vec3(1.0f * step, 0.0f, 0.0f), step);
        world.Step();
        woken = woken || !world.GetBody(ball).isAsleep;
    }
    Check(woken, "a person walking up to the ball wakes it");

    world.SetParked(person, true);
    Simulate(world, 5.0f);
    Check(world.GetBody(ball).isAsleep, "and when the person is gone the ball goes to sleep again, it is at speed " +
                                              std::to_string(glm::length(world.GetBody(ball).linearVelocity)));
}

void AStillPersonInAHeapDoesNotKeepItAwakeOrJoinTwoHeaps() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t left = AddBox(world, {-0.6f, 0.5f, 0.0f});
    const uint32_t right = AddBox(world, {0.6f, 0.5f, 0.0f});
    AddPerson(world, {0.0f, 0.0f, 0.0f});  // between the two boxes, touching both
    Simulate(world, 3.0f);
    Check(world.GetBody(left).isAsleep && world.GetBody(right).isAsleep, "boxes that touch a person who stands still go to sleep");
    Check(world.GetBody(left).sleepGroup != world.GetBody(right).sleepGroup, "in groups of their own, the person does not join them");
}

void ACrateOnAPersonFallsWhenThePersonIsTakenAway() {
    for (const bool parkThem : {true, false}) {
        PhysicsWorld world;
        AddGround(world);
        const uint32_t person = AddPerson(world, {0.0f, 0.0f, 0.0f});
        const uint32_t crate = AddBox(world, {0.0f, 1.8f + 0.4f + 0.01f, 0.0f}, {0.4f, 0.4f, 0.4f});
        Simulate(world, 3.0f);
        Check(world.GetBody(crate).isAsleep && Near(world.GetBody(crate).position.y, 2.2f, 0.05f), "a crate sleeps on a person's head");
        if (parkThem) {
            world.SetParked(person, true);
        } else {
            world.PlaceKinematic(person, {20.0f, 0.9f, 0.0f});
        }
        Simulate(world, 2.0f);
        Check(Near(world.GetBody(crate).position.y, 0.4f, 0.05f), std::string("and falls to the ground when the person is ") +
                                                                       (parkThem ? "parked" : "placed elsewhere"));
    }
}

void BodiesThatCannotBePushedArePassedThroughByPeople() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t heavy = AddBox(world, {2.0f, 0.5f, 0.0f});
    world.SetPushable(heavy, false);
    const uint32_t person = AddPerson(world, {0.0f, 0.0f, 0.0f});
    Simulate(world, 1.0f);
    Check(world.GetBody(heavy).isAsleep, "the crate sleeps");
    WalkPerson(world, person, 1.5f, 3.0f);
    Check(world.GetBody(heavy).isAsleep && Near(world.GetBody(heavy).position.x, 2.0f, 1e-4f), "and a person who walks through it neither wakes nor moves it");
    Check(Near(world.GetBody(person).position.x, 4.5f, 0.02f), "and is not held up");
}

void AParkedPersonTouchesNothing() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t ball = AddSphere(world, {0.0f, 0.3f, 0.0f}, 0.3f);
    const uint32_t person = AddPerson(world, {0.0f, 0.0f, 0.0f});  // inside the ball
    world.SetParked(person, true);
    Simulate(world, 2.0f);
    Check(Near(world.GetBody(ball).position.x, 0.0f, 1e-4f) && Near(world.GetBody(ball).position.z, 0.0f, 1e-4f),
          "a ball is not moved by a person who is parked inside it");
    Check(world.GetBody(ball).isAsleep, "and sleeps");
    world.SetParked(person, false);
    WalkPerson(world, person, 0.5f, 1.0f);
    Check(glm::length(world.GetBody(ball).position - glm::vec3(0.0f, 0.3f, 0.0f)) > 0.1f, "but when they are called back and walk they push it out");
}

void APersonIsPlacedAtOnceAndStops() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t person = AddPerson(world, {0.0f, 0.0f, 0.0f});
    WalkPerson(world, person, 1.5f, 1.0f);
    world.PlaceKinematic(person, {20.0f, 0.9f, 5.0f});
    Check(glm::length(world.GetBody(person).linearVelocity) == 0.0f, "a person who is placed stands still");
    world.Step();
    Check(Near(world.GetBody(person).position.x, 20.0f, 1e-5f) && Near(world.GetBody(person).position.z, 5.0f, 1e-5f), "at the place");
}

void APersonIsDrivenNoFasterThanTheTopSpeed() {
    PhysicsWorld world;
    AddGround(world);
    const uint32_t person = AddPerson(world, {0.0f, 0.0f, 0.0f});
    world.SetKinematicTarget(person, {100.0f, 0.9f, 0.0f}, 0.01f);
    Check(glm::length(world.GetBody(person).linearVelocity) <= 8.001f, "a target far away does not make a person fly");
    world.SetKinematicTarget(person, {std::numeric_limits<float>::quiet_NaN(), 0.9f, 0.0f}, 0.01f);
    Check(std::isfinite(world.GetBody(person).linearVelocity.x), "and a target that is not a number is ignored");
}

void TheSameWalkAlwaysGivesTheSameResult() {
    const auto run = [] {
        PhysicsWorld world;
        AddGround(world);
        for (int i = 0; i < 8; ++i) {
            AddSphere(world, {1.0f + 0.7f * static_cast<float>(i), 0.3f + 0.1f * static_cast<float>(i % 3), 0.2f * static_cast<float>(i % 2)}, 0.3f);
        }
        const uint32_t person = AddPerson(world, {0.0f, 0.0f, 0.0f});
        WalkPerson(world, person, 1.5f, 4.0f);
        std::vector<glm::vec3> positions;
        for (uint32_t i = 1; i <= 8; ++i) {
            positions.push_back(world.GetBody(i).position);
        }
        return positions;
    };
    Check(run() == run(), "a person pushing a row of balls gives the same result every time, to the last bit");
}

int main() {
    struct Test {
        const char* name;
        std::function<void()> run;
    };
    const std::vector<Test> tests = {
        {"ABodyInFreeFallFollowsGravity", ABodyInFreeFallFollowsGravity},
        {"ASphereComesToRestOnTheGround", ASphereComesToRestOnTheGround},
        {"ABoxLandsAndLiesFlat", ABoxLandsAndLiesFlat},
        {"ATiltedBoxFallsOverOntoAFace", ATiltedBoxFallsOverOntoAFace},
        {"ASphereRestsOnABox", ASphereRestsOnABox},
        {"ASphereBalancesOnASphereOnlyIfCentred", ASphereBalancesOnASphereOnlyIfCentred},
        {"AStackOfBoxesStaysStanding", AStackOfBoxesStaysStanding},
        {"AWallOfBoxesStaysStanding", AWallOfBoxesStaysStanding},
        {"ATallThinColumnOfBoxesDoesNotExplode", ATallThinColumnOfBoxesDoesNotExplode},
        {"ABallBouncesAsHighAsItsRestitutionAllows", ABallBouncesAsHighAsItsRestitutionAllows},
        {"ASlowBallDoesNotBounce", ASlowBallDoesNotBounce},
        {"ABoxSlidingOnTheGroundStopsWhereFrictionSaysItMust", ABoxSlidingOnTheGroundStopsWhereFrictionSaysItMust},
        {"AGripOnASlopeHoldsABoxStill", AGripOnASlopeHoldsABoxStill},
        {"AWeakGripLetsABoxSlideDownASlope", AWeakGripLetsABoxSlideDownASlope},
        {"ABoxRestsOnATurnedPlatform", ABoxRestsOnATurnedPlatform},
        {"AThinPlateStopsAFastBox", AThinPlateStopsAFastBox},
        {"AnImpulseWakesASleepingBody", AnImpulseWakesASleepingBody},
        {"ABallThatLandsOnASleepingBoxWakesIt", ABallThatLandsOnASleepingBoxWakesIt},
        {"TheSameWorldAlwaysGivesTheSameResult", TheSameWorldAlwaysGivesTheSameResult},
        {"TheResultDoesNotDependOnTheNumberOfThreads", TheResultDoesNotDependOnTheNumberOfThreads},
        {"AWalkingPersonPushesABallAheadAndIsNotSlowedByIt", AWalkingPersonPushesABallAheadAndIsNotSlowedByIt},
        {"AFastPersonDoesNotRunThroughABall", AFastPersonDoesNotRunThroughABall},
        {"KinematicBodiesTouchingNothingThatMovesStayWhereTheyAre", KinematicBodiesTouchingNothingThatMovesStayWhereTheyAre},
        {"NothingPushesAKinematicBody", NothingPushesAKinematicBody},
        {"ACrateThatFallsOnAPersonLandsOnTheGroundBesideThemOrOnTheirHead", ACrateThatFallsOnAPersonLandsOnTheGroundBesideThemOrOnTheirHead},
        {"AHeapSleepsBesideAStillPersonWakesForAWalkingOneAndSleepsAgain", AHeapSleepsBesideAStillPersonWakesForAWalkingOneAndSleepsAgain},
        {"AStillPersonInAHeapDoesNotKeepItAwakeOrJoinTwoHeaps", AStillPersonInAHeapDoesNotKeepItAwakeOrJoinTwoHeaps},
        {"ACrateOnAPersonFallsWhenThePersonIsTakenAway", ACrateOnAPersonFallsWhenThePersonIsTakenAway},
        {"BodiesThatCannotBePushedArePassedThroughByPeople", BodiesThatCannotBePushedArePassedThroughByPeople},
        {"AParkedPersonTouchesNothing", AParkedPersonTouchesNothing},
        {"APersonIsPlacedAtOnceAndStops", APersonIsPlacedAtOnceAndStops},
        {"APersonIsDrivenNoFasterThanTheTopSpeed", APersonIsDrivenNoFasterThanTheTopSpeed},
        {"TheSameWalkAlwaysGivesTheSameResult", TheSameWalkAlwaysGivesTheSameResult},
        {"APileSettlesDownAndNothingFallsThroughTheGround", APileSettlesDownAndNothingFallsThroughTheGround},
        {"ThePileNeverGainsEnergy", ThePileNeverGainsEnergy},
        {"ABoxOnABoxMakesFourContactsWithAnUpwardNormal", ABoxOnABoxMakesFourContactsWithAnUpwardNormal},
        {"BoxesThatAreApartMakeNoContacts", BoxesThatAreApartMakeNoContacts},
        {"TwoEdgesCrossingMakeOneContact", TwoEdgesCrossingMakeOneContact},
        {"ASphereInsideABoxIsPushedOutThroughTheNearestFace", ASphereInsideABoxIsPushedOutThroughTheNearestFace},
        {"BadInputDoesNotPoisonTheWorld", BadInputDoesNotPoisonTheWorld},
        {"ABodyStartingBadlyDoesNotSpreadToOthers", ABodyStartingBadlyDoesNotSpreadToOthers},
        {"AHeavyBallLandingOnASleepingBoxDoesNotPushItIntoTheGround", AHeavyBallLandingOnASleepingBoxDoesNotPushItIntoTheGround},
        {"ASleepingStackWakesAsAWhole", ASleepingStackWakesAsAWhole},
        {"APerfectlyElasticBounceKeepsItsHeightWhateverTheFramePhase", APerfectlyElasticBounceKeepsItsHeightWhateverTheFramePhase},
        {"BouncinessIsTheLargerOfTheTwoMaterials", BouncinessIsTheLargerOfTheTwoMaterials},
        {"SmallFastBodiesDoNotTunnelThroughAThinPlate", SmallFastBodiesDoNotTunnelThroughAThinPlate},
        {"BoxesMeetingHeadOnStopOrSwapSpeedsAsRestitutionSays", BoxesMeetingHeadOnStopOrSwapSpeedsAsRestitutionSays},
        {"ABoxStartingDeepInTheGroundRisesOutGently", ABoxStartingDeepInTheGroundRisesOutGently},
        {"ABoxHangingPastItsSupportTipsOverAndOneOverTheEdgeStays", ABoxHangingPastItsSupportTipsOverAndOneOverTheEdgeStays},
        {"ASpinningBodyKeepsItsOrientationNormal", ASpinningBodyKeepsItsOrientationNormal},
        {"CrossedEdgesAreFoundAsAnEdgeContact", CrossedEdgesAreFoundAsAnEdgeContact},
        {"ContactsOfAnOverhangingBoxStayOnTheFaceBelow", ContactsOfAnOverhangingBoxStayOnTheFaceBelow},
        {"AHeavyBoxOnLightOnesDoesNotScatterThem", AHeavyBoxOnLightOnesDoesNotScatterThem},
        {"ATenHighStackFallsAsleep", ATenHighStackFallsAsleep},
        {"ADenseHeapOfThreeHundredBodiesSettles", ADenseHeapOfThreeHundredBodiesSettles},
        {"AnElasticBoxDroppedAtAnyTiltNeverGainsEnergy", AnElasticBoxDroppedAtAnyTiltNeverGainsEnergy},
        {"ABoxWellAboveAPlateStillMakesAFaceContact", ABoxWellAboveAPlateStillMakesAFaceContact},
        {"TwoFastBallsMeetingHeadOnDoNotPassThroughEachOther", TwoFastBallsMeetingHeadOnDoNotPassThroughEachOther},
        {"AnImpossiblyHeavyFastBodyDoesNotCorruptTheGround", AnImpossiblyHeavyFastBodyDoesNotCorruptTheGround},
        {"ALongUpdateStepIsNotSlowMotion", ALongUpdateStepIsNotSlowMotion},
        {"ADisabledBodyStaysWhereItIsAndCostsNothing", ADisabledBodyStaysWhereItIsAndCostsNothing},
        {"AForceAcceleratesABodyAsNewtonSays", AForceAcceleratesABodyAsNewtonSays},
        {"AForceOffCentreSpinsTheBody", AForceOffCentreSpinsTheBody},
        {"AForceLastsOneStepOnly", AForceLastsOneStepOnly},
        {"ForcesOnBodiesThatCannotMoveOrAreNotNumbersAreIgnored", ForcesOnBodiesThatCannotMoveOrAreNotNumbersAreIgnored},
        {"ACapsuleHasTheMassAndInertiaOfItsVolume", ACapsuleHasTheMassAndInertiaOfItsVolume},
        {"ACapsuleWithNoStraightPartIsASphere", ACapsuleWithNoStraightPartIsASphere},
        {"TheBoundsOfACapsuleFollowItsAxis", TheBoundsOfACapsuleFollowItsAxis},
        {"BadCapsuleSizesAreCleanedUp", BadCapsuleSizesAreCleanedUp},
        {"ACapsuleLyingOnABoxTouchesItAtBothEnds", ACapsuleLyingOnABoxTouchesItAtBothEnds},
        {"AStandingCapsuleTouchesTheGroundAtOnePoint", AStandingCapsuleTouchesTheGroundAtOnePoint},
        {"ACapsuleTiltedOnABoxTouchesAtTheLowEndOnly", ACapsuleTiltedOnABoxTouchesAtTheLowEndOnly},
        {"ParallelCapsulesTouchAlongTheirLengthAndCrossedOnesAtOnePoint", ParallelCapsulesTouchAlongTheirLengthAndCrossedOnesAtOnePoint},
        {"AnyOrderOfTwoShapesGivesTheSameContactTurnedRound", AnyOrderOfTwoShapesGivesTheSameContactTurnedRound},
        {"ACapsuleAcrossAPostLeavesTheWayItCameIn", ACapsuleAcrossAPostLeavesTheWayItCameIn},
        {"CapsulesMeetingSpheresAndBoxesAtTheirRoundEnds", CapsulesMeetingSpheresAndBoxesAtTheirRoundEnds},
        {"ACapsuleThatIsNotQuiteUprightFallsOverAndLiesDown", ACapsuleThatIsNotQuiteUprightFallsOverAndLiesDown},
        {"ACapsuleLyingOnASlopeRollsDownIt", ACapsuleLyingOnASlopeRollsDownIt},
        {"ACapsuleTurnedOnTheSpotOnTheGroundStopsAndSleeps", ACapsuleTurnedOnTheSpotOnTheGroundStopsAndSleeps},
        {"ACapsuleRollsToAStopAcrossTheFloor", ACapsuleRollsToAStopAcrossTheFloor},
        {"CapsulesDroppedOnEachOtherAndOnABallSettleAndSleep", CapsulesDroppedOnEachOtherAndOnABallSettleAndSleep},
        {"AnElasticCapsuleDroppedAtAnyTiltNeverGainsEnergy", AnElasticCapsuleDroppedAtAnyTiltNeverGainsEnergy},
        {"AFastSmallCapsuleDoesNotPassThroughAThinPlate", AFastSmallCapsuleDoesNotPassThroughAThinPlate},
        {"ACapsuleThrownOntoAHeapOfBoxesLeavesItAsleep", ACapsuleThrownOntoAHeapOfBoxesLeavesItAsleep},
    };
    for (const Test& test : tests) {
        const int failuresBefore = failureCount;
        try {
            test.run();
        } catch (const std::exception& error) {
            ++failureCount;
            std::cerr << "  threw: " << error.what() << "\n";
        }
        std::cout << (failureCount == failuresBefore ? "PASS  " : "FAIL  ") << test.name << "\n";
    }
    std::cout << (failureCount == 0 ? "all physics tests passed\n" : "physics tests failed\n");
    return failureCount == 0 ? 0 : 1;
}
