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

}  // namespace

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
