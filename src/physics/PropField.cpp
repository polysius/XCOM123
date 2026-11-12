#include "physics/PropField.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace physics {
namespace {

constexpr float kSquareMargin = 2.0f;      // props start this far inside the edge of the square
constexpr float kCellSize = 2.0f;          // props start in cells this wide, so they do not overlap
constexpr int kMaxCellsAcross = 6;         // and in a heap no wider than this many cells
constexpr float kOffCentre = 10.0f;        // the heap is this far from the middle of the square, clear of a fountain
constexpr float kFirstLayerHeight = 3.0f;  // metres above the ground at which the lowest props start
constexpr float kLayerSpacing = 2.0f;      // more than the widest prop, so that none starts inside another
constexpr float kHighestFloor = 12.0f;     // a box with its top higher than this is not a floor: a tree or a roof
constexpr float kFallDepth = 80.0f;        // props this far below the ground under the heap have fallen out of the world

const glm::vec3 kWoods[] = {{0.52f, 0.36f, 0.20f}, {0.45f, 0.30f, 0.16f}, {0.60f, 0.45f, 0.26f}, {0.38f, 0.26f, 0.15f}};
const glm::vec3 kBarrelPaints[] = {{0.45f, 0.10f, 0.08f}, {0.10f, 0.22f, 0.40f}, {0.12f, 0.32f, 0.16f}, {0.30f, 0.30f, 0.32f}};
const glm::vec3 kBallColours[] = {{0.85f, 0.15f, 0.12f}, {0.95f, 0.75f, 0.10f}, {0.15f, 0.45f, 0.85f}, {0.90f, 0.90f, 0.85f}};

float Unit(std::mt19937& random) {
    return static_cast<float>(random() >> 8) * (1.0f / 16777216.0f);
}

// The largest square of the city, or a patch of ground around its centre when there are none.
OrientedRect ChooseDropArea(const CityLayout& city) {
    const OrientedRect* best = nullptr;
    for (const OrientedRect& plaza : city.plazas) {
        if (best == nullptr || plaza.Area() > best->Area()) {
            best = &plaza;
        }
    }
    if (best != nullptr) {
        return *best;
    }
    return OrientedRect(Rect{{-20.0f, -20.0f}, {20.0f, 20.0f}});
}

BodyInstance MakeInstance(glm::vec3 halfSize, glm::vec3 color, SurfaceMaterial material, float roughness, float seed) {
    BodyInstance instance{};
    instance.material = static_cast<float>(material);
    instance.halfSize = halfSize;
    instance.roughness = roughness;
    instance.color = color;
    instance.seed = seed;
    instance.orientation = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    return instance;
}

// The height of the highest solid thing under `point` that a prop could stand on: the ground, pavement and
// plaza, or the terrace the square stands on. Things above `ceiling` (trees, roofs) are not floors.
float GroundHeightAt(const CityLayout& city, glm::vec2 point, float ceiling) {
    float height = 0.0f;
    for (size_t i = 0; i < city.boxes.size(); ++i) {
        const BoxInstance& box = city.boxes[i];
        const float top = box.center.y + box.halfSize.y;
        if (city.boxParts[i] == CityPart::Water || top > ceiling || top <= height) {
            continue;
        }
        // The point in the box's own axes, where the footprint is axis aligned.
        const glm::vec2 offset = point - glm::vec2(box.center.x, box.center.z);
        const float cosine = std::cos(box.yaw);
        const float sine = std::sin(box.yaw);
        const float alongX = offset.x * cosine - offset.y * sine;
        const float alongZ = offset.x * sine + offset.y * cosine;
        if (std::abs(alongX) <= box.halfSize.x && std::abs(alongZ) <= box.halfSize.z) {
            height = top;
        }
    }
    return height;
}

}  // namespace

void PropField::Spawn(const CityLayout& city, const PropSettings& settings) {
    world = PhysicsWorld();
    boxBodies.clear();
    sphereBodies.clear();
    boxInstances.clear();
    sphereInstances.clear();

    const OrientedRect area = ChooseDropArea(city);
    const glm::vec2 centre = area.Center();

    // The city near the drop area is solid ground for the props.
    for (size_t i = 0; i < city.boxes.size(); ++i) {
        const BoxInstance& box = city.boxes[i];
        if (city.boxParts[i] == CityPart::Water) {
            continue;  // the river is no floor; props that fall in are lost to it
        }
        const float reach = settings.colliderRadius + 1.5f * std::max(box.halfSize.x, box.halfSize.z);
        if (glm::distance(glm::vec2(box.center.x, box.center.z), centre) > reach) {
            continue;
        }
        BodyDescription description;
        description.shape = Shape::Box(box.halfSize);
        description.position = box.center;
        description.orientation = glm::angleAxis(box.yaw, glm::vec3(0.0f, 1.0f, 0.0f));
        description.isStatic = true;
        description.material = {0.7f, 0.15f, 0.05f};
        world.AddBody(description);
    }

    // The props start in a heap: a grid of cells in the square, layer upon layer, so that none starts inside another.
    const glm::vec2 room = glm::max(area.Size() - 2.0f * kSquareMargin, glm::vec2(kCellSize));
    const int cellsX = std::clamp(static_cast<int>(room.x / kCellSize), 1, kMaxCellsAcross);
    const int cellsZ = std::clamp(static_cast<int>(room.y / kCellSize), 1, kMaxCellsAcross);
    const int perLayer = cellsX * cellsZ;
    const glm::vec2 size(static_cast<float>(cellsX) * kCellSize, static_cast<float>(cellsZ) * kCellSize);
    // The heap stands off the middle of the square, as far as the square allows.
    const glm::vec2 heapCentre = area.area.Center() + glm::vec2(std::min(kOffCentre, std::max(room.x - size.x, 0.0f) * 0.5f), 0.0f);
    const glm::vec2 low = heapCentre - size * 0.5f;
    dropCentre = area.FromFrame(heapCentre);

    // The first layer starts above the highest ground under the heap, which may be a terrace.
    float groundHeight = 0.0f;
    const glm::vec2 corners[] = {size * 0.5f, {-size.x * 0.5f, size.y * 0.5f}, {size.x * 0.5f, -size.y * 0.5f}, -size * 0.5f, {0.0f, 0.0f}};
    for (glm::vec2 corner : corners) {
        groundHeight = std::max(groundHeight, GroundHeightAt(city, area.FromFrame(heapCentre + corner), kHighestFloor));
    }
    fallLimit = groundHeight - kFallDepth;

    std::mt19937 random(settings.seed * 2654435761u + 17u);
    for (uint32_t i = 0; i < settings.count; ++i) {
        const int cell = static_cast<int>(i) % perLayer;
        const int layer = static_cast<int>(i) / perLayer;
        const glm::vec2 inFrame(low.x + (static_cast<float>(cell % cellsX) + 0.5f) * size.x / static_cast<float>(cellsX),
                                low.y + (static_cast<float>(cell / cellsX) + 0.5f) * size.y / static_cast<float>(cellsZ));
        const glm::vec2 ground = area.FromFrame(inFrame);

        BodyDescription description;
        description.position = {ground.x, groundHeight + kFirstLayerHeight + static_cast<float>(layer) * kLayerSpacing, ground.y};
        const glm::vec3 axis = glm::normalize(glm::vec3(Unit(random) + 0.1f, Unit(random), Unit(random)));
        description.orientation = glm::angleAxis(Unit(random) * 6.2831853f, axis);
        description.linearVelocity = {(Unit(random) - 0.5f) * 2.0f, 0.0f, (Unit(random) - 0.5f) * 2.0f};
        description.angularVelocity = {Unit(random) - 0.5f, Unit(random) - 0.5f, Unit(random) - 0.5f};

        const float seed = Unit(random);
        const uint32_t kind = i % 10;
        if (kind < 6) {  // a crate
            description.shape = Shape::Box({0.25f + 0.3f * Unit(random), 0.25f + 0.25f * Unit(random), 0.25f + 0.3f * Unit(random)});
            description.density = 450.0f;
            description.material = {0.6f, 0.1f, 0.05f};
            boxInstances.push_back(MakeInstance(description.shape.halfExtents, kWoods[i % 4], SurfaceMaterial::Plain, 0.85f, seed));
            boxBodies.push_back(world.AddBody(description));
        } else if (kind < 8) {  // a barrel, which is a box here, and hollow
            description.shape = Shape::Box({0.3f, 0.45f, 0.3f});
            description.density = 150.0f;
            description.material = {0.4f, 0.2f, 0.05f};
            boxInstances.push_back(MakeInstance(description.shape.halfExtents, kBarrelPaints[i % 4], SurfaceMaterial::Metal, 0.5f, seed));
            boxBodies.push_back(world.AddBody(description));
        } else {  // a ball
            const float radius = 0.2f + 0.25f * Unit(random);
            description.shape = Shape::Sphere(radius);
            description.density = 300.0f;
            description.material = {0.5f, 0.65f, 0.05f};
            sphereInstances.push_back(MakeInstance(glm::vec3(radius), kBallColours[i % 4], SurfaceMaterial::Plain, 0.4f, seed));
            sphereBodies.push_back(world.AddBody(description));
        }
    }
    RefreshInstances();
}

void PropField::Update(float seconds) {
    world.Update(seconds);
    // A prop that has gone through the river, which has no floor, would fall for ever and never sleep.
    for (const std::vector<uint32_t>* bodies : {&boxBodies, &sphereBodies}) {
        for (uint32_t index : *bodies) {
            if (world.GetBody(index).position.y < fallLimit) {
                world.Disable(index);
            }
        }
    }
    RefreshInstances();
}

void PropField::RefreshInstances() {
    const auto refresh = [this](const std::vector<uint32_t>& bodies, std::vector<BodyInstance>& instances) {
        for (size_t i = 0; i < bodies.size(); ++i) {
            const Body& body = world.GetBody(bodies[i]);
            instances[i].center = body.position;
            instances[i].orientation = glm::vec4(body.orientation.x, body.orientation.y, body.orientation.z, body.orientation.w);
        }
    };
    refresh(boxBodies, boxInstances);
    refresh(sphereBodies, sphereInstances);
}

}  // namespace physics
