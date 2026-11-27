// Tests of what a character wears and shows (no window, no GPU): skin, eyes, hair and clothes. Like the other test programs there
// is no test framework: each test is a function that calls Check, and main reports every failure it finds.
//
// What the appearance must be, and what these tests hold it to: the same every time on any number of threads; eyes that sit in the
// hollows the head carves for them without poking out of the head; hair that grows from the scalp and the brow and the jaw; garments
// that are closed shells, skinned like the body, that never touch it, whatever the build; and skin textures with values in range that
// change with the age and sex of the person.
//
// The default run tests two typical people and the five worst builds found so far (about two minutes), each dressed once and shared by
// every test. `appearance_tests --full` adds a sweep of fifteen more builds, which takes about four minutes; run it after a change to
// the clothes, the hair or the eyes. A word that is not --full filters the tests by name.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <unordered_map>
#include <string>
#include <vector>

#include "character/BodyShape.h"
#include "character/CharacterGenerator.h"
#include "character/DetailShapes.h"
#include "character/appearance/Appearance.h"
#include "character/appearance/HeadRegions.h"
#include "character/appearance/MeshTools.h"
#include "core/JobSystem.h"

namespace {

using namespace character;

int failureCount = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        ++failureCount;
        std::cerr << "  FAILED: " << what << "\n";
    }
}

CharacterParameters Person(float masculinity, float age, float stature, float mass, float muscularity, uint32_t seed = 1) {
    CharacterParameters parameters;
    parameters.masculinity = masculinity;
    parameters.ageYears = age;
    parameters.statureMeters = stature;
    parameters.massKg = mass;
    parameters.muscularity = muscularity;
    parameters.seed = seed;
    return parameters;
}

// Whether this is a full run (`appearance_tests --full`): the whole sweep of builds is tested, not only the default set, which takes a few
// times as long.
bool gFullRun = false;

// Two typical people: a tall heavy man and a short light woman.
std::vector<CharacterParameters> SomePeople() {
    return {Person(1.0f, 30.0f, 1.90f, 100.0f, 0.8f, 1), Person(0.0f, 25.0f, 1.55f, 48.0f, 0.2f, 2)};
}

// The builds that were found to bury cloth in the body: a short man of body mass index 15, a 2.05 m man of body mass index 45, and three
// random people. Every test of a person runs on these as well as on the typical people.
std::vector<CharacterParameters> WorstBuilds() {
    return {Person(1.0f, 50.0f, 1.45f, 15.0f * 1.45f * 1.45f, 0.2f, 1003), Person(1.0f, 40.0f, 2.05f, 45.0f * 2.05f * 2.05f, 0.5f, 1013), RandomParameters(7002),
            RandomParameters(7003), RandomParameters(5012)};
}

// The rest of the sweep, for a full run: a muscular woman, the ends of the range, another random person, and eight ordinary builds (body
// mass index 19 to 26).
std::vector<CharacterParameters> FullSweepBuilds() {
    std::vector<CharacterParameters> builds = {Person(0.0f, 28.0f, 1.72f, 66.0f, 0.9f, 4),
                                               Person(0.0f, 80.0f, 1.45f, 45.0f * 1.45f * 1.45f, 0.2f, 31),
                                               Person(1.0f, 60.0f, 1.45f, 45.0f * 1.45f * 1.45f, 0.5f, 32),
                                               Person(1.0f, 25.0f, 2.05f, 15.0f * 2.05f * 2.05f, 1.0f, 33),
                                               Person(0.0f, 30.0f, 1.45f, 15.0f * 1.45f * 1.45f, 0.0f, 34),
                                               Person(1.0f, 40.0f, 2.05f, 45.0f * 2.05f * 2.05f, 0.5f, 35),
                                               RandomParameters(7006)};
    for (uint32_t i = 0; i < 8; ++i) {
        const float height = 1.55f + 0.04f * static_cast<float>(i);
        const float bmi = 19.0f + 1.0f * static_cast<float>(i);
        builds.push_back(Person(static_cast<float>(i % 2), 22.0f + 7.0f * static_cast<float>(i), height, bmi * height * height, 0.2f + 0.1f * static_cast<float>(i % 5), 900 + i));
    }
    return builds;
}

// Small textures keep the tests quick; the shapes are the same at any texture size. Every garment is worn, and hair and a beard are
// grown, unless a test says otherwise.
AppearanceSettings TestSettings(const CharacterParameters& parameters) {
    AppearanceSettings settings = ChooseAppearance(parameters);
    settings.skinTileSize = 64;
    settings.eyeTextureSize = 64;
    settings.outfit.tileSize = 64;
    settings.outfit.top = TopKind::TShirtAndJacket;
    settings.outfit.legs = LegwearKind::Trousers;
    settings.outfit.feet = FootwearKind::Boots;
    if (settings.hairStyle == HairStyle::Bald) {
        settings.hairStyle = HairStyle::Medium;
    }
    if (parameters.masculinity > 0.8f) {
        settings.facialHair = FacialHair::Beard;
    }
    return settings;
}

struct Dressed {
    CharacterParameters parameters;
    CharacterMesh mesh;
    CharacterAppearance appearance;
};

Dressed Dress(const CharacterParameters& parameters) {
    Dressed dressed;
    dressed.parameters = parameters;
    // On the shared job system, which is the case that matters for the time of the tests; that the result does not depend on the
    // threads is a test of its own.
    dressed.mesh = GenerateCharacter(parameters, {}, &JobSystem::Shared());
    dressed.appearance = GenerateAppearance(dressed.mesh, TestSettings(parameters), &JobSystem::Shared());
    return dressed;
}

const std::vector<Dressed>& CachedPeople() {
    static std::vector<Dressed> cache;
    if (cache.empty()) {
        for (const CharacterParameters& person : SomePeople()) {
            cache.push_back(Dress(person));
        }
    }
    return cache;
}

const std::vector<Dressed>& CachedWorstBuilds() {
    static std::vector<Dressed> cache;
    if (cache.empty()) {
        for (const CharacterParameters& person : WorstBuilds()) {
            cache.push_back(Dress(person));
        }
    }
    return cache;
}

const std::vector<Dressed>& CachedFullSweep() {
    static std::vector<Dressed> cache;
    if (cache.empty()) {
        for (const CharacterParameters& person : FullSweepBuilds()) {
            cache.push_back(Dress(person));
        }
    }
    return cache;
}

// Everyone that the tests of a person run on. Each is dressed once and shared by every test: the typical people and the worst builds, and
// in a full run the rest of the sweep too.
std::vector<const Dressed*> AllPeople() {
    std::vector<const Dressed*> people;
    for (const std::vector<Dressed>* group : {&CachedPeople(), &CachedWorstBuilds()}) {
        for (const Dressed& person : *group) {
            people.push_back(&person);
        }
    }
    if (gFullRun) {
        for (const Dressed& person : CachedFullSweep()) {
            people.push_back(&person);
        }
    }
    return people;
}

// A mesh is a closed, consistently oriented surface when every directed edge of a triangle is matched by exactly one triangle that goes
// the other way.
int gBadEdges = 0;  // how many edges the last call of IsClosedAndOriented found that are not matched, for the failure message

bool IsClosedAndOriented(const RenderMesh& mesh) {
    gBadEdges = 0;
    // Directed edges by the indices of their ends: the surfaces tested here (eyes, the watertight shell of a garment) share a vertex between
    // all the triangles that meet at it.
    std::unordered_map<uint64_t, int> edges;
    edges.reserve(mesh.indices.size());
    const auto key = [](uint32_t from, uint32_t to) { return (static_cast<uint64_t>(from) << 32) | to; };
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        for (int c = 0; c < 3; ++c) {
            ++edges[key(mesh.indices[t + c], mesh.indices[t + (c + 1) % 3])];
        }
    }
    for (const auto& [edge, uses] : edges) {
        const uint32_t from = static_cast<uint32_t>(edge >> 32);
        const uint32_t to = static_cast<uint32_t>(edge & 0xFFFFFFFFu);
        const auto reverse = edges.find(key(to, from));
        if (uses != 1 || reverse == edges.end() || reverse->second != 1) {
            ++gBadEdges;
        }
    }
    return gBadEdges == 0 && !edges.empty();
}


bool IndicesAreValid(const RenderMesh& mesh) {
    if (mesh.indices.size() % 3 != 0 || mesh.normals.size() != mesh.positions.size() || mesh.uvs.size() != mesh.positions.size() ||
        mesh.boneIndices.size() != mesh.positions.size() || mesh.boneWeights.size() != mesh.positions.size()) {
        return false;
    }
    for (const uint32_t index : mesh.indices) {
        if (index >= mesh.positions.size()) {
            return false;
        }
    }
    for (size_t v = 0; v < mesh.positions.size(); ++v) {
        float sum = 0.0f;
        for (int slot = 0; slot < 4; ++slot) {
            sum += mesh.boneWeights[v][slot];
            if (mesh.boneWeights[v][slot] < 0.0f || mesh.boneIndices[v][slot] >= kJointCount) {
                return false;
            }
        }
        if (std::abs(sum - 1.0f) > 1e-3f || !std::isfinite(mesh.positions[v].x + mesh.positions[v].y + mesh.positions[v].z)) {
            return false;
        }
    }
    return true;
}

bool SameMesh(const RenderMesh& a, const RenderMesh& b) {
    return a.positions == b.positions && a.indices == b.indices && a.uvs == b.uvs && a.boneWeights == b.boneWeights;
}

bool SameImage(const ByteImage& a, const ByteImage& b) {
    return a.width == b.width && a.height == b.height && a.bytes == b.bytes;
}

bool SameAppearance(const CharacterAppearance& a, const CharacterAppearance& b) {
    if (!SameMesh(a.skin.mesh, b.skin.mesh) || !SameImage(a.skin.textures.albedo, b.skin.textures.albedo) || !SameImage(a.skin.textures.normal, b.skin.textures.normal) ||
        !SameImage(a.skin.textures.roughness, b.skin.textures.roughness) || !SameMesh(a.hair.scalp.mesh, b.hair.scalp.mesh) ||
        !SameMesh(a.hair.eyebrows.mesh, b.hair.eyebrows.mesh) || !SameMesh(a.hair.facial.mesh, b.hair.facial.mesh) || a.outfit.garments.size() != b.outfit.garments.size() ||
        !SameImage(a.eyes.textures.albedo, b.eyes.textures.albedo)) {
        return false;
    }
    for (int eye = 0; eye < 2; ++eye) {
        if (!SameMesh(a.eyes.eyes[eye].eyeball, b.eyes.eyes[eye].eyeball) || !SameMesh(a.eyes.eyes[eye].upperLid, b.eyes.eyes[eye].upperLid) ||
            !SameMesh(a.eyes.eyes[eye].lashes.mesh, b.eyes.eyes[eye].lashes.mesh)) {
            return false;
        }
    }
    for (size_t g = 0; g < a.outfit.garments.size(); ++g) {
        if (!SameMesh(a.outfit.garments[g].mesh, b.outfit.garments[g].mesh) || !SameImage(a.outfit.garments[g].textures.albedo, b.outfit.garments[g].textures.albedo) ||
            !SameImage(a.outfit.garments[g].textures.normal, b.outfit.garments[g].textures.normal)) {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Determinism
// ---------------------------------------------------------------------------------------------------------------------------------

void TheSameParametersGiveTheSameAppearanceOnAnyNumberOfThreads() {
    const CharacterParameters parameters = Person(0.8f, 52.0f, 1.78f, 80.0f, 0.5f, 77);
    const CharacterMesh mesh = GenerateCharacter(parameters, {}, &JobSystem::Shared());
    const AppearanceSettings settings = LowDetail(TestSettings(parameters));
    JobSystem jobs(3);
    const CharacterAppearance first = GenerateAppearance(mesh, settings, &jobs);
    const CharacterAppearance threaded = GenerateAppearance(mesh, settings, &JobSystem::Shared());
    Check(SameAppearance(first, threaded), "three worker threads and all the threads of the machine give the same appearance, whole outfit and all");
    // The calling thread alone is the slowest way, so it is tried on a lighter outfit (a shirt and shorts).
    AppearanceSettings light = settings;
    light.outfit.top = TopKind::TShirt;
    light.outfit.legs = LegwearKind::Shorts;
    light.outfit.feet = FootwearKind::None;
    const CharacterAppearance alone = GenerateAppearance(mesh, light);
    const CharacterAppearance alongside = GenerateAppearance(mesh, light, &jobs);
    Check(SameAppearance(alone, alongside), "and the calling thread alone gives the same appearance as three worker threads");

    CharacterParameters other = parameters;
    other.seed = 78;
    const CharacterMesh otherMesh = GenerateCharacter(other, {}, &JobSystem::Shared());
    const CharacterAppearance different = GenerateAppearance(otherMesh, LowDetail(TestSettings(other)), &JobSystem::Shared());
    Check(!SameImage(first.skin.textures.albedo, different.skin.textures.albedo), "another seed gives other skin");
}

void TheChoiceOfAppearanceFollowsTheSeed() {
    const CharacterParameters parameters = Person(0.9f, 40.0f, 1.80f, 85.0f, 0.5f, 5);
    const AppearanceSettings a = ChooseAppearance(parameters);
    const AppearanceSettings b = ChooseAppearance(parameters);
    Check(a.hairStyle == b.hairStyle && a.facialHair == b.facialHair && a.outfit.top == b.outfit.top && a.outfit.legs == b.outfit.legs && a.outfit.feet == b.outfit.feet,
          "the same person always gets the same choices");
    int women = 0;
    int womenWithFacialHair = 0;
    int styles[5] = {};
    for (uint32_t seed = 0; seed < 400; ++seed) {
        CharacterParameters woman = Person(0.0f, 30.0f, 1.65f, 60.0f, 0.3f, seed);
        const AppearanceSettings choice = ChooseAppearance(woman);
        ++women;
        womenWithFacialHair += choice.facialHair != FacialHair::None ? 1 : 0;
        ++styles[static_cast<int>(choice.hairStyle)];
    }
    Check(womenWithFacialHair == 0, "women do not grow beards: " + std::to_string(womenWithFacialHair) + " of " + std::to_string(women));
    Check(styles[static_cast<int>(HairStyle::Long)] + styles[static_cast<int>(HairStyle::Tied)] > styles[static_cast<int>(HairStyle::Short)] * 2,
          "women mostly have long or tied hair");
    int youngBald = 0;
    int oldBald = 0;
    int bearded = 0;
    for (uint32_t seed = 0; seed < 400; ++seed) {
        youngBald += ChooseHairStyle(Person(1.0f, 22.0f, 1.80f, 80.0f, 0.4f, seed)) == HairStyle::Bald ? 1 : 0;
        oldBald += ChooseHairStyle(Person(1.0f, 74.0f, 1.80f, 80.0f, 0.4f, seed)) == HairStyle::Bald ? 1 : 0;
        bearded += ChooseFacialHair(Person(1.0f, 35.0f, 1.80f, 80.0f, 0.4f, seed)) != FacialHair::None ? 1 : 0;
    }
    Check(oldBald > youngBald * 3 && oldBald > 40, "old men are bald far more often than young ones: " + std::to_string(oldBald) + " against " + std::to_string(youngBald));
    Check(bearded > 120 && bearded < 280, "about half of the men have facial hair: " + std::to_string(bearded) + " of 400");
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Eyes
// ---------------------------------------------------------------------------------------------------------------------------------

// Whether a point is in the flesh of the head or in the hollow that was carved for an eye: a point that is neither is outside the head,
// which an eyeball may be only where the lids open on to the air in front.
bool InFleshOrSocket(const BodyShape& head, const EyeSocket& socket, glm::vec3 p, float margin) {
    if (head.Distance(p) <= margin) {
        return true;
    }
    const glm::vec3 radii = socket.radii + glm::vec3(socket.smoothing + margin);
    const glm::vec3 d = (p - socket.centre) / radii;
    return glm::dot(d, d) <= 1.0f;
}

void EyesSitInTheSocketsAndDoNotPokeThroughTheHead() {
    for (const Dressed* personPointer : AllPeople()) {
        const Dressed& person = *personPointer;
        {
            const std::string who = " (seed " + std::to_string(person.parameters.seed) + ")";
            const HeadLandmarks landmarks = ComputeHeadLandmarks(person.mesh.parameters);
            const BodyShape head = BuildHeadShape(person.mesh.parameters);
            const float s = landmarks.scale;
            Check(std::abs(person.appearance.eyes.eyes[0].centre.x + person.appearance.eyes.eyes[1].centre.x) < 1e-6f, "the eyes are mirrored" + who);
            for (int i = 0; i < 2; ++i) {
                const EyeParts& eye = person.appearance.eyes.eyes[i];
                const EyeSocket& socket = landmarks.eyeSockets[i];
                Check(eye.radius > 0.0100f * s && eye.radius < 0.0125f * s, "an eyeball is about 23 mm across" + who);
                Check(glm::distance(glm::vec2(eye.centre.x, eye.centre.y), glm::vec2(socket.centre.x, socket.centre.y)) < 0.002f * s, "the eye is centred on its socket across the face" + who);
                // Behind the middle of the eyeball nothing may be outside the flesh of the head; in front the eyeball may fill the socket.
                int outside = 0;
                int rearOutside = 0;
                float frontmost = -1e9f;
                for (const glm::vec3& p : eye.eyeball.positions) {
                    frontmost = std::max(frontmost, p.z);
                    if (!InFleshOrSocket(head, socket, p, 0.0006f)) {
                        ++outside;
                    }
                    if (p.z < eye.centre.z && head.Distance(p) > 0.0006f) {
                        ++rearOutside;
                    }
                }
                Check(outside == 0, std::to_string(outside) + " eyeball vertices are outside both the head and its socket" + who);
                Check(rearOutside == 0, "the back half of the eyeball is inside the head" + who);
                Check(frontmost <= landmarks.browFront.z + 0.0005f * s, "the cornea does not reach beyond the brow" + who);
                Check(frontmost > eye.centre.z + 0.9f * eye.radius, "the cornea bulges in front of the sphere" + who);
                Check(IsClosedAndOriented(eye.eyeball) && MeshVolume(eye.eyeball.positions, eye.eyeball.indices) > 0.0f, "an eyeball is a closed shell with outward triangles" + who);
                Check(IndicesAreValid(eye.eyeball) && IndicesAreValid(eye.upperLid) && IndicesAreValid(eye.lowerLid) && IndicesAreValid(eye.lashes.mesh), "eye meshes have valid indices and skin weights" + who);
                for (const RenderMesh* lid : {&eye.upperLid, &eye.lowerLid}) {
                    Check(IsClosedAndOriented(*lid) && MeshVolume(lid->positions, lid->indices) > 0.0f, "a lid is a closed shell" + who);
                    float closest = 1e9f;
                    for (const glm::vec3& p : lid->positions) {
                        closest = std::min(closest, glm::distance(p, eye.centre));
                    }
                    Check(closest >= eye.radius, "a lid does not enter the eyeball" + who);
                }
                for (const uint8_t bone : eye.eyeball.boneIndices.front()) {
                    Check(bone == static_cast<uint8_t>(Index(Joint::Head)) || bone == 0, "the eye follows the head bone" + who);
                }
                Check(eye.eyeball.boneIndices.front()[0] == static_cast<uint8_t>(Index(Joint::Head)) && eye.eyeball.boneWeights.front()[0] == 1.0f, "with full weight" + who);
            }
        }
    }
}

void TheLandmarksAreWhereTheHeadIsHollow() {
    for (const Dressed& person : CachedPeople()) {
        const HeadLandmarks landmarks = ComputeHeadLandmarks(person.mesh.parameters);
        const BodyShape head = BuildHeadShape(person.mesh.parameters);
        const float s = landmarks.scale;
        for (const EyeSocket& socket : landmarks.eyeSockets) {
            // At the front of the socket there is air, and behind it the head is solid.
            Check(head.Distance(socket.centre + glm::vec3(0.0f, 0.0f, 0.010f * s)) > 0.0f, "in front of the centre of a socket is air");
            Check(head.Distance(socket.centre - glm::vec3(0.0f, 0.0f, 0.030f * s)) < 0.0f, "and 3 cm behind it is flesh");
        }
        Check(head.Distance(landmarks.noseTip - glm::vec3(0.0f, 0.0f, 0.004f * s)) < 0.002f * s, "the nose tip landmark is on the nose");
        Check(landmarks.crown.y > landmarks.chin.y + 0.15f * s && landmarks.crown.y < person.mesh.parameters.statureMeters + 0.01f, "the crown is above the chin and not above the stature");
    }
}

void EyeColourAndTextureFollowThePerson() {
    for (const Dressed& person : CachedPeople()) {
        const ByteImage& albedo = person.appearance.eyes.textures.albedo;
        Check(albedo.width == 64 && albedo.height == 64 && albedo.channels == 4, "the eye texture is the size asked for");
        // The pupil at the centre is nearly black, the sclera near the edge of the iris is light.
        const uint8_t* pupil = albedo.Pixel(32, 32);
        const uint8_t* sclera = albedo.Pixel(5, 32);
        Check(pupil[0] < 40 && pupil[1] < 40 && pupil[2] < 40, "the pupil is dark");
        Check(sclera[0] > 150 && sclera[1] > 140, "the white of the eye is light");
        Check(*person.appearance.eyes.textures.roughness.Pixel(32, 32) < 40, "the cornea is glossy");
    }
    // Iris colours differ between people.
    bool differ = false;
    for (const Dressed& person : CachedPeople()) {
        differ = differ || glm::distance(person.appearance.traits.irisColour, CachedPeople()[0].appearance.traits.irisColour) > 0.02f;
    }
    Check(differ, "people do not all have the same eye colour");
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Hair
// ---------------------------------------------------------------------------------------------------------------------------------

TriangleGrid HeadGrid(const CharacterMesh& mesh, std::vector<glm::vec3>& positions, std::vector<uint32_t>& indices) {
    for (const MeshPiece& piece : mesh.pieces) {
        if (piece.kind == MeshPieceKind::Head) {
            positions.assign(mesh.positions.begin() + piece.firstVertex, mesh.positions.begin() + piece.firstVertex + piece.vertexCount);
            indices.clear();
            for (uint32_t i = 0; i < piece.indexCount; ++i) {
                indices.push_back(mesh.indices[piece.firstIndex + i] - piece.firstVertex);
            }
        }
    }
    return TriangleGrid(positions, indices, 0.01f);
}

void HairGrowsFromTheScalpAndNeverEntersTheHead() {
    for (const Dressed& person : CachedPeople()) {
        const std::string who = " (seed " + std::to_string(person.parameters.seed) + ")";
        const HairSet& hair = person.appearance.hair;
        std::vector<glm::vec3> headPositions;
        std::vector<uint32_t> headIndices;
        const TriangleGrid head = HeadGrid(person.mesh, headPositions, headIndices);
        const HeadLandmarks landmarks = ComputeHeadLandmarks(person.mesh.parameters);
        const BodyShape headShape = BuildHeadShape(person.mesh.parameters);

        Check(hair.scalp.RibbonCount() > 1000, "a head of hair has locks: " + std::to_string(hair.scalp.RibbonCount()) + who);
        float farthestRoot = 0.0f;
        int outsideScalp = 0;
        for (const glm::vec3& root : hair.scalp.roots) {
            // A root is on the surface of the head, up to the little lift of the lock above it.
            const float distance = head.FindNearest(root, 0.02f).distance;
            farthestRoot = std::max(farthestRoot, distance);
            if (ScalpWeight(landmarks, person.appearance.traits, root) < 0.2f) {
                ++outsideScalp;
            }
        }
        Check(farthestRoot < 0.0005f, "roots lie on the surface of the scalp: " + std::to_string(farthestRoot * 1000.0f) + " mm" + who);
        Check(outsideScalp == 0, "every root is in the hairy part of the scalp" + who);
        Check(IndicesAreValid(hair.scalp.mesh) && hair.scalp.colours.size() == hair.scalp.mesh.positions.size(), "hair meshes are valid" + who);

        // No vertex of a lock is deep in the head (a lock is a ribbon a few millimetres wide, so its edge may dip a little).
        float deepest = 1e9f;
        for (const glm::vec3& p : hair.scalp.mesh.positions) {
            deepest = std::min(deepest, headShape.Distance(p));
        }
        Check(deepest > -0.0010f, "hair does not go through the head: deepest " + std::to_string(deepest * 1000.0f) + " mm" + who);

        // The eyebrows sit above the eyes, on the face.
        Check(hair.eyebrows.RibbonCount() > 200, "there are eyebrows" + who);
        const float eyeHeight = landmarks.eyeSockets[0].centre.y;
        float farthestBrowRoot = 0.0f;
        for (const glm::vec3& root : hair.eyebrows.roots) {
            farthestBrowRoot = std::max(farthestBrowRoot, head.FindNearest(root, 0.02f).distance);
            Check(root.y > eyeHeight + 0.006f * landmarks.scale && root.y < eyeHeight + 0.045f * landmarks.scale, "a brow hair is above its eye" + who);
        }
        Check(farthestBrowRoot < 0.0015f * landmarks.scale, "eyebrow roots are on the skin" + who);
    }
}

void HairStylesDifferAsTheirNamesSay() {
    const Dressed& base = CachedPeople()[1];  // a young woman, so that every style has plenty of hair
    float lowest[5] = {};
    float highest[5] = {};
    size_t locks[5] = {};
    for (int style = 0; style < 5; ++style) {
        AppearanceSettings settings = TestSettings(base.parameters);
        settings.hairStyle = static_cast<HairStyle>(style);
        const HairSet hair = BuildHair(base.mesh, base.appearance.traits, {settings.hairStyle, FacialHair::None}, nullptr, &JobSystem::Shared());
        locks[style] = hair.scalp.RibbonCount();
        lowest[style] = 1e9f;
        highest[style] = -1e9f;
        for (const glm::vec3& p : hair.scalp.mesh.positions) {
            lowest[style] = std::min(lowest[style], p.y);
            highest[style] = std::max(highest[style], p.y);
        }
    }
    const float stature = base.mesh.parameters.statureMeters;
    const HeadLandmarks landmarks = ComputeHeadLandmarks(base.mesh.parameters);
    Check(locks[0] == 0, "a bald head has no locks");
    Check(lowest[1] > landmarks.chin.y + 0.02f, "short hair stays above the jaw: " + std::to_string(lowest[1]));
    Check(highest[1] < stature + 0.045f, "short hair is a few centimetres thick on the crown: " + std::to_string(highest[1] - stature));
    Check(lowest[2] < lowest[1] - 0.04f, "medium hair is longer than short hair");
    Check(lowest[3] < lowest[2] - 0.10f, "long hair is longer than medium hair");
    Check(lowest[3] < landmarks.chin.y - 0.10f * stature / 1.75f, "long hair hangs well below the chin");
    Check(lowest[4] > lowest[3] + 0.02f && lowest[4] < landmarks.chin.y, "a ponytail hangs below the head but is shorter than the whole length of long hair");
}

void HairGreysWithAgeAndThinsWithAgeInMen() {
    int greyYoung = 0;
    int greyOld = 0;
    float densityOldMen = 0.0f;
    float densityOldWomen = 0.0f;
    constexpr int kPeople = 200;
    for (uint32_t seed = 0; seed < kPeople; ++seed) {
        greyYoung += DeriveTraits(Person(0.5f, 22.0f, 1.7f, 70.0f, 0.4f, seed)).greyFraction > 0.2f ? 1 : 0;
        greyOld += DeriveTraits(Person(0.5f, 75.0f, 1.7f, 70.0f, 0.4f, seed)).greyFraction > 0.2f ? 1 : 0;
        densityOldMen += DeriveTraits(Person(1.0f, 75.0f, 1.7f, 70.0f, 0.4f, seed)).hairDensity;
        densityOldWomen += DeriveTraits(Person(0.0f, 75.0f, 1.7f, 70.0f, 0.4f, seed)).hairDensity;
    }
    Check(greyYoung == 0, "no hair of the young is grey");
    Check(greyOld > kPeople * 0.8, "most hair of the old is grey: " + std::to_string(greyOld));
    Check(densityOldMen < densityOldWomen - 0.05f * kPeople, "old men have thinner hair than old women");
}

void FacialHairGrowsOnTheJawAndNotOnTheLips() {
    for (const Dressed& person : CachedPeople()) {
        const HairSet& hair = person.appearance.hair;
        const bool hasBeard = person.appearance.settings.facialHair != FacialHair::None;
        Check(hasBeard == !hair.facial.mesh.positions.empty(), "facial hair is made if and only if it was asked for");
        if (!hasBeard) {
            continue;
        }
        std::vector<glm::vec3> headPositions;
        std::vector<uint32_t> headIndices;
        const TriangleGrid head = HeadGrid(person.mesh, headPositions, headIndices);
        const HeadLandmarks landmarks = ComputeHeadLandmarks(person.mesh.parameters);
        for (const glm::vec3& root : hair.facial.roots) {
            Check(head.FindNearest(root, 0.02f).distance < 0.0005f, "a beard root is on the skin");
            Check(LipWeight(landmarks, root) < 0.75f, "no beard root is on a lip");
            Check(BeardWeight(landmarks, root) > 0.2f, "a beard root is where beards grow");
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Clothes
// ---------------------------------------------------------------------------------------------------------------------------------

void GarmentsAreClosedShellsWithValidSkinning() {
    for (const Dressed* personPointer : AllPeople()) {
        const Dressed& person = *personPointer;
        {
            const std::string who = " (seed " + std::to_string(person.parameters.seed) + ")";
            const Outfit& outfit = person.appearance.outfit;
            Check(outfit.garments.size() == 4, "a shirt, a jacket, trousers and boots make four garments: " + std::to_string(outfit.garments.size()) + who);
            for (const Garment& garment : outfit.garments) {
                const std::string what = std::string(GarmentName(garment.kind)) + who;
                Check(IndicesAreValid(garment.mesh), "valid indices and skin weights, " + what);
                RenderMesh solid;
                solid.positions = garment.solid.positions;
                solid.indices = garment.solid.indices;
                const bool closed = IsClosedAndOriented(solid);
                Check(closed, "a closed shell, " + what + ": unmatched edges " + std::to_string(gBadEdges));
                const float solidVolume = MeshVolume(solid.positions, solid.indices);
                Check(solidVolume > 0.0f, "its triangles face outwards, " + what);
                // The unwrapped mesh is the same surface cut along the seams of the unwrap.
                Check(std::abs(MeshVolume(garment.mesh.positions, garment.mesh.indices) / solidVolume - 1.0f) < 0.02f, "and the unwrapped mesh encloses the same volume, " + what);
                Check(garment.vertexLayer.size() == garment.mesh.positions.size(), "a layer for every vertex, " + what);
                Check(garment.thicknessMeters >= 0.001f && garment.thicknessMeters <= 0.006f, "cloth is millimetres thick, " + what);
                Check(garment.materialKind == material::MaterialKind::Fabric || garment.materialKind == material::MaterialKind::Leather, "it maps to a procedural material, " + what);
                Check(garment.tint.r >= 0.0f && garment.tint.g >= 0.0f && garment.tint.b >= 0.0f && garment.tint.r <= 1.0f, "with a colour, " + what);
                Check(garment.textures.albedo.width == 4 * garment.textures.tileSize && garment.textures.normal.width == garment.textures.albedo.width, "and textures in the atlas layout, " + what);
            }
        }
    }
}

// What a surface says of a point: how far the point is from it and on which side (negative inside), and which way the surface faces there.
struct Probe {
    float signedDistance = 1e9f;  // 1e9 when no triangle is within the radius
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
};

// Probes a surface at every point, on the job system: each query is its own, so this is the same as asking one at a time.
std::vector<Probe> ProbeAll(const TriangleGrid& surface, const std::vector<glm::vec3>& points, float radius) {
    std::vector<Probe> probes(points.size());
    JobSystem::Shared().ParallelFor(points.size(), 256, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            const TriangleGrid::Nearest nearest = surface.FindNearest(points[i], radius);
            if (nearest.distance < 1e8f) {
                probes[i] = {nearest.signedDistance, nearest.surfaceNormal};
            }
        }
    });
    return probes;
}

// The least signed distance, from the surface of the character, of every vertex of a garment (on the inner and the outer surface and along
// the edge strip alike) and of the middle of every triangle of its watertight shell. Negative means inside the body.
float LeastSignedClearance(const TriangleGrid& character, const Garment& garment, glm::vec3* where = nullptr) {
    std::vector<glm::vec3> points = garment.solid.positions;
    const std::vector<uint32_t>& indices = garment.solid.indices;
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        points.push_back((garment.solid.positions[indices[t]] + garment.solid.positions[indices[t + 1]] + garment.solid.positions[indices[t + 2]]) / 3.0f);
    }
    const std::vector<Probe> probes = ProbeAll(character, points, 0.04f);
    float least = 1e9f;
    for (size_t i = 0; i < probes.size(); ++i) {
        if (probes[i].signedDistance < least && where != nullptr) {
            *where = points[i];
        }
        least = std::min(least, probes[i].signedDistance);
    }
    return least;
}

// What every outfit must be on every build: each garment at least the minimum clearance from the body on all its surfaces and nowhere inside
// it, the inner surface of each outside every layer below it, and the trousers over the top of the footwear.
void CheckOutfit(const CharacterMesh& mesh, const Outfit& outfit, const std::string& who) {
    const TriangleGrid body(mesh.positions, mesh.indices, 0.03f);
    for (const Garment& garment : outfit.garments) {
        const std::string what = std::string(GarmentName(garment.kind)) + who;
        glm::vec3 where(0.0f);
        const float clearance = LeastSignedClearance(body, garment, &where);
        Check(clearance >= 0.9f * kMinimumClearance, what + " stays " + std::to_string(clearance * 1000.0f) + " mm clear of the body, outside it, on every surface, worst at (" + std::to_string(where.x) + ", " + std::to_string(where.y) + ", " + std::to_string(where.z) + ")");
        Check(std::abs(MeasureClearance(body, garment, &JobSystem::Shared()) - garment.minimumClearance) < 1e-4f, "the garment's own record of its clearance agrees, " + what);
        RenderMesh solid;
        solid.positions = garment.solid.positions;
        solid.indices = garment.solid.indices;
        Check(IsClosedAndOriented(solid) && MeshVolume(solid.positions, solid.indices) > 0.0f, "a watertight shell with outward triangles, " + what + ": unmatched edges " + std::to_string(gBadEdges));
    }
    // Every vertex of the inner surface of a garment is outside the outer surface of every garment below it, not only every fifth.
    for (size_t upper = 1; upper < outfit.garments.size(); ++upper) {
        const Garment& garment = outfit.garments[upper];
        for (size_t lower = 0; lower < upper; ++lower) {
            const OuterSurface below(outfit.garments[lower]);
            const std::vector<glm::vec3> facing = VertexNormals(garment.solid.positions, garment.solid.indices);
            float least = 1e9f;
            glm::vec3 worst(0.0f);
            const std::vector<glm::vec3> innerPoints(garment.solid.positions.begin(), garment.solid.positions.begin() + garment.solid.outerFirstVertex);
            const std::vector<Probe> probes = ProbeAll(*below.grid, innerPoints, 0.04f);
            for (uint32_t v = 0; v < garment.solid.outerFirstVertex; ++v) {
                // The outer surface of a garment is a wall to be outside of, not its far wall: only a surface that faces the way the vertex does counts.
                // (The way a vertex of the inner surface faces is the way its partner on the outer surface does: the inner surface itself faces inwards.)
                if (probes[v].signedDistance < 1e8f && glm::dot(probes[v].normal, facing[garment.solid.outerFirstVertex + v]) > 0.3f) {
                    if (probes[v].signedDistance < least) {
                        worst = innerPoints[v];
                    }
                    least = std::min(least, probes[v].signedDistance);
                }
            }
            Check(least >= -0.0001f, std::string(GarmentName(garment.kind)) + " is outside the " + GarmentName(outfit.garments[lower].kind) + " below it" + who + ": " + std::to_string(least * 1000.0f) + " mm at (" + std::to_string(worst.x) + ", " + std::to_string(worst.y) + ", " + std::to_string(worst.z) + ")");
        }
    }
    // The trousers go down over the top of the footwear, by a centimetre at least.
    const Garment* trousers = nullptr;
    const Garment* footwear = nullptr;
    for (const Garment& garment : outfit.garments) {
        trousers = garment.kind == GarmentKind::Trousers ? &garment : trousers;
        footwear = garment.kind == GarmentKind::Shoes || garment.kind == GarmentKind::Boots ? &garment : footwear;
    }
    if (trousers != nullptr && footwear != nullptr) {
        float footTop = -1e9f;
        float trouserBottom = 1e9f;
        for (const glm::vec3& p : footwear->solid.positions) {
            footTop = std::max(footTop, p.y);
        }
        for (const glm::vec3& p : trousers->solid.positions) {
            trouserBottom = std::min(trouserBottom, p.y);
        }
        Check(trouserBottom <= footTop - 0.010f, "the trousers overlap the top of the footwear by a centimetre: " + std::to_string((footTop - trouserBottom) * 1000.0f) + " mm" + who);
    }
}

void GarmentsNeverTouchTheBody() {
    for (const Dressed* personPointer : AllPeople()) {
        const Dressed& person = *personPointer;
        {
            CheckOutfit(person.mesh, person.appearance.outfit, " (seed " + std::to_string(person.parameters.seed) + ")");
        }
    }
}

void LayersOfClothingStackWithoutTouching() {
    // CheckOutfit tests every vertex of every layer against every layer below it; here the stack is looked at as a whole for a build that
    // is not small: the jacket is further from the body than the shirt, and the shirt further than the skin.
    for (const Dressed& person : CachedPeople()) {
        const std::vector<Garment>& garments = person.appearance.outfit.garments;
        const auto find = [&](GarmentKind kind) -> const Garment* {
            for (const Garment& garment : garments) {
                if (garment.kind == kind) {
                    return &garment;
                }
            }
            return nullptr;
        };
        const Garment* shirt = find(GarmentKind::TShirt);
        const Garment* jacket = find(GarmentKind::Jacket);
        if (shirt == nullptr || jacket == nullptr) {
            continue;
        }
        const TriangleGrid body(person.mesh.positions, person.mesh.indices, 0.03f);
        double shirtMean = 0.0;
        double jacketMean = 0.0;
        int shirtCount = 0;
        int jacketCount = 0;
        for (uint32_t v = shirt->solid.outerFirstVertex; v < shirt->solid.positions.size(); ++v) {
            shirtMean += body.FindNearest(shirt->solid.positions[v], 0.1f).signedDistance;
            ++shirtCount;
        }
        for (uint32_t v = jacket->solid.outerFirstVertex; v < jacket->solid.positions.size(); ++v) {
            jacketMean += body.FindNearest(jacket->solid.positions[v], 0.1f).signedDistance;
            ++jacketCount;
        }
        Check(jacketMean / jacketCount > shirtMean / shirtCount + 0.003, "the jacket is further out from the body than the shirt, on average");
    }
}

void EveryoneIsDressedAndMostWearShoes() {
    int shoed = 0;
    int partlyDressed = 0;
    constexpr int kPeople = 1000;
    for (uint32_t seed = 0; seed < kPeople; ++seed) {
        const OutfitSettings outfit = ChooseAppearance(RandomParameters(seed)).outfit;
        partlyDressed += (outfit.top == TopKind::None || outfit.legs == LegwearKind::None) ? 1 : 0;
        shoed += outfit.feet != FootwearKind::None ? 1 : 0;
    }
    Check(partlyDressed == 0, "everyone wears something on the body and the legs: " + std::to_string(partlyDressed));
    Check(shoed > kPeople * 0.85 && shoed < kPeople * 0.95, "about nine in ten wear shoes or boots: " + std::to_string(shoed));
}

void LowDetailKeepsTheShapesAndSavesTheTextures() {
    const CharacterParameters parameters = Person(1.0f, 45.0f, 1.80f, 80.0f, 0.4f, 17);
    const CharacterMesh mesh = GenerateCharacter(parameters, {}, &JobSystem::Shared());
    AppearanceSettings full = TestSettings(parameters);
    full.outfit.top = TopKind::TShirt;  // a lighter outfit: what is compared is the textures and the cards
    full.outfit.legs = LegwearKind::Shorts;
    full.outfit.feet = FootwearKind::Shoes;
    full.skinTileSize = 256;
    full.eyeTextureSize = 256;
    full.outfit.tileSize = 256;
    const AppearanceSettings low = LowDetail(full);
    Check(low.hairStyle == full.hairStyle && low.outfit.top == full.outfit.top && low.outfit.feet == full.outfit.feet, "the low detail appearance is the same person in the same clothes");
    Check(low.skinTileSize < full.skinTileSize && low.outfit.tileSize < full.outfit.tileSize && low.eyeTextureSize < full.eyeTextureSize, "with smaller textures");
    const CharacterAppearance a = GenerateAppearance(mesh, full, &JobSystem::Shared());
    const CharacterAppearance b = GenerateAppearance(mesh, low, &JobSystem::Shared());
    Check(b.skin.textures.albedo.bytes.size() < a.skin.textures.albedo.bytes.size() / 3, "the skin textures are a fraction of the size");
    Check(b.hair.scalp.RibbonCount() < a.hair.scalp.RibbonCount() * 0.6, "there are fewer hair cards");
    Check(b.outfit.garments.size() == a.outfit.garments.size(), "and all the garments");
}

void GarmentsFollowTheSkeletonLikeTheBodyBeneathThem() {
    for (const Dressed& person : CachedPeople()) {
        for (const Garment& garment : person.appearance.outfit.garments) {
            int sameBone = 0;
            int tested = 0;
            for (size_t v = 0; v < garment.mesh.positions.size(); v += std::max<size_t>(1, garment.mesh.positions.size() / 300)) {
                size_t nearest = 0;
                float best = 1e9f;
                for (size_t b = 0; b < person.mesh.positions.size(); ++b) {
                    const float d = glm::distance(person.mesh.positions[b], garment.mesh.positions[v]);
                    if (d < best) {
                        best = d;
                        nearest = b;
                    }
                }
                const auto dominant = [](const std::array<uint8_t, 4>& bones, const std::array<float, 4>& weights) {
                    return bones[std::max_element(weights.begin(), weights.end()) - weights.begin()];
                };
                sameBone += dominant(garment.mesh.boneIndices[v], garment.mesh.boneWeights[v]) == dominant(person.mesh.boneIndices[nearest], person.mesh.boneWeights[nearest]) ? 1 : 0;
                ++tested;
            }
            Check(sameBone >= tested * 0.9, std::string(GarmentName(garment.kind)) + " follows the same bones as the body under it: " + std::to_string(sameBone) + " of " + std::to_string(tested));
        }
    }
}

void ShoesHaveASoleAndTheOutfitCanBeLeftOut() {
    const Dressed& person = CachedPeople()[0];
    Check(person.appearance.outfit.soleHeightMeters > 0.005f && person.appearance.outfit.soleHeightMeters < 0.03f,
          "boots reach a few centimetres below the bare foot: " + std::to_string(person.appearance.outfit.soleHeightMeters));

    OutfitSettings nothing;
    nothing.top = TopKind::None;
    nothing.legs = LegwearKind::None;
    nothing.feet = FootwearKind::None;
    nothing.tileSize = 64;
    Check(BuildOutfit(person.mesh, nothing, &JobSystem::Shared()).garments.empty(), "an outfit of nothing is no garment");
    OutfitSettings shortsOnly = nothing;
    shortsOnly.legs = LegwearKind::Shorts;
    const Outfit outfit = BuildOutfit(person.mesh, shortsOnly, &JobSystem::Shared());
    Check(outfit.garments.size() == 1 && outfit.garments[0].kind == GarmentKind::Shorts && outfit.soleHeightMeters == 0.0f, "shorts alone are one garment with no sole");
    // Shorts end above the knee, a t-shirt above the hip, and trousers below the knee.
    float lowest = 1e9f;
    for (const glm::vec3& p : outfit.garments[0].mesh.positions) {
        lowest = std::min(lowest, p.y);
    }
    Check(lowest > 0.25f * person.mesh.parameters.statureMeters && lowest < 0.45f * person.mesh.parameters.statureMeters, "shorts end between the knee and the hip");
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Skin
// ---------------------------------------------------------------------------------------------------------------------------------

void SkinMeshAndTexturesAreValid() {
    for (const Dressed* personPointer : AllPeople()) {
        const Dressed& person = *personPointer;
        {
            const std::string who = " (seed " + std::to_string(person.parameters.seed) + ")";
            const SkinResult& skin = person.appearance.skin;
            Check(IndicesAreValid(skin.mesh), "the skin mesh has valid indices and weights" + who);
            Check(skin.pieces.size() == person.mesh.pieces.size(), "one range for each piece of the character" + who);
            Check(skin.vertexData.size() == skin.mesh.positions.size(), "vertex data for every vertex" + who);

            // Every uv is inside the atlas, and no triangle spans more than a part of a tile (a triangle across a seam would).
            bool uvInside = true;
            float widest = 0.0f;
            for (const glm::vec2& uv : skin.mesh.uvs) {
                uvInside = uvInside && uv.x >= 0.0f && uv.x <= 1.0f && uv.y >= 0.0f && uv.y <= 1.0f;
            }
            for (size_t t = 0; t + 2 < skin.mesh.indices.size(); t += 3) {
                const float a = skin.mesh.uvs[skin.mesh.indices[t]].x;
                const float b = skin.mesh.uvs[skin.mesh.indices[t + 1]].x;
                const float c = skin.mesh.uvs[skin.mesh.indices[t + 2]].x;
                widest = std::max(widest, std::max({a, b, c}) - std::min({a, b, c}));
            }
            Check(uvInside, "texture coordinates are inside the atlas" + who);
            Check(widest < 0.25f / 2.0f, "no triangle runs across a seam of the unwrap: widest " + std::to_string(widest) + who);

            // The unwrapped surface is the surface of the character: the same area, up to the cuts at the seams.
            double area = 0.0;
            for (size_t t = 0; t + 2 < skin.mesh.indices.size(); t += 3) {
                area += 0.5 * glm::length(glm::cross(skin.mesh.positions[skin.mesh.indices[t + 1]] - skin.mesh.positions[skin.mesh.indices[t]],
                                                     skin.mesh.positions[skin.mesh.indices[t + 2]] - skin.mesh.positions[skin.mesh.indices[t]]));
            }
            double original = 0.0;
            for (size_t t = 0; t + 2 < person.mesh.indices.size(); t += 3) {
                original += 0.5 * glm::length(glm::cross(person.mesh.positions[person.mesh.indices[t + 1]] - person.mesh.positions[person.mesh.indices[t]],
                                                         person.mesh.positions[person.mesh.indices[t + 2]] - person.mesh.positions[person.mesh.indices[t]]));
            }
            Check(std::abs(area / original - 1.0) < 0.005, "the unwrap keeps the area of the surface: " + std::to_string(area / original) + who);

            for (const SkinVertexData& data : skin.vertexData) {
                if (!(data.tone > 0.8f && data.tone < 1.2f && data.redness >= 0.0f && data.redness <= 1.0f && data.thickness >= 0.0f && data.thickness <= 1.0f && data.curvature >= 0.0f &&
                      data.curvature <= 1.0f)) {
                    Check(false, "vertex data in range" + who);
                    break;
                }
            }
        }
    }
}

// The texels that carry skin: those whose normal map alpha (the thickness) was set from a triangle, which is every one but the filler.
void SkinTexturesHaveValuesInRange() {
    for (const Dressed& person : CachedPeople()) {
        const std::string who = " (seed " + std::to_string(person.parameters.seed) + ")";
        const SkinTextures& textures = person.appearance.skin.textures;
        Check(textures.albedo.width == 256 && textures.albedo.height == 128, "the atlas is 4 by 2 tiles" + who);
        Check(textures.normal.width == 256 && textures.roughness.width == 256 && textures.roughness.channels == 1, "the three maps have the same layout" + who);

        double luma = 0.0;
        int unitNormals = 0;
        int uphillNormals = 0;
        int roughnessInRange = 0;
        int pixels = textures.albedo.width * textures.albedo.height;
        uint8_t lowest = 255;
        uint8_t highest = 0;
        for (int y = 0; y < textures.albedo.height; ++y) {
            for (int x = 0; x < textures.albedo.width; ++x) {
                const uint8_t* a = textures.albedo.Pixel(x, y);
                luma += 0.2126 * a[0] + 0.7152 * a[1] + 0.0722 * a[2];
                lowest = std::min(lowest, a[1]);
                highest = std::max(highest, a[1]);
                const uint8_t* n = textures.normal.Pixel(x, y);
                const glm::vec3 normal(n[0] / 127.5f - 1.0f, n[1] / 127.5f - 1.0f, n[2] / 127.5f - 1.0f);
                unitNormals += std::abs(glm::length(normal) - 1.0f) < 0.03f ? 1 : 0;
                uphillNormals += normal.z > 0.5f ? 1 : 0;
                const uint8_t r = *textures.roughness.Pixel(x, y);
                roughnessInRange += r >= 50 && r <= 235 ? 1 : 0;
            }
        }
        luma /= pixels;
        Check(luma > 20.0 && luma < 235.0, "the albedo is neither black nor white: " + std::to_string(luma) + who);
        Check(highest - lowest > 40, "the albedo has variation: " + std::to_string(highest - lowest) + who);
        Check(unitNormals > pixels * 0.99, "normals are unit vectors" + who);
        Check(uphillNormals == pixels, "normals face out of the surface" + who);
        Check(roughnessInRange == pixels, "roughness is neither mirror nor chalk" + who);
    }
}

// The mean brightness of the albedo over the skin of the person, in the order of skin tones.
void SkinColoursAreBelievableAndSpanTheRange() {
    float darkest = 1.0f;
    float lightest = 0.0f;
    for (uint32_t seed = 0; seed < 300; ++seed) {
        const PersonTraits traits = DeriveTraits(Person(0.5f, 40.0f, 1.7f, 70.0f, 0.4f, seed));
        darkest = std::min(darkest, traits.melanin);
        lightest = std::max(lightest, traits.melanin);
        const float brightness = traits.skinColour.r + traits.skinColour.g + traits.skinColour.b;
        Check(brightness > 0.2f && brightness < 1.4f, "a skin colour is a believable reflectance");
        Check(traits.skinColour.r > traits.skinColour.g && traits.skinColour.g > traits.skinColour.b, "skin is redder than it is green or blue");
    }
    Check(darkest < 0.2f && lightest > 0.8f, "skin tones cover the range: " + std::to_string(darkest) + " to " + std::to_string(lightest));
}

// How much relief the skin of the head has: the mean of the sideways lean of the normals over the head tile.
double MeanRelief(const SkinTextures& textures) {
    const int tile = textures.tileSize;
    const int left = 1 * tile;  // the head is tile 5: second row, second column
    const int top = 1 * tile;
    double sum = 0.0;
    for (int y = top; y < top + tile; ++y) {
        for (int x = left; x < left + tile; ++x) {
            const uint8_t* n = textures.normal.Pixel(x, y);
            sum += std::hypot(n[0] / 127.5 - 1.0, n[1] / 127.5 - 1.0);
        }
    }
    return sum / (static_cast<double>(tile) * tile);
}

void AgeAddsWrinklesSpotsAndGreyAndSexChangesTheFace() {
    const PersonTraits young = DeriveTraits(Person(0.5f, 20.0f, 1.7f, 70.0f, 0.4f, 9));
    const PersonTraits old = DeriveTraits(Person(0.5f, 78.0f, 1.7f, 70.0f, 0.4f, 9));
    Check(old.wrinkles > young.wrinkles + 0.4f, "an old face is more wrinkled");
    Check(old.ageSpots >= young.ageSpots, "and has more age spots");
    Check(old.greyFraction > young.greyFraction + 0.3f, "and greyer hair");
    Check(young.wrinkles < 0.05f, "a young face has no wrinkles");

    // The same person at 20 and at 78: the skin of the head has more relief when old.
    const CharacterParameters youngParameters = Person(0.5f, 20.0f, 1.70f, 70.0f, 0.4f, 9);
    CharacterParameters oldParameters = youngParameters;
    oldParameters.ageYears = 78.0f;
    const CharacterMesh youngMesh = GenerateCharacter(youngParameters, {}, &JobSystem::Shared());
    const CharacterMesh oldMesh = GenerateCharacter(oldParameters, {}, &JobSystem::Shared());
    AppearanceSettings settings = TestSettings(youngParameters);
    settings.skinTileSize = 128;
    settings.outfit.top = TopKind::None;  // the skin is what is compared
    settings.outfit.legs = LegwearKind::None;
    settings.outfit.feet = FootwearKind::None;
    const CharacterAppearance youngSkin = GenerateAppearance(youngMesh, settings, &JobSystem::Shared());
    const CharacterAppearance oldSkin = GenerateAppearance(oldMesh, settings, &JobSystem::Shared());
    Check(MeanRelief(oldSkin.skin.textures) > MeanRelief(youngSkin.skin.textures) * 1.05, "the normal map of an old head has more relief: " + std::to_string(MeanRelief(oldSkin.skin.textures)) + " against " +
                                                                                              std::to_string(MeanRelief(youngSkin.skin.textures)));

    // Sex: men have a heavier brow, so the eyebrows sit differently; and the skin is a little ruddier.
    double manRuddiness = 0.0;
    double womanRuddiness = 0.0;
    for (uint32_t seed = 0; seed < 200; ++seed) {
        manRuddiness += DeriveTraits(Person(1.0f, 40.0f, 1.8f, 80.0f, 0.4f, seed)).ruddiness;
        womanRuddiness += DeriveTraits(Person(0.0f, 40.0f, 1.65f, 60.0f, 0.4f, seed)).ruddiness;
    }
    Check(manRuddiness > womanRuddiness, "men have ruddier skin on average");
    const CharacterAppearance& man = CachedPeople()[0].appearance;
    const CharacterAppearance& woman = CachedPeople()[1].appearance;
    Check(man.hair.eyebrows.RibbonCount() > woman.hair.eyebrows.RibbonCount(), "men have fuller eyebrows");
}

void DifferentSeedsGiveDifferentFacesAndFeatures() {
    int differentIris = 0;
    int moles = 0;
    for (uint32_t seed = 1; seed <= 100; ++seed) {
        const PersonTraits a = DeriveTraits(Person(0.5f, 40.0f, 1.7f, 70.0f, 0.4f, seed));
        const PersonTraits b = DeriveTraits(Person(0.5f, 40.0f, 1.7f, 70.0f, 0.4f, seed + 1000));
        differentIris += glm::distance(a.irisColour, b.irisColour) > 0.01f ? 1 : 0;
        moles += a.moles > 0.5f ? 1 : 0;
    }
    Check(differentIris > 30, "iris colours vary with the seed: " + std::to_string(differentIris));
    Check(moles > 10 && moles < 90, "so do moles");
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Extreme builds
// ---------------------------------------------------------------------------------------------------------------------------------

void ExtremeBuildsKeepEyesHairAndClothesInOrder() {
    for (const Dressed& person : CachedWorstBuilds()) {
        const std::string who = " (seed " + std::to_string(person.parameters.seed) + ")";
        const CharacterAppearance& appearance = person.appearance;
        Check(appearance.hair.scalp.RibbonCount() > 500, "hair on every build" + who);
        Check(appearance.outfit.soleHeightMeters > 0.0f, "boots on every build" + who);
        float topOfHair = -1e9f;
        for (const glm::vec3& p : appearance.hair.scalp.mesh.positions) {
            topOfHair = std::max(topOfHair, p.y);
        }
        Check(topOfHair > person.mesh.parameters.statureMeters - 0.01f && topOfHair < person.mesh.parameters.statureMeters + 0.20f * person.mesh.parameters.statureMeters / 1.75f,
              "the hair is on top of the head" + who);
        // A garment lies within the bounds of the character plus a few centimetres of ease.
        float widest = 0.0f;
        for (const glm::vec3& p : person.mesh.positions) {
            widest = std::max(widest, std::abs(p.x));
        }
        for (const Garment& garment : appearance.outfit.garments) {
            float garmentWidest = 0.0f;
            for (const glm::vec3& p : garment.mesh.positions) {
                garmentWidest = std::max(garmentWidest, std::abs(p.x));
            }
            Check(garmentWidest < widest + 0.05f, std::string(GarmentName(garment.kind)) + " is not wider than the body by more than the ease" + who);
        }
    }
}

// The skin that is drawn (the copy of the character mesh that is fitted round the eyes) has no vertex inside an eyeball, and so no
// skin shows through the eye near the opening; the same on every build.
void NoSkinVertexIsInsideAnEyeball() {
    for (const Dressed* personPointer : AllPeople()) {
        const Dressed& person = *personPointer;
        {
            const std::string who = " (seed " + std::to_string(person.parameters.seed) + ")";
            const SkinResult& skin = person.appearance.skin;
            int inside = 0;
            float deepest = 0.0f;
            for (const MeshPiece& piece : skin.pieces) {
                if (piece.kind != MeshPieceKind::Head) {
                    continue;
                }
                for (uint32_t v = piece.firstVertex; v < piece.firstVertex + piece.vertexCount; ++v) {
                    for (const EyeParts& eye : person.appearance.eyes.eyes) {
                        const float depth = EyeballRadiusToward(eye, skin.mesh.positions[v]) - glm::distance(skin.mesh.positions[v], eye.centre);
                        if (depth > 0.0001f) {
                            ++inside;
                            deepest = std::max(deepest, depth);
                        }
                    }
                }
            }
            Check(inside == 0, std::to_string(inside) + " vertices of the head skin are inside an eyeball, up to " + std::to_string(deepest * 1000.0f) + " mm" + who);
        }
    }
}

// Hair and beard lie outside every garment: a beard over the collar of a jacket, hair over its shoulders.
void HairAndBeardLieOutsideTheClothes() {
    for (const Dressed* personPointer : AllPeople()) {
        const Dressed& person = *personPointer;
        {
            const std::string who = " (seed " + std::to_string(person.parameters.seed) + ")";
            for (const Garment& garment : person.appearance.outfit.garments) {
                const OuterSurface outer(garment);
                for (const RibbonSet* set : {&person.appearance.hair.scalp, &person.appearance.hair.facial, &person.appearance.hair.eyebrows}) {
                    int inside = 0;
                    float deepest = 0.0f;
                    for (const Probe& probe : ProbeAll(*outer.grid, set->mesh.positions, 0.03f)) {
                        if (probe.signedDistance < -0.0010f) {
                            ++inside;
                            deepest = std::min(deepest, probe.signedDistance);
                        }
                    }
                    Check(inside == 0, std::to_string(inside) + " hair vertices are inside the " + GarmentName(garment.kind) + ", up to " + std::to_string(-deepest * 1000.0f) + " mm" + who);
                }
            }
        }
    }
}

void TheStrandTextureHasStrandsAndGaps() {
    const HairTextures textures = MakeHairTextures(5);
    Check(textures.strands.width == 128 && textures.strands.height == 512 && textures.normal.width == 128, "the strand texture has its size");
    int opaque = 0;
    int opaqueAtTip = 0;
    int opaqueAtRoot = 0;
    for (int y = 0; y < textures.strands.height; ++y) {
        for (int x = 0; x < textures.strands.width; ++x) {
            const bool on = textures.strands.Pixel(x, y)[3] == 255;
            opaque += on ? 1 : 0;
            opaqueAtRoot += (on && y < 64) ? 1 : 0;
            opaqueAtTip += (on && y >= textures.strands.height - 64) ? 1 : 0;
            Check(textures.strands.Pixel(x, y)[3] == 0 || textures.strands.Pixel(x, y)[3] == 255, "alpha is a cut-out, 0 or 255");
        }
    }
    const double covered = static_cast<double>(opaque) / (textures.strands.width * textures.strands.height);
    Check(covered > 0.25 && covered < 0.75, "strands cover part of the card and leave gaps: " + std::to_string(covered));
    Check(opaqueAtTip < opaqueAtRoot * 0.7, "the card thins out towards the tips");
    Check(SameImage(textures.strands, MakeHairTextures(5).strands) && !SameImage(textures.strands, MakeHairTextures(6).strands), "the texture follows the seed");
}

void ACharacterWithoutAHeadPieceStillGetsClothes() {
    MeshSettings settings;
    settings.faceAndHands = false;
    const CharacterParameters parameters = Person(0.5f, 35.0f, 1.72f, 72.0f, 0.4f, 11);
    const CharacterMesh mesh = GenerateCharacter(parameters, settings, &JobSystem::Shared());
    AppearanceSettings lighter = TestSettings(parameters);
    lighter.outfit.top = TopKind::TShirt;
    lighter.outfit.legs = LegwearKind::Shorts;
    lighter.outfit.feet = FootwearKind::None;
    const CharacterAppearance appearance = GenerateAppearance(mesh, lighter, &JobSystem::Shared());
    Check(appearance.hair.scalp.mesh.positions.empty() && appearance.eyes.eyes[0].eyeball.positions.empty(), "no head piece, no hair and no eyes");
    Check(!appearance.outfit.garments.empty() && !appearance.skin.mesh.positions.empty(), "but skin and clothes");
}

}  // namespace

// With a word on the command line, only the tests whose names contain it run, which is quick when one test is being worked on.
int main(int argc, char** argv) {
    // `--full` tests the whole sweep of builds, not only the default set; any other word is a filter on the names of the tests.
    std::string filter;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--full") {
            gFullRun = true;
        } else {
            filter = argv[i];
        }
    }
    struct Test {
        const char* name;
        std::function<void()> run;
    };
    const Test tests[] = {
        {"TheSameParametersGiveTheSameAppearanceOnAnyNumberOfThreads", TheSameParametersGiveTheSameAppearanceOnAnyNumberOfThreads},
        {"TheChoiceOfAppearanceFollowsTheSeed", TheChoiceOfAppearanceFollowsTheSeed},
        {"EyesSitInTheSocketsAndDoNotPokeThroughTheHead", EyesSitInTheSocketsAndDoNotPokeThroughTheHead},
        {"TheLandmarksAreWhereTheHeadIsHollow", TheLandmarksAreWhereTheHeadIsHollow},
        {"EyeColourAndTextureFollowThePerson", EyeColourAndTextureFollowThePerson},
        {"HairGrowsFromTheScalpAndNeverEntersTheHead", HairGrowsFromTheScalpAndNeverEntersTheHead},
        {"HairStylesDifferAsTheirNamesSay", HairStylesDifferAsTheirNamesSay},
        {"HairGreysWithAgeAndThinsWithAgeInMen", HairGreysWithAgeAndThinsWithAgeInMen},
        {"FacialHairGrowsOnTheJawAndNotOnTheLips", FacialHairGrowsOnTheJawAndNotOnTheLips},
        {"GarmentsAreClosedShellsWithValidSkinning", GarmentsAreClosedShellsWithValidSkinning},
        {"GarmentsNeverTouchTheBody", GarmentsNeverTouchTheBody},
        {"LayersOfClothingStackWithoutTouching", LayersOfClothingStackWithoutTouching},
        {"GarmentsFollowTheSkeletonLikeTheBodyBeneathThem", GarmentsFollowTheSkeletonLikeTheBodyBeneathThem},
        {"ShoesHaveASoleAndTheOutfitCanBeLeftOut", ShoesHaveASoleAndTheOutfitCanBeLeftOut},
        {"SkinMeshAndTexturesAreValid", SkinMeshAndTexturesAreValid},
        {"SkinTexturesHaveValuesInRange", SkinTexturesHaveValuesInRange},
        {"SkinColoursAreBelievableAndSpanTheRange", SkinColoursAreBelievableAndSpanTheRange},
        {"AgeAddsWrinklesSpotsAndGreyAndSexChangesTheFace", AgeAddsWrinklesSpotsAndGreyAndSexChangesTheFace},
        {"DifferentSeedsGiveDifferentFacesAndFeatures", DifferentSeedsGiveDifferentFacesAndFeatures},
        {"ExtremeBuildsKeepEyesHairAndClothesInOrder", ExtremeBuildsKeepEyesHairAndClothesInOrder},
        {"EveryoneIsDressedAndMostWearShoes", EveryoneIsDressedAndMostWearShoes},
        {"LowDetailKeepsTheShapesAndSavesTheTextures", LowDetailKeepsTheShapesAndSavesTheTextures},
        {"NoSkinVertexIsInsideAnEyeball", NoSkinVertexIsInsideAnEyeball},
        {"HairAndBeardLieOutsideTheClothes", HairAndBeardLieOutsideTheClothes},
        {"TheStrandTextureHasStrandsAndGaps", TheStrandTextureHasStrandsAndGaps},
        {"ACharacterWithoutAHeadPieceStillGetsClothes", ACharacterWithoutAHeadPieceStillGetsClothes},
    };
    for (const Test& test : tests) {
        if (!filter.empty() && std::string(test.name).find(filter) == std::string::npos) {
            continue;
        }
        const int failuresBefore = failureCount;
        const auto started = std::chrono::steady_clock::now();
        try {
            test.run();
        } catch (const std::exception& error) {
            ++failureCount;
            std::cerr << "  threw: " << error.what() << "\n";
        }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        std::cout << (failureCount == failuresBefore ? "PASS  " : "FAIL  ") << test.name << " (" << static_cast<int>(seconds + 0.5) << " s)\n" << std::flush;
    }
    std::cout << (failureCount == 0 ? "all appearance tests passed\n" : "appearance tests failed\n");
    return failureCount == 0 ? 0 : 1;
}
