#include "character/CharacterGenerator.h"

#include <algorithm>
#include <cmath>

#include "character/Anthropometry.h"
#include "character/BodyShape.h"
#include "character/DetailShapes.h"
#include "character/ParallelEach.h"
#include "character/SurfaceExtraction.h"
#include "core/JobSystem.h"

namespace character {
namespace {

// The weights of every vertex: the four nearest bones among the nearest one, its parent and its children, smoothed
// over the surface so that one bone gives way to the next over a few centimetres and not at a line.
void ComputeSkinWeights(const BodyShape& shape, const Skeleton& skeleton, CharacterMesh& mesh, JobSystem* jobs) {
    constexpr float kBlendWidth = 0.040f;  // metres of the surface over which one bone gives way to the next, before smoothing
    constexpr int kSmoothingPasses = 4;
    const size_t vertexCount = mesh.positions.size();
    mesh.boneIndices.resize(vertexCount);
    mesh.boneWeights.resize(vertexCount);

    // One weight for every bone of every vertex, and which bones each vertex may follow.
    std::vector<std::array<float, kJointCount>> dense(vertexCount);
    std::vector<uint32_t> allowed(vertexCount, 0u);
    ParallelEach(jobs, static_cast<int>(vertexCount), [&](int v) {
        const glm::vec3 point = mesh.positions[v];
        float boneDistance[kJointCount];
        std::fill(boneDistance, boneDistance + kJointCount, 1e9f);
        for (size_t part = 0; part < shape.PartCount(); ++part) {
            const int bone = Index(shape.PartBone(part));
            boneDistance[bone] = std::min(boneDistance[bone], shape.PartDistance(part, point));
        }
        int nearest = 0;
        for (int bone = 1; bone < kJointCount; ++bone) {
            if (boneDistance[bone] < boneDistance[nearest]) {
                nearest = bone;
            }
        }
        // Candidates: the nearest bone, its parent and its children.
        for (int bone = 0; bone < kJointCount; ++bone) {
            const bool neighbour = bone == nearest || bone == skeleton.joints[nearest].parent || skeleton.joints[bone].parent == nearest;
            dense[v][bone] = 0.0f;
            if (neighbour) {
                dense[v][bone] = std::exp(-(boneDistance[bone] - boneDistance[nearest]) / kBlendWidth);
                allowed[v] |= 1u << bone;
            }
        }
    });

    // The neighbours of each vertex are the other corners of its triangles.
    std::vector<std::vector<uint32_t>> neighbours(vertexCount);
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        for (int corner = 0; corner < 3; ++corner) {
            neighbours[mesh.indices[t + corner]].push_back(mesh.indices[t + (corner + 1) % 3]);
            neighbours[mesh.indices[t + corner]].push_back(mesh.indices[t + (corner + 2) % 3]);
        }
    }
    std::vector<std::array<float, kJointCount>> smoothed(vertexCount);
    for (int pass = 0; pass < kSmoothingPasses; ++pass) {
        ParallelEach(jobs, static_cast<int>(vertexCount), [&](int v) {
            std::array<float, kJointCount> average{};
            for (const uint32_t other : neighbours[v]) {
                for (int bone = 0; bone < kJointCount; ++bone) {
                    average[bone] += dense[other][bone];
                }
            }
            const float share = neighbours[v].empty() ? 0.0f : 1.0f / static_cast<float>(neighbours[v].size());
            float total = 0.0f;
            for (int bone = 0; bone < kJointCount; ++bone) {
                // Only the bones this vertex may follow keep any weight: the torso still never pulls on an upper arm.
                const bool permitted = (allowed[v] >> bone & 1u) != 0u;
                smoothed[v][bone] = permitted ? 0.5f * dense[v][bone] + 0.5f * average[bone] * share : 0.0f;
                total += smoothed[v][bone];
            }
            for (float& value : smoothed[v]) {
                value /= std::max(total, 1e-9f);
            }
        });
        dense.swap(smoothed);
    }

    // The four heaviest bones of every vertex, scaled to add up to 1.
    ParallelEach(jobs, static_cast<int>(vertexCount), [&](int v) {
        std::array<float, kJointCount> weight = dense[v];
        std::array<uint8_t, 4> indices{};
        std::array<float, 4> weights{};
        float total = 0.0f;
        for (int slot = 0; slot < 4; ++slot) {
            int best = -1;
            for (int bone = 0; bone < kJointCount; ++bone) {
                if (weight[bone] > 0.0f && (best < 0 || weight[bone] > weight[best])) {
                    best = bone;
                }
            }
            if (best < 0) {
                break;
            }
            indices[slot] = static_cast<uint8_t>(best);
            weights[slot] = weight[best];
            total += weight[best];
            weight[best] = 0.0f;
        }
        for (float& value : weights) {
            value /= total;
        }
        mesh.boneIndices[v] = indices;
        mesh.boneWeights[v] = weights;
    });
}

}  // namespace

MeshTopology MeasureTopology(const CharacterMesh& mesh, size_t pieceIndex) {
    MeshTopology topology;
    if (pieceIndex >= mesh.pieces.size()) {
        topology.badEdges = 1;  // there is no such piece, which is not a closed sphere
        return topology;
    }
    const MeshPiece& piece = mesh.pieces[pieceIndex];
    const uint32_t firstVertex = piece.firstVertex;
    const size_t vertexCount = piece.vertexCount;
    const uint32_t* indices = mesh.indices.data() + piece.firstIndex;

    // The pieces of the surface: groups of vertices joined by triangles.
    std::vector<int> parent(vertexCount);
    for (size_t v = 0; v < vertexCount; ++v) {
        parent[v] = static_cast<int>(v);
    }
    const auto find = [&](int v) {
        while (parent[v] != v) {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        return v;
    };
    std::vector<bool> used(vertexCount, false);
    for (size_t t = 0; t + 2 < piece.indexCount; t += 3) {
        const int a = static_cast<int>(indices[t] - firstVertex);
        const int b = static_cast<int>(indices[t + 1] - firstVertex);
        const int c = static_cast<int>(indices[t + 2] - firstVertex);
        parent[find(a)] = find(b);
        parent[find(b)] = find(c);
        used[a] = used[b] = used[c] = true;
    }
    long long usedVertices = 0;
    for (size_t v = 0; v < vertexCount; ++v) {
        usedVertices += used[v] ? 1 : 0;
        topology.pieces += (used[v] && find(static_cast<int>(v)) == static_cast<int>(v)) ? 1 : 0;
    }

    // The edges: each should be in exactly two triangles.
    std::vector<uint64_t> edges;
    edges.reserve(piece.indexCount);
    for (size_t t = 0; t + 2 < piece.indexCount; t += 3) {
        for (int corner = 0; corner < 3; ++corner) {
            uint32_t a = indices[t + corner];
            uint32_t b = indices[t + (corner + 1) % 3];
            if (a > b) {
                std::swap(a, b);
            }
            edges.push_back((static_cast<uint64_t>(a) << 32) | b);
        }
    }
    std::sort(edges.begin(), edges.end());
    long long distinctEdges = 0;
    for (size_t i = 0; i < edges.size();) {
        size_t end = i;
        while (end < edges.size() && edges[end] == edges[i]) {
            ++end;
        }
        ++distinctEdges;
        topology.badEdges += (end - i) != 2 ? 1 : 0;
        i = end;
    }
    topology.eulerCharacteristic = usedVertices - distinctEdges + static_cast<long long>(piece.indexCount / 3);
    return topology;
}

float MeshVolume(const std::vector<glm::vec3>& positions, const std::vector<uint32_t>& indices) {
    double volume = 0.0;
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        const glm::dvec3 a(positions[indices[i]]);
        const glm::dvec3 b(positions[indices[i + 1]]);
        const glm::dvec3 c(positions[indices[i + 2]]);
        volume += glm::dot(a, glm::cross(b, c)) / 6.0;
    }
    return static_cast<float>(volume);
}

namespace {

// Adds a finer piece to the end of the mesh, fastened to one bone with all its weight.
void AppendPiece(CharacterMesh& mesh, const Surface& surface, MeshPieceKind kind, Joint bone) {
    MeshPiece piece;
    piece.kind = kind;
    piece.firstVertex = static_cast<uint32_t>(mesh.positions.size());
    piece.vertexCount = static_cast<uint32_t>(surface.positions.size());
    piece.firstIndex = static_cast<uint32_t>(mesh.indices.size());
    piece.indexCount = static_cast<uint32_t>(surface.indices.size());
    mesh.positions.insert(mesh.positions.end(), surface.positions.begin(), surface.positions.end());
    mesh.normals.insert(mesh.normals.end(), surface.normals.begin(), surface.normals.end());
    for (const uint32_t index : surface.indices) {
        mesh.indices.push_back(index + piece.firstVertex);
    }
    mesh.boneIndices.insert(mesh.boneIndices.end(), surface.positions.size(), std::array<uint8_t, 4>{static_cast<uint8_t>(Index(bone)), 0, 0, 0});
    mesh.boneWeights.insert(mesh.boneWeights.end(), surface.positions.size(), std::array<float, 4>{1.0f, 0.0f, 0.0f, 0.0f});
    mesh.pieces.push_back(piece);
}

}  // namespace

CharacterMesh GenerateCharacter(const CharacterParameters& parameters, const MeshSettings& settings, JobSystem* jobs) {
    CharacterMesh mesh;
    mesh.parameters = Sanitised(parameters);
    mesh.skeleton = BuildSkeleton(mesh.parameters);
    const auto finiteOr = [](float value, float fallback) { return std::isfinite(value) ? value : fallback; };
    const MeshSettings defaults;
    const float voxel = std::clamp(finiteOr(settings.voxelMeters, defaults.voxelMeters), 0.006f, 0.04f);
    const float minimumRadius = 0.55f * voxel;  // nothing thinner than a cell and a bit, or it breaks up
    const float targetVolume = mesh.parameters.massKg / anthropometry::kBodyDensity;

    // Find the girth that gives the volume. The volume grows about with the square of the girth, since the lengths of
    // the limbs and trunk are fixed, so a few secant steps on the coarse volume are enough. The coarse volume is of the
    // body with a plain head and hands, which are close in size to the finer ones.
    float girth = 1.0f;
    float previousGirth = 0.0f;
    float previousVolume = 0.0f;
    for (int step = 0; step < 6; ++step) {
        const float volume = CoarseVolume(BuildBodyShape(mesh.skeleton, mesh.parameters, {.girth = girth, .minimumRadius = minimumRadius}), jobs);
        if (std::abs(volume / targetVolume - 1.0f) < 0.01f) {
            break;
        }
        float next = girth * std::sqrt(targetVolume / std::max(volume, 1e-4f));
        if (previousGirth > 0.0f && std::abs(volume - previousVolume) > 1e-5f) {
            next = girth + (targetVolume - volume) * (girth - previousGirth) / (volume - previousVolume);
        }
        previousGirth = girth;
        previousVolume = volume;
        girth = std::clamp(next, 0.45f, 2.2f);
    }

    // The head does not depend on the girth, so it is cut once. A hand has a cuff as wide as the forearm, which does, so the
    // hands are cut again if the girth ends up more than a hundredth from the one they were made for.
    Surface head;
    Surface hands[2];
    float detailVolume = 0.0f;
    float handsGirth = 0.0f;
    const float handVoxel = std::clamp(finiteOr(settings.handVoxelMeters, defaults.handVoxelMeters), 0.002f, 0.02f);
    const auto cutHands = [&](float forGirth) {
        handsGirth = forGirth;
        // The gaps between the fingers are thinner than two cells, which surface nets cannot hold: tetrahedra can.
        hands[0] = ExtractSoundSurface(BuildHandShape(mesh.skeleton, mesh.parameters, true, forGirth), handVoxel, CuttingMethod::Tetrahedra, jobs);
        hands[1] = ExtractSoundSurface(BuildHandShape(mesh.skeleton, mesh.parameters, false, forGirth), handVoxel, CuttingMethod::Tetrahedra, jobs);
    };
    const auto measureDetails = [&]() {
        detailVolume = MeshVolume(head.positions, head.indices) + MeshVolume(hands[0].positions, hands[0].indices) + MeshVolume(hands[1].positions, hands[1].indices);
    };
    if (settings.faceAndHands) {
        const float headVoxel = std::clamp(finiteOr(settings.headVoxelMeters, defaults.headVoxelMeters), 0.002f, 0.02f);
        head = ExtractSoundSurface(BuildHeadShape(mesh.parameters), headVoxel, CuttingMethod::SurfaceNets, jobs);
        cutHands(girth);
        measureDetails();
    }

    // Cut the body, then correct the girth with the volume of the mesh itself, which the coarse count only estimates.
    BodyShapeOptions options{.girth = girth, .minimumRadius = minimumRadius, .includeHead = !settings.faceAndHands, .includeHands = !settings.faceAndHands};
    Surface body;
    BodyShape shape;
    for (int pass = 0; pass < 3; ++pass) {
        options.girth = girth;
        shape = BuildBodyShape(mesh.skeleton, mesh.parameters, options);
        body = ExtractSoundSurface(shape, voxel, CuttingMethod::Tetrahedra, jobs);
        const float volume = MeshVolume(body.positions, body.indices) + detailVolume;
        const float error = volume / targetVolume;
        if (pass == 2 || std::abs(error - 1.0f) < 0.01f) {
            break;
        }
        girth = std::clamp(girth / std::sqrt(error), 0.45f, 2.2f);
        if (settings.faceAndHands && std::abs(girth / handsGirth - 1.0f) > 0.01f) {
            cutHands(girth);
            measureDetails();
        }
    }
    mesh.girth = girth;

    // The body first, with its weights, then the finer pieces, each fastened to one bone.
    mesh.positions = std::move(body.positions);
    mesh.normals = std::move(body.normals);
    mesh.indices = std::move(body.indices);
    ComputeSkinWeights(shape, mesh.skeleton, mesh, jobs);
    MeshPiece bodyPiece;
    bodyPiece.kind = MeshPieceKind::Body;
    bodyPiece.vertexCount = static_cast<uint32_t>(mesh.positions.size());
    bodyPiece.indexCount = static_cast<uint32_t>(mesh.indices.size());
    mesh.pieces.push_back(bodyPiece);
    if (settings.faceAndHands) {
        if (std::abs(mesh.girth / handsGirth - 1.0f) > 0.01f) {
            cutHands(mesh.girth);  // the last correction of the girth was not followed by another pass
        }
        AppendPiece(mesh, head, MeshPieceKind::Head, Joint::Head);
        AppendPiece(mesh, hands[0], MeshPieceKind::HandLeft, Joint::HandL);
        AppendPiece(mesh, hands[1], MeshPieceKind::HandRight, Joint::HandR);
    }
    mesh.volumeCubicMeters = MeshVolume(mesh.positions, mesh.indices);
    mesh.massKg = mesh.volumeCubicMeters * anthropometry::kBodyDensity;
    return mesh;
}

std::vector<glm::vec3> SkinPositions(const CharacterMesh& mesh, const Pose& pose) {
    const std::array<glm::mat4, kJointCount> skin = SkinMatrices(mesh.skeleton, pose);
    std::vector<glm::vec3> result(mesh.positions.size());
    for (size_t v = 0; v < mesh.positions.size(); ++v) {
        glm::vec3 sum(0.0f);
        for (int slot = 0; slot < 4; ++slot) {
            const float weight = mesh.boneWeights[v][slot];
            if (weight > 0.0f) {
                sum += weight * glm::vec3(skin[mesh.boneIndices[v][slot]] * glm::vec4(mesh.positions[v], 1.0f));
            }
        }
        result[v] = sum;
    }
    return result;
}

}  // namespace character
