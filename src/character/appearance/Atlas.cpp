#include "character/appearance/Atlas.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "character/ParallelEach.h"

namespace character {
namespace {

constexpr float kTwoPi = 6.28318530718f;
constexpr int kGutterTexels = 3;  // between the unwrapped surface and the edge of its tile

// The cylinder of a tile: where its axis starts, which way it runs (down the image), and which way is the front of the
// body, where u = 0.5.
struct TileFrame {
    glm::vec3 origin{0.0f};
    glm::vec3 down{0.0f, -1.0f, 0.0f};
    glm::vec3 front{0.0f, 0.0f, 1.0f};
    glm::vec3 right{1.0f, 0.0f, 0.0f};  // to the right of the viewer who looks at the front of the body
    float alongMin = 0.0f;
    float alongMax = 1.0f;
};

TileFrame MakeFrame(glm::vec3 origin, glm::vec3 downDirection) {
    TileFrame frame;
    frame.origin = origin;
    frame.down = glm::normalize(downDirection);
    // The front is the world's +Z with whatever runs along the axis taken out of it.
    const glm::vec3 forward(0.0f, 0.0f, 1.0f);
    frame.front = glm::normalize(forward - frame.down * glm::dot(forward, frame.down));
    frame.right = glm::cross(frame.front, frame.down);
    return frame;
}

std::array<TileFrame, kAtlasTileCount> MakeFrames(const Skeleton& skeleton) {
    const auto at = [&](Joint joint) { return skeleton[joint].restPosition; };
    const glm::vec3 straightDown(0.0f, -1.0f, 0.0f);
    std::array<TileFrame, kAtlasTileCount> frames;
    const auto set = [&](AtlasTile tile, glm::vec3 origin, glm::vec3 down) { frames[static_cast<int>(tile)] = MakeFrame(origin, down); };
    set(AtlasTile::Trunk, glm::vec3(0.0f, at(Joint::Pelvis).y, at(Joint::Pelvis).z), straightDown);
    set(AtlasTile::Head, glm::vec3(0.0f, at(Joint::Head).y, at(Joint::Head).z), straightDown);
    set(AtlasTile::ArmLeft, at(Joint::UpperArmL), at(Joint::HandL) - at(Joint::UpperArmL));
    set(AtlasTile::ArmRight, at(Joint::UpperArmR), at(Joint::HandR) - at(Joint::UpperArmR));
    set(AtlasTile::LegLeft, at(Joint::ThighL), at(Joint::FootL) - at(Joint::ThighL));
    set(AtlasTile::LegRight, at(Joint::ThighR), at(Joint::FootR) - at(Joint::ThighR));
    set(AtlasTile::HandLeft, at(Joint::HandL), at(Joint::HandL) - at(Joint::ForearmL));
    set(AtlasTile::HandRight, at(Joint::HandR), at(Joint::HandR) - at(Joint::ForearmR));
    return frames;
}

float AlongOf(const TileFrame& frame, glm::vec3 position) {
    return glm::dot(position - frame.origin, frame.down);
}

float AngleUOf(const TileFrame& frame, glm::vec3 position) {
    const glm::vec3 d = position - frame.origin;
    return std::atan2(glm::dot(d, frame.right), glm::dot(d, frame.front)) / kTwoPi + 0.5f;
}

}  // namespace

AtlasTile TileForBone(int bone) {
    switch (static_cast<Joint>(bone)) {
        case Joint::UpperArmL:
        case Joint::ForearmL:
            return AtlasTile::ArmLeft;
        case Joint::UpperArmR:
        case Joint::ForearmR:
            return AtlasTile::ArmRight;
        case Joint::ThighL:
        case Joint::CalfL:
        case Joint::FootL:
        case Joint::ToesL:
            return AtlasTile::LegLeft;
        case Joint::ThighR:
        case Joint::CalfR:
        case Joint::FootR:
        case Joint::ToesR:
            return AtlasTile::LegRight;
        case Joint::HandL:
            return AtlasTile::HandLeft;
        case Joint::HandR:
            return AtlasTile::HandRight;
        case Joint::Head:
            return AtlasTile::Head;
        default:
            return AtlasTile::Trunk;
    }
}

AtlasTile TileForPiece(MeshPieceKind kind) {
    switch (kind) {
        case MeshPieceKind::Head:
            return AtlasTile::Head;
        case MeshPieceKind::HandLeft:
            return AtlasTile::HandLeft;
        case MeshPieceKind::HandRight:
            return AtlasTile::HandRight;
        default:
            return AtlasTile::Trunk;
    }
}

TriangleMesh Unwrap(const TriangleMesh& mesh, const Skeleton& skeleton, int tileSize, int pieceTile) {
    std::array<TileFrame, kAtlasTileCount> frames = MakeFrames(skeleton);

    // Every triangle goes to one tile. A vertex that is shared by triangles of two tiles is made twice.
    struct TileMesh {
        TriangleMesh mesh;
        std::unordered_map<uint32_t, uint32_t> remap;  // vertex of the input -> vertex of this tile's mesh
    };
    std::array<TileMesh, kAtlasTileCount> tiles;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        int tile = pieceTile;
        if (tile < 0) {
            int votes[kAtlasTileCount] = {};
            for (int c = 0; c < 3; ++c) {
                ++votes[static_cast<int>(TileForBone(DominantBone(mesh.vertices[mesh.indices[t + c]])))];
            }
            tile = static_cast<int>(std::max_element(votes, votes + kAtlasTileCount) - votes);
        }
        TileMesh& target = tiles[tile];
        for (int c = 0; c < 3; ++c) {
            const uint32_t old = mesh.indices[t + c];
            const auto found = target.remap.find(old);
            if (found == target.remap.end()) {
                target.remap.emplace(old, static_cast<uint32_t>(target.mesh.vertices.size()));
                target.mesh.vertices.push_back(mesh.vertices[old]);
                target.mesh.indices.push_back(static_cast<uint32_t>(target.mesh.vertices.size() - 1));
            } else {
                target.mesh.indices.push_back(found->second);
            }
        }
    }

    TriangleMesh result;
    const float atlasWidth = static_cast<float>(tileSize * kAtlasColumns);
    const float atlasHeight = static_cast<float>(tileSize * kAtlasRows);
    for (int tile = 0; tile < kAtlasTileCount; ++tile) {
        TriangleMesh& part = tiles[tile].mesh;
        if (part.indices.empty()) {
            continue;
        }
        TileFrame& frame = frames[tile];
        frame.alongMin = 1e9f;
        frame.alongMax = -1e9f;
        for (const MeshVertex& vertex : part.vertices) {
            frame.alongMin = std::min(frame.alongMin, AlongOf(frame, vertex.position));
            frame.alongMax = std::max(frame.alongMax, AlongOf(frame, vertex.position));
        }
        const float alongRange = std::max(frame.alongMax - frame.alongMin, 1e-4f);
        for (MeshVertex& vertex : part.vertices) {
            vertex.uv = glm::vec2(AngleUOf(frame, vertex.position), (AlongOf(frame, vertex.position) - frame.alongMin) / alongRange);
        }

        // A triangle whose corners lie on both sides of the seam is cut at u = 1 and its far part moved back to u = 0.
        TriangleMesh crossing;
        TriangleMesh whole;
        whole.vertices = part.vertices;
        for (size_t t = 0; t + 2 < part.indices.size(); t += 3) {
            const uint32_t id[3] = {part.indices[t], part.indices[t + 1], part.indices[t + 2]};
            const float lowest = std::min({part.vertices[id[0]].uv.x, part.vertices[id[1]].uv.x, part.vertices[id[2]].uv.x});
            const float highest = std::max({part.vertices[id[0]].uv.x, part.vertices[id[1]].uv.x, part.vertices[id[2]].uv.x});
            if (highest - lowest <= 0.5f) {
                whole.indices.insert(whole.indices.end(), {id[0], id[1], id[2]});
                continue;
            }
            for (int c = 0; c < 3; ++c) {
                MeshVertex vertex = part.vertices[id[c]];
                vertex.uv.x += vertex.uv.x < 0.5f ? 1.0f : 0.0f;
                crossing.vertices.push_back(vertex);
                crossing.indices.push_back(static_cast<uint32_t>(crossing.vertices.size() - 1));
            }
        }
        std::vector<float> nearSide(crossing.vertices.size());
        std::vector<float> farSide(crossing.vertices.size());
        for (size_t v = 0; v < crossing.vertices.size(); ++v) {
            nearSide[v] = 1.0f - crossing.vertices[v].uv.x;
            farSide[v] = crossing.vertices[v].uv.x - 1.0f;
        }
        const TriangleMesh nearPart = ClipByField(crossing, nearSide);
        TriangleMesh farPart = ClipByField(crossing, farSide);
        for (MeshVertex& vertex : farPart.vertices) {
            vertex.uv.x -= 1.0f;
        }
        for (const TriangleMesh* extra : std::array<const TriangleMesh*, 2>{&nearPart, &farPart}) {
            const uint32_t offset = static_cast<uint32_t>(whole.vertices.size());
            whole.vertices.insert(whole.vertices.end(), extra->vertices.begin(), extra->vertices.end());
            for (const uint32_t index : extra->indices) {
                whole.indices.push_back(index + offset);
            }
        }

        // Into the atlas: this tile's cell, less the gutter.
        const float usable = static_cast<float>(tileSize - 2 * kGutterTexels);
        const float left = static_cast<float>((tile % kAtlasColumns) * tileSize + kGutterTexels);
        const float top = static_cast<float>((tile / kAtlasColumns) * tileSize + kGutterTexels);
        const uint32_t offset = static_cast<uint32_t>(result.vertices.size());
        for (MeshVertex vertex : whole.vertices) {
            const float u = glm::clamp(vertex.uv.x, 0.0f, 1.0f);
            const float v = glm::clamp(vertex.uv.y, 0.0f, 1.0f);
            vertex.uv = glm::vec2((left + u * usable) / atlasWidth, (top + v * usable) / atlasHeight);
            result.vertices.push_back(vertex);
        }
        for (const uint32_t index : whole.indices) {
            result.indices.push_back(index + offset);
        }
    }
    return result;
}

TexelField BakeTexels(const TriangleMesh& unwrapped, int tileSize) {
    TexelField field;
    field.width = tileSize * kAtlasColumns;
    field.height = tileSize * kAtlasRows;
    field.texels.assign(static_cast<size_t>(field.width) * field.height, Texel{});
    const glm::vec2 atlasSize(static_cast<float>(field.width), static_cast<float>(field.height));

    for (size_t t = 0; t + 2 < unwrapped.indices.size(); t += 3) {
        const MeshVertex* corner[3] = {&unwrapped.vertices[unwrapped.indices[t]], &unwrapped.vertices[unwrapped.indices[t + 1]],
                                       &unwrapped.vertices[unwrapped.indices[t + 2]]};
        const glm::vec2 p0 = corner[0]->uv * atlasSize;
        const glm::vec2 p1 = corner[1]->uv * atlasSize;
        const glm::vec2 p2 = corner[2]->uv * atlasSize;
        const float area = (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
        if (std::abs(area) < 1e-6f) {
            continue;
        }
        const float worldArea = 0.5f * glm::length(glm::cross(corner[1]->position - corner[0]->position, corner[2]->position - corner[0]->position));
        const float metersPerTexel = std::sqrt(worldArea / (0.5f * std::abs(area)));
        const int minX = std::max(0, static_cast<int>(std::floor(std::min({p0.x, p1.x, p2.x}))));
        const int maxX = std::min(field.width - 1, static_cast<int>(std::ceil(std::max({p0.x, p1.x, p2.x}))));
        const int minY = std::max(0, static_cast<int>(std::floor(std::min({p0.y, p1.y, p2.y}))));
        const int maxY = std::min(field.height - 1, static_cast<int>(std::ceil(std::max({p0.y, p1.y, p2.y}))));
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const glm::vec2 p(static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f);
                const float w0 = ((p1.x - p.x) * (p2.y - p.y) - (p2.x - p.x) * (p1.y - p.y)) / area;
                const float w1 = ((p2.x - p.x) * (p0.y - p.y) - (p0.x - p.x) * (p2.y - p.y)) / area;
                const float w2 = 1.0f - w0 - w1;
                if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) {
                    continue;
                }
                Texel& texel = field.At(x, y);
                texel.position = w0 * corner[0]->position + w1 * corner[1]->position + w2 * corner[2]->position;
                const glm::vec3 normal = w0 * corner[0]->normal + w1 * corner[1]->normal + w2 * corner[2]->normal;
                texel.normal = glm::length(normal) > 1e-6f ? glm::normalize(normal) : corner[0]->normal;
                texel.attribute = w0 * corner[0]->attribute + w1 * corner[1]->attribute + w2 * corner[2]->attribute;
                texel.metersPerTexel = metersPerTexel;
                texel.covered = true;
                texel.tile = static_cast<AtlasTile>((y / tileSize) * kAtlasColumns + x / tileSize);
                texel.tileUv = glm::vec2((static_cast<float>(x % tileSize) + 0.5f) / static_cast<float>(tileSize),
                                         (static_cast<float>(y % tileSize) + 0.5f) / static_cast<float>(tileSize));
            }
        }
    }

    // Grow the covered area: each pass gives an empty texel the data of a covered neighbour, within its own tile.
    for (int pass = 0; pass < kGutterTexels; ++pass) {
        std::vector<Texel> grown = field.texels;
        for (int y = 0; y < field.height; ++y) {
            for (int x = 0; x < field.width; ++x) {
                if (field.At(x, y).Used()) {
                    continue;
                }
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int nx = x + dx;
                        const int ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= field.width || ny >= field.height || (dx == 0 && dy == 0)) {
                            continue;
                        }
                        const bool sameTile = nx / tileSize == x / tileSize && ny / tileSize == y / tileSize;
                        const Texel& neighbour = field.At(nx, ny);
                        if (sameTile && neighbour.Used() && !grown[static_cast<size_t>(y) * field.width + x].Used()) {
                            Texel& target = grown[static_cast<size_t>(y) * field.width + x];
                            target = neighbour;
                            target.covered = false;
                            target.gutter = true;
                        }
                    }
                }
            }
        }
        field.texels.swap(grown);
    }
    return field;
}

ByteImage NormalMapFromHeights(const TexelField& texels, const std::vector<float>& heights, int tileSize, JobSystem* jobs) {
    const int width = texels.width;
    const int height = texels.height;
    ByteImage image(width, height, 4);
    ParallelEach(jobs, height, [&](int y) {
        for (int x = 0; x < width; ++x) {
            const Texel& texel = texels.At(x, y);
            glm::vec3 normal(0.0f, 0.0f, 1.0f);
            if (texel.Used()) {
                const auto heightAt = [&](int nx, int ny, float fallback) {
                    const bool inside = nx >= 0 && ny >= 0 && nx < width && ny < height && texels.At(nx, ny).Used() && nx / tileSize == x / tileSize && ny / tileSize == y / tileSize;
                    return inside ? heights[static_cast<size_t>(ny) * width + nx] : fallback;
                };
                const float centre = heights[static_cast<size_t>(y) * width + x];
                const float slopeX = (heightAt(x + 1, y, centre) - heightAt(x - 1, y, centre)) / (2.0f * texel.metersPerTexel);
                // Rows run down the image and the normal map's +Y runs up it.
                const float slopeY = (heightAt(x, y + 1, centre) - heightAt(x, y - 1, centre)) / (2.0f * texel.metersPerTexel);
                normal = glm::normalize(glm::vec3(-slopeX, slopeY, 1.0f));
            }
            uint8_t* pixel = image.Pixel(x, y);
            for (int c = 0; c < 3; ++c) {
                pixel[c] = static_cast<uint8_t>(std::lround(glm::clamp(0.5f + 0.5f * normal[c], 0.0f, 1.0f) * 255.0f));
            }
            pixel[3] = 255;
        }
    });
    return image;
}

}  // namespace character
