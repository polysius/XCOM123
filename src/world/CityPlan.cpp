#include "world/CityPlan.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>

namespace city {
namespace {

constexpr float kSplitMargin = 30.0f;     // closest two parallel streets may be, centre line to centre line
constexpr float kSiteMargin = 45.0f;      // landmark sites keep this much room to a parallel street, or touch it
constexpr float kCrossingMargin = 20.0f;  // a street crossing a gap keeps this far from the gap's ends
constexpr float kSnapDistance = 12.0f;    // new streets line up with a junction this close on the far side
constexpr float kMedianWidth = 12.0f;     // planted strip between the two carriageways of a boulevard
constexpr float kRiverPromenade = 8.0f;   // quay walk along each bank of the river, wall included
constexpr float kCanalPromenade = 4.0f;
constexpr float kSameLine = 0.01f;
constexpr float kNoLimit = std::numeric_limits<float>::max();

const DistrictStyle kStyles[] = {
    // block size     merge  plaza  park   lot width     depth        upper floors
    {45.0f, 78.0f, 0.10f, 0.08f, 0.02f, 5.5f, 10.0f, 9.0f, 13.0f, 3, 5},       // OldQuarter
    {95.0f, 150.0f, 0.25f, 0.05f, 0.04f, 15.0f, 26.0f, 14.0f, 18.0f, 4, 5},    // Grand
    {60.0f, 100.0f, 0.10f, 0.16f, 0.03f, 8.0f, 14.0f, 10.0f, 14.0f, 2, 4},     // Market
    {70.0f, 120.0f, 0.15f, 0.05f, 0.06f, 10.0f, 18.0f, 11.0f, 15.0f, 3, 5},    // Residential
    {110.0f, 220.0f, 0.40f, 0.05f, 0.60f, 14.0f, 22.0f, 12.0f, 16.0f, 1, 3},   // Parkland
};

enum class LineKind : uint8_t {
    Street,
    River,
    Canal,
    Boulevard,
};

// A major line laid across the whole city before the ordinary streets: a street, or a gap with a
// street on each side.
struct MajorLine {
    LineKind kind = LineKind::Street;
    bool alongX = true;     // runs along X, at a constant Z
    float position = 0.0f;  // Z for lines along X, X otherwise; the middle of a gap
    float gapWidth = 0.0f;  // centre line to centre line distance of the two streets of a gap
    float zoneMin = -kNoLimit;  // only regions whose centre along the line lies in this range are split
    float zoneMax = kNoLimit;
};

GapKind GapKindOf(LineKind kind) {
    switch (kind) {
    case LineKind::River: return GapKind::River;
    case LineKind::Canal: return GapKind::Canal;
    default: return GapKind::Median;
    }
}

// The centre lines of the streets a major line consists of.
std::vector<float> StreetPositions(const MajorLine& line) {
    if (line.gapWidth <= 0.0f) {
        return {line.position};
    }
    return {line.position - line.gapWidth * 0.5f, line.position + line.gapWidth * 0.5f};
}

bool IsFarFromAll(float position, const std::vector<float>& taken, float distance) {
    for (float other : taken) {
        if (std::abs(position - other) < distance) {
            return false;
        }
    }
    return true;
}

class Planner {
public:
    Planner(const CitySettings& citySettings, CityPlan& cityPlan)
        : settings(citySettings), plan(cityPlan), random(MixSeed(citySettings.seed, 0x504C414Eu)) {}

    void Run() {
        plan.halfSize = settings.citySize * 0.5f;
        plan.roadHalfWidth = settings.roadWidth * 0.5f;
        plan.streetHalfWidth = plan.roadHalfWidth + settings.sidewalkWidth;
        plan.farDistance = settings.districtOnly ? plan.halfSize : plan.halfSize + 1500.0f;
        plan.districtOnly = settings.districtOnly;

        AddRingRoad();
        landRegions.push_back({glm::vec2(-plan.halfSize), glm::vec2(plan.halfSize)});
        const std::vector<MajorLine> lines = ChooseMajorLines();
        plan.districts = DistrictMap(MixSeed(settings.seed, 0x44495354u), plan.halfSize, plan.historicCore);
        for (const MajorLine& line : lines) {
            ApplyLine(line);
        }
        AddMedianConnectors();
        if (!settings.districtOnly) {
            PlaceLandmarkSites();
        }
        const std::vector<Rect> regions = landRegions;
        for (const Rect& region : regions) {
            Subdivide(region);
        }
        AddMedianBlocks();
    }

private:
    void AddSegment(glm::vec2 a, glm::vec2 b) {
        plan.segments.push_back({a, b});
    }

    void AddRingRoad() {
        const float h = plan.halfSize;
        AddSegment({-h, -h}, {h, -h});
        AddSegment({h, -h}, {h, h});
        AddSegment({h, h}, {-h, h});
        AddSegment({-h, h}, {-h, -h});
    }

    // The river, the canal, boulevards, avenues with their bridges and the streets along the
    // river, in the order they are laid out. Later lines cross the gaps of earlier ones.
    // A district has no water to plan around: two streets across it, one each way, give it a few
    // larger blocks, and its old quarter lies near the middle.
    std::vector<MajorLine> ChooseDistrictLines() {
        const float size = settings.citySize;
        plan.historicCore = {random.Range(-0.12f, 0.12f) * size, random.Range(-0.12f, 0.12f) * size};
        std::vector<MajorLine> lines;
        for (bool alongX : {true, false}) {
            lines.push_back({LineKind::Street, alongX, random.Range(-0.3f, 0.3f) * size});
        }
        return lines;
    }

    std::vector<MajorLine> ChooseMajorLines() {
        if (settings.districtOnly) {
            return ChooseDistrictLines();
        }
        const float h = plan.halfSize;
        const float size = settings.citySize;
        const float streetWidth = plan.streetHalfWidth * 2.0f;
        std::vector<MajorLine> lines;

        // The river runs across the whole city along X, a little south of the centre.
        const float riverZ = random.Range(-0.14f, 0.06f) * size;
        const float riverGap = settings.riverWidth + 2.0f * kRiverPromenade + streetWidth;
        const float southQuay = riverZ - riverGap * 0.5f;
        const float northQuay = riverZ + riverGap * 0.5f;
        lines.push_back({LineKind::River, true, riverZ, riverGap});
        plan.historicCore = {random.Range(-0.12f, 0.12f) * size, riverZ};
        plan.riverStraightEnd = plan.halfSize + plan.streetHalfWidth + kRiverStraightMargin;
        plan.waterways.push_back({{{-plan.riverStraightEnd, riverZ - settings.riverWidth * 0.5f},
                                   {plan.riverStraightEnd, riverZ + settings.riverWidth * 0.5f}},
                                  kQuayWallTop});

        // The canal leaves the river northwards on one side of the city.
        const float canalSide = random.Sign();
        const float canalX = canalSide * random.Range(0.20f, 0.33f) * size;
        const float canalGap = settings.canalWidth + 2.0f * kCanalPromenade + streetWidth;
        std::vector<float> acrossRiver{-h, h};  // X of streets crossing the river, to keep bridges apart
        std::vector<float> blockedX;            // X positions north-south streets must avoid
        if (settings.canal) {
            lines.push_back({LineKind::Canal, false, canalX, canalGap, northQuay, kNoLimit});
            plan.waterways.push_back({{{canalX - settings.canalWidth * 0.5f, riverZ + settings.riverWidth * 0.5f},
                                       {canalX + settings.canalWidth * 0.5f, plan.farDistance}},
                                      kQuayWallTop - 0.05f});  // a different height so the corner walls never fight
            blockedX.push_back(canalX);
        }

        // Boulevards: one along X north of the river, one along Z crossing the river on the side away
        // from the canal, and one along X south of the river.
        const float boulevardGap = kMedianWidth + streetWidth;
        std::vector<float> alongXStreets{-h, h, southQuay, northQuay};
        if (settings.boulevardCount >= 1) {
            const float z = northQuay + (h - northQuay) * random.Range(0.42f, 0.58f);
            lines.push_back({LineKind::Boulevard, true, z, boulevardGap});
            alongXStreets.push_back(z);
        }
        if (settings.boulevardCount >= 2) {
            const float x = -canalSide * random.Range(0.08f, 0.18f) * size;
            lines.push_back({LineKind::Boulevard, false, x, boulevardGap});
            acrossRiver.push_back(x - boulevardGap * 0.5f);
            acrossRiver.push_back(x + boulevardGap * 0.5f);
            blockedX.push_back(x);
        }
        if (settings.boulevardCount >= 3) {
            const float z = southQuay - (southQuay + h) * random.Range(0.42f, 0.58f);
            lines.push_back({LineKind::Boulevard, true, z, boulevardGap});
            alongXStreets.push_back(z);
        }

        // Two avenues run the full height of the city and cross the river on bridges.
        for (int avenue = 0; avenue < 2; ++avenue) {
            for (int attempt = 0; attempt < 60; ++attempt) {
                const float x = random.Range(-0.42f, 0.42f) * size;
                if (IsFarFromAll(x, acrossRiver, 200.0f) && IsFarFromAll(x, blockedX, 80.0f)) {
                    lines.push_back({LineKind::Street, false, x});
                    acrossRiver.push_back(x);
                    blockedX.push_back(x);
                    break;
                }
            }
        }

        // Streets parallel to the river, one block back from each quay.
        const float southStreet = southQuay - random.Range(130.0f, 175.0f);
        const float northStreet = northQuay + random.Range(130.0f, 175.0f);
        for (float z : {southStreet, northStreet}) {
            if (IsFarFromAll(z, alongXStreets, boulevardGap * 0.5f + 45.0f)) {
                lines.push_back({LineKind::Street, true, z});
                alongXStreets.push_back(z);
            }
        }

        // Extra bridges, each with a short street on both banks up to the parallel streets.
        for (int bridge = 0; bridge < settings.extraBridgeCount; ++bridge) {
            for (int attempt = 0; attempt < 60; ++attempt) {
                const float x = random.Range(-0.45f, 0.45f) * size;
                if (IsFarFromAll(x, acrossRiver, 140.0f) && IsFarFromAll(x, blockedX, canalGap * 0.5f + 40.0f)) {
                    lines.push_back({LineKind::Street, false, x, 0.0f, southStreet, northStreet});
                    acrossRiver.push_back(x);
                    break;
                }
            }
        }

        // Bridges over the canal, with streets reaching a few hundred metres to either side.
        if (settings.canal) {
            for (int bridge = 0; bridge < 3; ++bridge) {
                for (int attempt = 0; attempt < 60; ++attempt) {
                    const float z = random.Range(northQuay + 60.0f, h - 60.0f);
                    if (IsFarFromAll(z, alongXStreets, boulevardGap * 0.5f + 70.0f)) {
                        lines.push_back({LineKind::Street, true, z, 0.0f, canalX - 260.0f, canalX + 260.0f});
                        alongXStreets.push_back(z);
                        break;
                    }
                }
            }
        }
        return lines;
    }

    void ApplyLine(const MajorLine& line) {
        // Gaps created by this line must not be crossed by it, so only look at the earlier ones.
        const size_t earlierGaps = plan.gaps.size();
        std::vector<Rect> next;
        for (const Rect& region : landRegions) {
            if (!SplitRegion(region, line, next)) {
                next.push_back(region);
            }
        }
        landRegions.swap(next);
        if (line.kind == LineKind::River || line.kind == LineKind::Canal) {
            return;  // water never crosses anything
        }
        for (size_t i = 0; i < earlierGaps; ++i) {
            CrossGap(plan.gaps[i], line);
        }
    }

    bool SplitRegion(const Rect& region, const MajorLine& line, std::vector<Rect>& out) {
        const bool alongX = line.alongX;
        const float alongCenter = alongX ? region.Center().x : region.Center().y;
        if (alongCenter < line.zoneMin || alongCenter > line.zoneMax) {
            return false;
        }
        const float low = AcrossMin(region, alongX);
        const float high = AcrossMax(region, alongX);
        const float first = line.position - line.gapWidth * 0.5f;
        const float second = line.position + line.gapWidth * 0.5f;
        if (first - low < kSplitMargin || high - second < kSplitMargin) {
            return false;
        }
        const float start = AlongMin(region, alongX);
        const float end = AlongMax(region, alongX);
        out.push_back(FromRanges(alongX, start, end, low, first));
        out.push_back(FromRanges(alongX, start, end, second, high));
        AddSegment(OnLine(alongX, start, first), OnLine(alongX, end, first));
        if (line.gapWidth > 0.0f) {
            AddSegment(OnLine(alongX, start, second), OnLine(alongX, end, second));
            plan.gaps.push_back({GapKindOf(line.kind), alongX, FromRanges(alongX, start, end, first, second), {}});
        }
        return true;
    }

    void CrossGap(Gap& gap, const MajorLine& line) {
        if (gap.alongX == line.alongX) {
            return;  // parallel lines never meet
        }
        const float gapCenterAlongLine = line.alongX ? gap.area.Center().x : gap.area.Center().y;
        if (gapCenterAlongLine < line.zoneMin || gapCenterAlongLine > line.zoneMax) {
            return;
        }
        const float start = AlongMin(gap.area, gap.alongX);
        const float end = AlongMax(gap.area, gap.alongX);
        for (float position : StreetPositions(line)) {
            if (position - start >= kCrossingMargin && end - position >= kCrossingMargin) {
                AddCrossing(gap, position);
            }
        }
    }

    // A street across a gap, from the street on one side to the street on the other.
    void AddCrossing(Gap& gap, float position) {
        const bool crossingAlongX = !gap.alongX;
        AddSegment(OnLine(crossingAlongX, AcrossMin(gap.area, gap.alongX), position),
                   OnLine(crossingAlongX, AcrossMax(gap.area, gap.alongX), position));
        gap.crossings.push_back(position);
    }

    // Short streets across boulevard medians wherever a stretch is long, so people can cross.
    void AddMedianConnectors() {
        for (Gap& gap : plan.gaps) {
            if (gap.kind != GapKind::Median) {
                continue;
            }
            std::vector<float> stops = gap.crossings;
            stops.push_back(AlongMin(gap.area, gap.alongX));
            stops.push_back(AlongMax(gap.area, gap.alongX));
            std::sort(stops.begin(), stops.end());
            for (size_t i = 0; i + 1 < stops.size(); ++i) {
                const float length = stops[i + 1] - stops[i];
                if (length <= 210.0f) {
                    continue;
                }
                const int pieces = static_cast<int>(std::ceil(length / random.Range(150.0f, 190.0f)));
                for (int piece = 1; piece < pieces; ++piece) {
                    AddCrossing(gap, stops[i] + length * static_cast<float>(piece) / static_cast<float>(pieces) +
                                         random.Range(-8.0f, 8.0f));
                }
            }
        }
    }

    // Finds the land region holding `area`, moving any edge that comes close to a street onto it.
    // Returns false when the area does not fit.
    bool FitSite(Rect& area) const {
        for (const Rect& region : landRegions) {
            if (!region.Contains(area.Center())) {
                continue;
            }
            if (area.min.x < region.min.x - kSameLine || area.min.y < region.min.y - kSameLine ||
                area.max.x > region.max.x + kSameLine || area.max.y > region.max.y + kSameLine) {
                return false;
            }
            const glm::vec2 requested = area.Size();
            if (area.min.x - region.min.x < kSiteMargin) area.min.x = region.min.x;
            if (area.min.y - region.min.y < kSiteMargin) area.min.y = region.min.y;
            if (region.max.x - area.max.x < kSiteMargin) area.max.x = region.max.x;
            if (region.max.y - area.max.y < kSiteMargin) area.max.y = region.max.y;
            const glm::vec2 grown = area.Size();
            return grown.x <= requested.x * 1.45f && grown.y <= requested.y * 1.45f;
        }
        return false;
    }

    bool TryAddSite(LandmarkKind kind, Rect area) {
        if (!FitSite(area)) {
            return false;
        }
        for (const LandmarkSite& other : plan.sites) {
            if (other.area.Expanded(kSiteMargin).Overlaps(area)) {
                return false;
            }
        }
        plan.sites.push_back({kind, area, false});
        return true;
    }

    // The cathedral stands on the river bank in the historic core, its long side along the quay.
    void PlaceCathedral() {
        const float h = plan.halfSize;
        const Gap* river = nullptr;
        for (const Gap& gap : plan.gaps) {
            if (gap.kind == GapKind::River) {
                river = &gap;
            }
        }
        if (river == nullptr) {
            return;
        }
        for (int attempt = 0; attempt < 60; ++attempt) {
            const float length = random.Range(150.0f, 175.0f);
            const float depth = random.Range(88.0f, 100.0f);
            const float centerX = glm::clamp(plan.historicCore.x + random.Range(-160.0f, 160.0f), -h + 120.0f, h - 120.0f);
            const bool southBank = random.Chance(0.5f);
            const float quay = southBank ? river->area.min.y : river->area.max.y;
            const float far = southBank ? quay - depth : quay + depth;
            const Rect area{{centerX - length * 0.5f, std::min(quay, far)}, {centerX + length * 0.5f, std::max(quay, far)}};
            if (TryAddSite(LandmarkKind::Cathedral, area)) {
                return;
            }
        }
    }

    void PlaceSite(LandmarkKind kind, glm::vec2 size, District first, District second) {
        const float reach = plan.halfSize * 0.85f;
        for (int attempt = 0; attempt < 120; ++attempt) {
            const glm::vec2 center(random.Range(-reach, reach), random.Range(-reach, reach));
            const District district = plan.districts.At(center);
            if (district != first && district != second) {
                continue;
            }
            const glm::vec2 halfSize = (random.Chance(0.5f) ? size : glm::vec2(size.y, size.x)) * 0.5f;
            if (TryAddSite(kind, {center - halfSize, center + halfSize})) {
                return;
            }
        }
    }

    void PlaceLandmarkSites() {
        PlaceCathedral();
        PlaceSite(LandmarkKind::Palace, {random.Range(180.0f, 200.0f), random.Range(140.0f, 158.0f)},
                  District::Grand, District::Residential);
        PlaceSite(LandmarkKind::MarketHall, {random.Range(115.0f, 130.0f), random.Range(98.0f, 110.0f)},
                  District::Market, District::OldQuarter);
        PlaceSite(LandmarkKind::ClockTower, {random.Range(85.0f, 95.0f), random.Range(72.0f, 82.0f)},
                  District::OldQuarter, District::Grand);
        for (int church = 0; church < 3; ++church) {
            PlaceSite(LandmarkKind::ParishChurch, {random.Range(92.0f, 102.0f), random.Range(70.0f, 78.0f)},
                      District::Residential, church == 0 ? District::OldQuarter : District::Market);
        }
    }

    std::vector<int> SitesInside(const Rect& region) const {
        std::vector<int> inside;
        for (size_t i = 0; i < plan.sites.size(); ++i) {
            if (!plan.sites[i].placed && region.Contains(plan.sites[i].area.Center())) {
                inside.push_back(static_cast<int>(i));
            }
        }
        return inside;
    }

    bool CutsSite(const std::vector<int>& sites, bool splitX, float position) const {
        for (int index : sites) {
            const Rect& area = plan.sites[index].area;
            const float low = splitX ? area.min.x : area.min.y;
            const float high = splitX ? area.max.x : area.max.y;
            if (position > low + kSameLine && position < high - kSameLine) {
                return true;
            }
        }
        return false;
    }

    // Splits `region` with a street at `position`: a street along Z when `splitX`, along X otherwise.
    void SplitAt(const Rect& region, bool splitX, float position) {
        Rect first = region;
        Rect second = region;
        if (splitX) {
            first.max.x = position;
            second.min.x = position;
            AddSegment({position, region.min.y}, {position, region.max.y});
        } else {
            first.max.y = position;
            second.min.y = position;
            AddSegment({region.min.x, position}, {region.max.x, position});
        }
        Subdivide(first);
        Subdivide(second);
    }

    // Cuts streets along the edges of the landmark sites in `region` until each site has a block.
    void SubdivideAroundSites(const Rect& region, const std::vector<int>& sites) {
        for (int index : sites) {
            const Rect& area = plan.sites[index].area;
            const std::pair<bool, float> edges[] = {
                {true, area.min.x}, {true, area.max.x}, {false, area.min.y}, {false, area.max.y}};
            for (const auto& [splitX, position] : edges) {
                const float low = splitX ? region.min.x : region.min.y;
                const float high = splitX ? region.max.x : region.max.y;
                if (position - low >= kSplitMargin && high - position >= kSplitMargin && !CutsSite(sites, splitX, position)) {
                    SplitAt(region, splitX, position);
                    return;
                }
            }
        }
        // No edge is left to cut along, so this region is the site's block. Should two sites share
        // it, which their spacing rules out, the first one wins.
        LandmarkSite& site = plan.sites[sites.front()];
        site.area = region;
        site.placed = true;
        plan.blocks.push_back({region, BlockUse::Landmark, sites.front()});
        for (size_t i = 1; i < sites.size(); ++i) {
            plan.sites[sites[i]].area = {};
        }
    }

    // Moves a split position onto a junction just across the region's edge, so streets continue
    // straight through intersections instead of stopping a few metres apart.
    float SnapToJunction(const Rect& region, bool splitX, float position, float low, float high) const {
        float best = position;
        float bestDistance = kSnapDistance;
        for (const Segment& segment : plan.segments) {
            for (glm::vec2 end : {segment.a, segment.b}) {
                const float across = splitX ? end.y : end.x;
                const float along = splitX ? end.x : end.y;
                const bool onEdge = std::abs(across - (splitX ? region.min.y : region.min.x)) < kSameLine ||
                                    std::abs(across - (splitX ? region.max.y : region.max.x)) < kSameLine;
                const float distance = std::abs(along - position);
                if (onEdge && along > low && along < high && distance < bestDistance) {
                    best = along;
                    bestDistance = distance;
                }
            }
        }
        return best;
    }

    void Subdivide(const Rect& region) {
        const std::vector<int> sites = SitesInside(region);
        if (!sites.empty()) {
            SubdivideAroundSites(region, sites);
            return;
        }
        const glm::vec2 size = region.Size();
        const DistrictStyle& style = StyleOf(plan.districts.At(region.Center()));
        const float longest = std::max(size.x, size.y);
        const bool keepWhole = longest <= style.maxBlock ||
                               (longest <= style.maxBlock * 1.7f && random.Chance(style.mergeChance));
        if (keepWhole) {
            plan.blocks.push_back({region, BlockUse::Buildings, -1});
            return;
        }
        // Cut across the longer side so blocks tend towards squares.
        const bool splitX = size.x >= size.y;
        const float low = splitX ? region.min.x : region.min.y;
        const float extent = longest;
        const float minPart = std::min(style.minBlock, extent * 0.5f - 1.0f);
        float position = low + extent * random.Range(0.35f, 0.65f);
        position = glm::clamp(position, low + minPart, low + extent - minPart);
        position = SnapToJunction(region, splitX, position, low + minPart * 0.75f, low + extent - minPart * 0.75f);
        SplitAt(region, splitX, position);
    }

    // The planted strips of the boulevards, cut into blocks by the streets crossing them.
    void AddMedianBlocks() {
        // Even short pieces become blocks, because their paving carries the pavements of the
        // streets crossing the boulevard.
        const float minLength = plan.roadHalfWidth * 2.0f + 0.5f;
        for (const Gap& gap : plan.gaps) {
            if (gap.kind != GapKind::Median) {
                continue;
            }
            std::vector<float> stops = gap.crossings;
            stops.push_back(AlongMin(gap.area, gap.alongX));
            stops.push_back(AlongMax(gap.area, gap.alongX));
            std::sort(stops.begin(), stops.end());
            for (size_t i = 0; i + 1 < stops.size(); ++i) {
                if (stops[i + 1] - stops[i] < minLength) {
                    continue;
                }
                const Rect area = FromRanges(gap.alongX, stops[i], stops[i + 1], AcrossMin(gap.area, gap.alongX),
                                             AcrossMax(gap.area, gap.alongX));
                plan.blocks.push_back({area, BlockUse::Median, -1});
            }
        }
    }

    const CitySettings& settings;
    CityPlan& plan;
    Random random;
    std::vector<Rect> landRegions;  // land between the major lines, before the ordinary streets
};

}  // namespace

const DistrictStyle& StyleOf(District district) {
    return kStyles[static_cast<size_t>(district)];
}

DistrictMap::DistrictMap(uint32_t seed, float halfSize, glm::vec2 historicCore) {
    Random random(seed);
    // Seed points on a jittered 4 x 4 grid, so districts are of similar size but irregular shape.
    constexpr int kGrid = 4;
    const float cell = halfSize * 2.0f / kGrid;
    for (int z = 0; z < kGrid; ++z) {
        for (int x = 0; x < kGrid; ++x) {
            const glm::vec2 center(-halfSize + (static_cast<float>(x) + 0.5f) * cell,
                                   -halfSize + (static_cast<float>(z) + 0.5f) * cell);
            const glm::vec2 jitter(random.Range(-0.3f, 0.3f), random.Range(-0.3f, 0.3f));
            sites.push_back({center + jitter * cell, District::Residential});
        }
    }
    // The three sites nearest the historic core form the old quarter, the next four the grand
    // district around it. Of the rest, two become market districts and one parkland.
    std::vector<size_t> order(sites.size());
    for (size_t i = 0; i < order.size(); ++i) {
        order[i] = i;
    }
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return glm::distance(sites[a].position, historicCore) < glm::distance(sites[b].position, historicCore);
    });
    for (size_t rank = 0; rank < order.size(); ++rank) {
        if (rank < 3) {
            sites[order[rank]].district = District::OldQuarter;
        } else if (rank < 7) {
            sites[order[rank]].district = District::Grand;
        }
    }
    std::vector<size_t> outer(order.begin() + 7, order.end());
    for (int pick = 0; pick < 3 && !outer.empty(); ++pick) {
        const size_t choice = static_cast<size_t>(random.Int(0, static_cast<int>(outer.size()) - 1));
        sites[outer[choice]].district = pick < 2 ? District::Market : District::Parkland;
        outer.erase(outer.begin() + static_cast<std::ptrdiff_t>(choice));
    }
}

District DistrictMap::At(glm::vec2 point) const {
    District nearest = District::Residential;
    float nearestDistance = std::numeric_limits<float>::max();
    for (const Site& site : sites) {
        const glm::vec2 offset = site.position - point;
        const float distance = glm::dot(offset, offset);
        if (distance < nearestDistance) {
            nearestDistance = distance;
            nearest = site.district;
        }
    }
    return nearest;
}

namespace {

constexpr float kDegree = 0.017453292f;

// Plans the two reaches of the river beyond its straight part. They use a random stream of their
// own, so that they cannot change anything the planner decides.
void PlanRiverReaches(const CitySettings& settings, CityPlan& plan) {
    constexpr float kStep = 60.0f;                 // distance between the vertices of a reach
    constexpr float kMaxStepTurn = 8.0f * kDegree;   // how much the heading changes at a vertex
    constexpr float kMaxDeviation = 40.0f * kDegree;  // how far the heading may leave straight outwards
    constexpr float kSteer = 3.0f * kDegree;         // the push back towards the river's line
    constexpr float kWanderLimit = 300.0f;           // beyond this the reach is steered back
    constexpr float kStraightenDistance = 500.0f;    // the reach straightens out over this stretch before the edge
    constexpr float kStraightenRate = 0.3f;          // the share of the heading's deviation undone at each vertex
    const float riverZ = plan.waterways[0].water.Center().y;
    for (int side = 0; side < 2; ++side) {
        const float outward = side == 0 ? 1.0f : -1.0f;  // east, then west
        Random random(MixSeed(settings.seed, 0x52495652u + static_cast<uint32_t>(side)));
        RiverReach reach{{}, settings.riverWidth * 0.5f, kQuayWallTop};
        glm::vec2 point{outward * plan.riverStraightEnd, riverZ};
        reach.points.push_back(point);
        // The first stretch continues the straight river, so the join is seamless.
        point.x += outward * kStep;
        reach.points.push_back(point);
        float deviation = 0.0f;
        while (std::abs(point.x) < plan.farDistance) {
            float turn = random.Range(-kMaxStepTurn, kMaxStepTurn);
            if (std::abs(point.y - riverZ) > kWanderLimit) {
                turn += point.y > riverZ ? -kSteer : kSteer;
            }
            if (std::abs(point.x) > plan.farDistance - kStraightenDistance) {
                turn = -deviation * kStraightenRate;  // over the last stretch the reach straightens out
            }
            deviation = std::clamp(deviation + turn, -kMaxDeviation, kMaxDeviation);
            glm::vec2 step = glm::vec2(outward * std::cos(deviation), std::sin(deviation)) * kStep;
            if (std::abs(point.x + step.x) >= plan.farDistance) {
                // The last step ends exactly at the edge of the land, so that no water runs past it.
                step *= (plan.farDistance - std::abs(point.x)) / std::abs(step.x);
                point += step;
                point.x = outward * plan.farDistance;
            } else {
                point += step;
            }
            reach.points.push_back(point);
        }
        plan.reaches.push_back(std::move(reach));
    }
}

}  // namespace

CityPlan PlanCity(const CitySettings& settings) {
    CityPlan plan;
    Planner(settings, plan).Run();
    if (!settings.districtOnly) {
        PlanRiverReaches(settings, plan);
    }
    return plan;
}

Rect WaterZone(const Waterway& waterway) {
    return waterway.water.Expanded(kQuayWallThickness);
}

namespace {
// Positions are compared at centimetre precision, so the same intersection reached from two
// segments always becomes one node.
std::pair<int64_t, int64_t> NodeKey(glm::vec2 point) {
    return {std::llround(point.x * 100.0f), std::llround(point.y * 100.0f)};
}
int64_t LineKey(float coordinate) {
    return std::llround(coordinate * 100.0f);
}
}  // namespace

StreetGraph BuildStreetGraph(const std::vector<Segment>& segments) {
    // Every segment end, grouped by the line it could lie on: Z for lines along X, X for the others.
    std::map<int64_t, std::vector<float>> endsByZ;
    std::map<int64_t, std::vector<float>> endsByX;
    for (const Segment& segment : segments) {
        for (glm::vec2 end : {segment.a, segment.b}) {
            endsByZ[LineKey(end.y)].push_back(end.x);
            endsByX[LineKey(end.x)].push_back(end.y);
        }
    }

    StreetGraph graph;
    std::map<std::pair<int64_t, int64_t>, uint32_t> nodeIds;
    auto nodeAt = [&](glm::vec2 point) {
        const auto [it, inserted] = nodeIds.emplace(NodeKey(point), static_cast<uint32_t>(graph.nodes.size()));
        if (inserted) {
            graph.nodes.push_back(point);
        }
        return it->second;
    };

    std::set<std::pair<uint32_t, uint32_t>> existing;
    for (const Segment& segment : segments) {
        const bool alongX = std::abs(segment.b.x - segment.a.x) >= std::abs(segment.b.y - segment.a.y);
        const float fixed = alongX ? segment.a.y : segment.a.x;
        const float start = alongX ? std::min(segment.a.x, segment.b.x) : std::min(segment.a.y, segment.b.y);
        const float end = alongX ? std::max(segment.a.x, segment.b.x) : std::max(segment.a.y, segment.b.y);

        // Split the segment wherever another one ends on it.
        std::vector<float> stops{start, end};
        const auto& ends = alongX ? endsByZ : endsByX;
        const auto found = ends.find(LineKey(fixed));
        if (found != ends.end()) {
            for (float along : found->second) {
                if (along > start + kSameLine && along < end - kSameLine) {
                    stops.push_back(along);
                }
            }
        }
        std::sort(stops.begin(), stops.end());
        stops.erase(std::unique(stops.begin(), stops.end(), [](float a, float b) { return b - a < kSameLine; }),
                    stops.end());

        for (size_t i = 0; i + 1 < stops.size(); ++i) {
            const uint32_t from = nodeAt(OnLine(alongX, stops[i], fixed));
            const uint32_t to = nodeAt(OnLine(alongX, stops[i + 1], fixed));
            if (from == to || !existing.insert({std::min(from, to), std::max(from, to)}).second) {
                continue;
            }
            graph.edges.push_back({from, to, glm::distance(graph.nodes[from], graph.nodes[to])});
        }
    }

    graph.edgesAtNode.resize(graph.nodes.size());
    for (uint32_t i = 0; i < graph.edges.size(); ++i) {
        graph.edgesAtNode[graph.edges[i].from].push_back(i);
        graph.edgesAtNode[graph.edges[i].to].push_back(i);
    }
    return graph;
}

}  // namespace city
