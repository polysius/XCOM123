#version 460
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

// Lighting for everything opaque: physically based shading from one sun with soft shadows,
// ambient light and reflections from the sky, procedural surface detail, and distance fog.
//
// The city is built from plain boxes, so all surface detail is procedural: each material below
// draws its pattern (windows, stone courses, roof tiles, cobbles, ripples) from the position on
// the box and a per-box seed. Every pattern fades to its average colour once it becomes smaller
// than a pixel, so distant walls and roofs do not shimmer.

layout(set = 0, binding = 1) uniform sampler2DShadow shadowMap;

layout(location = 0) in vec3 inWorldPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inAlbedo;
layout(location = 3) in vec3 inSurface;        // x: roughness, y: material id, z: per-object seed
layout(location = 4) in vec3 inLocalPosition;  // boxes: metres from the minimum corner; people: body position
layout(location = 5) in vec3 inBoxSize;        // boxes: full size in metres
layout(location = 6) in vec3 inLocalNormal;    // boxes: the face normal in the box's own axes

layout(location = 0) out vec4 outColor;

// Material ids, matching SurfaceMaterial in SceneTypes.h.
const int MATERIAL_GROUND = 0;
const int MATERIAL_FACADE = 1;
const int MATERIAL_PLAIN = 2;
const int MATERIAL_CHARACTER = 3;
const int MATERIAL_WATER = 4;
const int MATERIAL_STONE = 5;
const int MATERIAL_ROOF_TILE = 6;
const int MATERIAL_FOLIAGE = 7;
const int MATERIAL_GRASS = 8;
const int MATERIAL_GLASS = 9;
const int MATERIAL_METAL = 10;
const int MATERIAL_LAMP = 11;

// The whole-number part of a box's seed holds pattern flags and its fraction a random number.
// The flags match the ones in src/world/CityBuild.h.
const uint FACADE_NO_SHOPS = 1u;
const uint FACADE_UPPER_ONLY = 2u;
const uint FACADE_NARROW_BAYS = 4u;
const uint FACADE_WIDE_BAYS = 8u;
const uint FACADE_SHUTTERS = 16u;
const uint FACADE_RUSTICATED = 32u;
const uint FACADE_ARCHED = 64u;
const uint FACADE_STUCCO = 128u;
const uint ROOF_DORMERS = 1u;
const uint GLASS_STAINED = 1u;
const uint METAL_RAILING = 1u;
const uint METAL_VERDIGRIS = 2u;
const uint GROUND_SLABS = 1u;
const uint GROUND_COBBLES = 2u;
const uint GROUND_GRAVEL = 3u;

// Facade layout. The floor heights must match CityBuild.h so window rows meet the roof line.
const float GROUND_FLOOR_HEIGHT = 4.5;
const float UPPER_FLOOR_HEIGHT = 3.3;
const float WINDOW_BAY_WIDTH = 3.2;
const float NARROW_BAY_WIDTH = 2.4;
const float WIDE_BAY_WIDTH = 4.2;

// Sizes of the repeating patterns, in metres.
const float PAVING_SLAB_SIZE = 0.9;
const vec2 COBBLE_SIZE = vec2(0.24, 0.16);
const vec2 ASHLAR_SIZE = vec2(1.2, 0.6);
const vec2 ROOF_TILE_SIZE = vec2(0.3, 0.28);

struct Surface {
    vec3 albedo;
    float roughness;
    vec3 emissive;
    float occlusion;  // 1 = fully open to the sky, 0 = fully blocked
    float metallic;
    float reflectance;  // head-on reflectance of the non-metallic part
    float wrap;         // how far diffuse light wraps past the terminator (leaves let light through)
};

// Coordinates for the procedural patterns, and how quickly each one changes from pixel to pixel.
// They are computed once, before any branching, because screen-space derivatives (fwidth) are
// only reliable when neighbouring pixels all run the same code.
struct Pattern {
    float along;        // metres along the wall
    float bay;          // window bay coordinate; whole numbers are the edges between bays
    float upperFloor;   // floor coordinate above the ground floor
    float groundFloor;  // 0..1 up the ground floor
    vec2 tile;          // paving slab coordinate
    vec2 face;          // metres across the face: along and up a wall, or across the ground on a top face
    vec3 wallBlur;      // anti-aliasing blur for bay, upperFloor and groundFloor
    vec2 tileBlur;
    vec2 faceRate;      // metres of `face` covered by one pixel
    float wallSharpness;  // 1 when the window pattern is clearly visible, 0 when it is sub-pixel
    float tileSharpness;
    float metresPerPixel;
    float detailFade;     // 1 up close, 0 when fine noise would be smaller than a pixel
};

float Hash(vec2 p) {
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}

float ValueNoise(vec2 p) {
    vec2 cell = floor(p);
    vec2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float a = Hash(cell);
    float b = Hash(cell + vec2(1.0, 0.0));
    float c = Hash(cell + vec2(0.0, 1.0));
    float d = Hash(cell + vec2(1.0, 1.0));
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

// 1 inside [low, high] of a coordinate and 0 outside, with edges softened by `blur`.
float Band(float x, float low, float high, float blur) {
    return smoothstep(low - blur, low + blur, x) - smoothstep(high - blur, high + blur, x);
}

// Anti-aliasing helpers for a pattern whose cells are one unit wide, given how many cells one
// pixel covers. The blur never reaches zero because smoothstep needs two distinct edges.
vec2 PatternBlur(vec2 cellsPerPixel) {
    return clamp(cellsPerPixel * 1.5, vec2(1e-4), vec2(0.5));
}

float PatternSharpness(vec2 cellsPerPixel) {
    return 1.0 - smoothstep(0.15, 0.45, max(cellsPerPixel.x, cellsPerPixel.y));
}

// 1 on the joints between the cells of a unit grid, 0 inside the cells.
float GridJoints(vec2 cell, vec2 jointWidth, vec2 blur) {
    vec2 f = fract(cell);
    return 1.0 - Band(f.x, jointWidth.x, 1.0 - jointWidth.x, blur.x) * Band(f.y, jointWidth.y, 1.0 - jointWidth.y, blur.y);
}

// Shifts every other row of a grid by half a cell, as bricks, stones and tiles are laid.
vec2 Staggered(vec2 cell) {
    return vec2(cell.x + 0.5 * mod(floor(cell.y), 2.0), cell.y);
}

// 1 after sunset, falling to 0.15 in full daylight: how strongly lit windows stand out.
float DuskFactor() {
    return 1.0 - 0.85 * smoothstep(0.05, 0.45, frame.sunDirection.y);
}

// 0 in daylight, 1 once the sun is low: street lamps and clock faces switch on.
float NightFactor() {
    return 1.0 - smoothstep(0.05, 0.3, frame.sunDirection.y);
}

float BayWidth(uint flags) {
    if ((flags & FACADE_NARROW_BAYS) != 0u) {
        return NARROW_BAY_WIDTH;
    }
    return (flags & FACADE_WIDE_BAYS) != 0u ? WIDE_BAY_WIDTH : WINDOW_BAY_WIDTH;
}

Pattern ComputePattern(vec3 normal, uint flags) {
    Pattern pattern;
    // Which wall of the box this is depends on its own axes, not on how the box is turned.
    bool facesX = abs(inLocalNormal.x) > 0.5;
    bool facesUp = abs(normal.y) > 0.5;
    pattern.along = facesX ? inLocalPosition.z : inLocalPosition.x;
    float wallWidth = max(facesX ? inBoxSize.z : inBoxSize.x, 0.01);
    // Fit a whole number of bays across the wall so windows are never cut off at the corners.
    float bayCount = max(1.0, floor(wallWidth / BayWidth(flags)));
    pattern.bay = pattern.along / wallWidth * bayCount;
    // Attics and dormer storeys have no ground floor: their windows start at the bottom.
    float groundFloorHeight = (flags & FACADE_UPPER_ONLY) != 0u ? 0.0 : GROUND_FLOOR_HEIGHT;
    pattern.upperFloor = (inLocalPosition.y - groundFloorHeight) / UPPER_FLOOR_HEIGHT;
    pattern.groundFloor = inLocalPosition.y / GROUND_FLOOR_HEIGHT;
    pattern.tile = inWorldPosition.xz / PAVING_SLAB_SIZE;
    pattern.face = facesUp ? inWorldPosition.xz : vec2(pattern.along, inLocalPosition.y);

    vec3 wallRate = vec3(fwidth(pattern.bay), fwidth(pattern.upperFloor), fwidth(pattern.groundFloor));
    vec2 tileRate = fwidth(pattern.tile);
    pattern.faceRate = fwidth(pattern.face);
    pattern.wallBlur = clamp(wallRate * 1.5, vec3(1e-4), vec3(0.5));
    pattern.tileBlur = PatternBlur(tileRate);
    pattern.wallSharpness = PatternSharpness(wallRate.xy);
    pattern.tileSharpness = PatternSharpness(tileRate);
    pattern.metresPerPixel = length(fwidth(inWorldPosition));
    pattern.detailFade = 1.0 - smoothstep(0.1, 0.4, pattern.metresPerPixel);
    return pattern;
}

// ---------------------------------------------------------------------------------------------
// Walls
// ---------------------------------------------------------------------------------------------

// How far a point of a window bay is inside a round-headed opening, in metres: negative inside,
// positive outside. The opening is `halfWidth` either side of the middle of the bay, rises
// straight from `bottom` and ends in a semicircle, all measured in metres up the storey.
float ArchDistance(float acrossBay, float up, float halfWidth, float bottom, float top) {
    float springLine = top - halfWidth;
    vec2 fromCentre = vec2(abs(acrossBay), max(up - springLine, 0.0));
    return max(length(fromCentre) - halfWidth, bottom - up);
}

// The window mask of one opening, and in `ring` the stone surround that frames it.
float ArchedOpening(Pattern pattern, float bayFraction, float bayMetres, float up, vec2 acrossRange, vec2 upRange,
                    out float ring) {
    float halfWidth = (acrossRange.y - acrossRange.x) * 0.5 * bayMetres;
    float acrossBay = (bayFraction - 0.5 * (acrossRange.x + acrossRange.y)) * bayMetres;
    float distance = ArchDistance(acrossBay, up, halfWidth, upRange.x, upRange.y);
    float blur = max(pattern.metresPerPixel * 1.5, 1e-3);
    float opening = 1.0 - smoothstep(-blur, blur, distance);
    float outer = 1.0 - smoothstep(0.14 - blur, 0.14 + blur, distance);
    // A pale keystone wedge crowns the arch.
    float keystone = (1.0 - smoothstep(0.11 - blur, 0.11 + blur, abs(acrossBay)))
                   * smoothstep(upRange.y - 0.02 - blur, upRange.y - 0.02 + blur, up)
                   * (1.0 - smoothstep(upRange.y + 0.26 - blur, upRange.y + 0.26 + blur, up));
    ring = max(outer - opening, keystone);
    return opening;
}

// Smooth hand-troweled plaster: soft trowel swirls, repainted patches, hairline cracks, and a
// damp stain rising from the foot of the wall.
Surface StuccoFinish(Surface surface, Pattern pattern, float height, float seedRandom) {
    float swirls = ValueNoise(pattern.face * 2.2 + seedRandom * 29.0);
    float patches = ValueNoise(pattern.face * 0.45 + seedRandom * 61.0);
    surface.albedo *= mix(0.94, 1.04, swirls) * mix(0.93, 1.05, patches);
    float crackLine = 1.0 - smoothstep(0.0, 0.02, abs(ValueNoise(pattern.face * vec2(1.3, 0.9) + seedRandom * 13.0) - 0.5));
    surface.albedo *= 1.0 - 0.14 * crackLine * pattern.detailFade;
    surface.albedo *= mix(0.78, 1.0, smoothstep(0.3, 1.8, height));
    surface.roughness = 0.95;
    return surface;
}

Surface FacadeSurface(Surface surface, vec3 normal, Pattern pattern, uint flags, float seedRandom) {
    if (abs(normal.y) > 0.5) {
        surface.albedo *= 0.55;  // flat top of a building, mostly hidden under its roof
        return surface;
    }
    bool arched = (flags & FACADE_ARCHED) != 0u;
    float archRing = 0.0;
    // The width of one window bay in metres, found the same way as in ComputePattern.
    bool facesX = abs(inLocalNormal.x) > 0.5;
    float wallWidth = max(facesX ? inBoxSize.z : inBoxSize.x, 0.01);
    float bayMetres = wallWidth / max(1.0, floor(wallWidth / BayWidth(flags)));

    float height = inLocalPosition.y;
    float bayFraction = fract(pattern.bay);
    bool hasGroundFloor = (flags & FACADE_UPPER_ONLY) == 0u;
    bool hasShops = (flags & (FACADE_NO_SHOPS | FACADE_UPPER_ONLY)) == 0u;
    float windowMask;
    float floorIndex;
    float averageWindowCoverage;
    if (hasGroundFloor && height < GROUND_FLOOR_HEIGHT) {
        floorIndex = -1.0;
        if (hasShops) {
            // Tall shop fronts with a painted sign board above them.
            windowMask = Band(bayFraction, 0.12, 0.88, pattern.wallBlur.x) * Band(pattern.groundFloor, 0.05, 0.78, pattern.wallBlur.z);
            averageWindowCoverage = 0.76 * 0.73;
            if (arched) {
                windowMask = ArchedOpening(pattern, bayFraction, bayMetres, height, vec2(0.12, 0.88),
                                           vec2(0.05, 0.78) * GROUND_FLOOR_HEIGHT, archRing);
                averageWindowCoverage *= 0.9;
            }
            float signBoard = Band(bayFraction, 0.06, 0.94, pattern.wallBlur.x) * Band(pattern.groundFloor, 0.81, 0.93, pattern.wallBlur.z);
            vec3 paint = mix(vec3(0.08, 0.16, 0.12), vec3(0.32, 0.08, 0.07), step(0.5, Hash(vec2(floor(pattern.bay), seedRandom * 53.0))));
            surface.albedo = mix(surface.albedo, paint, signBoard * pattern.wallSharpness);
        } else {
            // A residential ground floor: smaller windows above a stone plinth.
            windowMask = Band(bayFraction, 0.32, 0.68, pattern.wallBlur.x) * Band(pattern.groundFloor, 0.3, 0.8, pattern.wallBlur.z);
            averageWindowCoverage = 0.36 * 0.5;
            if (arched) {
                windowMask = ArchedOpening(pattern, bayFraction, bayMetres, height, vec2(0.32, 0.68),
                                           vec2(0.3, 0.8) * GROUND_FLOOR_HEIGHT, archRing);
                averageWindowCoverage *= 0.9;
            }
        }
        if ((flags & FACADE_RUSTICATED) != 0u) {
            // Deep horizontal grooves cut into the stone of the ground floor.
            float groovesPerPixel = pattern.faceRate.y / 0.55;
            float groove = Band(fract(height / 0.55), 0.0, 0.08, PatternBlur(vec2(groovesPerPixel)).x);
            surface.albedo *= 1.0 - 0.3 * mix(0.08, groove, PatternSharpness(vec2(groovesPerPixel)));
        }
    } else {
        float floorFraction = fract(pattern.upperFloor);
        windowMask = Band(bayFraction, 0.30, 0.70, pattern.wallBlur.x) * Band(floorFraction, 0.18, 0.82, pattern.wallBlur.y);
        averageWindowCoverage = 0.40 * 0.64;
        floorIndex = floor(pattern.upperFloor);
        // A pale stone ledge along every floor line, and a stone surround around each window.
        float ledge = mix(0.11, Band(floorFraction, -0.05, 0.06, pattern.wallBlur.y), pattern.wallSharpness);
        float surround = Band(bayFraction, 0.25, 0.75, pattern.wallBlur.x) * Band(floorFraction, 0.13, 0.87, pattern.wallBlur.y);
        surface.albedo *= 1.0 + 0.15 * ledge + (arched ? 0.0 : 0.1) * surround * pattern.wallSharpness;
        if (arched) {
            windowMask = ArchedOpening(pattern, bayFraction, bayMetres, floorFraction * UPPER_FLOOR_HEIGHT, vec2(0.30, 0.70),
                                       vec2(0.18, 0.82) * UPPER_FLOOR_HEIGHT, archRing);
            averageWindowCoverage *= 0.9;
        }
        if ((flags & FACADE_SHUTTERS) != 0u) {
            // Painted wooden shutters folded back either side of the window.
            float shutters = (Band(bayFraction, 0.15, 0.29, pattern.wallBlur.x) + Band(bayFraction, 0.71, 0.85, pattern.wallBlur.x))
                           * Band(floorFraction, 0.18, 0.82, pattern.wallBlur.y);
            vec3 paint = seedRandom < 0.4 ? vec3(0.12, 0.22, 0.14) : (seedRandom < 0.75 ? vec3(0.22, 0.28, 0.33) : vec3(0.32, 0.2, 0.12));
            surface.albedo = mix(surface.albedo, paint, shutters * pattern.wallSharpness);
        }
    }
    // Far away the pattern is smaller than a pixel: use its average so the wall keeps the same
    // overall brightness instead of shimmering.
    windowMask = mix(averageWindowCoverage, windowMask, pattern.wallSharpness);

    // Large soft stains so walls are not one flat colour, and a darker plinth at the foot.
    float weathering = ValueNoise(vec2(pattern.along * 0.35, height * 0.12) + seedRandom * 57.0);
    surface.albedo *= mix(0.82, 1.06, weathering) * mix(0.8, 1.0, smoothstep(0.5, 0.7, height));
    surface.albedo *= 1.0 + 0.2 * archRing * pattern.wallSharpness;
    if ((flags & FACADE_STUCCO) != 0u) {
        surface = StuccoFinish(surface, pattern, height, seedRandom);
    }
    // Little sky reaches the bottom of a narrow street, and walls are dirtier there.
    surface.occlusion *= mix(0.55, 1.0, smoothstep(0.0, 9.0, inWorldPosition.y));

    // Some windows are lit from inside; they stand out more as the sun goes down.
    vec2 windowId = vec2(floor(pattern.bay) + dot(normal, vec3(17.0, 0.0, 31.0)), floorIndex);
    float isLit = mix(0.2, step(0.8, Hash(windowId + seedRandom * 91.0)), pattern.wallSharpness);
    vec3 interiorLight = vec3(1.0, 0.68, 0.38) * 1.6 * isLit * DuskFactor();

    vec3 glass = vec3(0.02, 0.025, 0.03);
    surface.albedo = mix(surface.albedo, glass, windowMask);
    surface.roughness = mix(surface.roughness, 0.05, windowMask);
    surface.emissive += interiorLight * windowMask;
    return surface;
}

// Dressed stone laid in courses: quay walls, bridges, churches and the palace.
Surface StoneSurface(Surface surface, vec3 normal, Pattern pattern, float seedRandom) {
    vec2 cell = Staggered(pattern.face / ASHLAR_SIZE);
    vec2 cellsPerPixel = pattern.faceRate / ASHLAR_SIZE;
    float sharpness = PatternSharpness(cellsPerPixel);
    float joint = mix(0.08, GridJoints(cell, vec2(0.02, 0.04), PatternBlur(cellsPerPixel)), sharpness);
    float tone = mix(1.0, mix(0.86, 1.1, Hash(floor(cell) + seedRandom * 17.0)), sharpness);
    float weathering = ValueNoise(pattern.face * vec2(0.4, 0.2) + seedRandom * 31.0);
    surface.albedo *= tone * mix(0.8, 1.05, weathering) * mix(1.0, 0.62, joint);
    surface.roughness = mix(surface.roughness, 0.95, joint);
    if (abs(normal.y) < 0.5) {
        // Rain leaves vertical streaks, and the stone is grimier near the ground.
        float streaks = ValueNoise(vec2(pattern.face.x * 3.0, pattern.face.y * 0.15 + seedRandom * 7.0));
        surface.albedo *= mix(1.0, mix(0.82, 1.0, streaks), pattern.detailFade);
        surface.occlusion *= mix(0.6, 1.0, smoothstep(-4.0, 6.0, inWorldPosition.y));
    }
    return surface;
}

// ---------------------------------------------------------------------------------------------
// Roofs
// ---------------------------------------------------------------------------------------------

// Rows of slates or clay tiles, with moss in places and dormer windows on mansard slopes.
Surface RoofSurface(Surface surface, vec3 normal, Pattern pattern, uint flags, float seedRandom) {
    vec2 cell = Staggered(pattern.face / ROOF_TILE_SIZE);
    vec2 cellsPerPixel = pattern.faceRate / ROOF_TILE_SIZE;
    float sharpness = PatternSharpness(cellsPerPixel);
    vec2 blur = PatternBlur(cellsPerPixel);
    vec2 f = fract(cell);
    // Each row overlaps the one below it, leaving a thin shadow line under its lower edge.
    float rowShade = mix(0.62, 1.0, smoothstep(0.0, 0.25 + blur.y, f.y));
    float gap = 1.0 - Band(f.x, 0.04, 0.96, blur.x);
    float tile = mix(0.86, rowShade * (1.0 - 0.3 * gap), sharpness);
    float tone = mix(1.0, mix(0.85, 1.12, Hash(floor(cell) + seedRandom * 23.0)), sharpness);
    float moss = smoothstep(0.6, 0.9, ValueNoise(pattern.face * 0.25 + seedRandom * 41.0));
    surface.albedo = mix(surface.albedo * tile * tone, vec3(0.12, 0.14, 0.08), moss * 0.35);
    surface.roughness = mix(surface.roughness, 0.9, moss);

    if (abs(normal.y) < 0.5 && (flags & ROOF_DORMERS) != 0u) {
        // One dormer per window bay of the facade below, with a painted frame and a lit window.
        float bayFraction = fract(pattern.bay);
        float heightBlur = max(pattern.faceRate.y * 1.5, 1e-4);
        float height = inLocalPosition.y;
        float dormer = Band(bayFraction, 0.30, 0.70, pattern.wallBlur.x) * Band(height, 0.5, 2.3, heightBlur);
        float window = Band(bayFraction, 0.36, 0.64, pattern.wallBlur.x) * Band(height, 0.75, 2.05, heightBlur);
        dormer = mix(0.4 * 0.5, dormer, pattern.wallSharpness);
        window = mix(0.28 * 0.4, window, pattern.wallSharpness);
        surface.albedo = mix(surface.albedo, vec3(0.74, 0.72, 0.68), dormer);
        surface.albedo = mix(surface.albedo, vec3(0.02, 0.025, 0.03), window);
        surface.roughness = mix(surface.roughness, 0.05, window);
        float isLit = mix(0.2, step(0.75, Hash(vec2(floor(pattern.bay) + dot(normal, vec3(17.0, 0.0, 31.0)), seedRandom * 77.0))),
                          pattern.wallSharpness);
        surface.emissive += vec3(1.0, 0.68, 0.38) * 1.6 * isLit * DuskFactor() * window;
    }
    return surface;
}

// ---------------------------------------------------------------------------------------------
// Ground
// ---------------------------------------------------------------------------------------------

Surface GroundSurface(Surface surface, vec3 normal, vec3 position, Pattern pattern, uint groundPattern) {
    if (abs(normal.y) < 0.5) {
        // The sides of pavements and squares are granite kerbstones.
        float grain = ValueNoise(pattern.face * vec2(1.5, 6.0));
        surface.albedo = mix(surface.albedo, vec3(0.46, 0.45, 0.43), 0.6) * mix(0.85, 1.05, grain);
        surface.roughness = 0.7;
        return surface;
    }

    float coarse = ValueNoise(position.xz * 0.6);
    float fine = mix(0.5, ValueNoise(position.xz * 3.7), pattern.detailFade);
    surface.albedo *= mix(0.78, 1.12, coarse * 0.6 + fine * 0.4);

    if (groundPattern == GROUND_SLABS) {
        // Stone slabs with darker joints.
        vec2 slabFraction = fract(pattern.tile);
        float slab = Band(slabFraction.x, 0.04, 0.96, pattern.tileBlur.x) * Band(slabFraction.y, 0.04, 0.96, pattern.tileBlur.y);
        slab = mix(0.85, slab, pattern.tileSharpness);
        float slabShade = mix(1.0, mix(0.9, 1.08, Hash(floor(pattern.tile))), pattern.tileSharpness);
        surface.albedo *= mix(0.55, 1.0, slab) * slabShade;
    } else if (groundPattern == GROUND_COBBLES) {
        // Granite setts in staggered rows, each one domed and darker towards its edges.
        vec2 cell = Staggered(position.xz / COBBLE_SIZE);
        vec2 cellsPerPixel = pattern.faceRate / COBBLE_SIZE;
        float sharpness = PatternSharpness(cellsPerPixel);
        vec2 f = fract(cell);
        vec2 towardsCentre = min(f, 1.0 - f) * 2.0;
        float dome = smoothstep(0.0, 0.6, min(towardsCentre.x, towardsCentre.y));
        float stone = mix(0.45, 1.0, dome) * mix(0.8, 1.15, Hash(floor(cell)));
        surface.albedo *= mix(0.78, stone, sharpness);
        surface.roughness *= mix(1.0, mix(1.1, 0.85, dome), sharpness);
    } else if (groundPattern == GROUND_GRAVEL) {
        // Fine grit, faded out sooner than other detail because it is so small.
        float gritFade = 1.0 - smoothstep(0.02, 0.08, pattern.metresPerPixel);
        float grit = mix(0.5, ValueNoise(position.xz * 23.0), gritFade);
        surface.albedo *= mix(0.8, 1.15, grit);
        surface.roughness = 1.0;
    } else {
        // Asphalt with darker patched repairs.
        float patches = smoothstep(0.62, 0.7, ValueNoise(position.xz * 0.08 + 13.0));
        surface.albedo *= mix(1.0, 0.8, patches);
    }
    return surface;
}

Surface GrassSurface(Surface surface, vec3 normal, vec3 position, Pattern pattern, float seedRandom) {
    if (abs(normal.y) < 0.5) {
        surface.albedo *= 0.55;  // the soil edge of a lawn
        return surface;
    }
    // Drier, yellower patches, fine tufts, and the faint stripes a mower leaves.
    float patches = ValueNoise(position.xz * 0.12 + seedRandom * 19.0);
    surface.albedo = mix(surface.albedo, surface.albedo * vec3(1.25, 1.1, 0.65), smoothstep(0.55, 0.9, patches) * 0.7);
    float tufts = mix(0.5, ValueNoise(position.xz * 6.0), pattern.detailFade);
    surface.albedo *= mix(0.75, 1.15, tufts);
    vec2 stripesPerPixel = pattern.faceRate / 5.0;
    float stripe = Band(fract(position.x / 5.0), 0.0, 0.5, PatternBlur(stripesPerPixel).x);
    surface.albedo *= mix(0.94, 1.06, mix(0.5, stripe, PatternSharpness(stripesPerPixel)));
    surface.roughness = 0.95;
    return surface;
}

// ---------------------------------------------------------------------------------------------
// Water, leaves, glass, metal and lamps
// ---------------------------------------------------------------------------------------------

// The slope of one travelling wave; deep-water waves travel faster the longer they are.
vec2 WaveSlope(vec2 position, vec2 direction, float wavelength, float steepness, float time) {
    float k = 2.0 * PI / wavelength;
    float speed = sqrt(9.81 / k) * 0.5;
    float phase = k * (dot(direction, position) - speed * time);
    return direction * (steepness * cos(phase));
}

// Calm river water: a few overlapping waves tilt the normal, and the sky does the rest through
// the Fresnel reflection in the lighting below.
Surface WaterSurface(Surface surface, inout vec3 normal, vec3 position, Pattern pattern) {
    if (normal.y < 0.5) {
        return surface;  // the sides are hidden behind the quay walls
    }
    float time = frame.cameraPosition.w;
    vec2 p = position.xz;
    vec2 slope = WaveSlope(p, vec2(0.8, 0.6), 9.0, 0.035, time)
               + WaveSlope(p, vec2(-0.5, 0.866), 5.0, 0.03, time);
    // Short ripples fade out with distance before they could alias.
    vec2 ripples = WaveSlope(p, vec2(0.954, -0.3), 2.3, 0.025, time)
                 + WaveSlope(p, vec2(-0.2, -0.98), 1.1, 0.02, time);
    slope += ripples * pattern.detailFade;
    normal = normalize(vec3(-slope.x, 1.0, -slope.y));
    surface.roughness = 0.05;
    surface.reflectance = 0.02;
    return surface;
}

// Crowns of trees and hedges: clumps of leaves that break up the flat faces of the boxes.
Surface FoliageSurface(Surface surface, inout vec3 normal, vec3 position, Pattern pattern) {
    float clumps = ValueNoise(position.xz * 0.9 + position.y * 0.7) * 0.6
                 + ValueNoise(position.zy * 2.3 + position.x * 1.3) * 0.4;
    float leaves = mix(0.5, clumps, pattern.detailFade);
    // Each tree gets a slightly different shade of green.
    float hue = Hash(floor(position.xz / 6.0));
    surface.albedo *= mix(0.55, 1.25, leaves) * mix(vec3(0.9, 1.0, 0.8), vec3(1.1, 1.0, 0.92), hue);
    float heightInCrown = smoothstep(0.0, max(inBoxSize.y, 0.1), inLocalPosition.y);
    surface.occlusion *= mix(0.45, 1.0, leaves) * mix(0.55, 1.0, heightInCrown);
    // Tilt the normal with the clumps so light falls unevenly across each face.
    vec2 tilt = vec2(ValueNoise(position.xy * 1.1 + 3.7), ValueNoise(position.zy * 1.1 + 9.1)) - 0.5;
    normal = normalize(normal + (vec3(tilt.x, 0.0, tilt.y) * 0.9 + vec3(0.0, 0.25, 0.0)) * pattern.detailFade);
    surface.roughness = 0.85;
    surface.wrap = 0.5;
    return surface;
}

Surface GlassSurface(Surface surface, Pattern pattern, uint flags, float seedRandom) {
    bool stained = (flags & GLASS_STAINED) != 0u;
    // Panes held in a grid of glazing bars, finer in stained-glass windows.
    vec2 paneSize = stained ? vec2(0.6, 0.8) : vec2(1.0, 1.4);
    vec2 cell = pattern.face / paneSize;
    vec2 cellsPerPixel = pattern.faceRate / paneSize;
    float sharpness = PatternSharpness(cellsPerPixel);
    float bars = mix(0.12, GridJoints(cell, vec2(0.05), PatternBlur(cellsPerPixel)), sharpness);
    vec3 paneColor = surface.albedo;
    if (stained) {
        float pick = Hash(floor(cell) + seedRandom * 7.0);
        vec3 tint = pick < 0.3 ? vec3(0.35, 0.04, 0.04)
                  : pick < 0.6 ? vec3(0.04, 0.08, 0.35)
                  : pick < 0.8 ? vec3(0.35, 0.25, 0.03) : vec3(0.05, 0.22, 0.08);
        paneColor = mix(surface.albedo, tint, sharpness);
        // Candlelight shines out through the coloured glass after dark.
        surface.emissive += paneColor * 2.5 * DuskFactor() * (1.0 - bars);
        paneColor *= 0.4;
    }
    surface.albedo = mix(paneColor, vec3(0.03), bars);
    surface.roughness = mix(0.05, 0.5, bars);
    return surface;
}

// Copper that has weathered blue green: dull patina in patches and in rain streaks, with the
// brighter colour of the metal left only where the weather has not reached it.
Surface VerdigrisFinish(Surface surface, Pattern pattern, float seedRandom) {
    float patches = ValueNoise(pattern.face * vec2(1.7, 0.9) + seedRandom * 37.0);
    float streaks = ValueNoise(vec2(pattern.face.x * 4.0, pattern.face.y * 0.3 + seedRandom * 11.0));
    float patina = smoothstep(0.25, 0.75, mix(patches, streaks, 0.4));
    vec3 oxide = vec3(0.30, 0.60, 0.52);
    vec3 bareCopper = vec3(0.45, 0.28, 0.17);
    surface.albedo = mix(bareCopper, oxide, mix(0.85, patina, pattern.detailFade)) * mix(0.8, 1.1, patches);
    surface.metallic = mix(0.5, 0.1, patina);
    surface.roughness = mix(0.4, 0.8, patina);
    return surface;
}

Surface MetalSurface(Surface surface, vec3 normal, Pattern pattern, uint flags, float seedRandom) {
    surface.metallic = 0.6;
    bool vertical = abs(normal.y) < 0.5;
    if (vertical && (flags & METAL_RAILING) != 0u) {
        // Wrought-iron railings: thin balusters between a top rail and a bottom rail. The gaps
        // show the shadowed wall behind; they are shaded dark rather than cut out, which would
        // cost the early depth test for every box.
        float barsPerPixel = pattern.faceRate.x / 0.14;
        float bars = Band(fract(pattern.face.x / 0.14), 0.0, 0.35, PatternBlur(vec2(barsPerPixel)).x);
        float heightBlur = max(pattern.faceRate.y * 1.5, 1e-4);
        float top = max(inBoxSize.y - 0.1, 0.0);
        float rails = smoothstep(top - heightBlur, top + heightBlur, inLocalPosition.y)
                    + 1.0 - smoothstep(0.08 - heightBlur, 0.08 + heightBlur, inLocalPosition.y);
        float solid = mix(0.45, Saturate(max(bars, rails)), PatternSharpness(vec2(barsPerPixel)));
        surface.albedo = mix(vec3(0.02), surface.albedo, solid);
        surface.occlusion *= mix(0.3, 1.0, solid);
        surface.metallic *= solid;
    } else {
        // Standing seams on sheet-metal roofs and spires, and streaks of weathering.
        float seamsPerPixel = pattern.faceRate.x / 0.5;
        float seam = Band(fract(pattern.face.x / 0.5), 0.0, 0.06, PatternBlur(vec2(seamsPerPixel)).x);
        seam = mix(0.06, seam, PatternSharpness(vec2(seamsPerPixel)));
        float streaks = ValueNoise(pattern.face * vec2(1.0, 0.3));
        surface.albedo *= mix(1.0, 0.75, seam) * mix(0.8, 1.1, streaks);
        if ((flags & METAL_VERDIGRIS) != 0u) {
            surface = VerdigrisFinish(surface, pattern, seedRandom);
            surface.albedo *= mix(1.0, 0.8, seam);
        }
    }
    return surface;
}

// Lanterns and clock faces: pale glass by day, glowing once the sun is low.
Surface LampSurface(Surface surface) {
    surface.emissive += vec3(1.0, 0.74, 0.45) * 4.0 * NightFactor();
    surface.roughness = 0.2;
    return surface;
}

// Painted wood, cloth and plain trim: only a little soft weathering.
Surface PlainSurface(Surface surface, Pattern pattern, float seedRandom) {
    surface.albedo *= mix(0.92, 1.05, ValueNoise(pattern.face * 0.8 + seedRandom * 13.0));
    return surface;
}

// ---------------------------------------------------------------------------------------------
// Lighting
// ---------------------------------------------------------------------------------------------

// GGX normal distribution: how many microfacets face the half vector.
float DistributionGgx(float NdotH, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

// Smith-Schlick geometry term: how many microfacets are neither hidden nor in shadow.
float GeometrySmith(float NdotV, float NdotL, float roughness) {
    float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
    float viewTerm = NdotV / (NdotV * (1.0 - k) + k);
    float lightTerm = NdotL / (NdotL * (1.0 - k) + k);
    return viewTerm * lightTerm;
}

vec3 FresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

float SunShadow(vec3 position, vec3 normal, float NdotL) {
    // Move the lookup point off the surface along its normal to avoid self-shadowing ("acne").
    vec3 offsetPosition = position + normal * (0.04 + 0.12 * (1.0 - NdotL));
    vec4 lightClip = frame.lightViewProjection * vec4(offsetPosition, 1.0);
    vec3 lightNdc = lightClip.xyz / lightClip.w;
    vec2 uv = lightNdc.xy * 0.5 + 0.5;
    if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0))) || lightNdc.z > 1.0) {
        return 1.0;  // outside the area the shadow map covers
    }

    // 16 hardware-filtered taps (percentage-closer filtering) give soft shadow edges.
    float texel = frame.settings.x;
    float lit = 0.0;
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
            vec2 offset = (vec2(x, y) - 1.5) * texel;
            lit += textureLod(shadowMap, vec3(uv + offset, lightNdc.z), 0.0);
        }
    }
    lit /= 16.0;

    // Fade out towards the edge of the shadow map instead of stopping abruptly.
    float edge = min(min(uv.x, uv.y), min(1.0 - uv.x, 1.0 - uv.y));
    return mix(1.0, lit, smoothstep(0.0, 0.06, edge));
}

vec3 ApplyFog(vec3 color, vec3 position) {
    vec3 toPoint = position - frame.cameraPosition.xyz;
    float viewDistance = length(toPoint);
    vec3 viewDirection = toPoint / max(viewDistance, 0.001);
    // Haze is thicker close to the ground, as in real air.
    float heightFalloff = exp(-max(position.y, 0.0) * 0.02);
    float density = frame.settings.y * mix(0.35, 1.0, heightFalloff);
    float fogAmount = 1.0 - exp(-viewDistance * density);
    return mix(color, SkyRadiance(viewDirection, false), fogAmount);
}

void main() {
    // The geometric normal places the shadow lookup; materials may tilt the shading normal.
    vec3 geometricNormal = normalize(inNormal);
    vec3 N = geometricNormal;
    vec3 P = inWorldPosition;
    vec3 V = normalize(frame.cameraPosition.xyz - P);
    vec3 L = frame.sunDirection.xyz;

    // A tiny bias keeps the whole-number flags intact through interpolation.
    uint flags = uint(max(inSurface.z + 0.0005, 0.0));
    float seedRandom = fract(inSurface.z);
    Pattern pattern = ComputePattern(N, flags);

    Surface surface;
    surface.albedo = inAlbedo;
    surface.roughness = inSurface.x;
    surface.emissive = vec3(0.0);
    surface.occlusion = 1.0;
    surface.metallic = 0.0;
    surface.reflectance = 0.04;  // most non-metals reflect about 4% of light head-on
    surface.wrap = 0.0;

    int material = int(inSurface.y + 0.5);
    if (material == MATERIAL_FACADE) {
        surface = FacadeSurface(surface, N, pattern, flags, seedRandom);
    } else if (material == MATERIAL_GROUND) {
        surface = GroundSurface(surface, N, P, pattern, flags);
    } else if (material == MATERIAL_CHARACTER) {
        // Darker towards the feet, where the body and the ground block the sky.
        surface.occlusion = mix(0.6, 1.0, smoothstep(0.0, 1.6, inLocalPosition.y));
    } else if (material == MATERIAL_STONE) {
        surface = StoneSurface(surface, N, pattern, seedRandom);
    } else if (material == MATERIAL_ROOF_TILE) {
        surface = RoofSurface(surface, N, pattern, flags, seedRandom);
    } else if (material == MATERIAL_WATER) {
        surface = WaterSurface(surface, N, P, pattern);
    } else if (material == MATERIAL_FOLIAGE) {
        surface = FoliageSurface(surface, N, P, pattern);
    } else if (material == MATERIAL_GRASS) {
        surface = GrassSurface(surface, N, P, pattern, seedRandom);
    } else if (material == MATERIAL_GLASS) {
        surface = GlassSurface(surface, pattern, flags, seedRandom);
    } else if (material == MATERIAL_METAL) {
        surface = MetalSurface(surface, N, pattern, flags, seedRandom);
    } else if (material == MATERIAL_LAMP) {
        surface = LampSurface(surface);
    } else if (material == MATERIAL_PLAIN) {
        surface = PlainSurface(surface, pattern, seedRandom);
    }

    float roughness = clamp(surface.roughness, 0.04, 1.0);
    vec3 H = normalize(V + L);
    float NdotLSigned = dot(N, L);
    float NdotL = max(NdotLSigned, 0.0);
    float NdotV = max(dot(N, V), 1e-4);
    float NdotH = max(dot(N, H), 0.0);
    float VdotH = max(dot(V, H), 0.0);

    // Sunlight. Metals tint their reflection with their own colour and have no diffuse part.
    vec3 F0 = mix(vec3(surface.reflectance), surface.albedo, surface.metallic);
    vec3 diffuseColor = surface.albedo * (1.0 - surface.metallic);
    vec3 F = FresnelSchlick(VdotH, F0);
    vec3 specular = DistributionGgx(NdotH, roughness) * GeometrySmith(NdotV, NdotL, roughness) * F
                  / (4.0 * NdotV * NdotL + 1e-4);
    float diffuseNdotL = Saturate((NdotLSigned + surface.wrap) / (1.0 + surface.wrap));
    vec3 diffuse = (1.0 - F) * diffuseColor / PI;
    float geometricNdotL = max(dot(geometricNormal, L), 0.0);
    float shadow = (geometricNdotL > 0.0) ? SunShadow(P, geometricNormal, geometricNdotL) : 0.0;
    vec3 sunLight = (diffuse * diffuseNdotL + specular * NdotL) * frame.sunColor.rgb * shadow;
    // Sunlight seen through leaves when looking towards the sun.
    sunLight += diffuseColor * frame.sunColor.rgb * surface.wrap * 0.3 * pow(Saturate(dot(-V, L)), 4.0) * (1.0 - NdotL);

    // Ambient light: the sky above, and sunlight bounced off the ground below.
    vec3 skyLight = SkyRadiance(vec3(0.0, 1.0, 0.0), false) * 0.6
                  + SkyRadiance(normalize(vec3(N.x, 0.25, N.z)), false) * 0.4;
    vec3 groundBounce = vec3(0.32, 0.29, 0.25) * (frame.sunColor.rgb * max(L.y, 0.0) * 0.1 + 0.01);
    vec3 ambientLight = mix(groundBounce, skyLight, N.y * 0.5 + 0.5);

    // Reflections of the sky, blurred towards the ambient colour on rough surfaces. Rippled
    // water can reflect below the horizon, which the sky model does not cover, so it is lifted.
    vec3 reflected = reflect(-V, N);
    reflected = normalize(vec3(reflected.x, max(reflected.y, 0.02), reflected.z));
    vec3 ambientFresnel = F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(1.0 - NdotV, 5.0);
    vec3 reflection = mix(SkyRadiance(reflected, false), ambientLight, roughness);
    vec3 ambient = ((1.0 - ambientFresnel) * diffuseColor * ambientLight + ambientFresnel * reflection) * surface.occlusion;

    vec3 color = sunLight + ambient + surface.emissive;
    outColor = vec4(ApplyFog(color, P), 1.0);
}
