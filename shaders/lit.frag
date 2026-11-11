#version 460
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

// Lighting for everything opaque: physically based shading from one sun with soft shadows,
// ambient light and reflections from the sky, procedural surface detail, and distance fog.

layout(set = 0, binding = 1) uniform sampler2DShadow shadowMap;

layout(location = 0) in vec3 inWorldPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inAlbedo;
layout(location = 3) in vec3 inSurface;        // x: roughness, y: material id, z: per-object seed
layout(location = 4) in vec3 inLocalPosition;  // boxes: metres from the minimum corner; people: body position
layout(location = 5) in vec3 inBoxSize;        // boxes: full size in metres

layout(location = 0) out vec4 outColor;

// Material ids, matching SurfaceMaterial in SceneTypes.h.
const int MATERIAL_GROUND = 0;
const int MATERIAL_FACADE = 1;
const int MATERIAL_CHARACTER = 3;

// Facade layout. The floor heights must match CityGenerator.cpp so window rows meet the roof line.
const float GROUND_FLOOR_HEIGHT = 4.5;
const float UPPER_FLOOR_HEIGHT = 3.3;
const float WINDOW_BAY_WIDTH = 3.2;
const float PAVING_SLAB_SIZE = 0.9;

struct Surface {
    vec3 albedo;
    float roughness;
    vec3 emissive;
    float occlusion;  // 1 = fully open to the sky, 0 = fully blocked
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
    vec3 wallBlur;      // anti-aliasing blur for bay, upperFloor and groundFloor
    vec2 tileBlur;
    float wallSharpness;  // 1 when the window pattern is clearly visible, 0 when it is sub-pixel
    float tileSharpness;
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

// 1 inside [low, high] of a repeating 0..1 coordinate and 0 outside, with edges softened by `blur`.
float Band(float x, float low, float high, float blur) {
    return smoothstep(low - blur, low + blur, x) - smoothstep(high - blur, high + blur, x);
}

Pattern ComputePattern(vec3 normal) {
    Pattern pattern;
    bool facesX = abs(normal.x) > 0.5;
    pattern.along = facesX ? inLocalPosition.z : inLocalPosition.x;
    float wallWidth = max(facesX ? inBoxSize.z : inBoxSize.x, 0.01);
    // Fit a whole number of bays across the wall so windows are never cut off at the corners.
    float bayCount = max(1.0, floor(wallWidth / WINDOW_BAY_WIDTH));
    pattern.bay = pattern.along / wallWidth * bayCount;
    pattern.upperFloor = (inLocalPosition.y - GROUND_FLOOR_HEIGHT) / UPPER_FLOOR_HEIGHT;
    pattern.groundFloor = inLocalPosition.y / GROUND_FLOOR_HEIGHT;
    pattern.tile = inWorldPosition.xz / PAVING_SLAB_SIZE;

    vec3 wallRate = vec3(fwidth(pattern.bay), fwidth(pattern.upperFloor), fwidth(pattern.groundFloor));
    vec2 tileRate = fwidth(pattern.tile);
    pattern.wallBlur = min(wallRate * 1.5, vec3(0.5));
    pattern.tileBlur = min(tileRate * 1.5, vec2(0.5));
    pattern.wallSharpness = 1.0 - smoothstep(0.15, 0.45, max(wallRate.x, wallRate.y));
    pattern.tileSharpness = 1.0 - smoothstep(0.15, 0.45, max(tileRate.x, tileRate.y));
    float metresPerPixel = length(fwidth(inWorldPosition));
    pattern.detailFade = 1.0 - smoothstep(0.1, 0.4, metresPerPixel);
    return pattern;
}

Surface FacadeSurface(Surface surface, vec3 normal, Pattern pattern) {
    if (abs(normal.y) > 0.5) {
        surface.albedo *= 0.55;  // flat top of a building, mostly hidden under its roof
        return surface;
    }

    float height = inLocalPosition.y;
    float bayFraction = fract(pattern.bay);
    float windowMask;
    float floorIndex;
    float averageWindowCoverage;
    if (height < GROUND_FLOOR_HEIGHT) {
        // Tall shop fronts on the ground floor.
        windowMask = Band(bayFraction, 0.12, 0.88, pattern.wallBlur.x) * Band(pattern.groundFloor, 0.05, 0.78, pattern.wallBlur.z);
        averageWindowCoverage = 0.76 * 0.73;
        floorIndex = -1.0;
    } else {
        float floorFraction = fract(pattern.upperFloor);
        windowMask = Band(bayFraction, 0.30, 0.70, pattern.wallBlur.x) * Band(floorFraction, 0.18, 0.82, pattern.wallBlur.y);
        averageWindowCoverage = 0.40 * 0.64;
        floorIndex = floor(pattern.upperFloor);
        // A pale stone ledge along every floor line.
        float ledge = mix(0.11, Band(floorFraction, -0.05, 0.06, pattern.wallBlur.y), pattern.wallSharpness);
        surface.albedo *= 1.0 + 0.15 * ledge;
    }
    // Far away the pattern is smaller than a pixel: use its average so the wall keeps the same
    // overall brightness instead of shimmering.
    windowMask = mix(averageWindowCoverage, windowMask, pattern.wallSharpness);

    // Large soft stains so walls are not one flat colour.
    float weathering = ValueNoise(vec2(pattern.along * 0.35, height * 0.12) + inSurface.z * 57.0);
    surface.albedo *= mix(0.82, 1.06, weathering);
    // Little sky reaches the bottom of a narrow street, and walls are dirtier there.
    surface.occlusion *= mix(0.55, 1.0, smoothstep(0.0, 9.0, height));

    // Some windows are lit from inside; they stand out more as the sun goes down.
    vec2 windowId = vec2(floor(pattern.bay) + dot(normal, vec3(17.0, 0.0, 31.0)), floorIndex);
    float isLit = mix(0.2, step(0.8, Hash(windowId + inSurface.z * 91.0)), pattern.wallSharpness);
    float dusk = 1.0 - 0.85 * smoothstep(0.05, 0.45, frame.sunDirection.y);
    vec3 interiorLight = vec3(1.0, 0.68, 0.38) * 1.6 * isLit * dusk;

    vec3 glass = vec3(0.02, 0.025, 0.03);
    surface.albedo = mix(surface.albedo, glass, windowMask);
    surface.roughness = mix(surface.roughness, 0.05, windowMask);
    surface.emissive += interiorLight * windowMask;
    return surface;
}

Surface GroundSurface(Surface surface, vec3 position, Pattern pattern) {
    float coarse = ValueNoise(position.xz * 0.6);
    float fine = mix(0.5, ValueNoise(position.xz * 3.7), pattern.detailFade);
    surface.albedo *= mix(0.78, 1.12, coarse * 0.6 + fine * 0.4);

    // A seed of 1 marks paved surfaces (pavements and squares): stone slabs with darker joints.
    if (inSurface.z > 0.5) {
        vec2 slabFraction = fract(pattern.tile);
        float slab = Band(slabFraction.x, 0.04, 0.96, pattern.tileBlur.x) * Band(slabFraction.y, 0.04, 0.96, pattern.tileBlur.y);
        slab = mix(0.85, slab, pattern.tileSharpness);
        float slabShade = mix(1.0, mix(0.9, 1.08, Hash(floor(pattern.tile))), pattern.tileSharpness);
        surface.albedo *= mix(0.55, 1.0, slab) * slabShade;
    }
    return surface;
}

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
    vec3 N = normalize(inNormal);
    vec3 P = inWorldPosition;
    vec3 V = normalize(frame.cameraPosition.xyz - P);
    vec3 L = frame.sunDirection.xyz;
    Pattern pattern = ComputePattern(N);

    Surface surface;
    surface.albedo = inAlbedo;
    surface.roughness = inSurface.x;
    surface.emissive = vec3(0.0);
    surface.occlusion = 1.0;

    int material = int(inSurface.y + 0.5);
    if (material == MATERIAL_FACADE) {
        surface = FacadeSurface(surface, N, pattern);
    } else if (material == MATERIAL_GROUND) {
        surface = GroundSurface(surface, P, pattern);
    } else if (material == MATERIAL_CHARACTER) {
        // Darker towards the feet, where the body and the ground block the sky.
        surface.occlusion = mix(0.6, 1.0, smoothstep(0.0, 1.6, inLocalPosition.y));
    }

    float roughness = clamp(surface.roughness, 0.04, 1.0);
    vec3 H = normalize(V + L);
    float NdotL = max(dot(N, L), 0.0);
    float NdotV = max(dot(N, V), 1e-4);
    float NdotH = max(dot(N, H), 0.0);
    float VdotH = max(dot(V, H), 0.0);

    // Sunlight. Everything here is non-metallic, which reflects about 4% of light head-on.
    vec3 F0 = vec3(0.04);
    vec3 F = FresnelSchlick(VdotH, F0);
    vec3 specular = DistributionGgx(NdotH, roughness) * GeometrySmith(NdotV, NdotL, roughness) * F
                  / (4.0 * NdotV * NdotL + 1e-4);
    vec3 diffuse = (1.0 - F) * surface.albedo / PI;
    float shadow = (NdotL > 0.0) ? SunShadow(P, N, NdotL) : 0.0;
    vec3 sunLight = (diffuse + specular) * frame.sunColor.rgb * NdotL * shadow;

    // Ambient light: the sky above, and sunlight bounced off the ground below.
    vec3 skyLight = SkyRadiance(vec3(0.0, 1.0, 0.0), false) * 0.6
                  + SkyRadiance(normalize(vec3(N.x, 0.25, N.z)), false) * 0.4;
    vec3 groundBounce = vec3(0.32, 0.29, 0.25) * (frame.sunColor.rgb * max(L.y, 0.0) * 0.1 + 0.01);
    vec3 ambientLight = mix(groundBounce, skyLight, N.y * 0.5 + 0.5);

    // Reflections of the sky, blurred towards the ambient colour on rough surfaces.
    vec3 ambientFresnel = F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(1.0 - NdotV, 5.0);
    vec3 reflection = mix(SkyRadiance(reflect(-V, N), false), ambientLight, roughness);
    vec3 ambient = ((1.0 - ambientFresnel) * surface.albedo * ambientLight + ambientFresnel * reflection) * surface.occlusion;

    vec3 color = sunLight + ambient + surface.emissive;
    outColor = vec4(ApplyFog(color, P), 1.0);
}
