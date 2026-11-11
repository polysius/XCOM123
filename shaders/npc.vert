#version 460

// Instanced people. The walk cycle is animated here on the GPU: legs and arms swing around the
// hips and shoulders, so the CPU only has to send a position, heading and walk phase per person.

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in float inPart;            // body part id, see BodyPart in Meshes.h
layout(location = 3) in vec4 inPositionHeading;  // xyz: feet position, w: heading in radians
layout(location = 4) in vec4 inColorHeight;      // rgb: clothing colour, w: height scale
layout(location = 5) in vec4 inAnimation;        // x: walk phase in radians, y: stride 0..1

layout(push_constant) uniform PushConstants {
    mat4 viewProjection;
} push;

layout(location = 0) out vec3 outWorldPosition;
layout(location = 1) out vec3 outNormal;
layout(location = 2) out vec3 outAlbedo;
layout(location = 3) out vec3 outSurface;        // x: roughness, y: material id, z: seed
layout(location = 4) out vec3 outLocalPosition;  // position on the body, metres above the feet
layout(location = 5) out vec3 outBoxSize;        // unused for people

const int PART_HEAD = 1;
const int PART_LEFT_LEG = 2;
const int PART_RIGHT_LEG = 3;
const int PART_LEFT_ARM = 4;
const int PART_RIGHT_ARM = 5;

const float MATERIAL_CHARACTER = 3.0;
const vec3 HIP = vec3(0.0, 0.9, 0.0);
const vec3 SHOULDER = vec3(0.0, 1.45, 0.0);

vec3 RotateAroundX(vec3 v, float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return vec3(v.x, v.y * c - v.z * s, v.y * s + v.z * c);
}

// Turns a vector around the vertical axis. An angle of 0 keeps +Z pointing along +Z.
vec3 RotateAroundY(vec3 v, float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return vec3(v.x * c + v.z * s, v.y, -v.x * s + v.z * c);
}

float Hash(float n) {
    return fract(sin(n) * 43758.5453);
}

void main() {
    int part = int(inPart + 0.5);
    float phase = inAnimation.x;
    float stride = inAnimation.y;
    float swing = sin(phase) * 0.5 * stride;  // about 30 degrees at full stride

    vec3 position = inPosition;
    vec3 normal = inNormal;
    if (part == PART_LEFT_LEG || part == PART_RIGHT_LEG) {
        float angle = (part == PART_LEFT_LEG) ? swing : -swing;
        position = HIP + RotateAroundX(position - HIP, angle);
        normal = RotateAroundX(normal, angle);
    } else if (part == PART_LEFT_ARM || part == PART_RIGHT_ARM) {
        // Each arm swings opposite to the leg on the same side, as in a natural walk.
        float angle = (part == PART_LEFT_ARM) ? -swing * 0.8 : swing * 0.8;
        position = SHOULDER + RotateAroundX(position - SHOULDER, angle);
        normal = RotateAroundX(normal, angle);
    }
    // A small bounce, highest when the legs pass each other.
    position.y += abs(cos(phase)) * 0.035 * stride;
    position *= inColorHeight.w;

    float heading = inPositionHeading.w;
    vec3 worldPosition = inPositionHeading.xyz + RotateAroundY(position, heading);

    // The skin tone comes from a hash of the clothing colour, so it stays the same for each person
    // even as people enter and leave the screen (which reorders the instances).
    vec3 clothing = inColorHeight.rgb;
    float skinTone = Hash(dot(clothing, vec3(12.9898, 78.233, 37.719)));
    vec3 skin = mix(vec3(0.80, 0.60, 0.46), vec3(0.36, 0.24, 0.17), skinTone);
    vec3 albedo = clothing;
    if (part == PART_HEAD) {
        albedo = skin;
    } else if (part == PART_LEFT_LEG || part == PART_RIGHT_LEG) {
        albedo = clothing * 0.35 + vec3(0.02);  // darker trousers or skirts
    }

    outWorldPosition = worldPosition;
    outNormal = RotateAroundY(normal, heading);
    outAlbedo = albedo;
    outSurface = vec3(0.75, MATERIAL_CHARACTER, skinTone);
    outLocalPosition = position;
    outBoxSize = vec3(1.0);
    gl_Position = push.viewProjection * vec4(worldPosition, 1.0);
}
