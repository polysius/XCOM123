#version 460

// Instanced boxes: every instance stretches the unit cube to its own size, turns it about the
// vertical axis and moves it into place.
// Used for the whole static city, in both the shadow pass and the scene pass.

layout(location = 0) in vec3 inPosition;  // unit cube corner, -1..1
layout(location = 1) in vec3 inNormal;
layout(location = 2) in float inPart;     // unused for boxes
layout(location = 3) in vec4 inCenterMaterial;     // xyz: centre, w: material id
layout(location = 4) in vec4 inHalfSizeRoughness;  // xyz: half size, w: roughness
layout(location = 5) in vec4 inColorSeed;          // rgb: base colour, w: per-object seed
layout(location = 6) in vec4 inOrientation;        // x: yaw in radians about +Y, the rest unused

layout(push_constant) uniform PushConstants {
    mat4 viewProjection;  // the camera's in the scene pass, the sun's in the shadow pass
} push;

layout(location = 0) out vec3 outWorldPosition;
layout(location = 1) out vec3 outNormal;
layout(location = 2) out vec3 outAlbedo;
layout(location = 3) out vec3 outSurface;        // x: roughness, y: material id, z: seed
layout(location = 4) out vec3 outLocalPosition;  // metres from the box's minimum corner
layout(location = 5) out vec3 outBoxSize;        // full size of the box in metres
layout(location = 6) out vec3 outLocalNormal;    // the face normal before the box turns, to tell walls apart

// Turns a vector about +Y. A positive angle turns +Z towards +X, as for people (see npc.vert).
vec3 RotateAroundY(vec3 v, float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return vec3(v.x * c + v.z * s, v.y, -v.x * s + v.z * c);
}

void main() {
    vec3 halfSize = inHalfSizeRoughness.xyz;
    float yaw = inOrientation.x;
    vec3 worldPosition = inCenterMaterial.xyz + RotateAroundY(inPosition * halfSize, yaw);

    outWorldPosition = worldPosition;
    outNormal = RotateAroundY(inNormal, yaw);
    outLocalNormal = inNormal;
    outAlbedo = inColorSeed.rgb;
    outSurface = vec3(inHalfSizeRoughness.w, inCenterMaterial.w, inColorSeed.w);
    outLocalPosition = (inPosition + 1.0) * halfSize;
    outBoxSize = halfSize * 2.0;
    gl_Position = push.viewProjection * vec4(worldPosition, 1.0);
}
