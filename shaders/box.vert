#version 460

// Instanced boxes: every instance stretches the unit cube to its own position and size.
// Used for the whole static city, in both the shadow pass and the scene pass.

layout(location = 0) in vec3 inPosition;  // unit cube corner, -1..1
layout(location = 1) in vec3 inNormal;
layout(location = 2) in float inPart;     // unused for boxes
layout(location = 3) in vec4 inCenterMaterial;     // xyz: centre, w: material id
layout(location = 4) in vec4 inHalfSizeRoughness;  // xyz: half size, w: roughness
layout(location = 5) in vec4 inColorSeed;          // rgb: base colour, w: per-object seed

layout(push_constant) uniform PushConstants {
    mat4 viewProjection;  // the camera's in the scene pass, the sun's in the shadow pass
} push;

layout(location = 0) out vec3 outWorldPosition;
layout(location = 1) out vec3 outNormal;
layout(location = 2) out vec3 outAlbedo;
layout(location = 3) out vec3 outSurface;        // x: roughness, y: material id, z: seed
layout(location = 4) out vec3 outLocalPosition;  // metres from the box's minimum corner
layout(location = 5) out vec3 outBoxSize;        // full size of the box in metres

void main() {
    vec3 halfSize = inHalfSizeRoughness.xyz;
    vec3 worldPosition = inCenterMaterial.xyz + inPosition * halfSize;

    outWorldPosition = worldPosition;
    outNormal = inNormal;
    outAlbedo = inColorSeed.rgb;
    outSurface = vec3(inHalfSizeRoughness.w, inCenterMaterial.w, inColorSeed.w);
    outLocalPosition = (inPosition + 1.0) * halfSize;
    outBoxSize = halfSize * 2.0;
    gl_Position = push.viewProjection * vec4(worldPosition, 1.0);
}
