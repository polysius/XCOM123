#version 460

// Instanced rigid bodies: every instance stretches the unit cube (or the unit sphere) to its own size,
// turns it by a full orientation and moves it into place. Unlike box.vert, which only turns about
// the vertical axis, a body can lie at any angle, because it has tumbled to rest there.
// Used for the props, in both the shadow pass and the scene pass.

layout(location = 0) in vec3 inPosition;  // unit mesh point, -1..1
layout(location = 1) in vec3 inNormal;
layout(location = 2) in float inPart;     // unused for bodies
layout(location = 3) in vec4 inCenterMaterial;     // xyz: centre, w: material id
layout(location = 4) in vec4 inHalfSizeRoughness;  // xyz: half size, w: roughness
layout(location = 5) in vec4 inColorSeed;          // rgb: base colour, w: per-object seed
layout(location = 6) in vec4 inOrientation;        // a unit quaternion: xyz is the axis part, w the scalar part

layout(push_constant) uniform PushConstants {
    mat4 viewProjection;  // the camera's in the scene pass, the sun's in the shadow pass
} push;

layout(location = 0) out vec3 outWorldPosition;
layout(location = 1) out vec3 outNormal;
layout(location = 2) out vec3 outAlbedo;
layout(location = 3) out vec3 outSurface;        // x: roughness, y: material id, z: seed
layout(location = 4) out vec3 outLocalPosition;  // metres from the body's minimum corner
layout(location = 5) out vec3 outBoxSize;        // full size of the body in metres
layout(location = 6) out vec3 outLocalNormal;    // the normal before the body turns, to tell faces apart

// Turns a vector by a unit quaternion.
vec3 Rotate(vec3 v, vec4 q) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

void main() {
    vec3 halfSize = inHalfSizeRoughness.xyz;
    vec3 worldPosition = inCenterMaterial.xyz + Rotate(inPosition * halfSize, inOrientation);

    outWorldPosition = worldPosition;
    outNormal = Rotate(inNormal, inOrientation);
    outLocalNormal = inNormal;
    outAlbedo = inColorSeed.rgb;
    outSurface = vec3(inHalfSizeRoughness.w, inCenterMaterial.w, inColorSeed.w);
    outLocalPosition = (inPosition + 1.0) * halfSize;
    outBoxSize = halfSize * 2.0;
    gl_Position = push.viewProjection * vec4(worldPosition, 1.0);
}
