#version 460
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

void main() {
    // Turn the pixel back into the world-space direction the camera is looking through it.
    vec2 ndc = inUv * 2.0 - 1.0;
    vec4 farPoint = frame.inverseViewProjection * vec4(ndc, 0.0, 1.0);
    vec3 direction = normalize(farPoint.xyz / farPoint.w - frame.cameraPosition.xyz);
    outColor = vec4(SkyRadiance(direction, true), 1.0);
}
