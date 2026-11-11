#version 460

// Maps the HDR scene into the range a display can show: exposure, a filmic tone curve and a
// vignette. The window uses an sRGB format, so the GPU applies gamma encoding on write.
//
// With dynamic resolution the scene only fills the top-left part of the HDR image. The
// window's UVs are scaled onto that part, and the bilinear sampler upsamples it.

layout(set = 0, binding = 0) uniform sampler2D hdrScene;

layout(push_constant) uniform PushConstants {
    vec4 settings;      // x: exposure
    vec4 sourceRegion;  // xy: UV scale of the drawn part, zw: largest UV to sample (half a texel inside its edge)
} push;

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

// Krzysztof Narkowicz's fit of the ACES filmic tone curve: soft highlights, rich contrast.
vec3 AcesFilmic(vec3 x) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

float InterleavedGradientNoise(vec2 pixel) {
    return fract(52.9829189 * fract(dot(pixel, vec2(0.06711056, 0.00583715))));
}

void main() {
    vec2 sourceUv = min(inUv * push.sourceRegion.xy, push.sourceRegion.zw);
    vec3 color = AcesFilmic(texture(hdrScene, sourceUv).rgb * push.settings.x);

    // Gently darken the corners, as a real camera lens does. This uses the window's UVs, so the
    // vignette does not move with the render scale.
    float distanceFromCenter = length(inUv - 0.5);
    color *= mix(1.0, 0.72, smoothstep(0.35, 0.85, distanceFromCenter));

    // A little noise, about one display step in size, hides banding in smooth gradients like the sky.
    // The step is larger in dark tones once gamma encoding is applied, so the noise follows sqrt(color).
    float noise = InterleavedGradientNoise(gl_FragCoord.xy) - 0.5;
    color += noise * 2.0 * sqrt(max(color, vec3(0.0))) / 255.0;
    outColor = vec4(color, 1.0);
}
