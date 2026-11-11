// Shared by the scene shaders. Keep FrameUniforms in sync with the C++ struct in VulkanRenderer.cpp.

layout(set = 0, binding = 0) uniform FrameUniforms {
    mat4 viewProjection;
    mat4 inverseViewProjection;
    mat4 lightViewProjection;
    vec4 cameraPosition;  // xyz: camera position in world space, w: time in seconds
    vec4 sunDirection;    // xyz: unit vector pointing towards the sun
    vec4 sunColor;        // rgb: sun radiance
    vec4 settings;        // x: shadow map texel size (UV units), y: fog density
} frame;

const float PI = 3.14159265359;

float Saturate(float x) {
    return clamp(x, 0.0, 1.0);
}

// An inexpensive stylised sky. It is not a physical scattering model, but it follows the sun:
// blue overhead during the day, a warm horizon at sunrise and sunset, dark at night, and a glow
// around the sun. The lighting shader samples it for ambient light, reflections and fog, so the
// whole scene picks up the sky's colours.
vec3 SkyRadiance(vec3 direction, bool includeSunDisk) {
    vec3 sun = frame.sunDirection.xyz;
    float sunHeight = sun.y;

    float daylight = smoothstep(-0.15, 0.35, sunHeight);
    vec3 zenith = mix(vec3(0.010, 0.015, 0.040), vec3(0.16, 0.34, 0.78), daylight);
    vec3 horizon = mix(vec3(0.95, 0.45, 0.22), vec3(0.70, 0.80, 0.95), smoothstep(0.0, 0.5, sunHeight));
    horizon *= mix(0.05, 1.0, smoothstep(-0.2, 0.05, sunHeight));

    float up = max(direction.y, 0.0);
    vec3 sky = mix(horizon, zenith, pow(up, 0.45));
    // Below the horizon, fade to a darker haze so distant ground is never pitch black.
    float down = max(-direction.y, 0.0);
    sky = mix(sky, horizon * 0.45, Saturate(down * 3.0));

    float sunAlignment = max(dot(direction, sun), 0.0);
    vec3 sunTint = frame.sunColor.rgb;
    sky += sunTint * (0.035 * pow(sunAlignment, 5.0) + 0.10 * pow(sunAlignment, 40.0));
    if (includeSunDisk) {
        sky += sunTint * 12.0 * smoothstep(0.9994, 0.9997, sunAlignment);
    }
    return sky * 1.5;
}
