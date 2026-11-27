#version 460

// Instanced skinned characters. Every vertex follows up to four joints: the vertex is moved by the weighted sum of their
// skinning matrices, which carry the rest pose to the pose, and the character's world matrix then places it in the world.
//
// The matrices of every character of the frame sit one after the other in the palette buffer, and an instance finds its
// own by an offset counted in matrices. SkinPosition and SkinNormal in src/render/SkinningMath.cpp are this same blend on
// the CPU; keep them in step. Used for the characters, in both the shadow pass and the scene pass.

layout(location = 0) in vec3 inPosition;  // in the rest pose, in the character's own space
layout(location = 1) in vec3 inNormal;
layout(location = 2) in uvec4 inJoints;   // the joints that pull on the vertex
layout(location = 3) in vec4 inWeights;   // how hard each of them pulls; they add up to 1
layout(location = 4) in mat4 inWorld;     // takes the character's own space into the world (locations 4 to 7)
layout(location = 8) in vec4 inColorRoughness;  // rgb: base colour, w: roughness
layout(location = 9) in uint inPaletteOffset;   // index of the first matrix of this character's palette
layout(location = 10) in vec2 inMaterialSeed;   // x: material id, y: per-character seed

layout(std430, set = 0, binding = 5) readonly buffer Palette {
    mat4 matrices[];
} palette;

layout(push_constant) uniform PushConstants {
    mat4 viewProjection;  // the camera's in the scene pass, the sun's in the shadow pass
} push;

layout(location = 0) out vec3 outWorldPosition;
layout(location = 1) out vec3 outNormal;
layout(location = 2) out vec3 outAlbedo;
layout(location = 3) out vec3 outSurface;        // x: roughness, y: material id, z: seed
layout(location = 4) out vec3 outLocalPosition;  // position on the body, metres above the feet
layout(location = 5) out vec3 outBoxSize;        // unused for characters
layout(location = 6) out vec3 outLocalNormal;    // the same as outNormal, as for people
layout(location = 7) out float outFreeBody;      // 0: characters are not free bodies

void main() {
    mat4 blended = inWeights.x * palette.matrices[inPaletteOffset + inJoints.x]
                 + inWeights.y * palette.matrices[inPaletteOffset + inJoints.y]
                 + inWeights.z * palette.matrices[inPaletteOffset + inJoints.z]
                 + inWeights.w * palette.matrices[inPaletteOffset + inJoints.w];

    vec3 posedPosition = (blended * vec4(inPosition, 1.0)).xyz;
    vec3 posedNormal = normalize(mat3(blended) * inNormal);
    vec3 worldPosition = (inWorld * vec4(posedPosition, 1.0)).xyz;
    vec3 worldNormal = normalize(mat3(inWorld) * posedNormal);

    outWorldPosition = worldPosition;
    outNormal = worldNormal;
    outLocalNormal = worldNormal;
    outFreeBody = 0.0;
    outAlbedo = inColorRoughness.rgb;
    outSurface = vec3(inColorRoughness.w, inMaterialSeed.x, inMaterialSeed.y);
    outLocalPosition = posedPosition;
    outBoxSize = vec3(1.0);
    gl_Position = push.viewProjection * vec4(worldPosition, 1.0);
}
