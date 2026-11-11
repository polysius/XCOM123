#version 460

// One triangle that covers the whole screen, generated from the vertex index (no vertex buffer).
// Its depth is 0, the far plane with reversed depth, so the sky lands behind everything else.

layout(location = 0) out vec2 outUv;

void main() {
    vec2 uv = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    outUv = uv;
    gl_Position = vec4(uv * 2.0 - 1.0, 0.0, 1.0);
}
