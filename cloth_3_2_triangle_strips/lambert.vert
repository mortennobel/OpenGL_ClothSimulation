#version 150

uniform mat4 mvp;
uniform mat3 normalMatrix;

in vec3 position;
in vec3 normal;
in vec2 uv;

out vec2 vUV;
out vec3 vNormal;

void main ()
{
    vNormal = normalize(normalMatrix * normal);
    vUV = uv;
    gl_Position = mvp * vec4(position, 1.0);
}