#version 150

uniform mat4 mvp;
uniform mat3 normalMatrix;

in vec3 position;
in vec3 normal;
in vec4 color;

out vec4 vColor;
out vec3 vNormal;

void main ()
{
    vNormal = normalize(normalMatrix * normal);
    vColor = color;
    gl_Position = mvp * vec4(position, 1.0);
}