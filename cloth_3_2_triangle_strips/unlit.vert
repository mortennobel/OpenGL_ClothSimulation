#version 150

uniform mat4 mvp;

in vec3 color;
in vec3 position; 
out vec3 vColor;

void main ()
{
    vColor = color;
    gl_Position = mvp * vec4(position, 1.0);
}