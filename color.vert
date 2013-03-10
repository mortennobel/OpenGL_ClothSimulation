#version 430

uniform vec2 p;

layout(location = 0) in vec4 position;
layout(location = 1) in vec4 color;

out vec4 colorV;

void main (void)
{
    colorV = color;
    gl_Position = vec4(p, 0.0, 0.0) + position;
}