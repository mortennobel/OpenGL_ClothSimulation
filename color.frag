#version 430

in vec4 colorV;
layout(location = 0) out vec4 fragColor;

void main(void)
{
    fragColor = colorV;
}