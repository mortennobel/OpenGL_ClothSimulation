varying vec4 vColor;
varying vec3 vNormal;

void main ()
{
    vNormal = normalize(gl_NormalMatrix * gl_Normal);
    vColor = gl_Color;
    gl_Position = ftransform();
}