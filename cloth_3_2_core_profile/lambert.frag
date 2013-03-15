#version 150

uniform vec4 lightAmbient[2];
uniform vec4 lightDiffuse[2];
uniform vec4 lightPosition[2];
uniform vec4 lightModelAmbient;

in vec4 vColor;
in vec3 vNormal;
 
out vec4 fragColor;
 
void main()
{
	vec3 n = normalize(vNormal);
	if (!gl_FrontFacing){
		n *= -1.0;
	}
    vec4 color =  lightModelAmbient * vColor;
	for (int i=0;i<2;i++){
		vec4 diffuse = vColor * lightDiffuse[i];
		color += vColor * lightAmbient[i];

		vec3 lightDir = normalize(lightPosition[i].xyz);
		
		float NdotL = max(dot(n,lightDir),0.0);
		if (NdotL > 0.0) {
			color += diffuse * NdotL;
		}
	}
    fragColor = color;
}