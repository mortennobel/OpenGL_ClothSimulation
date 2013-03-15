varying vec4 vColor;
varying vec3 vNormal;
 
void main()
{
	vec3 n = normalize(vNormal);
	if (!gl_FrontFacing){
		n *= -1.0;
	}
    vec4 color =  gl_LightModel.ambient * vColor;
	for (int i=0;i<2;i++){
		vec4 diffuse = vColor * gl_LightSource[i].diffuse;
		color += vColor * gl_LightSource[i].ambient;

		vec3 lightDir = normalize(gl_LightSource[i].position.xyz);
		
		float NdotL = max(dot(n,lightDir),0.0);
		if (NdotL > 0.0) {
			color += diffuse * NdotL;
		}
	}
    gl_FragColor = color;
 
}