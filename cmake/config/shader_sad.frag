uniform sampler2D tex1;
uniform sampler2D tex2;
uniform sampler2D tex4;
uniform sampler2D tex8;
uniform sampler2D tex16;
uniform sampler2D tex32;
uniform float disp;

void main(void)
{
	vec4 texval1 = texture2D( tex1, vec2(gl_TexCoord[0]));
	vec4 texval2 = texture2D( tex2, vec2(gl_TexCoord[0]));
	vec4 texval4 = texture2D( tex4, vec2(gl_TexCoord[0]));
	vec4 texval8 = texture2D( tex8, vec2(gl_TexCoord[0]));
	vec4 texval16 = texture2D( tex16, vec2(gl_TexCoord[0]));
	vec4 texval32 = texture2D( tex32, vec2(gl_TexCoord[0]));
	
	vec4 sad = texval1 + texval2 + 1.0*texval4 + 1.0*texval8 + 1.0*texval16 + 1.0*texval32;
	vec4 c = vec4(disp,disp,disp,1.0);
	gl_FragDepth = sad.r / (1.0 + sad.r);
	gl_FragColor = c;
}