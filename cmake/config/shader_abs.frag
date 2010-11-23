uniform sampler2D texL;
uniform sampler2D texR;

void main()
{	
	vec4 texval0 = texture2D( texL, vec2(gl_TexCoord[0]));
	vec4 texval1 = texture2D( texR, vec2(gl_TexCoord[1]));
	
	gl_FragColor = abs(texval0 - texval1);
}
