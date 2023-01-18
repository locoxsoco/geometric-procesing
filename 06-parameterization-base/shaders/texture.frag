#version 330

in vec4 fragColor;
in vec2 fTexCoord;

out vec4 outColor;

void main()
{
	vec2 grid = floor(10.0 * fTexCoord);
	float value = mod(grid.x + grid.y, 2.0);
	
	outColor = vec4(value) * fragColor;
}



