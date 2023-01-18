#version 330

in vec3 position;
in vec3 normal;
in vec4 color;
in vec2 texCoord;

out vec4 fragColor;

void main()
{
	// Compute position in clipping coords from texture coords
	gl_Position = vec4(2.0 * texCoord - 1.0, 0.0, 1.0);

	fragColor = color;
}

