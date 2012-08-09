/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#version 120
uniform samplerCube observerViewCube;

void main(void)
{

	// texcoords go from 0.0 to 1.0
	// divide coord x at 0.0, 0.25, 0.50, 0.75, 1.0
	// divide coord y at 0.0, 0.25, 0.50, 0.75

	vec3 v;
	bool noface;

	noface=false;

	if (gl_TexCoord[0].y < 0.25) {
		// bottom row
		if ((0.25 <= gl_TexCoord[0].x) && (gl_TexCoord[0].x < 0.5)) {
			// -Z
			float s = (gl_TexCoord[0].x - 0.25) * 4.0;
			float t = (gl_TexCoord[0].y) * 4.0;
			float sc = 2*s-1;
			float tc = 2*t-1;
			v = vec3( tc, -sc, -1.0);
		} else {
			noface=true;
		}
	}
	else if ((0.25 <= gl_TexCoord[0].y) && (gl_TexCoord[0].y < 0.5)) {
		// middle row
		if ((0.0 <= gl_TexCoord[0].x) && (gl_TexCoord[0].x < 0.25)) {
			// +Y
			float s = (gl_TexCoord[0].x       ) * 4.0;
			float t = (gl_TexCoord[0].y - 0.25) * 4.0;
			float sc = 2*s-1;
			float tc = 2*t-1;
			v = vec3( sc, 1.0, tc);
		}
		else if ((0.25 <= gl_TexCoord[0].x) && (gl_TexCoord[0].x < 0.5)) {
			// +X
			float s = (gl_TexCoord[0].x - 0.25) * 4.0;
			float t = (gl_TexCoord[0].y - 0.25) * 4.0;
			float sc = 2*s-1;
			float tc = 2*t-1;
			v = vec3( 1.0, -sc, tc);
		}
		else if ((0.5 <= gl_TexCoord[0].x) && (gl_TexCoord[0].x < 0.75)) {
			// -Y
			float s = (gl_TexCoord[0].x - 0.5 ) * 4.0;
			float t = (gl_TexCoord[0].y - 0.25) * 4.0;
			float sc = 2*s-1;
			float tc = 2*t-1;
			v = vec3( -sc, -1.0, tc);
		}
		else if ((0.75 <= gl_TexCoord[0].x) && (gl_TexCoord[0].x < 1.0)) {
			// -X
			float s = (gl_TexCoord[0].x - 0.75) * 4.0;
			float t = (gl_TexCoord[0].y - 0.25) * 4.0;
			float sc = 2*s-1;
			float tc = 2*t-1;
			v = vec3( -1.0, sc, tc);
		} else {
			noface=true;
		}
	}
	else if ((0.5 <= gl_TexCoord[0].y) && (gl_TexCoord[0].y < 0.75)) {
		// top row
		if ((0.25 <= gl_TexCoord[0].x) && (gl_TexCoord[0].x < 0.5)) {
			// +Z
			float s = (gl_TexCoord[0].x - 0.25) * 4.0;
			float t = (gl_TexCoord[0].y - 0.5) * 4.0;
			float sc = 2*s-1;
			float tc = 2*t-1;
			v = vec3( -tc, -sc, 1.0);
		} else {
			noface=true;
		}
	}
	else {
		noface=true;
	}

	if (noface) {
		gl_FragColor = vec4(0.2, 0.2, 0.3, 1.0); // dark blue-gray
	} else {
		gl_FragColor = textureCube(observerViewCube, v);
	}
}
