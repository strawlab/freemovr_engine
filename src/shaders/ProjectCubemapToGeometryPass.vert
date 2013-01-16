/* -*- Mode: C -*- */
#version 120
uniform vec3 ObserverPosition;
varying vec3 oVec;

void main(void)
{
    // calculate direction from observer to vertex position
    oVec = gl_Vertex.xyz - ObserverPosition;

    // leave vertex position unchanged from normal OpenGL operation
    //gl_Position	= ftransform();

    // unwrap to texture coords
    gl_Position = vec4(gl_MultiTexCoord0.st*2-1, -1, 1.0);
}

