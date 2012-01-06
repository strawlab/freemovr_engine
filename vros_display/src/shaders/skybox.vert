/* -*- Mode: C -*- */
#version 120
varying vec3 oVec;

void main(void)
{
  vec4 vert_world = gl_Vertex;
  vec4 vert_eye = gl_ModelViewMatrix * vert_world;
  vec4 vert_clip = gl_ProjectionMatrix * vert_eye;
  gl_Position	= vert_clip;
  oVec = normalize( gl_Vertex.xyz);
}

