/* -*- Mode: C -*- */
#version 120
varying vec3 oVec;

void main(void)
{
  vec4 vert_world = gl_Vertex;
  vec4 vert_eye = gl_ModelViewMatrix * vert_world;
  vec4 vert_clip = gl_ProjectionMatrix * vert_eye;

  mat3 rot90 = mat3( 1.0, 0.0, 0.0,
                     0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0);

  vec3 rotated_dir = rot90 * gl_Vertex.xyz;
  gl_Position	= vert_clip;
  oVec = normalize( rotated_dir.xyz);
}

