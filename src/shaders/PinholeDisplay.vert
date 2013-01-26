/* -*- Mode: C -*- */
#version 120
uniform mat4 modelview;
uniform mat4 projection;

void main(void)
{
  vec4 vert_world = gl_Vertex;
  vec4 vert_eye = modelview*vert_world;
  vec4 vert_clip = projection*vert_eye;
  gl_Position	= vert_clip;
}

