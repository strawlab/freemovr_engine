/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#version 120

varying float x;
varying float y;
varying float z;

void main(void)
{
  gl_Position=ftransform(); // standard OpenGL vertex transform
  x = gl_Vertex.x;
  y = gl_Vertex.y;
  z = gl_Vertex.z;
}
