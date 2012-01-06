/* -*- Mode: C -*- */
#version 120
varying vec2 ProjectorCoord;

void main(void)
{
  gl_Position=ftransform();
  ProjectorCoord = gl_MultiTexCoord0.xy;
}
