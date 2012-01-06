/* -*- Mode: C -*- */
#version 120
uniform samplerCube skybox;
varying vec3 oVec;

void main(void)
{
  gl_FragData[0] = textureCube(skybox, oVec);
}

