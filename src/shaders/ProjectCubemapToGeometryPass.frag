/* -*- Mode: C -*- */
#version 120
uniform samplerCube observerViewCube;
varying vec3 oVec; // vector from observer to vertex position

void main(void)
{
  //gl_FragData[0] = textureCube(observerViewCube, oVec);
  gl_FragData[0] = vec3(0,1,1,1);
}
