/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#version 120
uniform float edge_height;
varying float x;
varying float y;
varying float z;

void main(void)
{
    gl_FragColor = vec4(x+0.5, y+0.5, z, 1);
}
