/* -*- Mode: C; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#version 120
void main(void)
{
    gl_Position	= gl_Vertex; // -1 to 1
    gl_TexCoord[0] = (gl_Vertex+1)*0.5; // 0 to 1
}
