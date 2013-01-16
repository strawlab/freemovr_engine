/* -*- Mode: C; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4 -*- */
#version 120
uniform sampler2D BackgroundImage;

void main(void)
{
	gl_FragColor = texture2D(BackgroundImage, gl_TexCoord[0].xy);
}
