/* -*- Mode: C -*- */
#version 120
uniform sampler2D inputGeometryTexture;
uniform sampler2DRect p2g;
uniform bool show_geom_coords;
varying vec2 ProjectorCoord;

void main(void)
{
  float eps=1e-10;
  vec2 GeomCoord = texture2DRect(p2g, ProjectorCoord).xy;
  if ((GeomCoord.x <= eps) && (GeomCoord.y <= eps)) {
    discard;
  } else {
    if (show_geom_coords) {
      gl_FragData[0] = vec4(GeomCoord,0.0,1.0);
    } else {
      gl_FragData[0] = texture2D(inputGeometryTexture,GeomCoord);
    }
  }
}
