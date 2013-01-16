/* -*- Mode: C -*- */
// #version 120 /* Works OK on VMWare v 1.2, but not nVidia 1.2 (but OK on nVidia 1.4 with minor changes) */
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
