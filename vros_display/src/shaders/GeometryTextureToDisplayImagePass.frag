/* -*- Mode: C -*- */
#version 140
uniform sampler2D inputGeometryTexture;
uniform sampler2DRect p2g;
uniform bool show_geom_coords;
in vec2 ProjectorCoord;
out vec4 MyFragColor;

void main(void)
{
  float eps=1e-10;
  vec2 GeomCoord = texture(p2g, ProjectorCoord).xy;
  if ((GeomCoord.x <= eps) && (GeomCoord.y <= eps)) {
    discard;
  } else {
    if (show_geom_coords) {
      MyFragColor = vec4(GeomCoord,0.0,1.0);
    } else {
      MyFragColor = texture(inputGeometryTexture,GeomCoord);
    }
  }
}
