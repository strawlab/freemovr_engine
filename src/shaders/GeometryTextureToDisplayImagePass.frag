#version 140
#extension GL_ARB_texture_rectangle : enable

uniform sampler2D inputGeometryTexture;
uniform sampler2DRect p2g;
uniform bool show_geom_coords;
uniform float display_gamma;
varying vec2 ProjectorCoord;

void main(void)
{
  float eps=1e-10;
  vec3 GeomCoord = texture2DRect(p2g, ProjectorCoord).xyz;
  if ((GeomCoord.x <= eps) && (GeomCoord.y <= eps)) {
    discard;
  } else {
    if (show_geom_coords) {
      gl_FragData[0] = vec4(GeomCoord.xy,0.0,1.0);
    } else if (display_gamma > 0) {
      // linearize texture, mask with 3rd channel and then gamma-correct it 
      vec3 colorG = pow(texture2D(inputGeometryTexture, GeomCoord.xy).rgb, vec3(2.2));
      gl_FragData[0] = vec4(pow(colorG*GeomCoord.z,  vec3(1.0 / display_gamma)), 1.0);
    } else {
      // no gamma correction
      gl_FragData[0] = texture2D(inputGeometryTexture,GeomCoord.xy);
    }
  }
}

/* -*- Mode: C -*- */
