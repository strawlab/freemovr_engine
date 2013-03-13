/* -*- Mode: C -*- */
//#version 140 // Keep this commented out so that we trick Intel drivers into accepting as v1.2 and NVidia as 1.4

#extension GL_ARB_texture_rectangle : enable

uniform sampler2D inputGeometryTexture;
uniform sampler2DRect p2g;
uniform bool show_geom_coords;
uniform float display_gamma;
varying vec2 ProjectorCoord;

void main(void)
{
  float eps=-0.5;
  vec3 GeomCoord = texture2DRect(p2g, ProjectorCoord).xyz;
  if ((GeomCoord.x <= eps) && (GeomCoord.y <= eps)) {
    discard;
  } else {
    if (show_geom_coords) {
      gl_FragData[0] = vec4(GeomCoord.xy,0.0,1.0);
    } else {
      vec3 color = texture2D(inputGeometryTexture, GeomCoord.xy).rgb; // lookup texture
      vec3 luminance_corrected = color*GeomCoord.z;                   // multiply with luminance mask
      gl_FragData[0] = vec4(pow(luminance_corrected,  vec3(1.0 / display_gamma)), 1.0); // correct for display gamma
    }
  }
}
