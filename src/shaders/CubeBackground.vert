#version 120      // inverse requires version 1.40

varying vec3 cube_direction;  // output direction for cubemap lookup

void main(void)
{
	// use the vertex XY as normalized screen coords and put z at the far plane
    gl_Position   =  vec4(gl_Vertex.xy, gl_DepthRange.far, 1.0);
    // we use 'transpose' instead of 'inverse' because we only have PS 1.2 at the moment
    // 'inverse' would work even with skewed trafos
    // it works because we remove the translation from the modelview mat
	cube_direction  =  (transpose(mat3(gl_ModelViewMatrix)) * (gl_ProjectionMatrixInverse * gl_Vertex).xyz).xzy;
}
