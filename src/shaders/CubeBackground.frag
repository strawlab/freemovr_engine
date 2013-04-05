
varying vec3  cube_direction;   // input direction for cubemap lookup
uniform samplerCube skybox;		// cubemap (Duh!)

void main(void)
{
    gl_FragColor = textureCube(skybox, cube_direction);
}
