import numpy as np

import matplotlib.pyplot as plt

from pymvg.plot_utils import plot_camera

def get_3d_verts(geom):
    allw = []
    res_u = 32
    res_v = 5
    for tc1 in np.linspace(0,1,res_v):
        tc = np.vstack( (
                np.linspace(0,1.,res_u),
                tc1*np.ones( (res_u,) ),
                )).T
        world = geom.model.texcoord2worldcoord(tc)
        allw.append(world)

    allw = np.concatenate(allw)
    return allw

def plot_camera( ax, display, scale=0.2):
    C = display.get_camcenter()
    C.shape=(3,)
    ax.plot( [C[0]], [C[1]], [C[2]], 'ko', ms=5 )

    world_coords = display.project_camera_frame_to_3d( [[scale,0,0],
                                                        [0,scale,0],
                                                        [0,0,scale],
                                                        [0,0,-scale],
                                                        [0,0,0],
                                                        [0,scale,0],
                                                        [0,0,scale]] )

    for i in range(3):
        c = 'rgb'[i]
        vv = world_coords[i]
        v = np.vstack( ([C],[vv]) )
        ax.plot( v[:,0], v[:,1], v[:,2], c+'-' )

    uv_raw = np.array([[0,0],
                       [0,display.height],
                       [display.width, display.height],
                       [display.width, 0],
                       [0,0]])
    pts3d_near = display.project_pixel_to_3d_ray( uv_raw, distorted=True, distance=0.1*scale)
    pts3d_far = display.project_pixel_to_3d_ray( uv_raw, distorted=True, distance=scale)
    # ring at near depth
    ax.plot( pts3d_near[:,0], pts3d_near[:,1], pts3d_near[:,2], 'k-' )
    # ring at far depth
    ax.plot( pts3d_far[:,0], pts3d_far[:,1], pts3d_far[:,2], 'k-' )
    # connectors
    for i in range(len(pts3d_near)-1):
        pts3d = np.vstack((pts3d_near[i,:],pts3d_far[i,:]))
        ax.plot( pts3d[:,0], pts3d[:,1], pts3d[:,2], 'k-' )

    ax.text( C[0], C[1], C[2], display.name )
    ax.text( pts3d_far[0,0], pts3d_far[0,1], pts3d_far[0,2], 'UL' )
