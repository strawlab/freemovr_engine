import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import roslib
roslib.load_manifest('vros_display')
from simple_geom import Cylinder, Vec3

def make_xy_circle_at_z(z, r=0.5, n=1024):
    theta = np.linspace(0,2*np.pi,n+1)
    x=r*np.cos(theta)
    y=r*np.sin(theta)
    z=z*np.ones_like(x)
    verts = np.array([x,y,z])
    cond = ( y < -0.2 )
    #verts = verts[:,cond]
    return verts

def get_rotation(face):
    if face==0: # -X
        R = np.array( [[-1,0,0],
                       [0,01,0],
                       [0,0,1]])
    elif face==1: # -Y
        R = np.array( [[0,-1,0],
                       [-1,0,0],
                       [0,0,1]])
    elif face==2: # -Z
        R = np.array( [[0,0,-1],
                       [0,-1,0],
                       [1,0,0]])
    elif face==3: # +X
        R = np.array( [[1,0,0],
                       [0,-1,0],
                       [0,0,1]])
    elif face==4: # +Y
        R = np.array( [[0,1,0],
                       [1,0,0],
                       [0,0,1]])
    elif face==5: # +Z
        R = np.array( [[0,0,1],
                       [0,-1,0],
                       [-1,0,0]])
    return R

def to_cubemap(verts):
    verts_mag = np.sqrt(np.sum(verts**2,axis=0))
    verts_norm = verts/verts_mag
    x,y,z = verts_norm
    ax,ay,az = abs(verts_norm)
    conds = []
    _ = conds.append
    _( (ax >= ay) & (ax >= az) & (x <= 0) )# 0: -X
    _( (ay >  ax) & (ay >= az) & (y <= 0) )# 1: -Y
    _( (az >  ax) & (az >  ay) & (z <= 0) )# 2: -Z
    _( (ax >= ay) & (ax >= az) & (x >= 0) )# 3: +X
    _( (ay >  ax) & (ay >= az) & (y >= 0) )# 4: +Y
    _( (az >  ax) & (az >  ay) & (z >= 0) )# 5: +Z
    conds = np.array(conds)

    cube = np.nan*np.ones_like(verts)
    for face in range(6):
        cond = conds[face]
        if not np.sum(cond):
            continue
        R = get_rotation(face)

        # rotate this face's points into +X direction
        this_verts = np.dot(R,verts_norm[:,cond])

        # find spherical coords (r==1)
        inclination = np.arccos(this_verts[2])
        azimuth = np.arctan2( this_verts[1], this_verts[0] )

        # project spherical points onto plane at +X
        y = np.tan( azimuth )
        cosi = np.cos( inclination )
        r = np.sqrt( (1 + y**2 ) / ( 1 - cosi**2) )
        z = r*cosi

        assert np.alltrue( (y >= -1) & (y <= 1) & (z >= -1) & (z <= 1))

        cube[0,cond]=face
        cube[1,cond]=y
        cube[2,cond]=z
    return cube

def plot_cubemap(ax, cube):
    x0a=[3,2,1,1,0,1]
    y0a=[1,1,0,1,1,2]
    for face in range(6):
        cond = (cube[0]==face)
        y = cube[1,cond]/2.0+0.5
        z = cube[2,cond]/2.0+0.5
        x0 = x0a[face]
        y0 = y0a[face]
        ax.plot(x0+y, y0+z, 'k.', ms=0.3)

        ax.plot( [x0, x0+1, x0+1, x0,   x0],
                 [y0, y0,   y0+1, y0+1, y0] )
        ax.text( x0, y0, face )

def vec3(a,b,c):
    return dict(x=a, y=b, z=c)

if 1:
    verts_world1 = np.hstack([make_xy_circle_at_z(0.0), make_xy_circle_at_z(0.9), make_xy_circle_at_z(1.0)])
    #observer_pos = np.array([[0.22], [0.22], [0.9]])
    observer_pos = np.array([[0.4], [0], [0.95]])
    verts_observer = verts_world1-observer_pos

    verts_observer_cube = to_cubemap( verts_observer )

    cyl = Cylinder(base=vec3(0,0,0), axis=vec3(0,0,1), radius=0.5)
    observer_pos_all = np.repeat( observer_pos, verts_world1.shape[1], axis=1)
    wc = cyl.get_first_surface( observer_pos_all.T, verts_world1.T )
    tc = cyl.worldcoord2texcoord( wc )

    if 0:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter( verts_world1[0],
                    verts_world1[1],
                    verts_world1[2], )
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

    if 0:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter( verts_observer[0],
                    verts_observer[1],
                    verts_observer[2], )
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

    if 1:
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        plot_cubemap( ax, verts_observer_cube )
        ax.set_aspect('equal')
        fname = 'mpl_cubemap.png'
        fig.savefig(fname)
        print 'saved',fname

    if 1:
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        ax.plot( tc[:,0], tc[:,1], 'k.' )
        ax.set_aspect('equal')
        fname = 'mpl_geometry.png'
        fig.savefig(fname)
        print 'saved',fname
    plt.show()
