import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import scipy.misc

import roslib
roslib.load_manifest('flyvr')
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

def from_cubemap(cube):
    result = np.zeros_like(cube)
    for face in range(6):
        cond = cube[0]==face
        y = cube[1,cond]
        z = cube[2,cond]
        x = np.ones_like(y)

        v = np.array([x,y,z])
        R = get_rotation(face).T
        v2 = np.dot(R,v)
        result[:,cond] = v2
    return result

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

        line,=ax.plot( [x0, x0+1, x0+1, x0,   x0],
                       [y0, y0,   y0+1, y0+1, y0], lw=5 )
        ax.text( x0, y0, face )

def generate_text_cube():
    verts = []
    for face in range(6):
        Y,Z = np.mgrid[-1:1:20j, -1:1:20j]
        FACE = face*np.ones_like(Y)
        fv = np.array( [FACE.ravel(), Y.ravel(), Z.ravel() ])
        verts.append(fv)
    verts = np.hstack(verts)
    faces = verts[0]
    v3 = from_cubemap(verts)
    return v3, faces

def vec3(a,b,c):
    return dict(x=a, y=b, z=c)

def normalize(a):
    assert a.ndim==2
    assert a.shape[1]==3
    mag = np.array([np.sqrt(np.sum(a**2,axis=1))]).T
    result = a/mag

    for row in [0,30, 440*512 + 100]:
        print 'row %d: orig: (%.2f, %.2f, %.2f), mag: %.2f'%(row,
                                                             float(a[row,0]),
                                                             float(a[row,1]),
                                                             float(a[row,2]),
                                                             float(mag[row]))
        print '  normalized: (%.2f, %.2f, %.2f)'%(result[row,0],
                                                  result[row,1],
                                                  result[row,2])
        print '  len %.2f'%(np.sqrt(np.sum(result[row]**2)),)
        result[row] = [0,0,1]
    return result

if 1:
    verts_world1 = np.hstack([make_xy_circle_at_z(0.0), make_xy_circle_at_z(0.9), make_xy_circle_at_z(1.0)])
    observer_pos = np.array([[0.22], [0.22], [0.9]])
    #observer_pos = np.array([[0.4], [0], [0.95]])
    #observer_pos = np.array([[0.0], [0.0], [0.0]])
    verts_observer = verts_world1-observer_pos

    verts_observer_cube = to_cubemap( verts_observer )

    cyl = Cylinder(base=vec3(0,0,0), axis=vec3(0,0,1), radius=0.5)

    mode = 'cubemap'
    if mode=='cubemap':
        v3, faces = generate_text_cube()
        observer_pos_all = np.repeat( observer_pos, v3.shape[1], axis=1)

        wc = cyl.get_first_surface( observer_pos_all.T, v3.T )
        tc = cyl.worldcoord2texcoord( wc )
    elif mode=='edge':
        observer_pos_all = np.repeat( observer_pos, verts_world1.shape[1], axis=1)
        wc = cyl.get_first_surface( observer_pos_all.T, verts_world1.T )
        tc = cyl.worldcoord2texcoord( wc )

    rez = 512
    V,U = np.mgrid[0:1:rez*1j,0:1:rez*1j]
    tc_grid = np.array([U.ravel(),V.ravel()]).T
    dense_geom = cyl.texcoord2worldcoord(tc_grid)
    diff = dense_geom-observer_pos[:,0]
    #oVec = normalize(diff)
    oVec = diff
    oVec = diff+1

    geom_image = np.zeros( (rez,rez,3) )
    if 0:
        geom_image[:, :, 0].flat = diff[:,0]
        geom_image[:, :, 1].flat = diff[:,1]
        geom_image[:, :, 2].flat = diff[:,2]

    elif 0:
        geom_image[:, :, 0].flat = U.flat
        geom_image[:, :, 1].flat = 0
        geom_image[:, :, 2].flat = 0
    elif 0:
        # show raw cylinder geometry
        geom_image[:, :, 0].flat = dense_geom[:,0]
        geom_image[:, :, 1].flat = 0
        geom_image[:, :, 2].flat = 0
    elif 1:
        # show direction of cylinder from observer
        geom_image[:, :, 0].flat = oVec[:,0]#*0.5 + 0.5
        geom_image[:, :, 1].flat = oVec[:,1]#*0.5 + 0.5
        geom_image[:, :, 2].flat = oVec[:,2]#*0.5 + 0.5

    if 0:
        for row in [0, 127, 128, 129, 130]:#32, 94, 493, 9503]:
            uv = tc_grid[row]
            orig = dense_geom[row]

            i0 = row%rez
            i1 = row//rez
            xyz2 = geom_image[ i1, i0, :]
            print 'row %d: (%s) -> (%s) -> (%s) -> (%s)'%(row,uv,orig,diff[row], xyz2)

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

    if 0:
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
        if mode=='cubemap':
            for face in range(6):
                cond = faces==face
                ax.plot( tc[cond,0],
                         tc[cond,1], '.-' )
        elif mode=='edge':
            ax.plot( tc[:,0], tc[:,1], 'k.' )
        else:
            pass
        ax.set_aspect('equal')
        fname = 'mpl_geometry.png'
        fig.savefig(fname,transparent=True)
        print 'saved',fname

    if 0:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for face in range(6):
            cond = faces==face
            ax.plot( v3[:,cond][0],
                        v3[:,cond][1],
                        v3[:,cond][2], )
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')


    if 1:
        geom_image = np.clip(geom_image,0,1)
        gi2 = (geom_image*255).astype(np.uint8)
        fname = 'geom_image.png'
        scipy.misc.imsave(fname,gi2[::-1])
        print 'saved',fname
    if 0:
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        ax.imshow( (geom_image[:,:,2] > 0.05), origin='lower', interpolation='nearest')
    #plt.show()
