#!/usr/bin/env python
import numpy as np
import os

# ROS imports
import roslib; roslib.load_manifest('freemoovr')
import tf.transformations
from freemoovr.calib.fit_shapes import fit_cylinder

PLOT=int(os.environ.get('PLOT',0))
if PLOT:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

def mag(vec):
    return np.sqrt(np.sum(vec**2))

def norm(vec):
    return vec/mag(vec)

def generate_random_cylinder_points( base, up, r, n=10 ):
    # uniformly distribute points in height and theta
    base = np.array(base)
    up = np.array(up)
    assert base.shape==(3,)
    assert up.shape==(3,)
    rshape = (2,n)
    rnd_h_norm, rnd_theta_norm = np.random.uniform(size=rshape)

    uh = up[:,np.newaxis]*rnd_h_norm
    on_axis = base[:,np.newaxis] + uh
    rnd_theta = rnd_theta_norm*2*np.pi
    on_xy_circle = np.array([ r*np.cos(rnd_theta),
                              r*np.sin(rnd_theta),
                              np.zeros_like(rnd_theta) ])

    updir = norm(up)
    zaxis = np.array((0,0,1.0))
    assert np.allclose(mag(updir),1)
    assert np.allclose(mag(zaxis),1)
    r_axis = np.cross( updir, zaxis )
    cosa = np.dot( updir, zaxis )
    r_angle = np.arccos(cosa)

    quat = tf.transformations.quaternion_about_axis(  -r_angle, r_axis )
    R =  tf.transformations.quaternion_matrix(quat)[:3,:3]
    on_tilted_circle = np.dot( R, on_xy_circle)
    final = on_tilted_circle + on_axis
    return final

def test_cylinder_fit():
    base = np.array( (1.2, 3.4, 5.6) )
    up = np.array( (1.0,0,0.5) )
    radius = .05

    np.random.seed(3)
    pts = generate_random_cylinder_points( base, up, radius, n=5000 )

    if PLOT:
        fig = plt.figure()
        ax3d = fig.add_subplot(211, projection='3d')
        ax3d.plot( pts[0], pts[1], pts[2], '.' )
        ax3d.set_xlabel('x')
        ax3d.set_ylabel('y')
        ax3d.set_zlabel('z')

        if 1:
            # force equal aspect
            sz = mag(up)
            v = np.array( [[-sz,-sz,-sz],
                           [ sz,-sz,-sz],
                           [ sz, sz,-sz],
                           [-sz, sz,-sz],
                           [-sz,-sz, sz],
                           [ sz,-sz, sz],
                           [ sz, sz, sz],
                           [-sz, sz, sz]])
            v = v + base
            v = v.T
            ax3d.plot( v[0], v[1], v[2], 'k.' )
        ax3d = fig.add_subplot(212, projection='3d')
    else:
        ax3d = None

    r = fit_cylinder( pts, ax3d )
    if PLOT:
        ax3d.set_xlabel('PC1')
        ax3d.set_ylabel('PC2')
        ax3d.set_zlabel('PC3')

        plt.show()

    center = base + up*0.5
    # convention: z is positive
    if up[2] < 0:
        up = -up

    assert np.allclose( center, r['center'], rtol=1e-2, atol=1e-2 )
    assert np.allclose( up, r['cyl_axis'], rtol=1e-1, atol=1e-2 )
    assert np.allclose( radius, r['radius'], rtol=1e-2, atol=1e-2 )
