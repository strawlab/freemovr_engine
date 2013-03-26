import numpy as np

# ROS imports
import roslib; roslib.load_manifest('flyvr')
import simple_geom

def nan_shape_allclose( a,b, **kwargs):
    if a.shape != b.shape:
        return False
    good_a = ~np.isnan(a)
    good_b = ~np.isnan(b)
    if not np.alltrue( good_a == good_b):
        return False
    aa = a[good_a]
    bb = b[good_b]
    return np.allclose( aa, bb, **kwargs)

def test_cyl():
    base = {'x':0, 'y':0, 'z':0}
    axis = {'x':0, 'y':0, 'z':1}
    radius = 1
    cyl = simple_geom.Cylinder(base=base, axis=axis, radius=radius)

    # several look-at locations
    b=np.array([(0,0,0),
                (0,1,0),
                (0,-1,0),
                (0,0,10),
                (0,0,-10),
                ])

    # the look-from location is essentially (+inf,0,0)
    a=np.zeros( b.shape )
    a[:,0] = 1e10

    actual = cyl.get_first_surface(a,b)
    expected = np.array([(1,0,0),
                         (0,1,0),
                         (0,-1,0),
                         (np.nan,np.nan,np.nan),
                         (np.nan,np.nan,np.nan),
                         ])
    assert nan_shape_allclose( actual, expected)

def test_sphere():
    center = {'x':0, 'y':0, 'z':0}
    radius = 1
    sphere = simple_geom.Sphere(center=center, radius=radius)

    # several look-at locations
    b=np.array([
                (0,1,0),
                (0,2,0),
                ])

    # the look-from location is essentially (+inf,0,0)
    a=np.zeros( b.shape )
    a[:,0] = 10
    a[:,1] = 1

    actual = sphere.get_first_surface(a,b)
    expected = np.array([
                         (0,1,0),
                         (np.nan, np.nan, np.nan),
                         ])
    assert nan_shape_allclose( actual, expected)

def test_sphere2():
    center = {'x':0, 'y':0, 'z':0}
    radius = 1
    sphere = simple_geom.Sphere(center=center, radius=radius)

    # several look-at locations
    b=np.array([(0,0,0),
                (0,1,0),
                (0,-1,0),
                (0,0,1),
                (0,0,10),
                (0,0,-10),
                ])

    # the look-from location is essentially (+inf,0,0)
    a=np.zeros( b.shape )
    a[:,0] = 1e10

    actual = sphere.get_first_surface(a,b)
    expected = np.array([(1,0,0),
                         (0,1,0),
                         (0,-1,0),
                         (0,0,1),
                         (np.nan,np.nan,np.nan),
                         (np.nan,np.nan,np.nan),
                         ])
    assert nan_shape_allclose( actual, expected)

    actual_wcs = sphere.worldcoord2texcoord( actual )
    expected_wcs = np.array([[0, 0.5],
                             [0.25, 0.5],
                             [0.75, 0.5],
                             [0.0, 1.0],
                             [np.nan, np.nan],
                             [np.nan, np.nan],
                             ])
    assert nan_shape_allclose( actual_wcs, expected_wcs)
