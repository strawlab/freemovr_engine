import numpy as np

# ROS imports
import roslib; roslib.load_manifest('flyvr')
import flyvr.simple_geom as simple_geom

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

def test_worldcoord_roundtrip():

    # PlanarRectangle
    ll = {'x':0, 'y':0, 'z':0}
    lr = {'x':1, 'y':0, 'z':0}
    ul = {'x':0, 'y':1, 'z':0}

    # Cylinder
    base = {'x':0, 'y':0, 'z':0}
    axis = {'x':0, 'y':0, 'z':1}
    radius = 1

    # Sphere
    center = {'x':0, 'y':0, 'z':0}
    radius = 1

    inputs = [ (simple_geom.PlanarRectangle, dict(lowerleft=ll, upperleft=ul, lowerright=lr)),
               (simple_geom.Cylinder, dict(base=base, axis=axis, radius=radius)),
               (simple_geom.Sphere, dict(center=center, radius=radius)),
               ]
    for klass, kwargs in inputs:
        yield check_worldcoord_roundtrip, klass, kwargs

def check_worldcoord_roundtrip(klass,kwargs):
    model = klass(**kwargs)

    eps = 0.001 # avoid 0-2pi wrapping issues on sphere and cylinder
    tc1 = np.array( [[eps,eps],
                     [eps,1-eps],
                     [1-eps,1-eps],
                     [1-eps,eps],
                     [0.5,eps],
                     [eps, 0.5]] )
    wc1 = model.texcoord2worldcoord(tc1)
    tc2 = model.worldcoord2texcoord(wc1)
    wc2 = model.texcoord2worldcoord(tc2)
    assert nan_shape_allclose( tc1, tc2)
    assert nan_shape_allclose( wc1, wc2 )

def disabled_tst_rect():
    # not finished yet
    ll = {'x':0, 'y':0, 'z':0}
    lr = {'x':1, 'y':0, 'z':0}
    ul = {'x':0, 'y':1, 'z':0}

    rect = simple_geom.PlanarRectangle(lowerleft=ll, upperleft=ul, lowerright=lr)

    # several look-at locations
    b=np.array([(0,0,0),
                (0,1,0),
                (0,-1,0),
                (0,0.5,0),
                (0,0,1),
                ])

    # the look-from location is essentially (0,0,+inf)
    a=np.zeros( b.shape )
    a[:,2] = 1e10

    actual = rect.get_first_surface(a,b)
    expected = np.array([(0,0,0),
                         (0,1,0),
                         (np.nan,np.nan,np.nan),
                         (0,0.5,0),
                         (0,0,0),
                         ])
    assert nan_shape_allclose( actual, expected)

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
