import numpy as np
import yaml

# ROS imports
import roslib; roslib.load_manifest('flyvr')
import flyvr.simple_geom as simple_geom
from pymvg.camera_model import CameraModel

def get_sample_camera():
    yaml_str = """header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
height: 494
width: 659
distortion_model: plumb_bob
D:
  - -0.331416226762
  - 0.143584747016
  - 0.00314558656669
  - -0.00393597842852
  - 0.0
K:
  - 516.385667641
  - 0.0
  - 339.167079537
  - 0.0
  - 516.125799368
  - 227.379935241
  - 0.0
  - 0.0
  - 1.0
R:
  - 1.0
  - 0.0
  - 0.0
  - 0.0
  - 1.0
  - 0.0
  - 0.0
  - 0.0
  - 1.0
P:
  - 444.369750977
  - 0.0
  - 337.107817516
  - 0.0
  - 0.0
  - 474.186859131
  - 225.062742824
  - 0.0
  - 0.0
  - 0.0
  - 1.0
  - 0.0
binning_x: 0
binning_y: 0
roi:
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False"""
    d = yaml.load(yaml_str)
    cam1 = CameraModel.from_dict(d,extrinsics_required=False)

    eye = (10,20,30)
    lookat = (11,20,30)
    up = (0,-1,0)
    cam = cam1.get_view_camera(eye, lookat, up)

    return cam

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

def test_models():
    # PlanarRectangle
    ll = {'x':0, 'y':0, 'z':0}
    lr = {'x':1, 'y':0, 'z':0}
    ul = {'x':0, 'y':1, 'z':0}

    # Cylinder
    base = {'x':0, 'y':0, 'z':0}
    axis = {'x':0, 'y':0, 'z':1}
    radius = 1

    # Sphere
    center = {'x':1.23, 'y':4.56, 'z':7.89}
    radius = 1

    inputs = [ (simple_geom.PlanarRectangle, dict(lowerleft=ll, upperleft=ul, lowerright=lr)),
               (simple_geom.Cylinder, dict(base=base, axis=axis, radius=radius)),
               (simple_geom.Sphere, dict(center=center, radius=radius)),
               ]
    for klass, kwargs in inputs:
        yield check_worldcoord_roundtrip, klass, kwargs
        yield check_surface_intersection, klass, kwargs

def check_surface_intersection(klass,kwargs):
    model = klass(**kwargs)

    a = np.array([[  0,   0,    0],
                  [100, 100,    0],
                  [100,   0, -100],
                  [  0,   0,    1],
                  ])
    b = np.array([ model.get_center() ]*len(a))
    surf = model.get_first_surface(a,b)
    rel_dist = model.get_relative_distance_to_first_surface(a,b)
    s = b-a
    dist = np.sqrt(np.sum((rel_dist[:,np.newaxis]*s)**2,axis=1))
    dist_actual = np.sqrt(np.sum((a-surf)**2,axis=1))

    assert nan_shape_allclose( dist, dist_actual)

def check_worldcoord_roundtrip(klass,kwargs):
    model = klass(**kwargs)

    eps = 0.001 # avoid 0-2pi wrapping issues on sphere and cylinder
    u = np.expand_dims(np.linspace(eps,1-eps,20.),1)
    v = np.expand_dims(np.linspace(eps,1-eps,20.),0)
    U, V = np.broadcast_arrays(u,v)
    tc1 = np.vstack((U.flatten(),V.flatten())).T
    wc1 = model.texcoord2worldcoord(tc1)
    tc2 = model.worldcoord2texcoord(wc1)
    wc2 = model.texcoord2worldcoord(tc2)

    # for some models, not every texcoord is valid
    bad = np.isnan(wc1[:,0])
    tc1_valid = np.array( tc1, copy=True )
    tc1_valid[ bad, : ] = np.nan

    assert nan_shape_allclose( tc1_valid, tc2 )
    assert nan_shape_allclose( wc1, wc2, atol=1e-7)

def test_rect():
    ll = {'x':0, 'y':0, 'z':0}
    lr = {'x':1, 'y':0, 'z':0}
    ul = {'x':0, 'y':1, 'z':0}

    rect = simple_geom.PlanarRectangle(lowerleft=ll, upperleft=ul, lowerright=lr)

    zval = 20.0
    # several look-at locations
    b=np.array([(0,0,0),
                (0,1,0),
                (0,-1,zval),
                (0,0.5,0),
                (0,0,1),
                (0,0,2*zval),
                (0,0,1.0001*zval),
                ])

    a=np.zeros( b.shape )
    a[:,2] = zval

    actual = rect.get_first_surface(a,b)
    expected = np.array([(0,0,0),
                         (0,1,0),
                         (np.nan,np.nan,np.nan),
                         (0,0.5,0),
                         (0,0,0),
                         (np.nan,np.nan,np.nan),
                         (np.nan,np.nan,np.nan),
                         ])

    assert nan_shape_allclose( actual, expected)

    actual_norms = rect.worldcoord2normal( actual )
    expected_norms = np.array([(0,0,1),
                               (0,0,1),
                               (np.nan,np.nan,np.nan),
                               (0,0,1),
                               (0,0,1),
                               (np.nan,np.nan,np.nan),
                               (np.nan,np.nan,np.nan),
                               ])
    assert nan_shape_allclose( actual_norms, expected_norms)

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

    actual_norms = cyl.worldcoord2normal( actual )
    expected_norms = expected
    assert nan_shape_allclose( actual_norms, expected_norms)


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

    actual_tcs = sphere.worldcoord2texcoord( actual )
    expected_tcs = np.array([[0, 0.5],
                             [0.25, 0.5],
                             [0.75, 0.5],
                             [0.0, 1.0],
                             [np.nan, np.nan],
                             [np.nan, np.nan],
                             ])
    assert nan_shape_allclose( actual_tcs, expected_tcs)

    actual_norms = sphere.worldcoord2normal( actual )
    expected_norms = expected
    assert nan_shape_allclose( actual_norms, expected_norms)


def test_geom_class():
    cam = get_sample_camera()

    d = {'model':'cylinder',
         'base':{'x':0,'y':0,'z':0},
         'axis':{'x':0,'y':0,'z':1},
         'radius':1.0}
    geom = simple_geom.Geometry(geom_dict=d)
    wcs = geom.compute_for_camera_view(cam,'world_coords')
    tcs = geom.compute_for_camera_view(cam,'texture_coords')
    dist = geom.compute_for_camera_view(cam,'distance')
    angle = geom.compute_for_camera_view(cam,'incidence_angle')
