# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-

# ROS imports
import roslib; roslib.load_manifest('vros_display')
import rosbag

# standard Python stuff
import json
import numpy as np

class Vec3:
    def __init__(self,x=0, y=0, z=0):
        self.x=x
        self.y=y
        self.z=z

    def to_dict(self):
        #this dict is usually used for serializing, and some libraries have trouble serializing
        #numpy types (im looking at you ROS parameter server API)
        return dict(x=float(self.x),y=float(self.y),z=float(self.z))

def point_dict_to_vec(d):
    return Vec3(**d)

def range_0_2pi(angle):
    """put angle in range [0,2*pi]"""
    # Given: np.fmod( -1, 3) == -1
    pi2 = 2*np.pi
    return np.fmod((np.fmod(angle,pi2) + pi2),pi2)

class ModelBase:
    def get_first_surface(self, a, b):
        """return point on surface closest to point a in direction of point b.

        a is Nx3 array of points
        b is Nx3 array of points

        return Nx3 array of points
        """
        raise NotImplementedError

    def to_geom_dict(self):
        raise NotImplementedError

class Cylinder(ModelBase):
    def __init__(self, base=None, axis=None, radius=None):
        self.base = point_dict_to_vec(base)
        self.axis = point_dict_to_vec(axis)
        self.radius = radius
        if self.base.x != 0 or self.base.y != 0 or self.base.z != 0:
            raise NotImplementedError("not tested when cylinder not at origin")
        if self.axis.x != 0 or self.axis.y != 0:
            raise NotImplementedError("only right cylinder currently supported")
        if self.axis.z <= 0:
            raise NotImplementedError("only cylinder z>0 currently supported")


        # keep in sync with DisplaySurfaceGeometry.cpp
        self._radius = radius
        self._matrix = np.eye(3) # currently we're forcing vertical cylinder, so this is OK
        self._height = self.axis.z - self.base.z
        self._base = np.expand_dims(np.array( (self.base.x, self.base.y, self.base.z) ),1)

    def to_geom_dict(self):
        return dict(
            axis=self.axis.to_dict(),
            base=self.base.to_dict(),
            radius=float(self.radius),
            model="cylinder")

    def texcoord2worldcoord(self,tc):
        # Parse inputs
        tc = np.array(tc,copy=False)
        assert tc.ndim==2
        assert tc.shape[1]==2
        tc = tc.T

        # keep in sync with DisplaySurfaceGeometry.cpp
        frac_theta = tc[0]
        frac_height = tc[1]

        angle = frac_theta * 2.0*np.pi
        c = np.cos(angle)
        s = np.sin(angle)
        r = self._radius

        vec = np.vstack((c*r, s*r, frac_height*self._height))
        result = np.dot( self._matrix, vec ) + self._base
        return result.T

    def worldcoord2texcoord(self,wc):
        # Parse inputs
        wc = np.array(wc,copy=False)
        assert wc.ndim==2
        assert wc.shape[1]==3
        wc = wc.T

        x,y,z = wc
        x0 = x-self.base.x
        y0 = y-self.base.y
        z0 = z-self.base.z

        angle = np.arctan2( y0, x0 )
        height = z0

        tc0 = range_0_2pi(angle)/(2*np.pi)
        tc1 = z0/self._height
        result = np.vstack((tc0,tc1))
        return result.T

    def get_first_surface(self,a,b):
        """return point on surface closest to point a in direction of point b.

        a is Nx3 array of points
        b is Nx3 array of points

        return Nx3 array of points
        """
        a = np.array(a,copy=False)
        assert a.ndim==2
        assert a.shape[1]==3
        inshape = a.shape

        b = np.array(b,copy=False)
        assert b.ndim==2
        assert b.shape[1]==3
        assert b.shape==inshape

        # Since our cylinder is upright, we project our line into 2D,
        # solve for the intersection with the circle.

        a = a.T
        b = b.T

        # Move so that cylinder base is at (0,0).
        ax = a[0] - self.base.x
        ay = a[1] - self.base.y
        az = a[2] - self.base.z

        bx = b[0] - self.base.x
        by = b[1] - self.base.y
        bz = b[2] - self.base.z

        del a, b

        # Now create vector between points a and b
        sx = bx-ax
        sy = by-ay
        sz = bz-az
        r = self.radius

        # Solve for the intersections between line and circle (see sympy_line_circle.py for math)
        t0 = (-ax*sx - ay*sy + (-ax**2*sy**2 + 2*ax*ay*sx*sy - ay**2*sx**2 + r**2*sx**2 + r**2*sy**2)**(0.5))/(sx**2 + sy**2)
        t1 = (ax*sx + ay*sy + (-ax**2*sy**2 + 2*ax*ay*sx*sy - ay**2*sx**2 + r**2*sx**2 + r**2*sy**2)**(0.5))/(-sx**2 - sy**2)
        tt = np.vstack((t0,t1))

        # We want t to be > 0 (in direction from camera center to
        # point) but the closest one, so the smallest value meeting
        # this criterion.

        tt[tt <= 0] = np.nan # behind camera - invalid

        # find Z coordinate of each intersection
        zz = az+sz*tt

        # intersections not on cylinder are invalid
        tt[zz < 0] = np.nan
        tt[zz > self.axis.z] = np.nan

        tmin = np.nanmin(tt, axis=0) # find closest to camera
        x = ax+sx*tmin
        y = ay+sy*tmin
        z = az+sz*tmin

        result = np.vstack((x,y,z)).T
        assert result.shape==inshape
        return result

class Sphere(ModelBase):
    def __init__(self, center=None, radius=None):
        self.center = point_dict_to_vec(center)
        self.radius = radius

        # keep in sync with DisplaySurfaceGeometry.cpp
        self._radius = radius
        self._center = np.expand_dims(np.array( (self.center.x, self.center.y, self.center.z) ),1)

    def to_geom_dict(self):
        return dict(
            center=self.center.to_dict(),
            radius=float(self.radius),
            model="sphere")

    def texcoord2worldcoord(self,tc):
        # Parse inputs
        tc = np.array(tc,copy=False)
        assert tc.ndim==2
        assert tc.shape[1]==2
        tc = tc.T

        # keep in sync with DisplaySurfaceGeometry.cpp
        frac_az = tc[0]
        frac_el = tc[1]

        az = frac_az * 2.0*np.pi # 0 - 2pi
        el = frac_el*np.pi - np.pi/2 # -pi/2 - pi/2

        ca = np.cos(az)
        sa = np.sin(az)

        ce = np.cos(el)
        se = np.sin(el)

        r = self._radius

        vec = np.vstack((r*ca*ce, r*sa*ce, r*se))
        result = vec + self._center
        return result.T

    def worldcoord2texcoord(self,wc):
        # Parse inputs
        wc = np.array(wc,copy=False)
        assert wc.ndim==2
        assert wc.shape[1]==3
        wc = wc.T

        x,y,z = wc
        x0 = x-self.center.x
        y0 = y-self.center.y
        z0 = z-self.center.z
        r = np.sqrt( x0**2 + y0**2 + z0**2 )

        az = np.arctan2( y0, x0 )
        el_rad = np.arcsin( z0/r )
        el = el_rad / np.pi + 0.5

        tc0 = range_0_2pi(az)/(2*np.pi)
        tc1 = el
        result = np.vstack((tc0,tc1))
        return result.T

    def get_first_surface(self,a,b):
        """return point on surface closest to point a in direction of point b.

        a is Nx3 array of points
        b is Nx3 array of points

        return Nx3 array of points
        """
        a = np.array(a,copy=False)
        assert a.ndim==2
        assert a.shape[1]==3
        inshape = a.shape

        b = np.array(b,copy=False)
        assert b.ndim==2
        assert b.shape[1]==3
        assert b.shape==inshape

        a = a.T
        b = b.T

        # Move so that sphere center is at (0,0).
        ax = a[0] - self.center.x
        ay = a[1] - self.center.y
        az = a[2] - self.center.z

        bx = b[0] - self.center.x
        by = b[1] - self.center.y
        bz = b[2] - self.center.z

        del a, b

        # Now create vector between points a and b
        sx = bx-ax
        sy = by-ay
        sz = bz-az
        r = self.radius

        # Solve for the intersections between line and sphere (see sympy_line_sphere.py for math)
        t0,t1 = [(ax*sx + ay*sy + az*sz + (-ax**2*sy**2 - ax**2*sz**2 + 2*ax*ay*sx*sy + 2*ax*az*sx*sz - ay**2*sx**2 - ay**2*sz**2 + 2*ay*az*sy*sz - az**2*sx**2 - az**2*sy**2 + r**2*sx**2 + r**2*sy**2 + r**2*sz**2)**(0.5))/(-sx**2 - sy**2 - sz**2), (-ax*sx - ay*sy - az*sz + (-ax**2*sy**2 - ax**2*sz**2 + 2*ax*ay*sx*sy + 2*ax*az*sx*sz - ay**2*sx**2 - ay**2*sz**2 + 2*ay*az*sy*sz - az**2*sx**2 - az**2*sy**2 + r**2*sx**2 + r**2*sy**2 + r**2*sz**2)**(0.5))/(sx**2 + sy**2 + sz**2)]

        tt = np.vstack((t0,t1))

        # We want t to be > 0 (in direction from camera center to
        # point) but the closest one, so the smallest value meeting
        # this criterion.

        tt[tt <= 0] = np.nan # behind camera - invalid

        tmin = np.nanmin(tt, axis=0) # find closest to camera
        x = ax+sx*tmin
        y = ay+sy*tmin
        z = az+sz*tmin

        result = np.vstack((x,y,z)).T
        assert result.shape==inshape
        return result

def get_distance_between_point_and_ray( c, a, b ):
    """return distance between point c and ray from a in direction of point b.

    c is Nx3 array of points
    a is Nx3 array of points
    b is Nx3 array of points

    return Nx3 array of points
    """
    c = np.array(c,copy=False)
    assert c.ndim==2
    assert c.shape[1]==3
    inshape = c.shape

    a = np.array(a,copy=False)
    assert a.ndim==2
    assert a.shape[1]==3
    assert a.shape==inshape

    b = np.array(b,copy=False)
    assert b.ndim==2
    assert b.shape[1]==3
    assert b.shape==inshape

    c = c.T
    a = a.T
    b = b.T

    # Move so that sphere center is at (0,0).
    ax = a[0] - c[0]
    ay = a[1] - c[1]
    az = a[2] - c[2]

    bx = b[0] - c[0]
    by = b[1] - c[1]
    bz = b[2] - c[2]

    del a, b

    # Now create vector between points a and b
    sx = bx-ax
    sy = by-ay
    sz = bz-az

    # See sympy_line_point.py
    t = -(ax*sx + ay*sy + az*sz)/(sx**2 + sy**2 + sz**2)
    t = np.max(t,0) # get point a if t opposite from b

    # find the point
    x = ax+sx*t
    y = ay+sy*t
    z = az+sz*t
    verts = np.vstack((x,y,z))

    # now find the distance
    dist = np.sqrt(np.sum((verts-c)**2,axis=0))
    return dist

class Geometry:
    def __init__(self, filename=None, geom_dict=None):
        if filename and not geom_dict:
            geom_dict = json.loads( open(filename).read() )
        elif geom_dict and not filename:
            pass
        else:
            raise Exception("must supply filename OR geometry dict")

        if geom_dict['model']=='cylinder':
            self.model = Cylinder(base=geom_dict['base'],
                                  axis=geom_dict['axis'],
                                  radius=geom_dict['radius'])
        elif geom_dict['model']=='sphere':
            self.model = Sphere(center=geom_dict['center'],
                                radius=geom_dict['radius'])
        else:
            raise ValueError("unknown model type: %s"%geom_dict['model'])

    def compute_for_camera_view(self, camera, what='world_coords'):
        shape = (camera.height, camera.width)
        y = np.expand_dims(np.arange(camera.height),1)
        x = np.expand_dims(np.arange(camera.width),0)

        XX, YY = np.broadcast_arrays(x, y)
        assert XX.shape == shape
        assert YY.shape == shape

        XX = XX.flatten()
        YY = YY.flatten()

        distorted = np.vstack((XX,YY)).T

        ray = camera.project_pixel_to_3d_ray(distorted, distorted=True )

        camcenter = camera.camcenter_like(ray)
        world_coords = self.model.get_first_surface(camcenter,ray)

        if what=='world_coords':
            rx, ry, rz = world_coords.T
            # reshape into image-sized arrays
            rx.shape = (camera.height, camera.width, 1)
            ry.shape = (camera.height, camera.width, 1)
            rz.shape = (camera.height, camera.width, 1)
            output = np.concatenate((rx,ry,rz),axis=2)
            assert output.shape == (camera.height, camera.width, 3)

        elif what == 'texture_coords':
            texcoords = self.model.worldcoord2texcoord(world_coords)
            tc0, tc1 = texcoords.T
            # reshape into image-sized arrays
            tc0.shape = (camera.height, camera.width, 1)
            tc1.shape = (camera.height, camera.width, 1)
            output = np.concatenate((tc0,tc1),axis=2)
            assert output.shape == (camera.height, camera.width, 2)
        return output

def angle_between_vectors(v1, v2):
    dot = np.dot(v1, v2)
    len_a = np.sqrt(np.dot(v1, v1))
    len_b = np.sqrt(np.dot(v2, v2))
    if len_a == 0 or len_b == 0:
        return 0
    return np.arccos(dot / (len_a * len_b))
