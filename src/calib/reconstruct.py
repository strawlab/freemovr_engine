import json

import roslib;
roslib.load_manifest('flyvr')
roslib.load_manifest('tf')
import tf

import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate

import calib.visualization
import simple_geom

class PointCloudTransformer(object):
    def __init__(self, scale=None, translate=None, rotate=None):
        # set default values or take passed arguments
        if scale is None:
            self._s = 1.0
        else:
            self._s = scale

        if translate is None:
            self._t = np.zeros((1,3))
        else:
            self._t = translate

        if rotate is None:
            self._R = np.eye(3)
        else:
            self._R = rotate

        self._t = np.array(self._t)
        assert self._t.ndim == 2
        assert self._t.shape == (1,3)

        self._R = np.array(self._R)
        assert self._R.ndim == 2
        assert self._R.shape == (3,3)

    def save_as_json(self, fname):
        s = float(self._s)
        t = map( float, [i for i in self._t[0]])
        R = []
        for row in self._R:
            R.append( map( float, [i for i in row] ) )
        result = {'s':s,
                  't':t,
                  'R':R}
        buf = json.dumps(result)
        with open(fname,mode='w') as fd:
            fd.write(buf)

    @staticmethod
    def from_json(fname):
        with open(fname,mode='r') as fd:
            alignment = json.loads(fd.read())

        s=alignment['s']

        t=alignment['t']
        t = np.array(t)
        t.shape = (1,3)

        R=alignment['R']
        R = np.array(R)

        recon = PointCloudTransformer(scale=s, translate=t, rotate=R)
        return recon

    def move_cloud(self, arr):
        #do it lazily because arr is not homogenous coords
        new = np.dot(self._s * self._R,arr.T).T
        return new - self._t

    def get_transformation(self):
        return self._s,self._R,self._t

    def get_transformation_matrix(self):
        s,R,t = self.get_transformation()
        M = np.zeros((4,4),dtype=np.float)
        M[:3,:3] = s*R
        M[:3,3] = t
        M[3,3]=1.0
        return M

class CylinderPointCloudTransformer(PointCloudTransformer):
    """
    Transforms a cloud of points to be about the origin and helps with creating
    texture coordinates for a geometry
    """
    def __init__(self, cx,cy,cz,ax,ay,az,radius,arr):
        super(CylinderPointCloudTransformer,self).__init__()

        assert arr.ndim==2
        assert arr.shape[1]==3

        #we need to create a rotation matrix to apply to the 3d points to align them on z axis
        axis = (ax, ay, az)
        rotation_axis = np.cross(axis, (0, 0, 1))
        rotation_angle = simple_geom.angle_between_vectors((0, 0, 1), axis)
        rotation_quaternion = tf.transformations.quaternion_about_axis(rotation_angle, axis)

        print "FIXING SCALE " * 5
        self._s = 1.0/radius

        #rotate points
        Rh = tf.transformations.rotation_matrix(rotation_angle, rotation_axis)
        self._R = Rh[0:3,0:3]
        new = np.dot(self._s * self._R,arr.T).T

        #move this to the origin
        cx,cy,_ = np.mean(new,axis=0)
        _,_,zmin = np.min(new,axis=0)
        _,_,zmax = np.max(new,axis=0)
        self._t = np.array([[cx,cy,zmin]])

        self.cloud = new - self._t

        height = zmax - zmin

        self._cyl = simple_geom.Cylinder(
                        base=dict(x=0,y=0,z=0),
                        axis=dict(x=0,y=0,z=height),
                        radius=radius)

    def in_cylinder_coordinates(self, arr):
        return self._cyl.worldcoord2texcoord(arr)

    def get_geom_dict(self):
        return self._cyl.to_geom_dict()

def interpolate_pixel_cords(points_2d, values_1d, img_width, img_height, method='cubic', fill_value=np.nan):
    assert points_2d.ndim == 2
    assert values_1d.ndim == 1
    assert points_2d.shape[0] == values_1d.shape[0]

    if method == "none":
        res = np.zeros((img_height, img_width), dtype=np.float)
        res.fill(fill_value)
        for p,v in zip(points_2d, values_1d):
            res[p[1],p[0]] = v
    else:
        grid_y, grid_x = np.mgrid[0:img_height, 0:img_width]
        res = scipy.interpolate.griddata(
                    points_2d,
                    values_1d,
                    (grid_x, grid_y),
                    method=method,
                    fill_value=fill_value)

    return res


