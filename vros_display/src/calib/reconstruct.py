import roslib;
roslib.load_manifest('vros_display')
roslib.load_manifest('tf')
import tf

import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate

import calib.visualization
import simple_geom

class CylinderPointCloudTransformer(object):
    """
    Transforms a cloud of points to be about the origin and helps with creating
    texture coordinates for a geometry
    """
    def __init__(self, cx,cy,cz,ax,ay,az,radius,arr):
        assert arr.ndim==2
        assert arr.shape[1]==3
        self._cloud_orig = arr

        #we need to create a rotation matrix to apply to the 3d points to align them on z axis
        axis = (ax, ay, az)
        rotation_axis = np.cross(axis, (0, 0, 1))
        rotation_angle = simple_geom.angle_between_vectors((0, 0, 1), axis)
        rotation_quaternion = tf.transformations.quaternion_about_axis(rotation_angle, axis)
        
        self._s = 1.0

        #rotate points
        Rh = tf.transformations.rotation_matrix(rotation_angle, rotation_axis)
        self._R = Rh[0:3,0:3]
        new = np.dot(self._R,arr.T).T

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

    def move_cloud(self, arr):
        new = np.dot(self._R,arr.T).T
        return new - self._t

    def in_cylinder_coordinates(self, arr):
        return self._cyl.worldcoord2texcoord(arr)

    def get_geom_dict(self):
        return self._cyl.to_geom_dict()
        
    def get_transformation(self):
        return self._s,self._R,self._t

    def get_transformation_matrix(self):
        s,R,t = self.get_transformation()
        M = np.zeros((4,4),dtype=np.float)
        M[:3,:3] = s*R
        M[:3,3] = t
        M[3,3]=1.0
        return M

def interpolate_pixel_cords(points_2d, values_1d, img_width, img_height, method='cubic', fill_value=np.nan):
    assert points_2d.ndim == 2
    assert values_1d.ndim == 1

    grid_y, grid_x = np.mgrid[0:img_height, 0:img_width]

    res = scipy.interpolate.griddata(
                points_2d,
                values_1d,
                (grid_x, grid_y),
                method=method,
                fill_value=fill_value)

    return res
        

