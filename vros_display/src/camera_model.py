#!/usr/bin/env python

# ROS imports
import roslib; roslib.load_manifest('vros_display')
import rosbag
import tf.transformations
import vros_display.msg
import sensor_msgs

import numpy as np
import warnings

# helper functions ---------------

def point_msg_to_tuple(d):
    return d.x, d.y, d.z

def parse_rotation_msg(rotation, force_matrix=False):
    # rotation could either be a quaternion or a 3x3 matrix

    if hasattr(rotation,'x') and hasattr(rotation,'y') and hasattr(rotation,'z') and hasattr(rotation,'w'):
        # convert quaternion message to tuple
        rotation = quaternion_msg_to_tuple(rotation)

    if len(rotation)==4:
        if force_matrix:
            rotation = tf.transformations.quaternion_matrix(rotation)[:3,:3]
        return rotation

    if len(rotation) != 9:
        raise ValueError('expected rotation to be a quaternion or 3x3 matrix')
    rotation = np.array( rotation )
    rotation.shape = 3,3
    return rotation

def quaternion_msg_to_tuple(d):
    return d.x, d.y, d.z, d.w

def _undistort( xd, yd, D):
    # See OpenCV modules/imgprc/src/undistort.cpp
    x = np.array(xd,copy=True)
    y = np.array(yd,copy=True)

    k1, k2, t1, t2, k3 = D[:5]
    k = list(D)
    if len(k)==5:
        k = k + [0,0,0]

    for i in range(5):
        r2 = x*x + y*y
        icdist = (1 + ((k[7]*r2 + k[6])*r2 + k[5])*r2)/(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2)
        delta_x = 2.0 * (t1)*x*y + (t2)*(r2 + 2.0*x*x)
        delta_y = (t1) * (r2 + 2.0*y*y)+2.0*(t2)*x*y
        x = (xd-delta_x)*icdist
        y = (yd-delta_y)*icdist
    return x,y


def my_rq(M):
    """RQ decomposition, ensures diagonal of R is positive"""
    import scipy.linalg
    R,K = scipy.linalg.rq(M)
    n = R.shape[0]
    for i in range(n):
        if R[i,i]<0:
            # I checked this with Mathematica. Works if R is upper-triangular.
            R[:,i] = -R[:,i]
            K[i,:] = -K[i,:]
    return R,K

def center(P):
    orig_determinant = np.linalg.det
    def determinant( A ):
        return orig_determinant( np.asarray( A ) )
    # camera center
    X = determinant( [ P[:,1], P[:,2], P[:,3] ] )
    Y = -determinant( [ P[:,0], P[:,2], P[:,3] ] )
    Z = determinant( [ P[:,0], P[:,1], P[:,3] ] )
    T = -determinant( [ P[:,0], P[:,1], P[:,2] ] )

    C_ = np.array( [[ X/T, Y/T, Z/T ]] ).T
    return C_

def is_rotation_matrix(R):
    # check if rotation matrix is really a pure rotation matrix
    testI = np.dot(R.T,R)
    if not np.allclose( testI, np.eye(len(R)) ):
        return False
    dr = abs(np.linalg.det(R))
    if not np.allclose(dr,1):
        return False
    return True

# main class

class CameraModel:
    def __init__(self,
                 translation=None,
                 rotation=None,
                 intrinsics=None,
                 name=None,
                 ):
        if translation is None:
            translation = (0,0,0)
        if rotation is None:
            rotation = np.eye(3)
        if name is None:
            name = 'camera'

        if 1:
            # Initialize the camera calibration from a CameraInfo message.
            msg = intrinsics
            self.width = msg.width
            self.height = msg.height
            shape = (msg.height, msg.width)

            self.P = np.array(msg.P,dtype=np.float)
            self.P.shape = (3,4)
            if not np.allclose(self.P[:,3], np.zeros((3,))):
                raise NotImplementedError('not tested when 4th column of P is nonzero')

            self.K = np.array( msg.K, dtype=np.float)
            self.K.shape = (3,3)
            assert self.K.ndim == 2

            self.distortion = np.array(msg.D, dtype=np.float)
            if len(self.distortion) == 5:
                self.distortion.shape = (5,1)
            elif len(self.distortion) == 8:
                self.distortion.shape = (8,1)
            else:
                raise ValueError('distortion can have only 5 or 8 entries')

            assert self.distortion.ndim==2

            self.rect = np.array( msg.R, dtype=np.float )
            self.rect.shape = (3,3)
            if np.allclose(self.rect,np.eye(3)):
                self.rect = None

        self.translation=np.array(translation,copy=True)
        rotation = np.array(rotation)
        if rotation.ndim==2:
            assert rotation.shape==(3,3)
            self.rot = rotation
        else:
            assert rotation.ndim==1
            assert rotation.shape==(4,)
            self.rot = tf.transformations.quaternion_matrix(rotation)[:3,:3]
        assert is_rotation_matrix(self.rot)

        self.name = name

        t = np.array(self.translation)
        t.shape = 3,1
        self.Rt = np.hstack((self.rot,t))

        K = self.P[:3,:3]

        self.pmat = np.dot( K, self.Rt )

        # Cache a few values
        self.rot_inv = np.linalg.pinv(self.rot)
        self.t_inv = -np.dot( self.rot_inv, t )
        self._opencv_compatible = (K[0,1]==0)

        # And a final check
        if K[0,1] != 0.0:
            if np.sum(abs(self.distortion)) != 0.0:
                raise NotImplementedError('distortion/undistortion for skewed pixels not implemented')

    # -------------------------------------------------
    # getters

    def is_opencv_compatible(self):
        """True iff there is no skew"""
        return self._opencv_compatible

    def get_name(self):
        return self.name

    def get_extrinsics_as_msg(self):
        msg = vros_display.msg.MatrixTransform()
        msg.translation.x, msg.translation.y, msg.translation.z = self.translation
        msg.rotation = list(self.rot.flatten())
        return msg
    def get_intrinsics_as_msg(self):
        i = sensor_msgs.msg.CameraInfo()
        # these are from image_geometry ROS package in the utest.cpp file
        i.height = self.height
        i.width = self.width
        i.distortion_model = 'plumb_bob'
        i.D = list(self.distortion.flatten())
        i.K = list(self.K.flatten())
        i.R = list(self.get_rect().flatten())
        i.P = list(self.P.flatten())
        return i

    def get_camcenter(self):
        return self.t_inv[:,0] # drop dimension

    def get_rotation(self):
        return self.rot

    def get_translation(self):
        return self.translation

    def get_K(self):
        return self.K

    def get_D(self):
        return self.distortion

    def get_rect(self):
        if self.rect is None:
            return np.eye(3)
        else:
            return self.rect

    def get_P(self):
        return self.P

    def fx(self):
        return self.P[0,0]

    def fy(self):
        return self.P[1,1]

    def cx(self):
        return self.P[0,2]

    def cy(self):
        return self.P[1,2]

    def Tx(self):
        return self.P[0,3]

    def Ty(self):
        return self.P[1,3]

    def save_to_bagfile(self,fname):
        bagout = rosbag.Bag(fname, 'w')
        topic = self.name + '/tf'
        bagout.write(topic, self.get_extrinsics_as_msg())
        topic = self.name + '/camera_info'
        bagout.write(topic, self.get_intrinsics_as_msg())
        bagout.close()

    def get_mirror_camera(self):
        """return a copy of this camera whose x coordinate is (image_width-x)"""
        if 0:

            # Method 1: flip the extrinsic coordinates to a LH
            # system. (Adjust camera center for distortion.)

            # Implementation note: I guess this should work, but it is
            # not quite right for some reason.

            flipr = np.eye(3)
            flipr[0,0] = -1
            rnew = np.dot(flipr,self.rot)
            C = self.get_camcenter()
            tnew = -np.dot(rnew, C)
            i = self.get_intrinsics_as_msg()
            i.K[2] = (self.width-i.K[2])
            i.P[2] = (self.width-i.P[2])
            camnew = CameraModel( translation = tnew,
                                  rotation = rnew,
                                  intrinsics = i,
                                  name = self.name + '_mirror',
                                  )
            return camnew
        elif 1:

            # Method 2: keep extrinsic coordinates, but flip intrinsic
            # parameter so that a mirror image is rendered.

            i = self.get_intrinsics_as_msg()
            i.K[0] = -i.K[0]
            i.P[0] = -i.P[0]

            # Now, do we flip about optical center or just the X
            # coordinate?

            if 1:

                # This flips the X coordinate but preserves the
                # optical center.

                i.K[2] = (self.width-i.K[2])
                i.P[2] = (self.width-i.P[2])

            camnew = CameraModel( translation = self.translation,
                                  rotation = self.rot,
                                  intrinsics = i,
                                  name = self.name + '_mirror',
                                  )
            return camnew



    # --------------------------------------------------
    # image coordinate operations

    def undistort(self, nparr):
        # See http://opencv.willowgarage.com/documentation/cpp/camera_calibration_and_3d_reconstruction.html#cv-undistortpoints

        # Parse inputs
        nparr = np.array(nparr,copy=False)
        assert nparr.ndim==2
        assert nparr.shape[1]==2

        if np.sum(abs(self.distortion)) == 0.0:
            # no distortion necessary, just copy inputs
            return np.array(nparr,copy=True)

        u = nparr[:,0]
        v = nparr[:,1]

        # prepare parameters
        K = self.get_K()

        fx = K[0,0]
        cx = K[0,2]
        fy = K[1,1]
        cy = K[1,2]

        # P=[fx' 0 cx' tx; 0 fy' cy' ty; 0 0 1 tz]

        P = self.get_P()
        fxp = P[0,0]
        cxp = P[0,2]
        fyp = P[1,1]
        cyp = P[1,2]

        # Apply intrinsic parameters to get normalized, distorted coordinates
        xpp = (u-cx)/fx
        ypp = (v-cy)/fy

        # Undistort
        (xp,yp) = _undistort( xpp, ypp, self.get_D() )

        # Now rectify
        R = self.rect
        if R is None:
            x = xp
            y = yp
        else:
            assert R.shape==(3,3)
            uh = np.vstack( (xp,yp,np.ones_like(xp)) )
            XYWt = np.dot(R, uh)
            X = XYWt[0,:]
            Y = XYWt[1,:]
            W = XYWt[2,:]
            x = X/W
            y = Y/W

        # Finally, get (undistorted) pixel coordinates
        up = x*fxp + cxp
        vp = y*fyp + cyp

        return np.vstack( (up,vp) ).T

    def distort(self, nparr):
        # See http://opencv.willowgarage.com/documentation/cpp/camera_calibration_and_3d_reconstruction.html#cv-undistortpoints

        # Based on code in pinhole_camera_model.cpp of ROS image_geometry package.

        # Parse inputs
        nparr = np.array(nparr,copy=False)
        assert nparr.ndim==2
        assert nparr.shape[1]==2

        if np.sum(abs(self.distortion)) == 0.0:
            # no distortion necessary, just copy inputs
            return np.array(nparr,copy=True)

        uv_rect_x = nparr[:,0]
        uv_rect_y = nparr[:,1]

        # prepare parameters
        P = self.get_P()

        fx = P[0,0]
        cx = P[0,2]
        Tx = P[0,3]
        fy = P[1,1]
        cy = P[1,2]
        Ty = P[1,3]

        x = (uv_rect_x - cx - Tx)/fx
        y = (uv_rect_y - cy - Ty)/fy

        if self.rect is not None:
            R = self.rect.T
            xy1 = np.vstack((x,y,np.ones_like(x)))
            X,Y,W = np.dot(R, xy1)
            xp = X/W
            yp = Y/W
        else:
            xp = x
            yp = y
        r2 = xp*xp + yp*yp
        r4 = r2*r2
        r6 = r4*r2
        a1 = 2*xp*yp
        D = self.distortion
        k1 = D[0]; k2=D[1]; p1=D[2]; p2=D[3]; k3=D[4]
        barrel = 1 + k1*r2 + k2*r4 + k3*r6
        if len(D)==8:
            barrel /= (1.0 + D[5]*r2 + D[6]*r4 + D[7]*r6)
        xpp = xp*barrel + p1*a1 + p2*(r2+2*(xp*xp))
        ypp = yp*barrel + p1*(r2+2*(yp*yp)) + p2*a1;

        K = self.K
        u = xpp*K[0,0] + K[0,2]
        v = ypp*K[1,1] + K[1,2]
        return np.vstack( (u,v) ).T

    # --------------------------------------------------
    # 3D <-> image coordinate operations

    def project_pixel_to_3d_ray(self, nparr, distorted=True ):
        if distorted:
            nparr = self.undistort(nparr)
        # now nparr is undistorted (aka rectified) 2d point data

        # Parse inputs
        nparr = np.array(nparr,copy=False)
        assert nparr.ndim==2
        assert nparr.shape[1]==2
        uv_rect_x = nparr[:,0]
        uv_rect_y = nparr[:,1]

        # transform to 3D point in camera frame
        x = (uv_rect_x - self.cx() - self.Tx()) / self.fx()
        y = (uv_rect_y - self.cy() - self.Ty()) / self.fy()
        z = np.ones_like(x)
        ray_cam = np.vstack((x,y,z))

        # transform to world frame
        ray_world_ori = np.dot(self.rot_inv,ray_cam)
        ray_world = ray_world_ori + self.t_inv
        return ray_world.T

    def project_3d_to_pixel(self, pts3d, distorted=True):
        pts3d = np.array(pts3d,copy=False)
        assert pts3d.ndim==2
        assert pts3d.shape[1]==3

        # homogeneous and transposed
        pts3d_h = np.empty( (4,pts3d.shape[0]) )
        pts3d_h[:3,:] = pts3d.T
        pts3d_h[3] = 1

        # undistorted homogeneous image coords
        cc = np.dot(self.pmat, pts3d_h)

        # project
        pc = cc[:2]/cc[2]
        u, v = pc

        if distorted:
            # distort (the currently undistorted) image coordinates
            nparr = np.vstack((u,v)).T
            u,v = self.distort( nparr ).T
        return np.vstack((u,v)).T

    def project_3d_to_camera_frame(self, pts3d):
        pts3d = np.array(pts3d,copy=False)
        assert pts3d.ndim==2
        assert pts3d.shape[1]==3

        # homogeneous and transposed
        pts3d_h = np.empty( (4,pts3d.shape[0]) )
        pts3d_h[:3,:] = pts3d.T
        pts3d_h[3] = 1

        # undistorted homogeneous image coords
        cc = np.dot(self.Rt, pts3d_h)

        return cc.T

    # --------------------------------------------
    # misc. helpers

    def camcenter_like(self,nparr):
        nparr = np.array(nparr,copy=False)
        assert nparr.ndim==2
        assert nparr.shape[1]==3
        return np.zeros( nparr.shape ) + self.t_inv.T

# factory function

def load_camera_from_bagfile( bag_fname ):
    """factory function for class CameraModel"""
    bag = rosbag.Bag(bag_fname, 'r')
    camera_name = None
    translation = None
    rotation = None
    intrinsics = None

    for topic, msg, t in bag.read_messages():
        if 1:
            parts = topic.split('/')
            if parts[0]=='':
                parts = parts[1:]
            topic = parts[-1]
            parts = parts[:-1]
            if len(parts)>1:
                this_camera_name = '/'.join(parts)
            else:
                this_camera_name = parts[0]
            # empty, this_camera_name, topic = parts
            # assert empty==''
        if camera_name is None:
            camera_name = this_camera_name
        else:
            assert this_camera_name == camera_name

        if topic == 'tf':
            translation = msg.translation
            rotation = msg.rotation
        elif topic == 'camera_info':
            intrinsics = msg
        else:
            print 'skipping message',topic
            continue

    bag.close()

    if translation is None or rotation is None:
        raise ValueError('no extrinsic parameters in bag file')
    if intrinsics is None:
        raise ValueError('no intrinsic parameters in bag file')

    result = CameraModel(translation=point_msg_to_tuple(translation),
                         rotation=parse_rotation_msg(rotation),
                         intrinsics=intrinsics,
                         name=camera_name,
                         )
    return result


def load_camera_from_pmat( pmat, width=None, height=None, name='cam', _depth=0 ):
    pmat = np.array(pmat)
    assert pmat.shape==(3,4)
    c = center(pmat)
    M = pmat[:,:3]
    K,R = my_rq(M)
    a = K[2,2]
    if a==0:
        warnings.warn('ill-conditioned intrinsic camera parameters')
    else:
        if a != 1.0:
            if _depth > 0:
                raise ValueError('cannot scale this pmat: %s'%( repr(pmat,)))
            new_pmat = pmat/a
            cam = load_camera_from_pmat( new_pmat, width=width, height=height, name=name, _depth=_depth+1)
            return cam

    t = -np.dot(R,c)

    P = np.zeros( (3,4) )
    P[:3,:3]=K

    i = sensor_msgs.msg.CameraInfo()
    i.width = width
    i.height = height
    i.D = [0,0,0,0,0]
    i.K = list(K.flatten())
    i.R = list(np.eye(3).flatten())
    i.P = list(P.flatten())
    result = CameraModel(translation = t,
                         rotation = R,
                         intrinsics = i,
                         name=name)
    return result
