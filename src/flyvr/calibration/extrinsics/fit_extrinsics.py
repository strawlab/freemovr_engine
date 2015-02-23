# ROS imports
import roslib; roslib.load_manifest('flyvr')

import scipy.optimize
from pymvg.camera_model import CameraModel
from pymvg.util import get_rotation_matrix_and_quaternion
import flyvr.simple_geom as simple_geom
import numpy as np
import os
import cv2

PLOT=int(os.environ.get('PLOT',0))
if PLOT:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from plot_utils import get_3d_verts, plot_camera

import roslib; roslib.load_manifest('flyvr')
from tf.transformations import quaternion_from_matrix, \
    quaternion_matrix, rotation_from_matrix, rotation_matrix, \
    quaternion_about_axis
from flyvr.cvnumpy import rodrigues2matrix, matrix2rodrigues

def matrix2quaternion( R ):
    rnew = np.eye(4)
    rnew[:3,:3] = R
    return quaternion_from_matrix(R)

def quaternion2matrix( q ):
    R =  quaternion_matrix(q)
    return R[:3,:3]

class ObjectiveFunctionFancy:
    """Find pose using world-space object point relations as the error term.

    For a similar idea, see 'Pose Estimation using Four Corresponding
    Points' by Liu and Wong, 1998. This method uses arbitrary numbers
    of points and (so far) does not use the Gauss-Newton method nor
    require calculation of a Jacobian, although those things would be
    straitforward from here.
    """

    def __init__(self,base_cam,X3d,x2d):
        self.base_cam = base_cam
        self.X3d = X3d
        self.x2d = x2d
        intrinsics = self.base_cam.to_dict()
        del intrinsics['Q']
        del intrinsics['translation']
        del intrinsics['name']
        self.intrinsic_dict = intrinsics
        self._obj_dist = []
        self.npts = len(self.X3d)

        self.d_actual = self.compute_distance_vector( self.X3d )
        self.alpha = 1.0

    def compute_distance_vector(self, pts ):
        result = []
        for i in range(self.npts):
            for j in range(self.npts):
                if i<j:
                    d = pts[i]-pts[j]
                    result.append(np.sqrt(np.sum(d**2)))
        return np.array(result)

    def compute_shape_scalar(self, pts):
        # Compute some value that changes based on the chirality of
        # the object. Here we use eqn 4 from Liu and Wong.
        v21 = pts[2]-pts[1]
        v23 = pts[2]-pts[3]
        v20 = pts[2]-pts[0]
        return -np.dot(np.cross(v21,v23),v20)

    def get_start_guess(self):
        # pts = camera.project_pixel_to_3d_ray(self.x2d)
        # vecs = pts - camera.get_camcenter()
        # distances = np.sqrt(np.sum(vecs**2,axis=1))

        return np.ones( (len(self.x2d),) )

    def make_cam_from_params(self, params):

        # find location of camcenter by finding point of best fit with
        # N spheres of radius params each centered at a point at
        # self.X3d

        raise NotImplementedError()

    def err(self, params):
        #x,y,z, rx, ry, rz = params
        camera = self.make_cam_from_params( params)
        pts_test = camera.project_pixel_to_3d_ray(self.x2d, distance=params)
        d_test = self.compute_distance_vector(pts_test)

        shape_test = self.compute_shape_scalar(pts_test)

        err_d = np.sum((d_test - self.d_actual)**2)
        err_shape = abs(shape_test - self.shape_actual)
        return (err_d + self.alpha*err_shape)

class ObjectiveFunction:
    """Find pose using reprojection error and shape term"""

    def __init__(self,base_cam,X3d,x2d,geom=None):
        self.base_cam = base_cam
        self.X3d = X3d
        self.x2d = x2d
        intrinsics = self.base_cam.to_dict()
        del intrinsics['Q']
        del intrinsics['translation']
        del intrinsics['name']
        self.intrinsic_dict = intrinsics
        self._obj_dist = []
        self.npts = len(self.X3d)
        if geom is not None:
            self.debug = True
        else:
            self.debug = False
        if PLOT and self.debug:
            plt.ion()
            self.fig = plt.figure()
            self.ax3d = self.fig.add_subplot(211, projection='3d')
            self.ax2d = self.fig.add_subplot(212)
            self.ax3d.set_xlabel('x')
            self.ax3d.set_ylabel('y')
            self.ax3d.set_zlabel('z')
            self.geom = geom
            self.plot_verts = get_3d_verts(self.geom)

    def get_start_guess(self):
        if 1:
            rod = matrix2rodrigues(self.base_cam.get_rotation())
            t = self.base_cam.get_translation()
            t.shape= (3,)
            rod.shape=(3,)
            return np.array(list(t)+list(rod),dtype=np.float)

        R = np.eye(4)
        R[:3,:3] = self.base_cam.get_rotation()

        angle, direction, point = rotation_from_matrix(R)
        q = quaternion_about_axis(angle,direction)
        #q = matrix2quaternion(R)
        if 1:
            R2 = rotation_matrix(angle, direction, point)
            #R2 = quaternion2matrix( q )
            try:
                assert np.allclose(R, R2)
            except:
                print
                print 'R'
                print R
                print 'R2'
                print R2
                raise

        C = self.base_cam.get_camcenter()
        result = list(C) + list(q)
        return result

    def make_cam_from_params(self, params):
        if 1:
            t = params[:3]
            rod = params[3:]
            rmat = rodrigues2matrix( rod )
            d = self.intrinsic_dict.copy()
            d['translation'] = t
            d['Q'] = rmat
            cam_model = CameraModel.from_dict(d)
            return cam_model

        C = params[:3]
        quat = params[3:]
        qmag = np.sqrt(np.sum(quat**2))
        quat = quat/qmag

        R,rquat=get_rotation_matrix_and_quaternion(quat)

        t = -np.dot(R, C)

        d = self.intrinsic_dict.copy()
        d['translation'] = t
        d['Q'] = R
        cam_model = CameraModel.from_dict(d)
        return cam_model

    def err(self, params):
        camera = self.make_cam_from_params( params)
        found = camera.project_3d_to_pixel(self.X3d)
        each_err = np.sqrt(np.sum((found - self.x2d)**2,axis=1))

        me = np.mean(each_err)
        if 0:
            print
            print 'params', params
            print 'found'
            print np.hstack( (found, self.x2d, each_err[:,np.newaxis]) )
            print 'mean reproj error: ',me
            print

        if PLOT and self.debug:
            assert len(each_err)==len(self.x2d)
            self.ax3d.cla()
            verts = self.plot_verts
            self.ax3d.plot( verts[:,0], verts[:,1], verts[:,2], 'ko' )
            plot_camera( self.ax3d, camera )

            self.ax2d.cla()
            self.ax2d.plot( self.x2d[:,0], self.x2d[:,1], 'go', mfc='none')
            self.ax2d.plot( found[:,0], found[:,1], 'rx', mfc='none')
            for i in range( len(found)):
                self.ax2d.plot( [found[i,0],self.x2d[i,0]],
                                [found[i,1],self.x2d[i,1]], 'k-' )


            plt.draw()
        if 0:
            df = found[1:]-found[:-1]
            #print 'found'
            #print found
            #print 'df'
            #print df
            bunching_penalty = 1.0/np.sum(df**2)
        #print 'mean reproj error:  % 20.1f   bunching penalty: % 20.1f '%(me,bunching_penalty)
        #return me + bunching_penalty
        return me

def fit_extrinsics_iterative(base_cam,X3d,x2d, geom=None):
    """find a camera with a better extrinsics than the input camera"""
    prestages = True
    if prestages:
        # pre-stage 1 - point the camera in the right direction
        world = np.array([np.mean( X3d, axis=0 )])
        image = np.array([np.mean( x2d, axis=0 )])
        obj = ObjectiveFunction(base_cam, world, image, geom=geom)
        result = scipy.optimize.fmin( obj.err, obj.get_start_guess(),ftol=5.0)
        base_cam = obj.make_cam_from_params(result)

    if prestages:
        # pre-stage 2 - get scale approximately OK
        world = X3d[:2,:]
        image = x2d[:2,:]
        obj = ObjectiveFunction(base_cam, world, image, geom=geom)
        result = scipy.optimize.fmin( obj.err, obj.get_start_guess())
        base_cam = obj.make_cam_from_params(result)

    if prestages:
        # pre-stage 3 - start rotations
        world = X3d[:3,:]
        image = x2d[:3,:]
        obj = ObjectiveFunction(base_cam, world, image, geom=geom)
        result = scipy.optimize.fmin( obj.err, obj.get_start_guess())
        base_cam = obj.make_cam_from_params(result)

    # now, refine our guess, held in base_cam

    last_fval = np.inf
    for i in range(10):
        cam = obj.make_cam_from_params(result)
        obj = ObjectiveFunction(cam, X3d, x2d, geom=geom)
        results = scipy.optimize.fmin( obj.err, obj.get_start_guess(),
                                       full_output=True )
        result, fval = results[:2]
        print 'fval, last_fval',fval, last_fval
        if fval > last_fval:
            # we're not getting better
            break
        eps = 1e-2 # this is pixel reprojection error here. don't need better than this.
        if abs(fval-last_fval) < eps:
            break
        last_fval=fval
    print 'did %d iterations'%(i+1,)

    if 0:
        obj = ObjectiveFunction(base_cam, X3d, x2d)#, geom=geom)
        results = scipy.optimize.anneal( obj.err, obj.get_start_guess(),
                                         learn_rate=0.5,
                                         full_output=True, maxeval=50000, T0=1000.0,
                                         maxiter=10000,
                                         #disp=True,
                                         )
        #print 'results',results
        result = results[0]
        if 1:
            result, Jmin, T, feval, iters, accept, retval = results
            print 'Jmin',Jmin
            print 'T',T
            print 'fevel',feval
            print 'iters',iters
            print 'accept',accept
            print 'retval',retval
    cam = obj.make_cam_from_params(result)

    if 1:
        found = cam.project_3d_to_pixel(X3d)
        orig = x2d
        reproj_error = np.sqrt(np.sum((found-orig)**2, axis=1))
        cum = np.mean(reproj_error)

        mean_cam_z = np.mean(cam.project_3d_to_camera_frame(X3d)[:,2])

    cam.name = base_cam.name
    result = dict(
        mean_err=cum,
        mean_cam_z = mean_cam_z,
        cam = cam)
    return result

def save_point_image(fname, sz, x2d ):
    im = np.zeros( (sz[1], sz[0]), dtype=np.uint8 )
    for xy in x2d:
        x,y=xy
        im[y-3:y+3,x-3:x+3] = 255
    scipy.misc.imsave(fname,im)

def fit_extrinsics(base_cam,X3d,x2d,geom=None):
    assert x2d.ndim==2
    assert x2d.shape[1]==2

    assert X3d.ndim==2
    assert X3d.shape[1]==3

    if 0:
        fname = 'x2d_'+base_cam.name + '.png'
        fname = fname.replace('/','-')
        save_point_image(fname, (base_cam.width, base_cam.height), x2d )
        #print 'saved pt debug image to',fname

    ipts = np.array(x2d,dtype=np.float64)
    opts = np.array(X3d,dtype=np.float64)

    K = np.array(base_cam.get_K(), dtype=np.float64)
    dist_coeffs = np.array( base_cam.get_D(), dtype=np.float64)

    retval, rvec, tvec = cv2.solvePnP( opts, ipts,
                                       K,
                                       dist_coeffs)
    assert retval

    # we get two possible cameras back, figure out which one has objects in front
    rmata = rodrigues2matrix( rvec )
    intrinsics = base_cam.to_dict()
    del intrinsics['Q']
    del intrinsics['translation']
    del intrinsics['name']
    d = intrinsics.copy()
    d['translation'] = tvec
    d['Q'] = rmata
    d['name'] = base_cam.name
    cam_model_a = CameraModel.from_dict(d)
    mza = np.mean(cam_model_a.project_3d_to_camera_frame(X3d)[:,2])

    # don't bother with second - it does not have a valid rotation matrix

    if 1:
        founda = cam_model_a.project_3d_to_pixel(X3d)
        erra = np.mean(np.sqrt(np.sum((founda-x2d)**2, axis=1)))

        cam_model = cam_model_a

    if 1:
        found = cam_model.project_3d_to_pixel(X3d)
        orig = x2d
        reproj_error = np.sqrt(np.sum((found-orig)**2, axis=1))
        cum = np.mean(reproj_error)

        mean_cam_z = np.mean(cam_model.project_3d_to_camera_frame(X3d)[:,2])

    if (mean_cam_z < 0 or cum > 20) and 0:
        # hmm, we have a flipped view of the camera.
        print '-'*80,'HACK ON'
        center, lookat, up = cam_model.get_view()
        #cam2 = cam_model.get_view_camera( -center, lookat, -up )
        cam2 = cam_model.get_view_camera( center, lookat, up )
        cam2.name=base_cam.name
        return fit_extrinsics_iterative(cam2,X3d,x2d, geom=geom)

    result = dict(cam=cam_model,
                  mean_err=cum,
                  mean_cam_z = mean_cam_z,
                  )
    return result

