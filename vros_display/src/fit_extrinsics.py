import scipy.optimize
import camera_model
import numpy as np
import cv

def numpy2opencv_image(arr):
    arr = np.array(arr)
    assert arr.ndim==2
    if arr.dtype in [np.float32]:
        result = cv.CreateMat( arr.shape[0], arr.shape[1], cv.CV_32FC1)
    elif arr.dtype in [np.float64, np.float]:
        result = cv.CreateMat( arr.shape[0], arr.shape[1], cv.CV_64FC1)
    elif arr.dtype in [np.uint8]:
        result = cv.CreateMat( arr.shape[0], arr.shape[1], cv.CV_8UC1)
    else:
        raise ValueError('unknown numpy dtype "%s"'%arr.dtype)
    for i in range(arr.shape[0]):
        for j in range(arr.shape[1]):
            result[i,j] = arr[i,j]
    return result

def opencv_image2numpy( cvimage ):
    pyobj = np.asarray(cvimage)
    if pyobj.ndim == 2:
        # new OpenCV version
        result = pyobj
    else:
        # old OpenCV, so hack this
        width = cvimage.width
        height = cvimage.height
        assert cvimage.channels == 1
        assert cvimage.nChannels == 1
        assert cvimage.depth == 32
        assert cvimage.origin == 0
        result = np.empty( (height,width), dtype=np.float )
        for i in range(height):
            for j in range(width):
                result[i,j] = cvimage[i,j]
    return result

def rodrigues2matrix(params):
    rvec = np.array(params)
    rvec.shape = (1,3)
    rvec = numpy2opencv_image(rvec.astype(np.float))
    Rmat = numpy2opencv_image(np.empty( (3,3) ))
    cv.Rodrigues2(rvec, Rmat)
    Rmat = opencv_image2numpy(Rmat)
    return Rmat

class ObjectiveFunction:
    def __init__(self,X3d,x2d,bagfile,name=None):
        if name is None:
            name = 'fit'
        self.name = name
        self.X3d = X3d
        self.x2d = x2d
        self.base_cam = camera_model.load_camera_from_bagfile( bagfile, extrinsics_required=False )
        self.intrinsics = self.base_cam.get_intrinsics_as_msg()

    def make_cam_from_params(self, params):
        x,y,z, rx, ry, rz = params
        translation = (x,y,z)
        rmat = rodrigues2matrix( (rx,ry,rz))
        cam_model = camera_model.CameraModel( translation=translation,
                                              rotation=rmat,
                                              intrinsics=self.intrinsics,
                                              name=self.name)
        return cam_model

    def err(self, params):
        #x,y,z, rx, ry, rz = params
        cam_model = self.make_cam_from_params( params)

        found = cam_model.project_3d_to_pixel(self.X3d)
        orig = self.x2d
        reproj_error = np.sqrt(np.sum((found-orig)**2, axis=1))
        cum = np.sum(reproj_error)

        if 0:
            print 'X3d'
            print self.X3d
            print 'found'
            print found

            print 'orig'
            print orig

            print 'reproj_error'
            print reproj_error
            print 'cum',cum
        return cum

def fit_extrinsics_old(bagfile_intrinsics,X3d,x2d,name=None):

    if name is None:
        name='fit'

    best_error = np.inf
    best_result = None

    thetas = [0, np.pi/2, np.pi, 3*np.pi/2]
    for rx in thetas:
        for ry in thetas:
            for rz in thetas:
                start_params = [3, 0.0, 0.5,  rx, ry, rz]
                obj = ObjectiveFunction(X3d,x2d,bagfile_intrinsics,
                                        name=name)
                result = scipy.optimize.fmin( obj.err, start_params )
                if obj.err(result) < best_error:
                    best_error = obj.err(result)
                    best_result = result
    cam = obj.make_cam_from_params(result)

    result = dict(
        start_params = start_params ,
        start_error = obj.err( start_params ),
        final_error = obj.err( result ),
        final_params = result,
        cam = cam)
    return result


def mk_object_points(X3d):
    assert X3d.ndim==2
    assert X3d.shape[1]==3
    opts = cv.CreateMat(len(X3d), 3, cv.CV_32FC1)
    for i in range(len(X3d)):
        for j in range(3):
            opts[i,j] = X3d[i,j]
    return opts

def mk_image_points(x2d):
    assert x2d.ndim==2
    assert x2d.shape[1]==2
    ipts = cv.CreateMat(len(x2d), 2, cv.CV_32FC1)
    for i in range(len(x2d)):
        for j in range(2):
            ipts[i,j] = x2d[i,j]
    return ipts

def fit_extrinsics(bagfile_intrinsics,X3d,x2d,name=None):
    assert x2d.ndim==2
    assert x2d.shape[1]==2

    assert X3d.ndim==2
    assert X3d.shape[1]==3

    ipts = mk_image_points(x2d)
    opts = mk_object_points(X3d)

    base_cam = camera_model.load_camera_from_bagfile( bagfile_intrinsics,
                                                      extrinsics_required=False )

    K = numpy2opencv_image(base_cam.get_K())
    D = numpy2opencv_image(base_cam.get_D())

    rvec = cv.CreateMat(1, 3, cv.CV_32FC1)
    tvec = cv.CreateMat(1, 3, cv.CV_32FC1)

    cv.FindExtrinsicCameraParams2( opts, ipts,
                                   K,
                                   D,
                                   rvec,
                                   tvec,
                                   False)
    rvec = opencv_image2numpy( rvec ); rvec.shape=(3,)
    tvec = opencv_image2numpy( tvec ); tvec.shape=(3,)

    rmat = rodrigues2matrix( rvec )
    cam_model = camera_model.CameraModel( translation=tvec,
                                          rotation=rmat,
                                          intrinsics=base_cam.get_intrinsics_as_msg(),
                                          name=base_cam.name)

    if 1:
        found = cam_model.project_3d_to_pixel(X3d)
        orig = x2d
        reproj_error = np.sqrt(np.sum((found-orig)**2, axis=1))
        cum = np.mean(reproj_error)

    result = dict(cam=cam_model,
                  mean_err=cum,
                  )
    return result

