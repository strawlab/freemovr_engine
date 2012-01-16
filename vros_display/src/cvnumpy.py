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

def rodrigues2matrix_cv(params):
    rvec = np.array(params)
    rvec.shape = (1,3)
    rvec = numpy2opencv_image(rvec.astype(np.float))
    Rmat = numpy2opencv_image(np.empty( (3,3) ))
    cv.Rodrigues2(rvec, Rmat)
    Rmat = opencv_image2numpy(Rmat)
    return Rmat

def rodrigues2matrix(params):
    # Written after the docs at
    # http://opencv.itseez.com/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#rodrigues

    try:
        rvec = np.array(params,dtype=np.float)
        rvec.shape = (1,3)
    except:
        print 'bad rvec',rvec
        raise

    theta = np.sqrt(np.sum(rvec**2))
    if theta==0:
        rvec = rvec
    else:
        rvec = rvec/theta
    r = rvec[0] # drop dim

    s = np.sin(theta)
    c = np.cos(theta)
    R = c*np.eye(3) + (1-c)*rvec*rvec.T + s*np.array([[0,   -r[2],  r[1]],
                                                      [r[2],  0,   -r[0]],
                                                      [-r[1], r[0],   0]])

    # -R.T might also be considered a valid rotation matrix, but it
    # -does not have an eigenvector of 1.

    return R

def matrix2rodrigues(R):
    rvec = np.empty( (1,3) )
    rvec = numpy2opencv_image(rvec)
    Rmat = numpy2opencv_image(R.astype(np.float))
    cv.Rodrigues2(Rmat, rvec)
    rvec = opencv_image2numpy(rvec)
    return rvec

def rodrigues2angle_axis(params):
    rvec = np.array(params)
    rvec.shape = (1,3)

    theta = np.sqrt(np.sum(rvec**2))
    if theta==0:
        rvec = rvec
    else:
        rvec = rvec/theta
    r = rvec[0] # drop dim
    return theta, r
