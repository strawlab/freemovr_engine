import numpy as np
import cv2

def rodrigues2matrix_cv(params):
    rvec = np.array(params,dtype=np.float64)
    rvec.shape = (1,3)
    Rmat, jacobian = cv2.Rodrigues(rvec)
    return Rmat

def rodrigues2matrix(params):
    # Written after the docs at
    # http://opencv.itseez.com/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#rodrigues

    try:
        rvec = np.array(params,dtype=np.float)
        rvec.shape = (1,3)
    except:
        print('bad rvec',rvec)
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
    Rmat = np.array(R,dtype=np.float64)
    assert Rmat.shape == (3,3)
    rvec, jacobian = cv2.Rodrigues(Rmat)
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
