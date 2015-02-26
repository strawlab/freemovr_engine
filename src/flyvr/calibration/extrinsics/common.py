import numpy
import cv2
import pymvg

def is_3d_point(point):
    return point.ndim == 1 and point.shape == (3,)

def is_3d_point_array(points):
    return points.ndim == 2 and points.shape[1] == 3

def is_2d_point(point):
    return point.ndim == 1 and point.shape == (2,)

def is_2d_point_array(points):
    return points.ndim == 2 and points.shape[1] == 2

def is_3d_vector(vec):
    # TODO
    return True

def is_rodrigues(vec):
    return vec.shape == (3,1)

def rotationmatrix_to_rodrigues(mat):
    return cv2.Rodrigues(mat)[0]

def rodrigues_to_rotationmatrix(rod):
    return cv2.Rodrigues(rod)[0]

def prepare_input(camera, points_3d, points_2d, extrinsics_guess):
    """prepares the input data for all other fitting functions"""

    assert is_3d_point_array(points_3d), "points_3d.shape != (N,3)"
    assert is_2d_point_array(points_2d), "points_2d.shape != (N,2)"
    assert points_3d.shape[0] == points_2d.shape[0], "len(points_3d) != len(points_2d)"

    if extrinsics_guess is None:
        USE_EXTRINSIC_GUESS = True
    else:
        # a guess for the camera position was provided!
        try:
            rvec, tvec = extrinsics_guess
        except (ValueError, TypeError):
            raise ValueError("extrinsics_guess should be [rvec, tvec]")

        assert is_rodrigues(rvec), "rvec is not a rodrigues rotation vector"
        assert is_3d_vector(tvec), "tvec is not a 3d vector"

        USE_EXTRINSIC_GUESS = False

    pts2d = numpy.array(points_2d, dtype=numpy.float64)
    pts3d = numpy.array(points_3d, dtype=numpy.float64)
    cam_matrix = numpy.array(camera.get_K(), dtype=numpy.float64)
    cam_dist_coeffs = numpy.array(camera.get_D(), dtype=numpy.float64)
    extguess = (rvec, tvec) if USE_EXTRINSIC_GUESS else tuple()

    return pts3d, pts2d, cam_matrix, cam_dist_coeffs, extguess
