
from .common import prepare_input
import cv2

def _cv2_solvepnp(camera, points_3d, points_2d, extrinsics_guess=None, flags=None, errmsg=""):
    """helper for cv2.solvepnp"""
    pts3d, pts2d, cam_matrix, cam_dist_coeffs, extguess = prepare_input(camera, points_3d, points_2d, extrinsics_guess)

    # Try fitting the extrinsics
    if not extguess:
        retval, rvec, tvec = cv2.solvePnP(pts3d, pts2d, camera_matrix, camera_dist_coeffs, flags=flags)
    else:
        rvec, tvec = extguess
        retval, rvec, tvec = cv2.solvePnP(pts3d, pts2d, camera_matrix, camera_dist_coeffs, rvec, tvec, 1, flags=flags)

    if not retval:
        raise RuntimeError("")
    return rvec, tvec, None



def pnp_iterative(camera, points_3d, points_2d, extrinsics_guess=None, params={}):
    """Fits the extrinsic matrices for a camera using 3d and 2d correspondences

    FIXME: parameters

    """
    return _cv2_solvepnp(camera, points_3d, points_2d, extrinsics_guess,
                         flags=cv2.CV_ITERATIVE,
                         errmsg="pnp_iterative did not succeed.")
pnp_iterative.params = {
        'extrinsics_guess_support': True,
        }


def pnp_p3p(camera, points_3d, points_2d, extrinsics_guess=None, params={}):
    """Fits the extrinsic matrices for a camera using 3d and 2d correspondences

    FIXME: parameters

    """
    return _cv2_solvepnp(camera, points_3d, points_2d, extrinsics_guess,
                         flags=cv2.CV_P3P,
                         errmsg="pnp_p3p did not succeed.")
pnp_p3p.params = {
        'extrinsics_guess_support': True,
        }

def pnp_epnp(camera, points_3d, points_2d, extrinsics_guess=None, params={}):
    """Fits the extrinsic matrices for a camera using 3d and 2d correspondences

    FIXME: parameters

    """
    return _cv2_solvepnp(camera, points_3d, points_2d, extrinsics_guess,
                         flags=cv2.CV_EPNP,
                         errmsg="pnp_epnp did not succeed.")
pnp_epnp.params = {
        'extrinsics_guess_support': True,
        }



def _cv2_solvepnpransac(camera, points_3d, points_2d, extrinsics_guess=None, params=None, flags=None, errmsg=""):
    """helper for cv2.solvepnp"""
    pts3d, pts2d, cam_matrix, cam_dist_coeffs, extguess = prepare_input(camera, points_3d, points_2d, extrinsics_guess)

    iterations_count = params.get('iterations_count', 100)
    reprojection_error = params.get('reprojection_error', 8.0)
    min_inliers_count = params.get('min_inliers_count', 100)

    # Try fitting the extrinsics
    if not extguess:
        rvec, tvec, inliers = cv2.solvePnPRansac(pts3d, pts2d, camera_matrix, camera_dist_coeffs,
                                                 iterationsCount=iterations_count,
                                                 reprojectionError=reprojection_error,
                                                 minInliersCount=min_inliers_count,
                                                 flags=flags)
    else:
        rvec, tvec = extguess
        rvec, tvec, inliers = cv2.solvePnPRansac(pts3d, pts2d, camera_matrix, camera_dist_coeffs, rvec, tvec, 1,
                                                 iterationsCount=iterations_count,
                                                 reprojectionError=reprojection_error,
                                                 minInliersCount=min_inliers_count,
                                                 flags=flags)
    return rvec, tvec, {'inliers': inliers}


def pnp_ransac_iterative(camera, points_3d, points_2d, extrinsics_guess=None, params={}):
    """Fits the extrinsic matrices for a camera using 3d and 2d correspondences

    FIXME: parameters

    """
    return _cv2_solvepnpransac(camera, points_3d, points_2d, extrinsics_guess,
                               params=params,
                               flags=cv2.CV_ITERATIVE,
                               errmsg="pnp_ransac_iterative did not succeed.")
pnp_ransac_iterative.params = {
        'extrinsics_guess_support': True,
        'iterations_count': int,
        'reprojection_error': float,
        'min_inliers_count': float
        }


def pnp_ransac_p3p(camera, points_3d, points_2d, extrinsics_guess=None, params={}):
    """Fits the extrinsic matrices for a camera using 3d and 2d correspondences

    FIXME: parameters

    """
    return _cv2_solvepnpransac(camera, points_3d, points_2d, extrinsics_guess,
                               params=params,
                               flags=cv2.CV_P3P,
                               errmsg="pnp_ransac_p3p did not succeed.")
pnp_ransac_p3p.params = {
        'extrinsics_guess_support': True,
        'iterations_count': int,
        'reprojection_error': float,
        'min_inliers_count': float
        }

def pnp_ransac_epnp(camera, points_3d, points_2d, extrinsics_guess=None, params={}):
    """Fits the extrinsic matrices for a camera using 3d and 2d correspondences

    FIXME: parameters

    """
    return _cv2_solvepnpransac(camera, points_3d, points_2d, extrinsics_guess,
                               params=params,
                               flags=cv2.CV_EPNP,
                               errmsg="pnp_ransac_epnp did not succeed.")
pnp_ransac_epnp.params = {
        'extrinsics_guess_support': True,
        'iterations_count': int,
        'reprojection_error': float,
        'min_inliers_count': float
        }