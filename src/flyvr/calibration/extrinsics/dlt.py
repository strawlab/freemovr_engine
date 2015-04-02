import numpy
import numpy.linalg
import scipy.optimize

from .common import prepare_input, rotationmatrix_to_rodrigues, rodrigues_to_rotationmatrix

import cv2

def create_matrix_A(hom_points_3d, hom_points_2d):

    assert hom_points_3d.ndim == 2 and hom_points_3d.shape[1] == 4
    assert hom_points_2d.ndim == 2 and hom_points_2d.shape[1] == 3
    assert hom_points_3d.shape[0] == hom_points_2d.shape[0]

    N = hom_points_3d.shape[0]

    _A = []
    for i in range(N):
        X, Y, Z, S = hom_points_3d[i]
        u, v, w = hom_points_2d[i]

        _A.append([  0,   0,   0,   0, -w*X, -w*Y, -w*Z, -w*S,  v*X,  v*Y,  v*Z,  v*S])
        _A.append([w*X, w*Y, w*Z, w*S,    0,    0,    0,    0, -u*X, -u*Y, -u*Z, -u*S])

    A = numpy.array(_A, dtype=numpy.float64)
    assert A.shape == (2*N, 12)
    return A


def get_normalize_2d_matrix(points_2d):
    # normalize 2d points to mean 0 and rms sqrt(2)
    pts_mean = points_2d.mean(axis=0)
    centered_pts_2d = points_2d - pts_mean
    s = 1 / numpy.linalg.norm(centered_pts_2d, axis=1).mean()
    xm, ym = pts_mean
    T = numpy.array([[s, 0, -s*xm],
                     [0, s, -s*ym],
                     [0, 0,  1 ]], dtype=numpy.float64)
    return T


def get_normalize_3d_matrix(points_3d):
    # normalize 3d points to mean 0 and rms sqrt(2)
    pts_mean = points_3d.mean(axis=0)
    centered_pts_3d = points_3d - pts_mean
    s = 1 / numpy.linalg.norm(centered_pts_3d, axis=1).mean()
    xm, ym, zm = pts_mean
    U = numpy.array([[s, 0, 0, -s*xm],
                     [0, s, 0, -s*ym],
                     [0, 0, s, -s*zm],
                     [0, 0, 0,  1 ]], dtype=numpy.float64)
    return U


def get_homogeneous_coordinates(points):
    assert points.ndim == 2
    assert points.shape[1] in [2, 3]
    if points.shape[1] == 3:
        assert not numpy.allclose(points[:,2], 1.)
    return numpy.hstack((points, numpy.ones((points.shape[0], 1))))


def get_normalized_points_2d(points_2d):
    T = get_normalize_2d_matrix(points_2d)
    Tinv = numpy.linalg.inv(T)
    hom_points_2d = get_homogeneous_coordinates(points_2d)
    normalized_points_2d = numpy.empty(hom_points_2d.shape)
    for i, x in enumerate(hom_points_2d):
        normalized_points_2d[i,:] = numpy.dot(T, x)
    return normalized_points_2d, T, Tinv


def get_normalized_points_3d(points_3d):
    U = get_normalize_3d_matrix(points_3d)
    hom_points_3d = get_homogeneous_coordinates(points_3d)
    normalized_points_3d = numpy.empty(hom_points_3d.shape)
    for i, x in enumerate(hom_points_3d):
        normalized_points_3d[i,:] = numpy.dot(U, x)
    return normalized_points_3d, U, Uinv

def _dlt(nhpts3d, nhpts2d):
    # get matrix A
    A = create_matrix_A(normalized_points_3d, normalized_points_2d)

    # solve via singular value decomposition
    # XXX: in numpy the returned V is already transposed!
    _, singular_values, VT = numpy.linalg.svd(A, full_matrices=False)
    sol_idx = numpy.argmin(singular_values)
    assert sol_idx == 11
    Pvec_n = VT.T[:,sol_idx]

    return Pvec_n


def direct_linear_transform(camera, points_3d, points_2d, extrinsics_guess=None, params={}):

    points_3d, points_2d, K, distortion, extguess = prepare_input(camera, points_3d, points_2d, extrinsics_guess)

    # Undistort
    points_2d = cv2.undistort(points_2d.reshape(-1,1,2), camera.get_K(), camera.get_D())
    points_2d = points_2d.reshape(-1,2)

    # Normalize 2d points and keep transformation matrix
    normalized_points_2d, _, Tinv = get_normalized_points_2d(points_2d)
    # Normalize 3d points and keep transformation matrix
    normalized_points_3d, U, _ = get_normalized_points_3d(points_3d)

    Pvec_n = _dlt(normalized_points_3d, normalized_points_2d)

    P_n = Pvec_n.reshape((3, 4))

    # Denormalize
    P = numpy.dot(Tinv, numpy.dot(P_n, U))

    # Recover rvec and tvec
    Kinv = numpy.linalg.inv(K)
    RT = numpy.dot(Kinv, P)
    # TODO: RODRIGUES!
    return rotationmatrix_to_rodrigues(RT[:,:3]), RT[:,3], None

direct_linear_transform.params = {
        'extrinsic_guess_support': False
        }


def hartley_gold_algorithm(camera, points_3d, points_2d, extrinsics_guess=None, params={}):

    # Undistort
    points_2d = cv2.undistort(points_2d.reshape(-1,1,2), camera.get_K(), camera.get_D())
    points_2d = points_2d.reshape(-1,2)

    # Normalize 2d points and keep transformation matrix
    normalized_points_2d, T, Tinv = get_normalized_points_2d(points_2d)
    # Normalize 3d points and keep transformation matrix
    normalized_points_3d, U, Uinv = get_normalized_points_3d(points_3d)

    if extrinsics_guess is None:
        Pvec_n = _dlt(normalized_points_3d, normalized_points_2d)
    else:
        K = camera.K
        assert extrinsics_guess.shape == (3, 4)
        rvec, tvec = extrinsics_guess
        RT = numpy.hstack((rodrigues_to_rotationmatrix(rvec), tvec.T))
        Pvec_n = numpy.dot(T, numpy.dot(K, numpy.dot(RT, Uinv))).flatten()

    # Function to minimize ||A*P|| = 0 with norm being sum over squared geometric distances
    def geometric_error(Pvec_n):
        P = Pvec_n.reshape(3, 4)
        proj_n_pts_2d = numpy.dot(P, normalized_points_3d.T).T
        proj_n_pts_2d /= proj_n_pts_2d[:,2].reshape(-1,1)
        return numpy.linalg.norm(proj_n_pts_2d - normalized_points_2d).mean()

    # constraints
    constraints = [{'type': 'eq',
                    'fun': lambda pvec: numpy.linalg.norm(pvec) - 1  # ||Pvec|| = 1
                   },{
                    'type': 'eq',
                    'fun': lambda pvec: numpy.linalg.norm(pvec[8:11]) - 1  # ||(p31, p32, p33)|| = 1
                   }]
    # options
    options = {
            'maxiter': 10000,
            'disp': False
            }

    constraint_idx = params.get('constraint', 1)

    # Minimize the camera_matrix_equation
    opt_result = scipy.optimize.minimize(fun=geometric_error,
                                         x0=Pvec_n,
                                         method='SLSQP',
                                         constraints=constraints[constraint_idx],
                                         options=options)
    if not opt_result.success:
        raise RuntimeError(opt_result.message)
    Pvec_n = opt_result.x

    P_n = Pvec_n.reshape((3, 4))
    # Denormalize
    P = numpy.dot(Tinv, numpy.dot(P_n, U))
    # Recover rvec and tvec
    Kinv = numpy.linalg.inv(K)
    RT = numpy.dot(Kinv, P)
    # TODO: RODRIGUES!
    return rotationmatrix_to_rodrigues(RT[:,:3]), RT[:,3], None

hartley_gold_algorithm.params = {
        'extrinsics_guess_support': True,
        'constraint': int
        }


