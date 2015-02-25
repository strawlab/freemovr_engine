import numpy
import numpy.linalg
import scipy.optimize

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


def direct_linear_transform(points_3d, points_2d, denormalize=True):

    # Normalize 2d points and keep transformation matrix
    T = get_normalize_2d_matrix(points_2d)
    Tinv = numpy.linalg.inv(T)
    hom_points_2d = get_homogeneous_coordinates(points_2d)
    normalized_points_2d = numpy.empty(hom_points_2d.shape)
    for i, x in enumerate(hom_points_2d):
        normalized_points_2d[i,:] = numpy.dot(T, x)

    # Normalize 3d points and keep transformation matrix
    U = get_normalize_3d_matrix(points_3d)
    hom_points_3d = get_homogeneous_coordinates(points_3d)
    normalized_points_3d = numpy.empty(hom_points_3d.shape)
    for i, x in enumerate(hom_points_3d):
        normalized_points_3d[i,:] = numpy.dot(U, x)

    # get matrix A
    A = create_matrix_A(normalized_points_3d, normalized_points_2d)

    # solve via singular value decomposition
    # XXX: in numpy the returned V is already transposed!
    _, singular_values, VT = numpy.linalg.svd(A, full_matrices=False)
    sol_idx = numpy.argmin(singular_values)
    Pvec_n = VT.T[:,sol_idx]  # that's why we need to transpose here...

    P_n = Pvec_n.reshape((3, 4))

    if denormalize:
        # Denormalize
        return numpy.dot(numpy.dot(Tinv, P_n), U)
    else:
        return Pvec_n, Tinv, U, normalized_points_3d, normalized_points_2d


def hartley_gold_algorithm(points_3d, points_2d):

    # Linear solution for Pvec_n
    Pvec_n, Tinv, U, n_pts_3d, n_pts_2d = direct_linear_transform(points_3d, points_2d,
                                                                  denormalize=False)

    # Function to minimize ||A*P|| = 0 with norm being sum over squared geometric distances
    def geometric_error(Pvec_n):
        P = Pvec_n.reshape(3, 4)
        proj_n_pts_2d = numpy.dot(P, n_pts_3d.T).T
        return numpy.sum((proj_n_pts_2d - n_pts_2d)**2)

    # constraints
    constraint0 = {
            'type': 'eq',
            'fun': lambda pvec: numpy.linalg.norm(pvec) - 1  # ||Pvec|| = 1
            }
    constraint1 = {
            'type': 'eq',
            'fun': lambda pvec: numpy.linalg.norm(pvec[8:11]) - 1  # ||(p31, p32, p33)|| = 1
            }
    # options
    options = {
            'maxiter': 10000,
            'disp': False
            }

    # Minimize the camera_matrix_equation
    opt_result = scipy.optimize.minimize(fun=geometric_error,
                                         x0=Pvec_n,
                                         method='SLSQP',
                                         constraints=constraint0,
                                         options=options)
    if not opt_result.success:
        raise RuntimeError(opt_result.message)
    Pvec_n = opt_result.x

    P_n = Pvec_n.reshape((3, 4))

    # Denormalize
    return numpy.dot(numpy.dot(Tinv, P_n), U)




#~~~ WIP testing arrays ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
X3d = numpy.array([[ 304.8,    0. ,    0. ],
                   [ 304.8,  152.4,    0. ],
                   [ 304.8,  152.4,  152.4],
                   [ 304.8,    0. ,  152.4],
                   [ 178. ,   85. ,   86. ],
                   [ 178. ,   85. ,   63. ]])
X2d = numpy.array([[ 120. ,  475. ],
                   [ 522. ,  460. ],
                   [ 497. ,   69.6],
                   [ 120. ,   76.2],
                   [ 344. ,  200. ],
                   [ 349. ,  298. ]])

M3d = []
M2d = []
for i in range(10):
    M3d.append(X3d + numpy.random.normal(size=X3d.shape))
    M2d.append(X2d + numpy.random.normal(size=X2d.shape))
M3d = numpy.vstack(M3d)
M2d = numpy.vstack(M2d)


