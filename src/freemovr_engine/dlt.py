import numpy as np
from . import ransac

def calc_reprojection_error( pmat, X3d, x2d ):
    X3dht = np.hstack( (X3d, np.ones( (len(X3d),1))) ).T
    x2dt = x2d.T

    X3dht = X3dht

    x2dh = np.dot(pmat, X3dht)
    found = x2dh[:2]/x2dh[2]
    orig = x2dt

    reproj_error = np.sqrt(np.sum((found-orig)**2, axis=0)) # L2 norm
    return {'mean':np.mean(reproj_error),
            'all':reproj_error}

class DltRansacModel:
    """linear system solved using linear least squares

    This class serves as an example that fulfills the model interface
    needed by the ransac() function.

    """
    def __init__(self,X3d,x2d,debug=False):
        self.X3dht = np.hstack((X3d, np.ones( (len(X3d),1)))).T
        self.x2dt = x2d.T
        self.fullB,self.fullc = build_Bc(X3d,x2d)
        self.debug = debug
    def fit(self, data):
        # calculate the DLT given a set of data

        # create index into B and c
        idx = np.repeat(data,2)*2
        idx[1::2] += 1

        # now slice our B and c arrays for this partition
        B = self.fullB[idx]
        c = self.fullc[idx]

        DLT_avec_results = np.linalg.lstsq(B,c)
        a_vec,residuals = DLT_avec_results[:2]
        Mhat = np.array(list(a_vec)+[1])
        Mhat.shape=(3,4)
        return Mhat
    def get_error( self, data, model):
        X3dht = self.X3dht[:,data]

        x2dh = np.dot(model, X3dht)
        x2d = x2dh[:2]/x2dh[2]
        found = x2d
        orig = self.x2dt[:,data]

        reproj_error = np.sum((found-orig)**2, axis=0) # L2 norm
        return reproj_error

def build_M(X3d,x2d):
    M = []

    assert len(X3d)==len(x2d)
    assert len(X3d) >= 6 # 2 equations and 11 unknowns means we need 6 points.

    for i in range(len(X3d)):
        X = X3d[i,0]
        Y = X3d[i,1]
        Z = X3d[i,2]
        x = x2d[i,0]
        y = x2d[i,1]

        M.append( [X, Y, Z, 1, 0, 0, 0, 0, 0, 0, 0, 0, -x] )
        M.append( [0, 0, 0, 0, X, Y, Z, 1, 0, 0, 0, 0, -y] )
        M.append( [0, 0, 0, 0, 0, 0, 0, 0, X, Y, Z, 1, -1] )
    return np.array(M)

def build_Bc(X3d,x2d):
    """Build matrix B and vector c to solve

    We want an equation of the form Bx = c where x is a vector of
    unknowns corresponding to the solution to the camera calibration.
    """
    B = []
    c = []

    assert len(X3d)==len(x2d)
    if len(X3d) < 6:
        print('WARNING: 2 equations and 11 unknowns means we need 6 points!')
    for i in range(len(X3d)):
        X = X3d[i,0]
        Y = X3d[i,1]
        Z = X3d[i,2]
        x = x2d[i,0]
        y = x2d[i,1]

        B.append( [X, Y, Z, 1, 0, 0, 0, 0, -x*X, -x*Y, -x*Z] )
        B.append( [0, 0, 0, 0, X, Y, Z, 1, -y*X, -y*Y, -y*Z] )

        c.append( x )
        c.append( y )
    return np.array(B), np.array(c)

# def center(P):
#     orig_determinant = np.linalg.det
#     def determinant( A ):
#         return orig_determinant( np.asarray( A ) )
#     # camera center
#     X = determinant( [ P[:,1], P[:,2], P[:,3] ] )
#     Y = -determinant( [ P[:,0], P[:,2], P[:,3] ] )
#     Z = determinant( [ P[:,0], P[:,1], P[:,3] ] )
#     T = -determinant( [ P[:,0], P[:,1], P[:,2] ] )

#     C_ = np.array( [[ X/T, Y/T, Z/T ]] ).T
#     return C_

def ransac_dlt(X3d, x2d,
               n = 6,     # six is minimum
               k = 200,   # do it 200 times
               t = 15.0,  # mean reprojection error should be less than 15
               d = 8,
               debug=False,
               ):
    """perform the DLT in RANSAC

    Params
    ------
    n: the minimum number of data values required to fit the model
    k: the maximum number of iterations allowed in the algorithm
    t: a threshold value for determining when a data point fits a model
    d: the number of close data values required to assert that a model fits well to data
    """
    model = DltRansacModel(X3d,x2d)
    data = np.arange( len(X3d) )


    return ransac.ransac(data,model,n,k,t,d,debug=debug,return_all=True)

def simple_dlt2(X3d, x2d):
    normalize = True
    if normalize:
        # normalize so that SVD has better numerical properties
        mx = np.mean(x2d[:,0])
        my = np.mean(x2d[:,1])
        s = 1.0/((mx+my)*0.5)
        N = np.array( [[ s, 0, -s*mx],
                       [ 0, s, -s*my],
                       [ 0, 0,  1]])

        x2dhT = np.hstack( (x2d, np.ones_like( x2d[:,np.newaxis,0] )) ).T
        xt = np.dot(N,x2dhT)
        x2dN = xt[:2].T

        use_2d = x2dN
    else:
        use_2d = x2d
    M = build_M(X3d,use_2d)
    U,s,V = np.linalg.svd(M, full_matrices=False)

    a = V[-1]

    assert a.shape == (13,)
    Mhat = a[:12]
    Mhat.shape=(3,4)

    Ninv = np.linalg.pinv(N)
    if normalize:
        Mhat = np.dot( Ninv, Mhat )
        assert Mhat.shape==(3,4)
    results = dict(#center = center(Mhat).T[0],
                   pmat = Mhat,
                   )
    return results

def simple_dlt(X3d, x2d):
    B,c = build_Bc(X3d,x2d)
    DLT_avec_results = np.linalg.lstsq(B,c)
    a_vec,residuals = DLT_avec_results[:2]
    Mhat = np.array(list(a_vec)+[1])
    Mhat.shape=(3,4)

    results = dict(#center = center(Mhat).T[0],
                   pmat = Mhat,
                   )
    return results

def dlt(X3d, x2d, ransac=True):
    X3d = np.array(X3d)
    x2d = np.array(x2d)
    if ransac:
        pmat,rd = ransac_dlt(X3d, x2d)
        result = dict(#center = center(pmat).T[0],
                      pmat = pmat,
                      )
        idxs = rd['inliers']
        result['X3d'] = X3d[idxs]
        result['x2d'] = x2d[idxs]

    else:
        result = simple_dlt(X3d, x2d)
        result['X3d'] = X3d
        result['x2d'] = x2d
    err = calc_reprojection_error( result['pmat'], result['X3d'], result['x2d'] )
    result['mean_reprojection_error'] = err['mean']
    result['reprojection_error'] = err['all']
    return result

def print_summary(results,n_pts=None):
    opts = np.get_printoptions()
    np.set_printoptions(precision=6, linewidth=150, suppress=True)
    try:
        if 1:
            import pymvg
            camera = pymvg.CameraModel.load_camera_from_M( results['pmat'], name='tmp' )

            testpts = results['X3d'][:n_pts]
            test2d = camera.project_3d_to_pixel(testpts, distorted=True)
            for i in range(len(testpts)):
                print('%s -> %s (expect %s, err %.1f)'%(testpts[i], test2d[i], results['x2d'][i], results['reprojection_error'][i]))
        #print '%d inliers, camera center: %s'%(len(results['x2d']), results['center'])
        print('mean err: %.1f'%(results['mean_reprojection_error'],))

    finally:
        np.set_printoptions(**opts)
