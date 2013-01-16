import numpy as np
import ransac

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

def build_Bc(X3d,x2d):
    B = []
    c = []

    assert len(X3d)==len(x2d)
    if len(X3d) < 6:
        print 'WARNING: 2 equations and 11 unknowns means we need 6 points!'
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

def ransac_dlt(X3d, x2d):
    model = DltRansacModel(X3d,x2d)
    data = np.arange( len(X3d) )

    # n - the minimum number of data values required to fit the model
    # k - the maximum number of iterations allowed in the algorithm
    # t - a threshold value for determining when a data point fits a model
    # d - the number of close data values required to assert that a model fits well to data

    n = 6    # six is minimum
    k = 200  # do it 200 times
    t = 15.0  # mean reprojection error should be less than 15
    d = 20   # 20 points and we're doing well

    return ransac.ransac(data,model,n,k,t,d,debug=True,return_all=True)

def simple_dlt(X3d, x2d):
    B,c = build_Bc(X3d,x2d)
    DLT_avec_results = np.linalg.lstsq(B,c)
    a_vec,residuals = DLT_avec_results[:2]
    Mhat = np.array(list(a_vec)+[1])
    Mhat.shape=(3,4)
    results = dict(center = center(Mhat).T[0],
                   pmat = Mhat,
                   #residuals=residuals[0],
                   )
    return results

def dlt(X3d, x2d, ransac=True):
    X3d = np.array(X3d)
    x2d = np.array(x2d)
    if ransac:
        pmat,rd = ransac_dlt(X3d, x2d)
        result = dict(center = center(pmat).T[0],
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

def print_summary(results):
    opts = np.get_printoptions()
    np.set_printoptions(precision=6, linewidth=150, suppress=True)
    try:
        if 1:
            import camera_model
            camera = camera_model.load_camera_from_pmat( results['pmat'], name='tmp' )

            testpts = results['X3d'][:20]
            test2d = camera.project_3d_to_pixel(testpts, distorted=True)
            for i in range(len(testpts)):
                print '%s -> %s (expect %s, err %.1f)'%(testpts[i], test2d[i], results['x2d'][i], results['reprojection_error'][i])
        print '%d inliers, camera center: %s'%(len(results['x2d']), results['center'])
        print 'mean err: %.1f'%(results['mean_reprojection_error'],)

    finally:
        np.set_printoptions(**opts)
