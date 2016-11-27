import numpy as np

def mag(vec):
    return np.sqrt(np.sum(vec**2))

def norm(vec):
    return vec/mag(vec)

class PCA:
    # this is the matplotlib.mlab PCA class without the sigma normalization
    def __init__(self, a):
        """
        compute the SVD of a and store data for PCA.  Use project to
        project the data onto a reduced set of dimensions

        Inputs:

          *a*: a numobservations x numdims array

        Attrs:

          *a* a centered version of input a

          *numrows*, *numcols*: the dimensions of a

          *mu* : a numdims array of means of a

          *fracs* : the proportion of variance of each of the principal components

          *Wt* : the weight vector for projecting a numdims point or array into PCA space

          *Y* : a projected into PCA space


        The factor loadings are in the Wt factor, ie the factor
        loadings for the 1st principal component are given by Wt[0]

        """
        n, m = a.shape
        if n<m:
            raise RuntimeError('we assume data in a is organized with numrows>numcols')

        self.numrows, self.numcols = n, m
        self.mu = a.mean(axis=0)

        a = self.center(a)

        self.a = a

        U, s, Vh = np.linalg.svd(a, full_matrices=False)


        Y = np.dot(Vh, a.T).T

        vars = s**2/float(len(s))
        self.fracs = vars/vars.sum()


        self.Wt = Vh
        self.Y = Y


    def project(self, x, minfrac=0.):
        'project x onto the principle axes, dropping any axes where fraction of variance<minfrac'
        x = np.asarray(x)

        ndims = len(x.shape)

        if (x.shape[-1]!=self.numcols):
            raise ValueError('Expected an array with dims[-1]==%d'%self.numcols)


        Y = np.dot(self.Wt, self.center(x).T).T
        mask = self.fracs>=minfrac
        if ndims==2:
            Yreduced = Y[:,mask]
        else:
            Yreduced = Y[mask]
        return Yreduced



    def center(self, x):
        'center the data using the mean from training set a'
        return (x - self.mu)



def fit_cylinder(pts,ax3d=None):
    a = pts.T
    p = PCA(a)
    assert len(p.fracs)==3

    eps = 0.07
    if p.fracs[0] > 1./3+eps and p.fracs[1] < 1./3-eps:
        # tall aspect cylinder
        cyl_axis_dir = norm(p.Wt[0])
        height = 2*np.max( abs( p.Y[:,0] ) )
        circ_dims = p.Y[:,1:]
    elif p.fracs[2] < 1./3-eps:
        # wide aspect cylinder
        cyl_axis_dir = norm(p.Wt[2])
        height = 2*np.max( abs( p.Y[:,2] ) )
        circ_dims = p.Y[:,:2]
    else:
        # aspect ratio is near unity
        raise NotImplementedError('PCA to identify cylinder failed')

    cyl_axis = cyl_axis_dir*height
    center = p.mu
    mean_r = np.mean(np.sqrt(np.sum(circ_dims**2,axis=1)))

    if ax3d is not None:
        pp = p.project(pts.T).T
        ax3d.plot( pp[0], pp[1], pp[2], 'g.' )

    # make z of axis always positive
    if cyl_axis[2] < 0:
        cyl_axis = -cyl_axis
    r = {'center':center,
         'cyl_axis':cyl_axis,
         'radius':mean_r,
         }
    return r
