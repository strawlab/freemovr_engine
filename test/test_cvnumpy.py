import roslib; roslib.load_manifest('freemovr_engine')

import freemovr_engine.cvnumpy as cvnumpy
import numpy as np

def test_rodrigues_cv():
    for x in np.linspace(-np.pi,np.pi,5):
        for y in np.linspace(-np.pi,np.pi,5):
            for z in np.linspace(-np.pi,np.pi,5):
                rvec = np.array((x,y,z))
                rmat = cvnumpy.rodrigues2matrix(rvec)
                rmat_cv = cvnumpy.rodrigues2matrix_cv(rvec)
                assert np.allclose(rmat, rmat_cv)
