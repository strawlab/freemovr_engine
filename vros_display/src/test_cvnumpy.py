import roslib; roslib.load_manifest('vros_display')

import cvnumpy
import numpy as np
from camera_model.camera_model import is_rotation_matrix

def test_rodrigues_cv():
    for x in np.linspace(-np.pi,np.pi,5):
        for y in np.linspace(-np.pi,np.pi,5):
            for z in np.linspace(-np.pi,np.pi,5):
                rvec = np.array((x,y,z))
                rmat = cvnumpy.rodrigues2matrix(rvec)
                rmat_cv = cvnumpy.rodrigues2matrix_cv(rvec)
                assert np.allclose(rmat, rmat_cv)
