import os
import numpy as np
from test_simple_geom import nan_shape_allclose

# ROS imports
import roslib; roslib.load_manifest('freemovr_engine')
import PyDisplaySurfaceArbitraryGeometry
import freemovr_engine.fixup_path as fixup_path

def test_arbitrary_geom():
    filename = fixup_path.fixup_path( '$(find freemovr_engine)/data/pyramid.osg' )
    model = PyDisplaySurfaceArbitraryGeometry.ArbitraryGeometry(filename=filename,precision=1e-6)

    # Use a few special texcoords because not all in range [0,1] are
    # valid for arbitrary geometries.

    eps = 1e-5;
    tc1 = np.array( [[eps,eps],
                     [0.1, 0.1],
                     [0.2, 0.1],
                     [0.4, 0.2],
                     ] )
    wc1 = model.texcoord2worldcoord(tc1)
    tc2 = model.worldcoord2texcoord(wc1)
    wc2 = model.texcoord2worldcoord(tc2)
    assert nan_shape_allclose( tc1, tc2)
    assert nan_shape_allclose( wc1, wc2 )
