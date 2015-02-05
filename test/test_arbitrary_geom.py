import os
import numpy as np
from test_simple_geom import nan_shape_allclose
import time

# ROS imports
import roslib; roslib.load_manifest('flyvr')
import PyDisplaySurfaceArbitraryGeometry
import flyvr.rosmsg2json as rosmsg2json

def test_arbitrary_geom():
    filename = rosmsg2json.fixup_path( '$(find flyvr)/data/pyramid.osg' )
    model = PyDisplaySurfaceArbitraryGeometry.ArbitraryGeometry(filename=filename)

    # Use a few special texcoords because not all in range [0,1] are
    # valid for arbitrary geometries.

    eps = 1e-5
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

def arbitrary_geom_speed():
    filename = rosmsg2json.fixup_path( '$(find flyvr)/data/pyramid.osg' )
    model = PyDisplaySurfaceArbitraryGeometry.ArbitraryGeometry(filename=filename)

    N = 330000
    u = np.linspace(0.1, 0.2, N)
    v = 0.1 * np.ones_like(u)
    tc1 = np.array( [u,v] ).T
    wc1 = model.texcoord2worldcoord(tc1)

if __name__=='__main__':
    vals = []
    n_loops = 3
    for i in range(n_loops):
        start = time.time()
        arbitrary_geom_speed()
        stop = time.time()
        vals.append( stop-start )
    best = np.min(vals)
    print 'best of %d: %s'%(n_loops,best)
