#!/usr/bin/env python

# ROS imports
import roslib; roslib.load_manifest('freemoovr_engine')
import rospy
import freemoovr_engine.simple_geom as simple_geom
import glob
import json
from fit_extrinsics import fit_extrinsics

import numpy as np

import sys

bagfile=sys.argv[1]

if 1:
    globber = '*-corners*.data'
    files = glob.glob(globber)
    files.sort()
    print 'all data files matching "%s"'%(globber,)
    print files
    for f  in files:
        parts = f.split('-')
        board = None
        for p in parts:
            if p.startswith('board'):
                boardname = p
                with open(boardname+'.json',mode='r') as fd:
                    buf = fd.read()
                board = json.loads(buf)
        assert board is not None

        xy = np.loadtxt(f)

        n_cols = board['n_cols']
        n_rows = board['n_rows']
        dim = board['dim']

        x = np.expand_dims(np.arange(n_cols)*dim,0)
        y = np.expand_dims(np.arange(n_rows)*dim,1)
        XX, YY = np.broadcast_arrays(x, y)
        assert XX.shape == (n_rows, n_cols)
        assert YY.shape == (n_rows, n_cols)
        ZZ = np.zeros_like(XX)

        X3d = np.array([ XX.flatten(), YY.flatten(), ZZ.flatten() ]).T
        x2d = xy

        results = fit_extrinsics( bagfile,
                                  X3d,
                                  x2d,
                                  name='hacked' )
        cam = results['cam']
        projected = cam.project_3d_to_pixel(X3d)

        if 0:
            print X3d[:20]
            print x2d[:20]
            print projected[:20]
        print f, results['mean_err']
