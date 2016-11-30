#!/usr/bin/env python
# ROS imports
import roslib; roslib.load_manifest('freemoovr')
import rospy
import rosbag

import pymvg

import numpy as np
import argparse
import json
import sys, glob
import matplotlib.pyplot as plt

from camera_calibration.calibrator import MonoCalibrator, ChessboardInfo
import cv2

def find_intrinsics(visualize=False):
    fatal = False
    good = []
    xys = []

    if 1:
        with open('setup.json',mode='r') as fd:
            setup = json.loads(fd.read())

    globber = '*-corners*.data'
    files = glob.glob(globber)
    files.sort()
    print 'all data files matching "%s"'%(globber,)
    print files
    for f in files:
        this_fatal = False
        parts = f.split('-')
        board = None
        for p in parts:
            if p.startswith('board'):
                boardname = p
                with open(boardname+'.json',mode='r') as fd:
                    buf = fd.read()
                boardd = json.loads(buf)
                board = ChessboardInfo()
                board.n_cols = boardd['n_cols']
                board.n_rows = boardd['n_rows']
                board.dim = boardd['dim']

        assert board is not None

        xy = np.loadtxt(f)
        xys.append(xy)
        if len(xy) != board.n_cols * board.n_rows:
            rospy.logfatal('Error: %d points in %s. Expected %d.'%(
                len(xy), f, board.n_cols * board.n_rows))
            this_fatal = True
            fatal = True
        if visualize:
            if 0:
                plt.figure()
            fmt = 'o-'
            label = f
            if this_fatal:
                fmt = 'kx:'
                label = 'BAD: '+ f
            plt.plot( xy[:,0], xy[:,1], fmt, mfc='none', label=label )
            if this_fatal:
                for i in range(len(xy)):
                    plt.text( xy[i,0],
                              xy[i,1],
                              str(i) )
        corners = [ (x,y) for (x,y) in xy ]
        good.append( (corners, board) )
    if visualize:
        plt.legend()
        plt.show()
    if fatal:
        sys.exit(1)

    mc = MonoCalibrator([ board ])#, cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST )
    mc.size = tuple(setup['size'])

    mc.cal_fromcorners(good)
    msg = mc.as_message()

    cam = pymvg.CameraModel.from_ros_like( intrinsics=msg )
    cam_mirror = cam.get_mirror_camera()

    if 1:

        # The intrinsics are valid for the whole physical display and
        # each virtual display.
        names = setup['names']

        for name in names:
            name = str(name) # remove unicode -- ROS bag fails on unicode topic name
            if 'mirror' in name:
                c = cam_mirror
            else:
                c = cam
            msg = c.get_intrinsics_as_msg()
            fname = 'display-intrinsic-cal-%s.bag'%( name.replace('/','-'), )
            bagout = rosbag.Bag(fname, 'w')
            topic = '/%s/camera_info'%name
            bagout.write(topic, msg)
            bagout.close()
            print 'saved to',fname

            print 'You can play this calibration with:\n'
            print 'rosbag play %s -l'%(fname,)
            print

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--visualize', action='store_true', default=False, help="show plot")
    args = parser.parse_args()

    find_intrinsics(visualize=args.visualize)
