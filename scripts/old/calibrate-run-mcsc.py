#!/usr/bin/env python

import roslib;
roslib.load_manifest('flyvr')
import rospy

from calib.io import MultiCalSelfCam

import os.path
import tempfile
import argparse

def run_mcsc(path):
    mcsc = MultiCalSelfCam(path)
    dest = mcsc.execute(blocking=True, dest=tempfile.mkdtemp(),silent=False)
    pcd = '/tmp/mcscpoints.pcd'
    MultiCalSelfCam.save_to_pcd(dirname=dest, fname=pcd)
    print 'saved to pcd file', pcd

if __name__ == '__main__':
    rospy.init_node('mcsc', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, required=True, help='path to mcsc dir')
    args = parser.parse_args()

    path = os.path.abspath(os.path.expanduser(args.path))
    assert os.path.exists(path)

    run_mcsc(path)

