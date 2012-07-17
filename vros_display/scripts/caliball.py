# standard Python imports
import argparse
import time
import os.path
import math
import fnmatch
import threading

import json
import yaml
import numpy as np
import scipy.ndimage
import scipy.misc
import cv,cv2

# ROS imports
import roslib;
roslib.load_manifest('vros_display')
roslib.load_manifest('motmot_ros_utils')
import rospy

import calib
from calib.io import MultiCalSelfCam, AllPointPickle
from rosutils.io import decode_url

import flydra
import flydra.align

rospy.init_node('caliball', anonymous=True)

ALL_DATA        = decode_url('package://flycave/data/calib-all/1208_3PROJ/')
FLYDRA_CALIB    = decode_url('package://flycave/calibration/flycave')
RERUN_MCSC      = False

config = yaml.load(open(decode_url('package://flycave/conf/calib-all.yaml')))

a = AllPointPickle()
a.initilize_from_directory(ALL_DATA)

MultiCalSelfCam.publish_calibration_points(
    FLYDRA_CALIB,
    topic_base='/flydracalib')

for n in (0,1,3,):
    ds = '/display_server%d' % n
    ids = [ds] + config['display_servers'][ds]
    dest = decode_url('package://flycave/calibration/ds%d' % n)

    print "*"*20
    print dest
    print "*"*20

    if RERUN_MCSC:
        mcsc =  MultiCalSelfCam(dest)
        mcsc.create_from_cams(
                        cam_ids=ids,
                        cam_resolutions=a.resolutions.copy(),
                        cam_points=a.results.copy(),
                        cam_calibrations={},
                        num_cameras_fill=0)
        mcsc.execute(blocking=True, dest=dest, copy_files=False, silent=False)
    MultiCalSelfCam.publish_calibration_points(
        dest,
        topic_base='/ds%d' % n)

rospy.spin()

