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

DS_PKL          = decode_url('package://flycave/calibration/triplets/')
RERUN_MCSC      = False

config = yaml.load(open(decode_url('package://flycave/conf/calib-all.yaml')))

for n in (0,1,3):
    src = DS_PKL + "/ds%d" % n
    ds = '/display_server%d' % n
    ids = [ds] + config['display_servers'][ds]

    a = AllPointPickle()
    a.initilize_from_directory(src)

    print "*"*20
    print src
    print "*"*20

    if RERUN_MCSC:
        mcsc =  MultiCalSelfCam(src)
        mcsc.create_from_cams(
                        cam_ids=ids,
                        cam_resolutions=a.resolutions.copy(),
                        cam_points=a.results.copy(),
                        cam_calibrations={},
                        num_cameras_fill=0)
        dest = mcsc.execute(blocking=True, copy_files=True, silent=False)
    else:
        dest = src + "/result"
    MultiCalSelfCam.publish_calibration_points(
        dest,
        topic_base='/ds%d' % n)

rospy.spin()

