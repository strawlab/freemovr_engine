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
import flydra.reconstruct

rospy.init_node('calibone', anonymous=True)

PKL_DIR = decode_url('package://flycave/calibration/triplets/ds1')
OUT_DIR = decode_url('package://flycave/calibration/triplets/ds1/result')

a = AllPointPickle()
a.initilize_from_directory(PKL_DIR)

if 0:
    mcsc =  MultiCalSelfCam(OUT_DIR)
    mcsc.create_from_cams(
            cam_ids=a.cameras,
            cam_resolutions=a.resolutions.copy(),
            cam_points=a.results.copy(),
            cam_calibrations={},
            num_cameras_fill=0)
    mcsc.execute(blocking=True, dest=OUT_DIR, copy_files=False, silent=False)
    MultiCalSelfCam.publish_calibration_points(OUT_DIR)

r = flydra.reconstruct.Reconstructor(cal_source=OUT_DIR)

cams = r.get_cam_ids()
print cams

pts = a.get_points_in_cameras(*r.get_cam_ids(), result_format=list)
pt = pts[0:3]
print pt

X = r.find3d(pt,return_line_coords=False)
print X

pt = r.find2d('display_server1',X)
print pt

print [r.find2d(c,X) for c in cams]


#rospy.spin()

