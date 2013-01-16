import os.path

import numpy as np

import roslib;
roslib.load_manifest('flyvr')
roslib.load_manifest('motmot_ros_utils')
import rospy

from calib.io import MultiCalSelfCam, AllPointPickle
from calib.visualization import create_point_cloud_message_publisher
from rosutils.io import decode_url
from rosutils.formats import camera_calibration_yaml_to_radfile

import flydra
import flydra.reconstruct


rospy.init_node('showpointspkl', anonymous=True)

LASER_PKL       = decode_url('package://flycave/calibration/laser')
FLYDRA_CALIB    = decode_url('package://flycave/calibration/flydra')

#make sure the flydra cameras are intrinsically calibrated
name_map = MultiCalSelfCam.get_camera_names_map(FLYDRA_CALIB)
for c in MultiCalSelfCam.read_calibration_names(FLYDRA_CALIB):
    camera_calibration_yaml_to_radfile(
            decode_url('package://flycave/calibration/cameras/%s.yaml' % c),
            os.path.join(FLYDRA_CALIB,name_map[c]))

fly = flydra.reconstruct.Reconstructor(cal_source=FLYDRA_CALIB)

laser = AllPointPickle()
laser.initilize_from_directory(LASER_PKL)

print "number of laser points",laser.num_points

#get all points visible in 2 or more cameras
#and use the flydra calibration to get the 3d coords
pts = laser.get_points_in_cameras(fly.get_cam_ids())
xyz = np.array([fly.find3d(pt,return_line_coords=False,undistort=True) for pt in pts])
create_point_cloud_message_publisher(
        xyz,
        topic_name='/flydracalib/points',
        publish_now=True,
        latch=True)

rospy.spin()
