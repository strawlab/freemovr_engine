#!/usr/bin/env python
import os

import roslib; roslib.load_manifest('flyvr')
import rosbag

import sensor_msgs.msg
import geometry_msgs.msg

import pymvg

import numpy as np

def test_bag():
    for i in range(1000):
        bagout = rosbag.Bag('/tmp/testbag.bag', 'w')
        topic = '/tf'
        extrinsics = geometry_msgs.msg.Transform()
        bagout.write(topic, extrinsics)
        topic = '/camera_info'
        intrinsics = sensor_msgs.msg.CameraInfo()
        intrinsics.distortion_model = ','*i
        intrinsics.K = list(np.random.rand(3,3).flatten())
        bagout.write(topic, intrinsics)
        bagout.close()

    c = pymvg.CameraModel.load_camera_simple()
    c.save_to_bagfile('/tmp/testbag.bag')

    fname = '/home/stowers/.ros/camera_info/Basler_21220788.yaml'
    assert os.path.exists(fname)
    c = pymvg.CameraModel.load_camera_from_file(fname)
    c.get_intrinsics_as_msg()
    c.save_to_bagfile('/tmp/testbag.bag')

    c = pymvg.CameraModel.load_camera_from_file('/tmp/testbag.bag')
    c.get_intrinsics_as_msg()
