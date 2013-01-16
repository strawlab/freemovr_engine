#!/usr/bin/env python

import roslib; roslib.load_manifest('camera_model')
import rosbag

import sensor_msgs.msg
import geometry_msgs.msg

import camera_model

import numpy as np

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

c = camera_model.load_default_camera()
c.save_to_bagfile('/tmp/testbag.bag')

c = camera_model.load_camera_from_file('/home/stowers/.ros/camera_info/Basler_21220788.yaml')
c.get_intrinsics_as_msg()
c.save_to_bagfile('/tmp/testbag.bag')

c = camera_model.load_camera_from_file('/tmp/testbag.bag')
c.get_intrinsics_as_msg()
