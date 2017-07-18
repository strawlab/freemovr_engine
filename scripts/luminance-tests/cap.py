#!/usr/bin/env python

import numpy as np
import scipy.misc
import time

import roslib;
roslib.load_manifest('freemovr_engine')
roslib.load_manifest('camera_trigger')
roslib.load_manifest('flycave')
roslib.load_manifest('std_srvs')
roslib.load_manifest('std_msgs')
roslib.load_manifest('motmot_ros_utils')
roslib.load_manifest('rosbag')
import rospy

import freemovr_engine.display_client as display_client
import calib.acquire

if __name__ == "__main__":
    dsn = "/display_server3"
    camn = "/Basler_21017520"
    nimgs = 20
    greys = range(0,255+5,5) + [64,127,191]

    col,row = 519,390
    ptsize = 3

    rospy.init_node('show_foo')

    dsc = display_client.DisplayServerProxy(dsn)
    dsc.enter_2dblit_mode()

    runner = calib.acquire.SequentialCameraRunner(
                                (calib.acquire.CameraHandler(camn),),
                                queue_depth=nimgs)

    for g in greys:
        print "GRAY %d" % g

        rgb = (g,g,g)

        arr = dsc.new_image(dsc.IMAGE_COLOR_BLACK, None)
        for i,c in enumerate(rgb):
            arr[row-ptsize:row+ptsize,col-ptsize:col+ptsize,i] = c

        dsc.show_pixels(arr)
        time.sleep(2)

        runner.get_images(nimgs)
        imgs = runner.result_as_nparray

        for n in range(nimgs):

            img = imgs[camn][:,:,n]
            assert img.dtype == np.uint8

            scipy.misc.imsave("gray%d_%d.png" % (g,n), img)

    print "DONE"
    rospy.spin()
