#!/usr/bin/env python

# standard Python imports
import argparse
import numpy as np
import time
import os.path
import pickle
import scipy
import math

# ROS imports
import roslib;
roslib.load_manifest('vros_display')
roslib.load_manifest('camera_trigger')
import rospy

# local vros_display imports
import display_client
import camera_trigger.srv
import std_srvs.srv

from calib.acquire import CameraHandler, SimultainousCameraRunner
from calib.io import MultiCalSelfCam

def all_of_the_things(topic_prefixes, display_server_prefixes, trigger_prefix):
    rospy.wait_for_service(trigger_prefix+'/set_framerate')

    #ensure trigger is running
    trigger_proxy = rospy.ServiceProxy(trigger_prefix+'/set_framerate', camera_trigger.srv.SetFramerate)
    trigger_proxy(0.0)

    trigger_proxy = rospy.ServiceProxy(trigger_prefix+'/trigger_once', std_srvs.srv.Empty)

    cam_handlers = [CameraHandler(prefix,debug=True) for prefix in topic_prefixes]
    runner = SimultainousCameraRunner(cam_handlers)

    for i in range(5):
        print "waiting"
        time.sleep(2)
        print "triggering"
        tstart = time.time()
        trigger_proxy()
        imgs = runner.get_images(tstart,n_per_camera=1)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--camera-topic-prefixes', type=str, required=True,
        help='camera topic prefix of the images used to view the projector (e.g. /camnode)',
        nargs='*')
    parser.add_argument(
        '--display-server-topic-prefixes', type=str,
        help='display server topic prefixs', required=True,
        nargs='*')
    parser.add_argument(
        '--trigger-prefix', type=str, default="/camera_trigger",
        help='camera trigger ROS prefix')

    argv = rospy.myargv()
    print argv
    args = parser.parse_args(argv[1:])
    
    rospy.init_node('multicamselfcal_everything', anonymous=True)

    all_of_the_things(topic_prefixes=args.camera_topic_prefixes,
                      display_server_prefixes=args.display_server_topic_prefixes,
                      trigger_prefix=args.trigger_prefix)

