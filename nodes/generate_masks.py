#!/usr/bin/env python

# standard Python imports
import argparse
import os.path

import yaml
import numpy as np
import scipy.misc

# ROS imports
import roslib;
roslib.load_manifest('flyvr')
roslib.load_manifest('motmot_ros_utils')
roslib.load_manifest('std_srvs')
roslib.load_manifest('camera_trigger')
import rospy

# local flyvr imports
import flyvr.display_client as display_client
import std_srvs.srv
import camera_trigger.srv

from flyvr.calibration.simple.acquire import CameraHandler, SimultaneousCameraRunner
from rosutils.io import decode_url

class GenMasks:

    MODE_WAIT = 0
    MODE_FINISHED = 1
    MODE_COLLECT = 2

    def __init__(self, display_servers, cams, mask_dir, trigger):
        self.mask_dir = mask_dir

        cam_handlers = []

        #turn on projectors
        for d in display_servers:
            dsc = display_client.DisplayServerProxy(d,wait=True)
            dsc.enter_2dblit_mode()
            dsc.show_pixels(dsc.new_image(dsc.IMAGE_COLOR_WHITE))

        for cam in cams:
            rospy.loginfo("Generating mask for cam %s" % cam)
            cam_handlers.append(CameraHandler(cam,debug=False))

        self.trigger_proxy_rate = rospy.ServiceProxy(trigger+'/set_framerate', camera_trigger.srv.SetFramerate)
        self.trigger_proxy_rate(1.0)
        rospy.loginfo("Set framerate to 1fps")

        self.runner = SimultaneousCameraRunner(cam_handlers)

        s = rospy.Service('~mask_start', std_srvs.srv.Empty, self._change_mode_start)
        s = rospy.Service('~mask_finish', std_srvs.srv.Empty, self._change_mode_finish)

        self.mode = self.MODE_WAIT

    def _change_mode_start(self, req):
        self.mode = self.MODE_COLLECT
        return std_srvs.srv.EmptyResponse()
    def _change_mode_finish(self, req):
        self.mode = self.MODE_FINISHED
        return std_srvs.srv.EmptyResponse()

    def run(self):
        while not rospy.is_shutdown() and self.mode != self.MODE_FINISHED:
            if self.mode == self.MODE_WAIT:
                pass
            elif self.mode == self.MODE_COLLECT:
                rospy.loginfo("Collecting images")
                self.runner.get_images(1)
                imgs = self.runner.result_as_nparray
                for cam in imgs:
                    fname = self.mask_dir + "/%s-new.png" % cam.split("/")[-1]
                    print fname, imgs[cam].shape,imgs[cam].dtype
                    scipy.misc.imsave(fname,imgs[cam].squeeze())
                    rospy.loginfo("wrote %s " % fname)
                self.mode = self.MODE_WAIT

            rospy.sleep(0.1)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--calib-config', type=str, default='package://flycave/conf/calib-all.yaml',
        help='path to calibration configuration yaml file')

    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])
    
    rospy.init_node('generate_masks')

    config = {}
    conffile = decode_url(args.calib_config)
    with open(conffile, 'r') as f:
        config = yaml.load(f)
        for k in ("display_servers", "mask_dir", "tracking_cameras"):
            if not config.has_key(k):
                parser.error("malformed calibration config, missing %s" % k)

    mask_dir = decode_url(config["mask_dir"])
    if not os.path.isdir(mask_dir):
        parser.error("mask dir not found")

    display_servers = config["display_servers"]

    all_cams = []
    for cam in config["tracking_cameras"]:
        all_cams.append(cam)

    c = GenMasks(display_servers=display_servers,
                 cams=all_cams,
                 mask_dir=mask_dir,
                 trigger="camera_trigger")
    c.run()

