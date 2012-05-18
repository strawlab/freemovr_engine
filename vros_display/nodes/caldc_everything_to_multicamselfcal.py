#!/usr/bin/env python

# standard Python imports
import argparse
import time
import os.path
import pickle
import scipy
import math

import numpy as np
import scipy.ndimage
import cv,cv2

import skimage.feature

# ROS imports
import roslib;
roslib.load_manifest('vros_display')
roslib.load_manifest('camera_trigger')
roslib.load_manifest('std_srvs')
import rospy

# local vros_display imports
import display_client
import camera_trigger.srv
import std_srvs.srv

from calib.acquire import CameraHandler, SimultainousCameraRunner
from calib.io import MultiCalSelfCam
from calib.imgproc import DotBGFeatureDetector

class Calib:

    MODE_POINTS = 0
    MODE_PROJECTOR_VIS_SETUP = 1
    MODE_PROJECTOR_VIS = 2
    MODE_PROJECTOR_IR = 3
    MODE_FINISHED = 4

    def __init__(self, tracking_cameras, display_servers, trigger, show_cameras, show_type, diff_thresh):
        rospy.wait_for_service(trigger+'/set_framerate')

        self.trigger_proxy_rate = rospy.ServiceProxy(trigger+'/set_framerate', camera_trigger.srv.SetFramerate)
        self.trigger_proxy_rate(0.0)
        self.trigger_proxy_once = rospy.ServiceProxy(trigger+'/trigger_once', std_srvs.srv.Empty)

        s = rospy.Service('~calib_mode_points', std_srvs.srv.Empty, self._change_mode_srv_points)
        s = rospy.Service('~calib_mode_projector_visible', std_srvs.srv.Empty, self._change_mode_srv_projvis)
        s = rospy.Service('~calib_mode_projector_ir', std_srvs.srv.Empty, self._change_mode_srv_projir)
        s = rospy.Service('~calib_finish', std_srvs.srv.Empty, self._change_mode_srv_fin)

        self.show_cameras = show_cameras
        if show_cameras:
            cv2.startWindowThread()

        self.results = {}
        self.num_results = 0
        self.mcsc = MultiCalSelfCam('./mcamall')

        self.display_servers = {}   #name : (display_client,[cams])
        projector_cameras = []
        for d,cams in display_servers.items():
            rospy.loginfo("Calibrating display server %s" % d)
            projector_cameras.extend(cams)
            self.results[d] = []
            dsc = display_client.DisplayServerProxy(d,wait=True)
            dsc.enter_2dblit_mode()
            dsc.show_pixels(dsc.new_image(dsc.IMAGE_COLOR_BLACK))
            self.display_servers[d] = (dsc,cams)

        self.detectors = {}
        cam_handlers = []
        for cam in tracking_cameras+projector_cameras:
            rospy.loginfo("Calibrating camera %s" % cam)
            self.detectors[cam] = DotBGFeatureDetector(
                                cam,
                                method="med",
                                show=show_type if (show_cameras[0] == "all" or cam in show_cameras) else "",
                                diff_thresh=diff_thresh)
            cam_handlers.append(CameraHandler(cam,debug=False))
            self.results[cam] = []

        self.runner = SimultainousCameraRunner(cam_handlers)
        self.change_mode(self.MODE_POINTS)

    def change_mode(self, mode):
        rospy.loginfo("Changing to mode %d" % mode)
        self.mode = mode
    def _change_mode_srv_points(self, req):
        self.change_mode(self.MODE_POINTS)
        return std_srvs.srv.EmptyResponse()
    def _change_mode_srv_projvis(self, req):
        self.change_mode(self.MODE_PROJECTOR_VIS_SETUP)
        return std_srvs.srv.EmptyResponse()
    def _change_mode_srv_projir(self, req):
        self.change_mode(self.MODE_PROJECTOR_IR)
        return std_srvs.srv.EmptyResponse()
    def _change_mode_srv_fin(self, req):
        self.change_mode(self.MODE_FINISHED)
        return std_srvs.srv.EmptyResponse()

    def _detect_points(self, runner, restrict={}):
        runner.get_images(1, self.trigger_proxy_once)
        imgs = runner.result_as_nparray
        detected = {}
        visible = 0
        for cam in imgs:
            if restrict and cam not in restrict:
                continue
            features = self.detectors[cam].detect(imgs[cam][:,:,0])
            if features:
                print "detect: ",cam,features
                #take the first point, but convert to pixel coords
                row,col = features[0]
                detected[cam] = (col,row)
                visible += 1
            else:
                detected[cam] = (np.nan, np.nan)
        return detected,visible

    def run(self):
        rospy.loginfo("Collecting backgrounds")
        #collect bg images
        self.runner.get_images(20, self.trigger_proxy_rate, [5], self.trigger_proxy_rate, [0])
        imgs = self.runner.result_as_nparray
        for cam in imgs:
            #collect the background model
            self.detectors[cam].compute_bg(imgs[cam])

        rospy.loginfo("Collecting points")
        while not rospy.is_shutdown() and self.mode != self.MODE_FINISHED:
            if self.mode == self.MODE_POINTS:
                detected,nvisible = self._detect_points(self.runner)
                if nvisible >= 3:
                    self.num_results += 1
                    for cam in detected:
                        print "save #%d (%d collected)" % (nvisible, self.num_results)
                        self.results[cam].append(detected[cam])
                    for ds in self.display_servers:
                        self.results[ds].append((np.nan, np.nan))
            elif self.mode == self.MODE_PROJECTOR_VIS_SETUP:
                for d,(dsc,cams) in self.display_servers.items():
                    midh = int(dsc.height)/2
                    midw = int(dsc.width)/2

                    rospy.loginfo("Lighting projector pixel %d,%d" % (midh,midw))

                    arr = dsc.new_image(dsc.IMAGE_COLOR_BLACK)
                    arr[midh-1:midh+1,midw-1:midw+1,:3] = dsc.IMAGE_COLOR_WHITE
                    dsc.show_pixels(arr)
                    self.change_mode(self.MODE_PROJECTOR_VIS)
            elif self.mode == self.MODE_PROJECTOR_VIS:
                for d,(dsc,cams) in self.display_servers.items():
                    detected,nvisible = self._detect_points(self.runner, restrict=cams)

            rospy.sleep(0.1)

        if self.show_cameras:
            cv2.destroyAllWindows()

        resolutions = {}
        for cam,det in self.detectors.items():
            resolutions[cam] = (det.img_width_px,det.img_height_px)
        for d,(ds,cams) in self.display_servers.items():
            resolutions[d] = (ds.width, ds.height)

        with open('allcalibresults.pkl','w') as f:
            pickle.dump(self.results,f)
        with open('allcalibresolution.pkl','w') as f:
            pickle.dump(resolutions,f)

        self.mcsc.create_from_cams(
                cam_ids=self.detectors.keys(),
                cam_resolutions=resolutions,
                cam_points=self.results)


if __name__ == '__main__':

    CONFIG={"trigger":"/camera_trigger",
            "display_servers":{"/display_server0":["/Basler_21220788"]},
            "tracking_cameras":["/Basler_21020228", "/Basler_21020229", "/Basler_21020232", "/Basler_21020233", "/Basler_21029382", "/Basler_21029383", "/Basler_21029385"]}

    CONFIG={"trigger":"/camera_trigger",
            "display_servers":{"/display_server0":[]},
            "tracking_cameras":["/Basler_21020228", "/Basler_21020229", "/Basler_21020232", "/Basler_21020233", "/Basler_21029382", "/Basler_21029383", "/Basler_21029385"]}

    CONFIG={"trigger":"/camera_trigger",
            "display_servers":{"/display_server0":[], #"/Basler_21220782", "/Basler_21220789"
                               "/display_server1":[],
                               "/display_server3":[]},
            "tracking_cameras":["/Basler_21020228", "/Basler_21020229", "/Basler_21020232", "/Basler_21020233", "/Basler_21029382", "/Basler_21029383", "/Basler_21029385"]}


#    CONFIG={"trigger":"/camera_trigger",
#            "display_servers":{"/display_server0":["/Basler_21220788"]},
#            "tracking_cameras":["/Basler_21029382", "/Basler_21029383"]}

    print "TODO: add service to change mode, do display pixel calibraion"

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--show-cameras', type=str, default=("",),
        help='show images with the given topics. see --show-type for a description of the available images types',
        nargs='*')
    parser.add_argument(
        '--show-type', type=str, default='IF',
        help='which images types to show. %s' % ', '.join(["%s=%s" % i for i in DotBGFeatureDetector.WIN_TYPES.items()]))
    parser.add_argument(
        '--diff-thresh', type=int, default=70,
        help='bg diff thresh')


    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])
    
    rospy.init_node('multicamselfcal_everything')

    c = Calib(tracking_cameras=CONFIG["tracking_cameras"],
              display_servers=CONFIG["display_servers"],
              trigger=CONFIG["trigger"],
              show_cameras=args.show_cameras,
              show_type=set(args.show_type),
              diff_thresh=args.diff_thresh)
    c.run()

