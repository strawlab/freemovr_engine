#!/usr/bin/env python

# standard Python imports
import argparse
import time
import os.path
import math
import fnmatch
import threading
import tempfile
import datetime
import collections
import traceback

import json
import yaml
import numpy as np
import numpy.linalg
import scipy.ndimage
import scipy.misc
import cv,cv2
import random

# ROS imports
import roslib;
roslib.load_manifest('flyvr')
roslib.load_manifest('camera_trigger')
roslib.load_manifest('flycave')
roslib.load_manifest('std_srvs')
roslib.load_manifest('std_msgs')
roslib.load_manifest('motmot_ros_utils')
roslib.load_manifest('rosbag')
import rospy
import rosbag

# local flyvr imports
import flyvr.display_client as display_client
import camera_trigger.srv
import std_srvs.srv
import flycave.srv
import flyvr.srv

import calib
import calib.imgproc
import calib.kdtree
from calib.acquire import CameraHandler, SimultainousCameraRunner, SequentialCameraRunner
from calib.imgproc import DotBGFeatureDetector, load_mask_image, add_crosshairs_to_nparr
from calib.sampling import gen_horiz_snake, gen_vert_snake, gen_spiral_snake
from calib.calibrationconstants import *

from rosutils.io import decode_url

import flydra.reconstruct

from flyvr.msg import Calib2DPoint, CalibMapping
from geometry_msgs.msg import Point32
from std_msgs.msg import UInt32, String

class Calib:

    def __init__(self, config, show_cameras, show_display_servers, show_type, debug):
        tracking_cameras = config["tracking_cameras"]
        trigger = "/camera_trigger"

        self.display_servers = config["display_servers"]
        self.ptsize = int(config["projector_point_size_px"])
        self.visible_thresh = int(config["bg_thresh_visible"])
        self.mask_dir = decode_url(config["mask_dir"])
        self.flydra = flydra.reconstruct.Reconstructor(
                        cal_source=decode_url(config["tracking_calibration"]))


        rospy.wait_for_service(trigger+'/set_framerate')
        self.trigger_proxy_rate = rospy.ServiceProxy(trigger+'/set_framerate', camera_trigger.srv.SetFramerate)
        self.trigger_proxy_once = rospy.ServiceProxy(trigger+'/trigger_once', std_srvs.srv.Empty)
        self.trigger_proxy_rate(0.0)

        self.pub_mode = rospy.Publisher('~mode', String)
        
        self.show_cameras = show_cameras
        self.show_display_servers = {}
        if show_cameras or show_display_servers or show_laser_scatter:
            cv2.startWindowThread()

        for d in self.display_servers:
            dsc = display_client.DisplayServerProxy(d,wait=True)
            dsc.enter_2dblit_mode()

            self.display_servers[d]["display_client"] = dsc
            rospy.loginfo("Calibrating %s" % d)

            if d in show_display_servers or show_display_servers[0] == "all":
                cv2.namedWindow(d)
                self.show_display_servers[d] = {}

        #ensure all the projectors are black
        self._light_proj_cache = {d:() for d in self.display_servers}
        for d in self._light_proj_cache:
            self._black_projector(d)
            
        #tracking (flydra) cameras and acquisition
        self.tracking_cameras = {}
        cam_handlers = []
        for cam in tracking_cameras:
            fd = DotBGFeatureDetector(
                    cam,
                    method="med",
                    show=show_type if (show_cameras[0] == "all" or cam in show_cameras) else "")
            if "detection" in debug:
                fd.enable_debug_detection()
            if "saveimages" in debug:
                fd.enable_debug_images("/mnt/ssd/CALIB/")
            if "benchmark" in debug:
                fd.enable_benchmark()
            self.tracking_cameras[cam] = fd
            cam_handlers.append(CameraHandler(cam,debug="acquisition" in debug))
            rospy.loginfo("Connecting to cam %s" % cam)
            self._set_bg_mask(cam, fd)
        self.runner = SimultainousCameraRunner(cam_handlers)
        
        self._calculate_background()

        self.mode_lock = threading.Lock()
        self.mode_args = tuple()

        s = rospy.Service('~calib_change_mode', flyvr.srv.CalibMode, self._on_change_mode)

        self.change_mode(CALIB_MODE_SLEEP)

    def change_mode(self, mode, *service_args):
        with self.mode_lock:
            self.mode = mode
            self.mode_args = service_args
            rospy.loginfo("Changing to mode -> %s (args %s)" % (mode,repr(self.mode_args)))

    def _on_change_mode(self, req):
        self.change_mode(req.mode, req.sa, req.fa, req.fb, req.fc)
        return flyvr.srv.CalibModeResponse()

    def _set_bg_mask(self, cam, detector):
        mask_name = os.path.join(self.mask_dir,cam.split("/")[-1]) + ".png"
        if os.path.exists(mask_name):
            arr = load_mask_image(mask_name)
            detector.set_mask(arr)
            rospy.loginfo("Setting %s mask = %s" % (cam,mask_name))


    def _calculate_background(self):
        rospy.loginfo("Collecting backgrounds")
        #collect bg images
        self.runner.get_images(20, self.trigger_proxy_rate, [5], self.trigger_proxy_rate, [0])
        imgs = self.runner.result_as_nparray
        for cam in imgs:
            #collect the background model
            self.tracking_cameras[cam].compute_bg(imgs[cam])
            rospy.loginfo("Calculate background for %s" % cam)
        rospy.loginfo("Collecting backgrounds finished")

    def _parse_ds_specified(self, args):
        try:
            spec = args[0]
            ds,vdisp = spec.split("/")
            vdispinfo = None
            centroid = []
            for vd in self.display_servers[ds]["virtualDisplays"]:
                if vd["id"] == vdisp:
                    vdispinfo = vd
                    colm = int(args[1])
                    rowm = int(args[2])
                    if colm > 0 and rowm > 0:
                        centroid = [colm,rowm]
            if not vdispinfo:
                rospy.logwarn("vdisp %s not found in display server %s" % (vdisp,ds))
                raise ValueError
        except (ValueError, KeyError):
            rospy.logwarn("invalid display_server/vdisp specifier")
            raise

        return ds,vdisp,vdispinfo,centroid

    def _black_projector(self, ds):
        self._light_proj_pixel(ds, None, None)

    def _light_proj_pixel(self, ds, row, col, black_others=True):
        row = math.floor(row) if row != None else None
        col = math.floor(col) if col != None else None

        print row,col

        if black_others:
            for ods in self._light_proj_cache:
                if ods != ds:
                    self._light_proj_pixel(ods, None, None, False)

        target = (col, row)
        if self._light_proj_cache[ds] == target:
            rospy.logdebug("not lighting projector %s col:%s row:%s" % (ds,col,row))
            return

        dsc = self.display_servers[ds]["display_client"]
        #create the image to send to the dsc
        arr = dsc.new_image(dsc.IMAGE_COLOR_BLACK, mask=None)
        
        if col != None:
            sz = self.ptsize
            ri = arr.shape[0]
            ci = arr.shape[1]
            arr[max(0,row-sz):min(row+sz,ri),max(0,col-sz):min(col+sz,ci),:3] = dsc.IMAGE_COLOR_WHITE

        dsc.show_pixels(arr)
        
        if ds in self.show_display_servers:
            handle = self.show_display_servers[ds]["handle"]
            img =  self.show_display_servers[ds]["visualizeimg"].copy()
            if col != None:
                add_crosshairs_to_nparr(arr=img, row=row, col=col, sz=-1, fill=255, chan=1)
            cv2.imshow(handle, img)
            if self.__ds_fmt:
                cv2.imwrite(self.__ds_fmt%{"time":time.time()},img)

        rospy.logdebug("lighting projector %s col:%s row:%s" % (ds,col,row))
        self._light_proj_cache[ds] = target

        rospy.sleep(0.5)

    def _detect_points(self, runner, thresh, restrict={}):
        runner.get_images(1, self.trigger_proxy_rate, [5], self.trigger_proxy_rate, [0])
        imgs = runner.result_as_nparray
        detected = {}
        visible = 0
        for cam in imgs:
            if restrict and cam not in restrict:
                continue
            features,dmax = self.tracking_cameras[cam].detect(imgs[cam][:,:,0], thresh)
            if features:
                if len(features) > 1:
                    rospy.logerr("multiple features not supported, taking the first one")

                rospy.logdebug("detect: %s: %s" % (cam,repr(features)))
                #take the first point
                row,col,lum = features[0]
                #numpy returns int64 here, which is not serializable, and also not needed. Just
                #convert to simple int
                #
                #convert to pixel coords (swap row/col)
                detected[cam] = (int(col),int(row))
                visible += 1

        return detected,visible

    def _detect_3d_point(self, runner, thresh):
        restrict = self.tracking_cameras.keys()
        detected,nvisible = self._detect_points(runner, thresh, restrict)
        xyz = None
        pts = None
        reproj = 0
        if nvisible >= 2:
            rospy.logdebug("#%d: (visible: %s)" % (nvisible, ','.join(detected.keys())))
            pts = []
            for d in detected:
                safe_name = d if d[0] != "/" else d[1:]
                pts.append( (safe_name,detected[d]) )
            xyz = self.flydra.find3d(pts,return_line_coords=False, undistort=True)
        if xyz != None:
            recon_3d = []
            for camid,(u,v) in pts:
                u2,v2 = self.flydra.find2d(camid,xyz,distorted=True)
                d = math.sqrt((u-u2)**2 + (v-v2)**2)
                recon_3d.append(d)
            reproj = np.mean(recon_3d)

            if reproj >= 10:
                xyz = None

        if xyz != None:
            rospy.loginfo("detect 3D: %s (%d visible, reproj:%.1f)" % (
                    repr(xyz),nvisible,reproj))

        return xyz,pts,nvisible,reproj

    def run(self):
        while not rospy.is_shutdown():
            with self.mode_lock:
                mode = self.mode
                service_args = self.mode_args
            if mode == CALIB_MODE_FINISHED:
                pass
                break

            elif mode == CALIB_MODE_SLEEP:
                pass
                
            elif mode == CALIB_MODE_MANUAL_TRACKING:
                xyz,pts,nvisible,reproj = self._detect_3d_point(self.runner, self.laser_thresh)
                col,row,lum = self._detect_laser_camera_2d_point(self.laser_thresh)

            elif mode == CALIB_MODE_MANUAL_PROJECTOR:
                try:
                    ds,vdisp,vdispinfo,centroid = self._parse_ds_specified(service_args)
                except (ValueError, KeyError):
                    self.change_mode(CALIB_MODE_SLEEP)
                    continue

                print centroid

                if centroid:                 
                    col,row = centroid
                else:
                    print "NOT SUPPORTED, PLEASE SPECIFY A COL and ROW"
                    self.change_mode(CALIB_MODE_SLEEP)
                    continue

                dsc = self.display_servers[ds]["display_client"]
                self._light_proj_pixel(ds, row=row, col=col)

                self.change_mode("DETECT")

            elif mode == "DETECT":
                xyz,pts,nvisible,reproj = self._detect_3d_point(self.runner, self.visible_thresh)
                print xyz

            rospy.sleep(0.1)

        #clean up all state
        if self.laser_proxy_power:
            self.laser_proxy_power(False)

        if self.show_cameras or self.show_display_servers:
            cv2.destroyAllWindows()

        self.data.close()

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--show-cameras', type=str, default=("",),
        help='show images with the given topics (or "all"). See --show-type for a description of the available images types',
        metavar="/Basler_NNN", nargs='*')
    parser.add_argument(
        '--show-type', type=str, default='F',
        help='which images types to show. %s' % (
                ', '.join(["%s=%s" % i for i in DotBGFeatureDetector.WIN_TYPES.items()])))
    parser.add_argument(
        '--calib-config', type=str, default='calib-cube.yaml',
        help='path to calibration configuration yaml file')
    parser.add_argument(
        '--show-display-servers', type=str, default=("",),
        help='show display servers with the given names (or "all") calibration in process',
        metavar="display_serverN", nargs='*')
    parser.add_argument(
        '--debug', type=str, default='',
        help='comma separated list of debug domains. '
             '[acquisition,detection,saveimages,control,benchmark]')

    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    rospy.init_node('calibration')

    config = {}
    conffile = decode_url(args.calib_config)
    with open(conffile, 'r') as f:
        config = yaml.load(f)
        for k in ("tracking_cameras", "display_servers"):
            if not config.has_key(k):
                parser.error("malformed calibration config, missing %s" % k)

    c = Calib(config,
              show_cameras=args.show_cameras,
              show_display_servers=args.show_display_servers,
              show_type=set(args.show_type),
              debug=args.debug.split(","))
    c.run()

