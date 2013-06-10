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

DisplayCorrespondence =     collections.namedtuple("DisplayCorrespondence",
                                ["col","row","vdisp","pan","tilt","x","y","z"])
PositionCorrenpondence =    collections.namedtuple("PositionCorrespondence",
                                ["x","y","z","col","row","vdisp","pan","tilt"])

def get_centre_of_vdisp(vdmask):
    """ returns col,row """
    rowm,colm,_ = scipy.ndimage.center_of_mass(vdmask)
    return colm,rowm

def generate_sampling_pixel_coords_5(vdmask,pts,space,img=None):
    colm,rowm = get_centre_of_vdisp(vdmask)

    m = np.array((colm,rowm))
    quadrants={(True,True):[],(True,False):[],(False,False):[],(False,True):[]}
    
    corners = [(colm,rowm)]

    if img != None:
        calib.imgproc.add_crosshairs_to_nparr(img, row=rowm, col=colm, chan=CHAN_R, sz=4)

    for pt in pts:
        col,row = pt
        p = np.array(pt)

        if img != None:
            calib.imgproc.add_crosshairs_to_nparr(img, row=row, col=col, chan=CHAN_R, sz=2)
        
        quad = m > p
        dist = numpy.linalg.norm(m-p)
        quadrants[tuple(quad)].append(tuple((dist,p)))
    
    for quad,ptdist in quadrants.items():
        dist,farpoint = sorted(ptdist, key=lambda ptd: ptd[0], reverse=True)[0]

        half = (m + farpoint) / 2
        c,r = (half + farpoint) / 2
        
        if img != None:
            calib.imgproc.add_crosshairs_to_nparr(img, row=r, col=c, chan=CHAN_R, sz=4)

        corners.append( (c,r) )
    
    return corners

def generate_sampling_pixel_coords(vdmask,pts,space,img=None):

    ncol = vdmask.shape[1]
    nrow = vdmask.shape[0]

    if img != None:
        #debugging....
        rowm,colm,_ = scipy.ndimage.center_of_mass(vdmask)
        calib.imgproc.add_crosshairs_to_nparr(img, row=rowm, col=colm, chan=0, sz=4)
        for pt in pts:
            col,row = pt
            p = np.array(pt)
            calib.imgproc.add_crosshairs_to_nparr(img, row=row, col=col, chan=1, sz=4)


    valid = []
    for col,row in gen_horiz_snake(w=ncol,h=nrow,sw=space,sh=space):
        if vdmask[row,col]:
            valid.append( (col,row) )

    for col,row in valid:
        if img != None:
            calib.imgproc.add_crosshairs_to_nparr(img, row=row, col=col, chan=2, sz=2)

    return valid

class DataIO:

    def __init__(self, directory):
        self.outdir = directory
        if not os.path.isdir(self.outdir):
            raise Exception("Dir %s does not exist" % self.outdir)

        self.num_points = 0
        
        self._display_tree = {}
        self._position_tree = calib.kdtree.create(dimensions=3, check_dimensions=False)
        
        self._pub_num_pts = rospy.Publisher('~num_points', UInt32)
        self._pub_mapping = rospy.Publisher('~mapping', CalibMapping)
        
        fn = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")+".bag"
        self._dest = os.path.join(self.outdir,fn)
        self._bag = rosbag.Bag(self._dest, 'w')
        rospy.loginfo("Saving to %s" % self._dest)

        self._pub_num_pts.publish(0)

    def clear_kdtree(self, ds=None):
        if ds is None:
            self._display_tree = {}
        elif ds in self._display_tree:
            del self._display_tree[ds]

    def save(self):
        self._bag.flush()
        
    def close(self):
        self._bag.close()
        rospy.loginfo("Saved to %s" % self._dest)

    def load(self, name, calibration_except=None, vis_callback_2d=None):
        if calibration_except is None:
            calibration_except = set()

        with rosbag.Bag(name, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=[CALIB_MAPPING_TOPIC]):

                viewport_desc = "%s/%s" % (msg.display_server,msg.vdisp)
                viewport_desc_all = "%s/all" % msg.display_server
                if (viewport_desc) in calibration_except or (viewport_desc_all in calibration_except):
                    continue

                self._add_mapping(msg)

                if vis_callback_2d:
                    vis_callback_2d(ds=msg.display_server, 
                                    col=msg.pixel_projector.x,
                                    row=msg.pixel_projector.y,
                                    pan=msg.pan,
                                    tilt=msg.tilt)

#        self._position_tree = self._position_tree.rebalance()
#        for tree in self._display_tree.values():
#            tree.rebalance()

    def _add_mapping(self, c):
        dcorr = DisplayCorrespondence(
                    col=c.pixel_projector.x,
                    row=c.pixel_projector.y,
                    vdisp=c.vdisp,
                    pan=c.pan,
                    tilt=c.tilt,
                    x=c.position.x,
                    y=c.position.y,
                    z=c.position.z)
        pcorr = PositionCorrenpondence(
                    x=c.position.x,
                    y=c.position.y,
                    z=c.position.z,
                    col=c.pixel_projector.x,
                    row=c.pixel_projector.y,
                    vdisp=c.vdisp,
                    pan=c.pan,
                    tilt=c.tilt)
                    
        self._position_tree.add(pcorr)

        try:
            self._display_tree[c.display_server]
        except KeyError:
            self._display_tree[c.display_server] = calib.kdtree.create(dimensions=2, check_dimensions=False)
        finally:
            self._display_tree[c.display_server].add(dcorr)
        
        self._bag.write(CALIB_MAPPING_TOPIC,c)
        self.num_points += 1
        self._pub_mapping.publish(c)
        self._pub_num_pts.publish(self.num_points)

    def get_display_correspondence(self, ds, col, row):
        dcorr = DisplayCorrespondence(
                    col=col,row=row,
                    vdisp="",pan=0,tilt=0,x=0,y=0,z=0)
        try:
            return self._display_tree[ds].search_nn(dcorr).data
        except KeyError:
            #OK, no data for display server yet
            return None
        except Exception:
            rospy.logwarn(
                "Unknown error getting correspondence for %r\n%s\nTree:\n\n" %(
                    dcorr,traceback.format_exc()))
            calib.kdtree.visualize(self._display_tree[ds])


    def add_mapping(self, **kwargs):
        c = CalibMapping()
        c.points = [Calib2DPoint(camera=pt[0],
                                         pixel=Point32(x=pt[1][0],y=pt[1][1])) \
                            for pt in kwargs["points"]]
        c.display_server = kwargs["display_server"]
        c.vdisp = kwargs["vdisp"]
        c.position = Point32(*kwargs["position"])
        c.pan = kwargs["pan"]
        c.tilt= kwargs["tilt"]
        c.pixel_projector = Point32(*kwargs["pixel_projector"])
        c.pixel_ptc_laser = Point32(*kwargs["pixel_ptc_laser"])
        c.pixel_ptc_projector = Point32(*kwargs["pixel_ptc_projector"])
        c.pixel_ptc_projector_luminance = float(kwargs["pixel_ptc_projector_luminance"])
        
        self._add_mapping(c)

show_laser_scatter = True
laser_handle = "pantilt"

class Calib:

    def __init__(self, config, show_cameras, show_display_servers, show_type, outdir, continue_calibration, calibration_except, enable_mouse_click, debug):
        tracking_cameras = config["tracking_cameras"]
        laser_camera = config["laser_camera"]
        laser = config["laser"]

        trigger = '/camera_trigger'

        if 0:
            ####SAVE EVERY IMAGE FOR NICE MOVIE
            basedir = "/mnt/ssd/CALIB_STEPS/"
            self.__ds_fmt = basedir+"%(time)f_ds.png"
            self.__l_fmt = basedir+"%(time)f_laser.png"
            self.__ptc_fmt = basedir+"%(time)f_ptc%(imgtype)s.png"
        else:
            self.__ds_fmt = None
            self.__l_fmt = None
            self.__ptc_fmt = None

        self.debug_control = "control" in debug

        #FIXME: make __getattr__ look into config for locals starting with cfg_
        self.projector_sleep = config["projector_sleep"]
        self.display_servers = config["display_servers"]
        self.mask_dir = decode_url(config["mask_dir"])
        self.ptsize = int(config["projector_point_size_px"])
        self.laser_range_pan = config["laser_range_pan"]
        self.laser_range_tilt = config["laser_range_tilt"]
        self.laser_expected_detect_location = config["laser_expected_detect_location"]
        self.laser_expected_detect_hist = config["laser_expected_detect_hist"]
        self.visible_thresh = int(config["bg_thresh_visible"])
        self.laser_thresh = int(config["bg_thresh_laser"])
        self.laser_search_size = config["laser_search_size"]
        self.laser_per_point_repeat_n_times = int(config["laser_per_point_repeat_n_times"])
        
        self.flydra = flydra.reconstruct.Reconstructor(
                        cal_source=decode_url(config["tracking_calibration"]))

        self.data = DataIO(outdir)

        rospy.wait_for_service(trigger+'/set_framerate')
        self.trigger_proxy_rate = rospy.ServiceProxy(trigger+'/set_framerate', camera_trigger.srv.SetFramerate)
        self.trigger_proxy_once = rospy.ServiceProxy(trigger+'/trigger_once', std_srvs.srv.Empty)
        self.trigger_proxy_rate(0.0)

        self.laser_proxy_power = rospy.ServiceProxy(laser+'/set_power', flycave.srv.SetPower)
        self.laser_proxy_pan = rospy.ServiceProxy(laser+'/set_pan', flycave.srv.SetFloat)
        self.laser_proxy_tilt = rospy.ServiceProxy(laser+'/set_tilt', flycave.srv.SetFloat)
        self.laser_proxy_brightness = rospy.ServiceProxy(laser+'/set_brightness', flycave.srv.SetFloat)

        rospy.wait_for_service(laser+'/set_power')
        rospy.wait_for_service(laser+'/set_brightness')
        rospy.wait_for_service(laser+'/set_pan')
        rospy.wait_for_service(laser+'/set_tilt')

        #move laser home
        self._laser_currpan, self._laser_currtilt = config["laser_home"]
        self.laser_proxy_brightness(config["laser_brightness"])
        self.laser_proxy_power(False)
        self.laser_proxy_pan(self._laser_currpan)
        self.laser_proxy_tilt(self._laser_currtilt)
        rospy.loginfo("Moving laser home to p:%s t:%s" % (self._laser_currpan, self._laser_currtilt))
        rospy.loginfo("Laser range p:%r -> t:%r" % (self.laser_range_pan, self.laser_range_tilt))

        self.pub_mode = rospy.Publisher('~mode', String)
        
        self.show_cameras = show_cameras
        self.show_display_servers = {}
        if show_cameras or show_display_servers or show_laser_scatter:
            cv2.startWindowThread()

        self.results = {}           #display_server : {vdisp : [(u,v,x,y,z),(u,v,x,y,z),...]}
        self.num_results = 0
        
        self._laser_handles = {}
        if show_laser_scatter:
            handle = laser_handle
            cv2.namedWindow(handle)
            sizepan = self.laser_range_pan[1] - self.laser_range_pan[0]
            sizetilt = self.laser_range_tilt[1] - self.laser_range_tilt[0]
            self._laser_handles[handle] = np.zeros((sizetilt+1,sizepan+1,3),dtype=np.uint8)
            cv2.imshow(handle, self._laser_handles[handle])

        self._click_queue = {} #display_server:[(col, row), ...]
        for d in self.display_servers:
            dsc = display_client.DisplayServerProxy(d,wait=True)
            dsc.enter_2dblit_mode()

            self.display_servers[d]["vdmask"] = {}
            for vdisp in self.display_servers[d]["virtualDisplays"]:
                vdispname = vdisp["id"]
                self.results[d] = dict(vdispname=list())
                self.display_servers[d]["vdmask"][vdispname] = dsc.get_virtual_display_mask(vdisp)

            self.display_servers[d]["display_client"] = dsc
            rospy.loginfo("Calibrating %s" % d)

            if d in show_display_servers or show_display_servers[0] == "all":
                cv2.namedWindow(d)
                self._click_queue[d] = []
                if enable_mouse_click:
                    cv2.setMouseCallback(d, self._display_server_window_click, d)
                    rospy.logwarn("ENABLING UNSTABLE CRASHY MOUSE CLICK SELECTION ON %s" % d)
                self.show_display_servers[d] = {}

                #get the masks for the lot
                allmask = dsc.get_display_mask()
                img = dsc.new_image(
                            color=255, mask=~allmask, nchan=3, dtype=np.uint8)
                self.show_display_servers[d] = dict(
                        handle=d,
                        visualizeimg=img)
                cv2.imshow(d, img)

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
        
        #laser camera acquisition
        self.laser_camera = laser_camera
        fd = DotBGFeatureDetector(
                    laser_camera,
                    method="morphbinary",
                    show=show_type if (show_cameras[0] == "all" or laser_camera in show_cameras) else "")
        if "detection" in debug:
            fd.enable_debug_detection()
        if "saveimages" in debug:
            fd.enable_debug_images("/mnt/ssd/CALIB/")
        if "benchmark" in debug:
            fd.enable_benchmark()
        self.laser_runner = SequentialCameraRunner(
                                (CameraHandler(laser_camera,"acquisition" in debug),),
                                queue_depth=1)
        rospy.loginfo("Connecting to cam %s" % laser_camera)
        self.laser_detector = fd
        self.laser_mask = load_mask_image(decode_url(config["laser_camera_mask"]))

        #need to recieve images (i.e. by calculating the background) before
        #being able to read the resoluions
        self._calculate_background()

        #now we know the resolutions we can show a debug target in the blue channel
        b = np.zeros((494, 659),dtype=np.uint8)
        cv2.circle(b, 
                tuple(self.laser_expected_detect_location),
                self.laser_expected_detect_hist, (255,0,0))
        self.laser_detector.add_debug_blue_channel(b)

        if self.__ptc_fmt:
            self.laser_detector.enable_debug_saveimages(self.__ptc_fmt)

        self._vdisptocalibrate = []            
        self._vdispinfo = {}

        self.mode_lock = threading.Lock()
        self.mode_args = tuple()

        if continue_calibration:
            path = decode_url(continue_calibration)
            if os.path.exists(path):
                self._load_previous_calibration(path, calibration_except)
            else:
                rospy.logerr("could not find requested calibration to load")

        s = rospy.Service('~clear_kdtree', std_srvs.srv.Empty, self._on_clear_kdtree)
        s = rospy.Service('~calib_change_mode', flyvr.srv.CalibMode, self._on_change_mode)
        s = rospy.Service('~set_visual_threshold', flyvr.srv.CalibSetFloat, self._on_set_visual_threshold)

        self.change_mode(CALIB_MODE_SLEEP)

    def change_mode(self, mode, *service_args):
        with self.mode_lock:
            self.mode = mode
            self.mode_args = service_args
            rospy.loginfo("Changing to mode -> %s (args %s)" % (mode,repr(self.mode_args)))

    def _display_server_window_click(self, event, col, row, flags, ds):
        if flags & cv.CV_EVENT_FLAG_LBUTTON:
            rospy.loginfo("%s queuing point col:%s row:%s" % (ds,col,row))
            self._click_queue[ds].append( (col,row) )
        elif flags & cv.CV_EVENT_FLAG_RBUTTON:
            rospy.loginfo("clearing queued points")
            self._click_queue[ds] = []

    def _on_clear_kdtree(self, req):
        self.data.clear_kdtree()
        if show_laser_scatter:
            handle = laser_handle
            sizepan = self.laser_range_pan[1] - self.laser_range_pan[0]
            sizetilt = self.laser_range_tilt[1] - self.laser_range_tilt[0]
            self._laser_handles[handle] = np.zeros((sizetilt+1,sizepan+1,3),dtype=np.uint8)
            cv2.imshow(handle, self._laser_handles[handle])
        return std_srvs.srv.EmptyResponse()

    def _on_set_visual_threshold(self, req):
        old = self.visible_thresh
        self.visible_thresh = int(req.data)
        rospy.loginfo("updating visible thresh = %s (was %s)" % (self.visible_thresh, old))
        return flyvr.srv.CalibSetFloatResponse()

    def _on_change_mode(self, req):
        self.change_mode(req.mode, req.sa, req.fa, req.fb, req.fc)
        return flyvr.srv.CalibModeResponse()

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

    def _set_bg_mask(self, cam, detector, clear_masks=False):
        if clear_masks:
            rospy.loginfo("Clearing %s mask" % cam)
            detector.clear_mask()
        else:
            mask_name = os.path.join(self.mask_dir,cam.split("/")[-1]) + ".png"
            if os.path.exists(mask_name):
                arr = load_mask_image(mask_name)
                detector.set_mask(arr)
                rospy.loginfo("Setting %s mask = %s" % (cam,mask_name))

    def _light_laser_pixel(self, pan, tilt, power):
        minpan,maxpan,npan = self.laser_range_pan
        mintilt,maxtilt,ntilt = self.laser_range_tilt
        
        pan = np.clip(pan,minpan,maxpan-1)
        tilt = np.clip(tilt,mintilt,maxtilt-1)
        
        dist = numpy.linalg.norm(
                np.array((self._laser_currpan,self._laser_currtilt)) -
                np.array((pan,tilt)))
        rospy.logdebug("laser %s: pan: %d tilt: %d dist: %.1f" % (
                            "on" if power else "off",
                            pan,tilt,dist))

        self._laser_currpan = pan
        self.laser_proxy_pan(pan)
        self._laser_currtilt = tilt
        self.laser_proxy_tilt(tilt)
        self.laser_proxy_power(power)

        if show_laser_scatter:
            handle = laser_handle
            img = self._laser_handles[handle].copy()
            add_crosshairs_to_nparr(
                        arr=img,
                        row=math.floor(tilt)+(0-self.laser_range_tilt[0]),
                        col=math.floor(pan)+(0-self.laser_range_pan[0]),
                        sz=-1,  fill=255, chan=1)
            cv2.imshow(handle, img)
            if self.__l_fmt:
                cv2.imwrite(self.__l_fmt%{"time":time.time()},img)
        
        return pan,tilt

    def _black_projector(self, ds):
        self._light_proj_pixel(ds, None, None)

    def _light_proj_pixel(self, ds, row, col, black_others=True):
        row = math.floor(row) if row != None else None
        col = math.floor(col) if col != None else None

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

    def _detect_laser_camera_2d_point(self, thresh, msgprefix=""):
        self.laser_runner.get_images(1)
        imgs = self.laser_runner.result_as_nparray

        if thresh == self.laser_thresh:
            self.laser_detector.set_mask(self.laser_mask, copy=False)

        img = imgs[self.laser_camera][:,:,0]
        features,dmax = self.laser_detector.detect(
                        img,
                        thresh,
                        exact_luminance=thresh != self.laser_thresh)

        if thresh == self.laser_thresh:
            self.laser_detector.clear_mask()

        if features:
            row,col,luminance = features[0]

            if len(features) > 1:
                rospy.logerr("multiple features not supported, taking the closest one")
                dist = 1e6 #any big number
                expected = np.array(self.laser_expected_detect_location)
                for feat in features:
                    _row,_col,_luminance = feat
                    _dist = numpy.linalg.norm(expected - np.array((_col,_row)))
                    if _dist < dist:
                        row,col,liminance = _row,_col,_luminance

            if thresh == self.laser_thresh:
                msg = msgprefix + "PTC laser"
                expected = np.array(self.laser_expected_detect_location)
                actual = np.array((col,row))
                dist = numpy.linalg.norm(expected - actual)
                if dist > self.laser_expected_detect_hist:
                    rospy.logwarn("misdetected laser location expected:%s got:%s (dist:%.1f) " % (
                        expected,actual,dist))
                    row = col = luminance = None
            elif thresh == self.visible_thresh:
                msg = msgprefix + "PTC visible"
            else:
                msg = msgprefix + "PTC (unknown threshold)"
            rospy.loginfo("detect 2D %s: col:%s row:%s lum:%s" % (msg,col,row,luminance))
            return col,row,luminance

        return None,None,dmax

    def _detect_3d_point(self, runner, thresh):
        restrict = self.tracking_cameras.keys()
        detected,nvisible = self._detect_points(runner, thresh, restrict)
        xyz = None
        pts = None
        reproj = 0
        if nvisible >= 2:
            rospy.logdebug("#%d: (%d visible: %s)" % (self.num_results, nvisible, ','.join(detected.keys())))
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

    def _load_previous_calibration(self, path, calibration_except=None):
        self.data.load(path, calibration_except, vis_callback_2d=self._show_correspondence)

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

    def _show_correspondence(self, ds, col, row, pan, tilt):
        if ds in self.show_display_servers:
            handle = self.show_display_servers[ds]["handle"]
            img =  self.show_display_servers[ds]["visualizeimg"]
            add_crosshairs_to_nparr(
                arr=img,
                row=row,
                col=col,
                sz=1, fill=255, chan=0)
            cv2.imshow(handle, img)
            if self.__ds_fmt:
                cv2.imwrite(self.__ds_fmt%{"time":time.time()},img)


        if show_laser_scatter:
            handle = laser_handle
            img = self._laser_handles[handle]
            try:
                t = math.floor(tilt+(0-self.laser_range_tilt[0]))
                p = math.floor(pan+(0-self.laser_range_pan[0]))
                img[t,p,:] = 255
            except:
                rospy.logerr("could not plot pan/tilt: tilt:%s pan:%s" % (tilt,pan))
            cv2.imshow(handle, img)
            if self.__l_fmt:
                cv2.imwrite(self.__l_fmt%{"time":time.time()},img)

    def run(self):
        while not rospy.is_shutdown():
            with self.mode_lock:
                mode = self.mode
                service_args = self.mode_args
            if mode == CALIB_MODE_FINISHED:
                self.data.save()
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

                if centroid:                 
                    col,row = centroid
                else:
                    dsc = self.display_servers[ds]["display_client"]
                    vdmask = dsc.get_virtual_display_mask(vdisp)
                    col,row = get_centre_of_vdisp(vdmask)

                self._light_proj_pixel(ds, row=row, col=col)
                col,row,lum = self._detect_laser_camera_2d_point(self.visible_thresh)

            elif mode == CALIB_MODE_MANUAL_CLICKED:
                tocal = []
                for ds,pts in self._click_queue.items():    
                    if pts:
                        for vdisp in self.display_servers[ds]["virtualDisplays"]:
                            dsc = self.display_servers[ds]["display_client"]
                            vdispname = vdisp["id"]
                            vdmask = dsc.get_virtual_display_mask(vdispname)
                            for col,row in pts:
                                if vdmask[row,col]:
                                    rospy.loginfo("clicked to select col:%s row:%s in %s" % (col,row,vdispname))
                                    tocal.append( (ds,vdispname,vdisp.copy(),(col,row)) )

                for ds in self.show_display_servers:
                    self._click_queue[ds] = []

                if tocal:
                    self._vdisptocalibrate = tocal
                    self.change_mode(CALIB_MODE_DISPLAY_SERVER_VDISP)
                    continue

                self.change_mode(CALIB_MODE_SLEEP)

            elif mode == CALIB_MODE_DISPLAY_SERVER:
                spec = service_args[0]
                if spec in self.display_servers:
                    options = [spec]
                    selected_vdisp = None
                else:
                    try:
                        ds,selected_vdisp = spec.split('/')
                        if ds not in self.display_servers:
                            raise ValueError
                        options = [ds]
                    except ValueError:
                        options = self.display_servers.keys()
                        selected_vdisp = None

                try:
                    pointspace = int(service_args[1])
                except:
                    pointspace = 40
                finally:
                    pointspace = np.clip(pointspace,0,200)
                    rospy.loginfo("calibrating display servers %r/%s with %d point space" % (
                                    options, selected_vdisp, pointspace))

                tocal = []
                for ds in options:
                    for vdisp in self.display_servers[ds]["virtualDisplays"]:
                        vdispname = vdisp['id']
                        
                        #limit vdisps to thos specified
                        if selected_vdisp != None and selected_vdisp != vdispname:
                            continue
                        
                        if ds in self.show_display_servers:
                            handle = self.show_display_servers[ds]["handle"]
                            img =  self.show_display_servers[ds]["visualizeimg"]
                        else:
                            img = None

                        dsc = self.display_servers[ds]["display_client"]
                        vdmask = dsc.get_virtual_display_mask(vdispname)
                        vdpts = dsc.get_virtual_display_points(vdispname)

                        centroids = generate_sampling_pixel_coords(vdmask,vdpts,pointspace,img)
                        for c in centroids:
                            tocal.append( (ds,vdispname,vdisp.copy(),c) )

                self._vdisptocalibrate = tocal
                self.change_mode(CALIB_MODE_DISPLAY_SERVER_VDISP)

            elif mode == CALIB_MODE_DISPLAY_SERVER_STOP:
                self._vdisptocalibrate = []
                self.change_mode(CALIB_MODE_DISPLAY_SERVER_VDISP)

            elif mode == CALIB_MODE_DISPLAY_SERVER_VDISP:
                if len(self._vdisptocalibrate):
                    ds,vdisp,vdispinfo,centroid = self._vdisptocalibrate.pop()
                #did the user request a vdisp calibration
                elif service_args and service_args[0]:
                    try:
                        ds,vdisp,vdispinfo,centroid = self._parse_ds_specified(service_args)
                    except (ValueError, KeyError):
                        self.change_mode(CALIB_MODE_SLEEP)
                        continue
                    self._vdisptocalibrate = []
                else:
                    rospy.loginfo("nothing to do")
                    self.change_mode(CALIB_MODE_SLEEP)
                    continue

                dsc = self.display_servers[ds]["display_client"]
                vdmask = dsc.get_virtual_display_mask(vdisp)
                    
                self._vdispinfo = vdispinfo

                searchpath = []
                if centroid:
                    #user has clicked or we have generated a dense sampling grid.
                    #to save time, start at previous closest location
                    pcorr = self.data.get_display_correspondence(ds, centroid[0], centroid[1])
                    if pcorr:
                        searchpath = [(pcorr.pan,pcorr.tilt),
                                      (pcorr.pan,pcorr.tilt)]
                else:
                    #find the centre of the vdisp by default
                    centroid = get_centre_of_vdisp(vdmask)
                    
                if not searchpath:
                    #start search at current location (thrice for reliability)
                    searchpath = [(self._laser_currpan,self._laser_currtilt),
                                  (self._laser_currpan,self._laser_currtilt),
                                  (self._laser_currpan,self._laser_currtilt)]
                    
                rospy.loginfo("Calibrating %s:%s@%s" % (ds,vdisp,repr(centroid)))
                colmid,rowmid = centroid

                #we have found the pixel location in the projector around which we
                #should search, using the laser. Now we get the laser close
                #enough to the pixel that subsequently we can later move the pixel
                #to the laser
                self._light_proj_pixel(ds, row=rowmid, col=colmid)

                minpan,maxpan,npan = self.laser_range_pan
                mintilt,maxtilt,ntilt = self.laser_range_tilt
                searchpath.extend( gen_vert_snake(
                                        w=maxpan,h=maxtilt,
                                        startw=minpan,starth=mintilt,
                                        sw=npan,sh=ntilt,
                                        linspace=True) )

                found = False
                for pan,tilt in searchpath:
                    if found:
                        break
                    #so it doesnt look like we are hung
                    self.pub_mode.publish(self.mode)
                    #ensure the laser is off
                    pan,tilt = self._light_laser_pixel(pan=pan,tilt=tilt,power=False)
                    col,row,lum = self._detect_laser_camera_2d_point(self.visible_thresh)
                    #we can see the projector pixel, somewhere
                    if col is not None:
                        expected = np.array(self.laser_expected_detect_location)
                        expdist = numpy.linalg.norm(expected - np.array((col,row)))
                        fine_dist = expdist
                        oldpan,oldtilt      = pan,tilt
                        foundpan,foundtilt  = pan,tilt
                        newpan,newtilt      = pan,tilt
                        if expdist < 150:
                            #we have a rough estimate, refine it
                            tries = 40
                            if self.debug_control:
                                rospy.loginfo("CTRL:MPTC:DETE rough n%d %r %r" % (tries,expdist,expected - np.array((col,row))))
                            
                            while (tries > 0):
                                #use the returned pan,tilt because of mechanical limits
                                #or future wrap-around
                                pan,tilt = self._light_laser_pixel(pan=newpan,tilt=newtilt,power=False)
                                if self.debug_control:
                                    rospy.loginfo("CTRL:MPTC:SHOW n%d p:%s t:%s (was p%s t%s)" % (
                                                                tries,
                                                                newpan,newtilt,
                                                                oldpan,oldtilt))

                                _col,_row,_lum = self._detect_laser_camera_2d_point(self.visible_thresh)
                                if _col is None:
                                    #we lost the pixel
                                    tries -= 1
                                    if self.debug_control:
                                        rospy.loginfo("CTRL:MPTC:MISS rough lost (thresh %f dmax %f)" % (
                                                                self.visible_thresh,_lum))

                                else:
                                    #we found the pixel
                                    col,row = _col, _row
                                    foundpan,foundtilt = pan,tilt
                                    fine_dist = numpy.linalg.norm(expected - np.array((col,row)))
                                    if self.debug_control:
                                        rospy.loginfo("CTRL:MPTC:DETE fine dist=%f (%r)" % (
                                                                fine_dist,
                                                                expected - np.array((col,row))))

                                    #already close enough
                                    if fine_dist < 3:
                                        tries = 0       #tries = 0 breaks from inner loop
                                        found = True    #break from outer loop
                                        if self.debug_control:
                                            rospy.loginfo("CTRL:MPTC:FIN rough dist=%r" % fine_dist)

                                    #control to get it closer
                                    else:
                                        tries -= 1
                                        diffcol,diffrow = expected - np.array((col,row))
                                        oldpan,oldtilt = pan,tilt
                                        newpan  = pan  + (np.sign(diffcol) * self._vdispinfo["p_laser_col"])
                                        newtilt = tilt + (np.sign(diffrow) * self._vdispinfo["p_laser_row"])

                                        
                            if not found and (fine_dist < 80):
                                found = True
                                if self.debug_control:
                                    rospy.loginfo("CTRL:MPTC:FIN rough dist=%r" % fine_dist)


                            #FIXME: there was a break here, I think that is superceded by using 
                            #foundpan and foundtilt
                        else:
                            rospy.loginfo("2D pixel too far from start location (dist:%.1f)" % (expdist))

                if found:
                    rospy.loginfo("found starting pixel: col:%s row:%s pan:%s tilt:%s (dist:%.1f)" % (
                                    colmid,rowmid,foundpan,foundtilt,fine_dist))
                    self._vdispinfo["panmid"] = foundpan
                    self._vdispinfo["tiltmid"] = foundtilt

                    self._vdispinfo["colmid"] = int(colmid)
                    self._vdispinfo["rowmid"] = int(rowmid)

                    self._vdispinfo["vdmask"] = vdmask
                    self._vdispinfo["dsc"] = dsc
                    self._vdispinfo["ds"] = ds
                    self._vdispinfo["width"] = self.display_servers[ds]["width"]
                    self._vdispinfo["height"] = self.display_servers[ds]["height"]

                    self._vdispinfo["pointsneeded"] = self.laser_per_point_repeat_n_times
                    
                    self._vdispinfo["homeattempt"] = 30

                    self.change_mode(CALIB_MODE_DISPLAY_SERVER_HOME)
                else:
                    rospy.logwarn("could not find find starting pixel col:%s row:%s" %(
                                    colmid,rowmid))
                
                if ds in self.show_display_servers:
                    handle = self.show_display_servers[ds]["handle"]
                    img =  self.show_display_servers[ds]["visualizeimg"]
                    cv2.imshow(handle, img)

            elif mode == CALIB_MODE_DISPLAY_SERVER_HOME:
                ds = self._vdispinfo["ds"]
                self._light_laser_pixel(self._vdispinfo["panmid"], self._vdispinfo["tiltmid"], power=True)
                self._black_projector(ds)
                
                self._vdispinfo["homeattempt"] -= 1
                if self._vdispinfo["homeattempt"] < 0:
                    rospy.logwarn("giving up, could not get laser home location (maybe reflection)")
                    self.change_mode(CALIB_MODE_DISPLAY_SERVER_VDISP)
                    continue
                
                col,row,lum = self._detect_laser_camera_2d_point(self.laser_thresh)
                if col is not None:
                    self.laser_proxy_power(False)
                    
                    #generate N points about the start - and include the
                    #start point several times
                    #
                    #this is the maximum number of points we will test, so be generous
                    #because this loop is exited when we have collected enough points,
                    #not when this list is empty (so we will not necessarily take this long,
                    #but we will take this long if we can't get a reconstruction)
                    #
                    #start with current location
                    p = self._vdispinfo["panmid"]
                    t = self._vdispinfo["tiltmid"]
                    #twice for reliability
                    currattempt3d = [(p,t),(p,t)]
                    #locatons spiraling out from the start
                    currattempt3d.extend(
                        (p,t) for p,t in gen_spiral_snake(
                                            self.laser_search_size,self.laser_search_size,
                                            0.5,
                                            startw=p,
                                            starth=t
                                         )
                    )
                    #we pop from this list, so reverse it
                    currattempt3d.reverse()
                    self._vdispinfo["currattempt3d"] = currattempt3d

                    self.change_mode(CALIB_MODE_DISPLAY_SERVER_LASER)

            elif mode == CALIB_MODE_DISPLAY_SERVER_LASER:
                try:
                    pan,tilt = self._vdispinfo["currattempt3d"].pop()
                    pan,tilt = self._light_laser_pixel(pan, tilt, power=True)
                    self._vdispinfo["currpan"] = pan
                    self._vdispinfo["currtilt"] = tilt
                    rospy.loginfo("laser making candidate 3D point at p:%s t:%s (%d remain)" % (pan,tilt,len(self._vdispinfo["currattempt3d"])))
                except IndexError:
                    rospy.logwarn("giving up, could not get a 3D reconstruction")
                    self.change_mode(CALIB_MODE_DISPLAY_SERVER_VDISP)                    

                ds = self._vdispinfo["ds"]
                self._black_projector(ds)
                
                xyz,pts,nvisible,reproj = self._detect_3d_point(self.runner, self.laser_thresh)
                #always do this detection to keep the basler camera updated... even
                #if there is a chance we throw away the result
                col,row,lum = self._detect_laser_camera_2d_point(self.laser_thresh)

                if xyz == None:
                    rospy.loginfo("no 3d point (visible in %d cams, reproj error: %f)" % (nvisible,reproj))
                    continue

                if col is not None:
                    self._vdispinfo["targetcol"] = col
                    self._vdispinfo["targetrow"] = row
                    self._vdispinfo["currxyz"] = xyz
                    self._vdispinfo["currpts"] = pts
                    #start at the previously detected middle of the field of
                    #view of the ptc camera
                    self._vdispinfo["projcol"] = self._vdispinfo["colmid"]
                    self._vdispinfo["projrow"] = self._vdispinfo["rowmid"]
                    self._vdispinfo["currattempt"] = 40
                    self.laser_proxy_power(False)
                    self.change_mode(CALIB_MODE_DISPLAY_SERVER_PROJECTOR)

            elif mode == CALIB_MODE_DISPLAY_SERVER_PROJECTOR:
                ds = self._vdispinfo["ds"]
                self.laser_proxy_power(False)
                
                self._vdispinfo["currattempt"] -= 1
                if self._vdispinfo["currattempt"] < 0:
                    rospy.logwarn("giving up, no 3D reconstruction (%d attempts remain)" % self._vdispinfo["currattempt"])
                    self.change_mode(CALIB_MODE_DISPLAY_SERVER_LASER)
                    continue
    
                self._light_proj_pixel(
                        ds,
                        row=self._vdispinfo["projrow"],
                        col=self._vdispinfo["projcol"])

                if self.debug_control:
                    rospy.loginfo("CTRL:MPRJ:SHOW n%d pr:%s pc:%s" % (
                                        self._vdispinfo["currattempt"],
                                        self._vdispinfo["projrow"],
                                        self._vdispinfo["projcol"]))

                col,row,lum = self._detect_laser_camera_2d_point(
                                    self.visible_thresh,
                                    msgprefix="attempt %d " % self._vdispinfo["currattempt"])

                #missed the projector pixel. try again (we are safe from looping
                #because of the currattempt test)
                if col is None:
                    if self.debug_control:
                        rospy.loginfo("CTRL:MPRJ:MISS no detect (thresh %f dmax %f)" % (
                                                self.visible_thresh, lum))
                    continue
                else:
                    if self.debug_control:
                        rospy.loginfo("CTRL:MPRJ:DETE cr:%s cc:%s" %(row,col))

                    targetcol = self._vdispinfo["targetcol"]
                    targetrow = self._vdispinfo["targetrow"]
                    projcol = self._vdispinfo["projcol"]
                    projrow = self._vdispinfo["projrow"]

                    colfound = rowfound = False
                    if abs(targetcol - col) > 1:
                        diff = abs(targetcol - col)
                        if diff > 5:
                            #p-control
                            adj = int(self._vdispinfo["dcol"] * diff * 0.3)
                        else:
                            #i-control
                            adj = self._vdispinfo["dcol"]
                        if targetcol > col:
                            projcol = self._vdispinfo["projcol"] + adj
                        else:
                            projcol = self._vdispinfo["projcol"] - adj
                    else:
                        colfound = True

                    if abs(targetrow - row) > 1:
                        diff = abs(targetrow - row)
                        if diff > 5:
                            #p-control
                            adj = int(self._vdispinfo["drow"] * diff * 0.3)
                        else:
                            #i-control
                            adj = self._vdispinfo["drow"]
                        if targetrow > row:
                            projrow = self._vdispinfo["projrow"] + adj
                        else:
                            projrow = self._vdispinfo["projrow"] - adj
                    else:
                        rowfound = True
                        
                    #clamp the projection range and then
                    self._vdispinfo["projcol"] = np.clip(projcol,0,self._vdispinfo["width"]-1)
                    self._vdispinfo["projrow"] = np.clip(projrow,0,self._vdispinfo["height"]-1)

                    if colfound and rowfound:
                        if self.debug_control:
                            rospy.loginfo("CTRL:MPRJ:FIN r:%s c:%s" %(
                                                self._vdispinfo["projrow"],
                                                self._vdispinfo["projcol"]))

                        if ds in self.show_display_servers or show_laser_scatter:
                            self._show_correspondence(
                                    ds=ds,
                                    col=self._vdispinfo["projcol"],
                                    row=self._vdispinfo["projrow"],
                                    pan=self._vdispinfo["currpan"],
                                    tilt=self._vdispinfo["currtilt"])

                        rospy.loginfo("FOUND CORRESPONDENCE: %s:%s@(%s,%s) -> %r" % (
                                ds,self._vdispinfo["id"],
                                self._vdispinfo["projcol"], self._vdispinfo["projrow"],
                                self._vdispinfo["currxyz"]))

                        self.data.add_mapping(
                                points=self._vdispinfo["currpts"],
                                display_server=ds,
                                vdisp=self._vdispinfo["id"],
                                position=self._vdispinfo["currxyz"].tolist(),
                                pan=self._vdispinfo["currpan"],
                                tilt=self._vdispinfo["currtilt"],
                                pixel_projector=(self._vdispinfo["projcol"],
                                                 self._vdispinfo["projrow"],
                                                 0),
                                pixel_ptc_laser=(self._vdispinfo["targetcol"],
                                                 self._vdispinfo["targetrow"],
                                                 0),
                                pixel_ptc_projector=(col,
                                                     row,
                                                     0),
                                pixel_ptc_projector_luminance=lum
                        )

                        self._vdispinfo["pointsneeded"] -= 1
                        if self._vdispinfo["pointsneeded"] > 0:
                            self.change_mode(CALIB_MODE_DISPLAY_SERVER_LASER)
                        else:
                            self.change_mode(CALIB_MODE_DISPLAY_SERVER_VDISP)

            elif mode == CALIB_MODE_RESTORE:
                self._load_previous_calibration(self.outdir, None)
                self.change_mode(CALIB_MODE_SLEEP)

            #publish state
            self.pub_mode.publish(self.mode)

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
        '--calib-config', type=str, default='package://flycave/conf/calib-all.yaml',
        help='path to calibration configuration yaml file')
    parser.add_argument(
        '--save-dir', type=str,
        default=os.path.expanduser('~/FLYDRA/flyvr-calibration/%s' % datetime.datetime.now().strftime("%b")),
        help='path to save calibration data')
    parser.add_argument(
        '--continue-calibration', type=str,
        help='path to previous calibration bag file')
    parser.add_argument(
        '--continue-calibration-except', type=str, default=("",), nargs='*',
        help='viewport strings describing parts of a calibration not to load')
    parser.add_argument(
        '--show-display-servers', type=str, default=("",),
        help='show display servers with the given names (or "all") calibration in process',
        metavar="display_serverN", nargs='*')
    parser.add_argument(
        '--debug', type=str, default='',
        help='comma separated list of debug domains. '
             '[acquisition,detection,saveimages,control,benchmark]')
    parser.add_argument(
        '--enable-mouse-click', action="store_true",
        help='enable mouse click point selection (CAUSES HANGS, UNSTABLE)')

    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    outdir = os.path.expanduser(os.path.abspath(args.save_dir))
    if not os.path.isdir(outdir):
        os.makedirs(outdir)
    
    rospy.init_node('calibration')

    config = {}
    conffile = decode_url(args.calib_config)
    with open(conffile, 'r') as f:
        config = yaml.load(f)
        for k in ("tracking_cameras", "display_servers", "mask_dir"):
            if not config.has_key(k):
                parser.error("malformed calibration config, missing %s" % k)

    c = Calib(config,
              show_cameras=args.show_cameras,
              show_display_servers=args.show_display_servers,
              show_type=set(args.show_type),
              outdir=outdir,
              continue_calibration=args.continue_calibration,
              calibration_except=args.continue_calibration_except,
              enable_mouse_click=args.enable_mouse_click,
              debug=args.debug.split(","))
    c.run()

