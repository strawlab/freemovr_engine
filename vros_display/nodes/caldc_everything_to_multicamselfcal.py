#!/usr/bin/env python

# standard Python imports
import argparse
import time
import os.path
import math
import fnmatch
import threading
import tempfile

import json
import yaml
import numpy as np
import scipy.ndimage
import scipy.misc
import cv,cv2

# ROS imports
import roslib;
roslib.load_manifest('vros_display')
roslib.load_manifest('camera_trigger')
roslib.load_manifest('flycave')
roslib.load_manifest('std_srvs')
roslib.load_manifest('std_msgs')
roslib.load_manifest('motmot_ros_utils')
import rospy

# local vros_display imports
import display_client
import camera_trigger.srv
import std_srvs.srv
import flycave.srv
import vros_display.srv

import calib
from calib.acquire import CameraHandler, SimultainousCameraRunner
from calib.io import MultiCalSelfCam, AllPointPickle
from calib.imgproc import DotBGFeatureDetector, load_mask_image
from rosutils.io import decode_url

from std_msgs.msg import UInt32, String

class Calib:

    MODE_SLEEP = 0
    MODE_POINTS_MANUAL = 1
    MODE_PROJECTOR_VIS_SETUP = 2
    MODE_PROJECTOR_VIS_LIGHT = 3
    MODE_PROJECTOR_VIS_DETECT = 4
    MODE_FINISHED = 6
    MODE_POINTS_AUTO_SETUP = 7
    MODE_POINTS_AUTO_LIGHT = 8
    MODE_POINTS_AUTO_DETECT = 9
    MODE_SAVE = 10
    MODE_RESTORE = 11
    MODE_SET_BACKGROUND = 12
    MODE_CLEAR_BACKGROUND = 13
    MODE_CALIBRATE = 14

    #for colored display in caldc_monitor, and for generating subsets
    #of calibration data from the pickle files
    MODE_POINT_TYPES = {
        MODE_POINTS_MANUAL:calib.POINT_TYPE_MANUAL,
        MODE_POINTS_AUTO_SETUP:calib.POINT_TYPE_LASER,
        MODE_POINTS_AUTO_LIGHT:calib.POINT_TYPE_LASER,
        MODE_POINTS_AUTO_DETECT:calib.POINT_TYPE_LASER,
        MODE_PROJECTOR_VIS_SETUP:calib.POINT_TYPE_PROJECTOR,
        MODE_PROJECTOR_VIS_LIGHT:calib.POINT_TYPE_PROJECTOR,
        MODE_PROJECTOR_VIS_DETECT:calib.POINT_TYPE_PROJECTOR,
    }

    def __init__(self, config, show_cameras, show_type, outdir, continue_calibration):
        tracking_cameras = config["tracking_cameras"]
        display_servers = config["display_servers"]
        trigger = config["trigger"]
        laser = config["laser"]
        self.laser_sleep = config["laser_sleep"]
        self.projector_sleep = config["projector_sleep"]

        self.mask_dir = decode_url(config["mask_dir"])
        self.ptsize = int(config["projector_point_size_px"])
        self.num_ds_pts = int(config["projector_num_points"])
        self.laser_range_pan = config.get("laser_range_pan",[0,180,8])
        self.laser_range_tilt = config.get("laser_range_tilt",[64,136,8])
        self.visible_thresh = int(config["bg_thresh_visible"])
        self.laser_thresh = int(config["bg_thresh_laser"])
        self.outdir = outdir

        if not os.path.isdir(self.outdir):
            raise Exception("Dir %s does not exist" % self.outdir)
        rospy.loginfo("Saving to %s" % self.outdir)

        rospy.wait_for_service(trigger+'/set_framerate')
        if laser:
            rospy.wait_for_service(laser+'/set_power')
            self.laser_proxy_power = rospy.ServiceProxy(laser+'/set_power', flycave.srv.SetPower)
            self.laser_proxy_pan = rospy.ServiceProxy(laser+'/set_pan', flycave.srv.SetFloat)
            self.laser_proxy_tilt = rospy.ServiceProxy(laser+'/set_tilt', flycave.srv.SetFloat)
            self.laser_proxy_power(False)
        else:
            self.laser_proxy_power = None

        self.trigger_proxy_rate = rospy.ServiceProxy(trigger+'/set_framerate', camera_trigger.srv.SetFramerate)
        self.trigger_proxy_once = rospy.ServiceProxy(trigger+'/trigger_once', std_srvs.srv.Empty)
        self.trigger_proxy_rate(0.0)

        s = rospy.Service('~calib_mode_points_manual', std_srvs.srv.Empty, self._change_mode_srv_points_manual)
        s = rospy.Service('~calib_mode_points_auto', std_srvs.srv.Empty, self._change_mode_srv_points_auto)
        s = rospy.Service('~calib_mode_projector_visible', vros_display.srv.CalibProjector, self._change_mode_srv_projvis)
        s = rospy.Service('~calib_finish', std_srvs.srv.Empty, self._change_mode_srv_fin)
        s = rospy.Service('~calib_save', vros_display.srv.CalibConfig, self._change_mode_srv_save)
        s = rospy.Service('~calib_restore', std_srvs.srv.Empty, self._change_mode_srv_restore)
        s = rospy.Service('~calib_set_mask', std_srvs.srv.Empty, self._change_mode_set_mask)
        s = rospy.Service('~calib_clear_mask', std_srvs.srv.Empty, self._change_mode_clear_mask)
        s = rospy.Service('~calib_calculate_background', std_srvs.srv.Empty, self._change_mode_calculate_background)
        s = rospy.Service('~calib_calibrate', vros_display.srv.CalibConfig, self._change_mode_calibrate)


        self.pub_num_pts = rospy.Publisher('~num_points', UInt32)
        self.pub_pts = rospy.Publisher('~points', String)
        self.pub_pcd = rospy.Publisher('~pcd_file', String, latch=True)

        self.show_cameras = show_cameras
        if show_cameras:
            cv2.startWindowThread()

        self.results = {}           #camera : [(u,v),(u,v),...]
        self.results_mode = {}      #camera : [mode that was in use for point detection self.MODE_xxx]
        self.num_results = 0
        self.mcsc = MultiCalSelfCam(self.outdir)

        self.display_servers = {}   #name : (display_client,[cams])
        projector_cameras = []
        for d,cams in display_servers.items():
            projector_cameras.extend(cams)
            self.results[d] = []
            self.results_mode[d] = []
            dsc = display_client.DisplayServerProxy(d,wait=True)
            dsc.enter_2dblit_mode()
            dsc.show_pixels(dsc.new_image(dsc.IMAGE_COLOR_BLACK))
            self.display_servers[d] = (dsc,cams)

        #detectors contains all cameras (i.e. not projectors)
        self.detectors = {}
        cam_handlers = []
        for cam in tracking_cameras+projector_cameras:
            fd = DotBGFeatureDetector(
                                cam,
                                method="med",
                                show=show_type if (show_cameras[0] == "all" or cam in show_cameras) else "")
            self.detectors[cam] = fd
            cam_handlers.append(CameraHandler(cam,debug=False))
            self.results[cam] = []
            self.results_mode[cam] = []

        #set bg masks by default
        self._set_bg_masks()

        #cam_ids contains everything we calibrate
        self.cam_ids = self.detectors.keys() + self.display_servers.keys()
        for c in self.cam_ids:
            rospy.loginfo("Calibrating %s" % c)

        self.runner = SimultainousCameraRunner(cam_handlers)

        #need to recieve images (i.e. by calculating the background) before
        #being able to read the resoluions
        self._calculate_background()

        self.resolutions = {}
        for cam,det in self.detectors.items():
            self.resolutions[cam] = (det.img_width_px,det.img_height_px)
        for d,(ds,cams) in self.display_servers.items():
            self.resolutions[d] = (ds.width, ds.height)

        if continue_calibration:
            path = os.path.abspath(os.path.expanduser(continue_calibration))
            self._load_previous_calibration(path)

        self.mode_lock = threading.Lock()
        self.service_args = tuple()
        self.change_mode(self.MODE_SLEEP)

    def change_mode(self, mode, *service_args):
        with self.mode_lock:
            self.mode = mode
            self.service_args = service_args
            rospy.loginfo("Changing to mode -> %d (args %s)" % (mode,repr(self.service_args)))
    def _change_mode_srv_points_manual(self, req):
        self.change_mode(self.MODE_POINTS_MANUAL)
        return std_srvs.srv.EmptyResponse()
    def _change_mode_srv_points_auto(self, req):
        self.change_mode(self.MODE_POINTS_AUTO_SETUP)
        return std_srvs.srv.EmptyResponse()
    def _change_mode_srv_projvis(self, req):
        if req.node and req.node in self.display_servers:
            selected = [req.node]
        else:
            selected = self.display_servers.keys()
        self.change_mode(self.MODE_PROJECTOR_VIS_SETUP,tuple(selected))
        return vros_display.srv.CalibProjectorResponse(selected)
    def _change_mode_srv_fin(self, req):
        self.change_mode(self.MODE_FINISHED)
        return std_srvs.srv.EmptyResponse()
    def _change_mode_srv_save(self, req):
        if req.cams:
            cam_ids = [c for c in req.cams if c in self.cam_ids]
        elif req.cam_filter:
            cam_ids = fnmatch.filter(self.cam_ids,req.cam_filter)
        else:
            cam_ids = self.cam_ids[:]
        self.change_mode(self.MODE_SAVE,cam_ids,req.undo_distortion,req.num_cameras_fill)
        return vros_display.srv.CalibConfigResponse()
    def _change_mode_calibrate(self, req):
        if req.cams:
            cam_ids = [c for c in req.cams if c in self.cam_ids]
        elif req.cam_filter:
            cam_ids = fnmatch.filter(self.cam_ids,req.cam_filter)
        else:
            cam_ids = self.cam_ids[:]
        self.change_mode(self.MODE_CALIBRATE,cam_ids,req.undo_distortion,req.num_cameras_fill)
        return vros_display.srv.CalibConfigResponse()
    def _change_mode_srv_restore(self, req):
        self.change_mode(self.MODE_RESTORE)
        return std_srvs.srv.EmptyResponse()
    def _change_mode_set_mask(self, req):
        self._set_bg_masks()
        return std_srvs.srv.EmptyResponse()
    def _change_mode_clear_mask(self, req):
        self._set_bg_masks(clear_masks=True)
        return std_srvs.srv.EmptyResponse()
    def _change_mode_calculate_background(self, req):
        self._calculate_background()
        return std_srvs.srv.EmptyResponse()

    def _get_points_coverage(self, cam):
        w,h = self.resolutions[cam]
        #swap h,w as resolution saves in image coord convention and scipy.misc.imsave saves
        #from matrix convention
        arr = np.zeros((h,w,4),dtype=np.uint8)
        arr[:,:,3]=255 #opaque
        for pt in self.results[cam]:
            #again swap image coords -> matrix notation
            col,row = pt
            if np.any(np.isnan(pt)):
                continue
            arr[row,col,1] = 255
        return arr

    def _calculate_background(self):
        rospy.loginfo("Collecting backgrounds")
        #collect bg images
        self.runner.get_images(20, self.trigger_proxy_rate, [5], self.trigger_proxy_rate, [0])
        imgs = self.runner.result_as_nparray
        for cam in imgs:
            #collect the background model
            self.detectors[cam].compute_bg(imgs[cam])
            rospy.loginfo("Calculate background for %s" % cam)
        rospy.loginfo("Collecting backgrounds finished")

    def _set_bg_masks(self, clear_masks=False):
        for cam in self.detectors:
            if clear_masks:
                rospy.loginfo("Clearing %s mask" % cam)
                self.detectors[cam].clear_mask()
            else:
                mask_name = os.path.join(self.mask_dir,cam.split("/")[-1]) + ".png"
                if os.path.exists(mask_name):
                    arr = load_mask_image(mask_name)
                    self.detectors[cam].set_mask(arr)
                    rospy.loginfo("Setting %s mask = %s" % (cam,mask_name))

    def _detect_points(self, runner, thresh, restrict={}):
        runner.get_images(1, self.trigger_proxy_once)
        imgs = runner.result_as_nparray
        detected = {}
        visible = 0
        for cam in imgs:
            if restrict and cam not in restrict:
                continue
            features = self.detectors[cam].detect(imgs[cam][:,:,0], thresh)
            if features:
                rospy.logdebug("detect: %s: %s" % (cam,repr(features)))
                #take the first point
                row,col = features[0]
                #numpy returns int64 here, which is not serializable, and also not needed. Just
                #convert to simple int
                #
                #convert to pixel coords (swap row/col)
                detected[cam] = (int(col),int(row))
                visible += 1

        return detected,visible

    def _detect_and_save_all_points(self, runner, thresh, mode, restrict={}):
        #FIXME: do detect_points once for vis, once for laser
        detected,nvisible = self._detect_points(runner, thresh, restrict)
        if nvisible >= 3:
            self.num_results += 1
            rospy.loginfo("#%d: (%d visible: %s)" % (self.num_results, nvisible, ','.join(detected.keys())))
            for cam in self.cam_ids:
                if cam in detected:
                    self.results[cam].append(detected[cam])
                else:
                    self.results[cam].append((np.nan, np.nan))
                try:
                    self.results_mode[cam].append(self.MODE_POINT_TYPES[mode])
                except KeyError:
                    pass

    def _save_results(self, cam_ids=None, use_calibrations=True, num_cameras_fill=-1):
        if not cam_ids:
            cam_ids = self.cam_ids[:]

        a = AllPointPickle()
        a.save_calibration(
            self.outdir,
            results=self.results,
            results_mode=self.results_mode,
            resolutions=self.resolutions)

        cam_calibrations = {}
        if use_calibrations:
            for cam in cam_ids:
                #strip the leading '/' for the filename
                calib = decode_url('package://flycave/calibration/cameras/%s.yaml' % cam.split('/')[-1])
                if os.path.isfile(calib):
                    cam_calibrations[cam] = calib

        self.mcsc.create_from_cams(
                cam_ids=cam_ids,
                cam_resolutions=self.resolutions,
                cam_points=self.results.copy(),
                cam_calibrations=cam_calibrations,
                num_cameras_fill=num_cameras_fill)

        rospy.loginfo("saved results to %s" % self.outdir)

    def _run_mcsc_and_calibrate(self):
        def calib_finished(_cmd,_desdir):
            pcd = _desdir+'/points.pcd'
            self.mcsc.publish_calibration_points(result_dir=_desdir)
            self.mcsc.save_to_pcd(pcd,result_dir=_desdir)
            self.pub_pcd.publish(pcd)
            rospy.loginfo("published points and saved pcd file: %s" % pcd)

        #store the calibration result in a temp directory
        self.mcsc.execute(
                blocking=False,
                cb=calib_finished,
                dest=tempfile.mkdtemp(prefix='mcsc'))

    def _load_previous_calibration(self, path):
        a = AllPointPickle()
        self.results,self.results_mode,self.resolutions,self.num_results = \
            a.load_previous_calibration(path)

    def run(self):
        while not rospy.is_shutdown():
            with self.mode_lock:
                mode = self.mode
                service_args = self.service_args
            if mode == self.MODE_FINISHED:
                break

            elif mode == self.MODE_SLEEP:
                pass

            elif mode == self.MODE_RESTORE:
                self._load_previous_calibration(self.outdir)
                self.change_mode(self.MODE_SLEEP)

            elif mode == self.MODE_POINTS_MANUAL:
                #all cameras
                cams = self.detectors.keys()
                self._detect_and_save_all_points(self.runner, self.laser_thresh, mode, restrict=cams)

            elif mode == self.MODE_PROJECTOR_VIS_SETUP:
                self._ds_pts = {}
                self._nds_pts = self.num_ds_pts
                #only one argument, a list of selected display servers
                selected, = service_args
                for d in selected:
                    vdisp = 'vdisp' #FIXME
                    dsc,cams = self.display_servers[d]
                    #generate npts random points
                    pts = []
                    npts = self.num_ds_pts
                    mask = dsc.get_virtual_display_mask(vdisp)
                    arr = dsc.new_image(color=0,nchan=1,dtype=mask.dtype)
                    rospy.loginfo("Randomly choosing projector pixels for %s in vdisp %s" % (d,vdisp))
                    while npts > 0:
                        arr.fill(0)
                        row = np.random.random_integers(self.ptsize,dsc.height-self.ptsize)
                        col = np.random.random_integers(self.ptsize,dsc.width-self.ptsize)
                        #check if the point is inside the vdisp mask
                        arr[row,col] = 1
                        if (arr * mask).any():
                            pts.append((row,col))
                            npts -= 1
                    rospy.loginfo("Chose %d points" % self.num_ds_pts)
                    self._ds_pts[d] = pts
                self._nds_pts -= 1 #we count down and use this as the index of tested points
                self.change_mode(self.MODE_PROJECTOR_VIS_LIGHT)
            elif mode == self.MODE_PROJECTOR_VIS_LIGHT:
                for d in self._ds_pts:
                    dsc,cams = self.display_servers[d]
                    row,col = self._ds_pts[d][self._nds_pts]
            
                    arr = dsc.new_image(dsc.IMAGE_COLOR_BLACK)
                    arr[row-self.ptsize:row+self.ptsize,col-self.ptsize:col+self.ptsize,:3] = dsc.IMAGE_COLOR_WHITE
                    dsc.show_pixels(arr)
                    
                    rospy.loginfo("Lighting projector %s pixel %d,%d (%d remain)" % (dsc.name,row,col,self._nds_pts))

                #extra sleep for the projector to settle
                self.change_mode(self.MODE_PROJECTOR_VIS_DETECT)
                rospy.sleep(self.projector_sleep)

            elif mode == self.MODE_PROJECTOR_VIS_DETECT:
                if self._nds_pts > 0:
                    for d in self._ds_pts:
                        dsc,cams = self.display_servers[d]
                        detected,nvisible = self._detect_points(self.runner, self.visible_thresh, restrict=cams)
                        if nvisible == 2:
                            self.num_results += 1
                            for cam in self.cam_ids:
                                if cam in detected:
                                    #detected in both cameras
                                    self.results[cam].append(detected[cam])
                                    self.results_mode[cam].append(self.MODE_POINT_TYPES[mode])
                                elif cam == d:
                                    #'detected' in projector
                                    row,col = self._ds_pts[d][self._nds_pts]
                                    # convert to pixel coords - see self._detect_points
                                    self.results[d].append((int(col), int(row)))
                                    self.results_mode[d].append(self.MODE_POINT_TYPES[mode])
                                else:
                                    self.results[cam].append((np.nan, np.nan))
                                    self.results_mode[cam].append(self.MODE_POINT_TYPES[mode])
                            rospy.loginfo("#%d: (projector:%s cam: %s)" % (self.num_results, d, ','.join(detected.keys())))
                    self._nds_pts -= 1
                    self.change_mode(self.MODE_PROJECTOR_VIS_LIGHT)
                else:
                    #turn off all projectors
                    for d,(dsc,cams) in self.display_servers.items():
                        dsc.show_pixels(dsc.new_image(dsc.IMAGE_COLOR_BLACK))
                    self.change_mode(self.MODE_SLEEP)

            elif mode == self.MODE_POINTS_AUTO_SETUP:
                self.laser_proxy_power(True)
                self._laser_pts = [(p,t) for p in range(*self.laser_range_pan)\
                                         for t in range(*self.laser_range_tilt)]
                self._nlaser_pts = len(self._laser_pts) - 1 #we count down to zero
                self.laser_proxy_power(True)
                self.change_mode(self.MODE_POINTS_AUTO_LIGHT)

            elif mode == self.MODE_POINTS_AUTO_LIGHT:
                pan,tilt = self._laser_pts[self._nlaser_pts]
                rospy.loginfo("Laser pan,tilt %d,%d (%d remain)" % (pan,tilt,self._nlaser_pts))
                self.laser_proxy_pan(pan)
                self.laser_proxy_tilt(tilt)
                #extra sleep for the laser to move
                self.change_mode(self.MODE_POINTS_AUTO_DETECT)
                rospy.sleep(self.laser_sleep)

            elif mode == self.MODE_POINTS_AUTO_DETECT:
                if self._nlaser_pts > 0:
                    self._detect_and_save_all_points(self.runner, self.laser_thresh, mode)
                    self._nlaser_pts -= 1
                    self.change_mode(self.MODE_POINTS_AUTO_LIGHT)
                else:
                    self.laser_proxy_power(False)
                    self.change_mode(self.MODE_SLEEP)

            elif mode == self.MODE_SAVE:
                try:
                    self._save_results(*service_args)
                except Exception, e:
                    rospy.loginfo("could not save results: %s" % e)
                self.change_mode(self.MODE_SLEEP)
            elif mode == self.MODE_CALIBRATE:
                try:
                    self._save_results(*service_args)
                    self._run_mcsc_and_calibrate()
                except Exception, e:
                    rospy.loginfo("could not save results: %s" % e)
                self.change_mode(self.MODE_SLEEP)

            #publish state
            self.pub_num_pts.publish(self.num_results)
            self.pub_pts.publish(json.dumps({
                                    'results':self.results,
                                    'results_mode':self.results_mode,
                                    'resolution':self.resolutions}))

            rospy.sleep(0.1)

        #clean up all state
        if self.laser_proxy_power:
            self.laser_proxy_power(False)

        if self.show_cameras:
            cv2.destroyAllWindows()

        #self._save_results()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--show-cameras', type=str, default=("",),
        help='show images with the given topics. see --show-type for a description of the available images types',
        nargs='*')
    parser.add_argument(
        '--show-type', type=str, default='F',
        help='which images types to show. %s' % (
                ', '.join(["%s=%s" % i for i in DotBGFeatureDetector.WIN_TYPES.items()])))
    parser.add_argument(
        '--calib-config', type=str, default='package://flycave/conf/calib-all.yaml',
        help='path to calibration configuration yaml file')
    parser.add_argument(
        '--save-dir', type=str, default='./mcamall',
        help='path to save calibration data')
    parser.add_argument(
        '--continue-calibration', type=str,
        help='path to previous calibration .pkl file')



    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    outdir = os.path.expanduser(os.path.abspath(args.save_dir))
    if not os.path.isdir(outdir):
        os.makedirs(outdir)
    
    rospy.init_node('multicamselfcal_everything')

    config = {}
    conffile = decode_url(args.calib_config)
    with open(conffile, 'r') as f:
        config = yaml.load(f)
        for k in ("tracking_cameras", "display_servers", "trigger", "mask_dir"):
            if not config.has_key(k):
                parser.error("malformed calibration config, missing %s" % k)

    c = Calib(config,
              show_cameras=args.show_cameras,
              show_type=set(args.show_type),
              outdir=outdir,
              continue_calibration=args.continue_calibration)
    c.run()

