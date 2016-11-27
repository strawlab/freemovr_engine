#!/usr/bin/env python

import sys
import glob
import argparse
import os.path
import subprocess

import roslib
roslib.load_manifest('flyvr')
roslib.load_manifest('rosbag')
roslib.load_manifest('motmot_ros_utils')
import rosbag
import rospy

import flyvr.simple_geom as simple_geom
import flyvr.display_client as display_client
import flyvr.exr as exr
import flyvr.tools.blend as blend

from flyvr.calibration.image_processing.imgproc import add_crosshairs_to_nparr
from flyvr.tools.visualization import create_pcd_file_from_points, create_point_cloud_message_publisher, show_pointcloud_3d_plot, create_cylinder_publisher, create_point_publisher
from flyvr.tools.reconstruct import interpolate_pixel_cords
from flyvr.calibration.image_processing.calibrationconstants import CALIB_MAPPING_TOPIC

from rosutils.io import decode_url
import flydra.reconstruct

import matplotlib.pyplot as plt
import numpy as np
import cv2

import pcl

X_INDEX = 0
Y_INDEX = 1
Z_INDEX = 2
L_INDEX = 3

class Calibrator:
    def __init__(self, visualize=True, debug=False, new_reconstructor="", mask_out=False, update_parameter_server=True):
        self.data    = {}
        self.masks   = {}
        self.cvimgs  = {}
        self.dscs    = {}

        self.geom = simple_geom.Cylinder(
                base=dict(x=0,y=0,z=0),
                axis=dict(x=0,y=0,z=1),
                radius=0.5)

        self.mask_out = mask_out
        self.visualize = visualize
        self.debug = debug
        self.update_parameter_server = update_parameter_server

        if new_reconstructor:
            self.flydra = flydra.reconstruct.Reconstructor(
                            cal_source=new_reconstructor)
            self.flydra_calib = new_reconstructor
        else:
            self.flydra = None
            self.flydra_calib = ''

        self.smoothed = None
        self.filenames = []

    @property
    def display_servers(self):
        return sorted(self.data)

    def load(self, b, nowait_display_server):
        with rosbag.Bag(b, 'r') as bag:
            rospy.loginfo("processing %s" % b)
            self.filenames.append(b)
            for topic, msg, t in bag.read_messages(topics=[CALIB_MAPPING_TOPIC]):
                key = msg.display_server
                try:
                    self.data[key]
                except KeyError:
                    self.data[key] = {}

                    dsc = display_client.DisplayServerProxy(key,
                                wait=not nowait_display_server,
                                prefer_parameter_server_properties=nowait_display_server)

                    mask = dsc.get_display_mask()
                    cvimg = dsc.new_image(color=255, mask=~mask, nchan=3, dtype=np.uint8)

                    self.dscs[key] = dsc
                    self.cvimgs[key] = cvimg
                    self.masks[key] = mask
                    if self.visualize:
                        cv2.namedWindow(key)

                finally:
                    #support old bag files that lacked luminace information
                    try:
                        luminance = msg.pixel_ptc_projector_luminance
                    except AttributeError:
                        luminance = 255

                    pixel = np.array((msg.pixel_projector.x, msg.pixel_projector.y))

                    #recompute 3D position
                    if self.flydra:
                        pts = []
                        for projpt in msg.points:
                            pts.append( (projpt.camera,(projpt.pixel.x,projpt.pixel.y)) )
                        xyz = self.flydra.find3d(pts,return_line_coords=False, undistort=True)
                    else:
                        xyz = np.array((msg.position.x,msg.position.y,msg.position.z))

                    try:
                        self.data[msg.display_server][msg.vdisp].append( [xyz,pixel,luminance] )
                    except KeyError:
                        self.data[msg.display_server][msg.vdisp] = [ [xyz,pixel,luminance] ]

    def interpolate_points(self, xyz_arr, points_2d_arr, dsc, interp_method):
        #interpolation in XYZ and not in texcordinates
        #to stop filter / wraparound effects (i.e. tex-coordinates in U/V)
        #wrap 0->1->0

        x = xyz_arr[:,0]
        y = xyz_arr[:,1]
        z = xyz_arr[:,2]

        x0 = interpolate_pixel_cords(
                points_2d_arr, x,
                img_width=dsc.width,
                img_height=dsc.height,
                method=interp_method)
        y0 = interpolate_pixel_cords(
                points_2d_arr, y,
                img_width=dsc.width,
                img_height=dsc.height,
                method=interp_method)
        z0 = interpolate_pixel_cords(
                points_2d_arr, z,
                img_width=dsc.width,
                img_height=dsc.height,
                method=interp_method)

        xs = x0.ravel()
        ys = y0.ravel()
        zs = z0.ravel()
        xyz_new = np.array( [xs, ys, zs] ).T

        uv = self.geom.worldcoord2texcoord(xyz_new)

        u0 = uv[:,0]
        v0 = uv[:,1]
        u0.shape = x0.shape
        v0.shape = x0.shape

        return u0, v0

    def show_vdisp_points(self, arr, ds, u0, v0, vdisp):
        plt.figure()

        plt.subplot(2,3,1)
        plt.imshow(arr[:,:,X_INDEX])
        plt.colorbar()
        plt.title('%s/%s X' % (ds,vdisp))

        plt.subplot(2,3,2)
        plt.imshow(arr[:,:,Y_INDEX])
        plt.colorbar()
        plt.title('%s/%s Y' % (ds,vdisp))

        plt.subplot(2,3,3)
        plt.imshow(arr[:,:,Z_INDEX])
        plt.colorbar()
        plt.title('%s/%s Z' % (ds,vdisp))

        plt.subplot(2,3,4)
        plt.imshow(u0)
        plt.title('%s/%s U' % (ds,vdisp))
        plt.colorbar()

        plt.subplot(2,3,5)
        plt.imshow(v0)
        plt.colorbar()
        plt.title('%s/%s V' % (ds,vdisp))

    def smooth(self, amount):
        rospy.loginfo("smoothing points with MLS filter %f" % amount)
        self.smoothed = amount

        all_3d = []
        for ds in self.display_servers:
            for vdisp in self.data[ds]:
                for xyz,pixel,lum in self.data[ds][vdisp]:
                    all_3d.append(xyz)

        create_point_cloud_message_publisher(all_3d,'/calibration/pre_smooth',
            publish_now=True, latch=True)

        p = pcl.PointCloud()
        p.from_array(np.array(all_3d,dtype=np.float32))
        smoothed = p.filter_mls(amount).to_list()

        i = 0
        all_3d = []
        for ds in self.display_servers:
            for vdisp in self.data[ds]:
                for xyz,pixel,lum in self.data[ds][vdisp]:
                    sxyz = smoothed[i]
                    xyz[0] = sxyz[0]; xyz[1] = sxyz[1]; xyz[2] = sxyz[2]
                    all_3d.append(xyz)
                    i += 1

        create_point_cloud_message_publisher(all_3d,'/calibration/post_smooth',
            publish_now=True, latch=True)


    def do_exr(self, interp_method, do_luminance, gamma, blend_curve):
        exrs = {ds:{} for ds in self.display_servers}

        do_xyz = ["x","y","z"]
        exrs_uvl = ["u","v","l","ui","vi","li"]

        comment = 'created from %s' % (", ".join(self.filenames),)
        if self.smoothed is not None:
            comment += '. MLS smoothed by %f' % self.smoothed
        if self.flydra_calib:
            comment += '. 3D reconstruction from %s' % self.flydra_calib
        comment += '. Interpolation method %s' % interp_method
        comment += '. Luminance blend %s (gamma: %.2f curve: %.2f)' % (do_luminance, gamma, blend_curve)
        rospy.loginfo(comment)

        if do_luminance:
            blender = blend.Blender(
                        True or self.visualize,
                        os.getcwd(),
                        debug_exr=self.debug,
                        exr_comments=comment
            )

        def alloc_exr_mask(ds, name):
            dsc = self.dscs[ds]
            exrs[ds][name] = {
                "exr":np.zeros((768,1024)),
            }
            exrs[ds][name]["exr"].fill(np.nan)

        def update_mask(ds, name, val, vdispmask=None):
            valid_val = ~np.isnan(val)
            if vdispmask is not None:
                valid = np.logical_or(vdispmask,valid_val)
            else:
                valid = valid_val
            exrs[ds][name]["exr"][valid] = val[valid]

        all_3d = []
        for ds in self.display_servers:
            dsc = self.dscs[ds]

            if self.visualize:
                cv2.namedWindow(ds)
                img = self.cvimgs[ds]

            for ax in do_xyz + exrs_uvl:
                alloc_exr_mask(ds, ax)

            ds_3d = []
            for vdisp in self.data[ds]:
                vdispmask = dsc.get_virtual_display_mask(vdisp, squeeze=True)

                vdisp_3d = []
                vdisp_2d = []
                vdisp_lum = []

                if self.visualize:
                    arr = np.zeros((768,1024,4))
                    arr.fill(np.nan)

                for xyz,pixel,lum in self.data[ds][vdisp]: #just do one vdisp
                    vdisp_2d.append(pixel)
                    vdisp_3d.append(xyz)
                    vdisp_lum.append(lum)

                    if self.visualize:
                        col = pixel[0]
                        row = pixel[1]
                        add_crosshairs_to_nparr(img, row=row, col=col, chan=2, sz=1)

                        arr[row-2:row+2,col-2:col+2,X_INDEX] = xyz[0]
                        arr[row-2:row+2,col-2:col+2,Y_INDEX] = xyz[1]
                        arr[row-2:row+2,col-2:col+2,Z_INDEX] = xyz[2]
                        arr[row-2:row+2,col-2:col+2,L_INDEX] = lum

                ds_3d.extend(vdisp_3d)

                vdisp_lum_arr = np.array(vdisp_lum, dtype=np.float)
                vdisp_2d_arr = np.array(vdisp_2d, dtype=np.float)
                vdisp_3d_arr = np.array(vdisp_3d, dtype=np.float)

                #construct the interpoolated geometry (uv) <-> 3d (xyz) mapping.
                ui,vi = self.interpolate_points(
                            vdisp_3d_arr,
                            vdisp_2d_arr,
                            dsc,
                            interp_method)
                #interpolate luminance
                li = interpolate_pixel_cords(
                            points_2d=vdisp_2d_arr,
                            values_1d=vdisp_lum_arr,
                            img_width=dsc.width,
                            img_height=dsc.height,
                            method=interp_method)
                update_mask(ds, "ui", ui, vdispmask)
                update_mask(ds, "vi", vi, vdispmask)
                update_mask(ds, "li", li, vdispmask)

                #and keep an unterpolated copy
                u,v = self.interpolate_points(
                            vdisp_3d_arr,
                            vdisp_2d_arr,
                            dsc,
                            "none")
                update_mask(ds, "u", u, vdispmask)
                update_mask(ds, "v", v, vdispmask)

                if self.debug:
                    for axnum,ax in enumerate(do_xyz):
                        update_mask(ds, ax,
                            interpolate_pixel_cords(
                                points_2d=vdisp_2d_arr,
                                values_1d=vdisp_3d_arr[:,axnum],
                                img_width=dsc.width,
                                img_height=dsc.height,
                                method="none")
                        )

                if self.visualize:
                    self.show_vdisp_points(arr, ds, ui, vi, vdisp)

            if do_luminance:
                blender.add_display_server(
                        ds,
                        dsc,
                        exrs[ds]["u"]["exr"].astype(np.float32),exrs[ds]["v"]["exr"].astype(np.float32),
                        exrs[ds]["ui"]["exr"].astype(np.float32),exrs[ds]["vi"]["exr"].astype(np.float32),
                )

            if self.visualize:
                cv2.imshow(ds, img)
            if self.debug:
                create_pcd_file_from_points(decode_url('%s.pcd' % ds),ds_3d)

            all_3d.extend(ds_3d)

        if self.debug:
            create_pcd_file_from_points(decode_url('all.pcd'),all_3d)
            create_point_cloud_message_publisher(all_3d,'/calibration/points',publish_now=True, latch=True)

        if do_luminance:
            blended = blender.blend(gamma, blend_curve)

        for ds in self.display_servers:
            dsc = self.dscs[ds]

            if do_luminance:
                final_li = blended[ds]
            else:
                #set the luminance channel to 1 (no blending) inside the viewport,
                #0 (off) outside
                final_li = np.ones_like(exrs[ds]["ui"]["exr"])
                final_li[np.isnan(exrs[ds]["ui"]["exr"])] = 0

            #in our shader convention -1 means no data, not NaN
            exrs[ds]["ui"]["exr"][np.isnan(exrs[ds]["ui"]["exr"])] = -1
            exrs[ds]["vi"]["exr"][np.isnan(exrs[ds]["vi"]["exr"])] = -1
            final_ui = exrs[ds]["ui"]["exr"]
            final_vi = exrs[ds]["vi"]["exr"]

            if self.visualize:
                plt.figure()
                plt.imshow(final_ui)
                plt.colorbar()
                plt.title('%s U' % ds)

                plt.figure()
                plt.imshow(final_vi)
                plt.colorbar()
                plt.title('%s V' % ds)

                plt.figure()
                plt.imshow(final_li)
                plt.colorbar()
                plt.title('%s L' % ds)

            exrpath = "%s.exr" % ds
            exr.save_exr(
                    exrpath,
                    r=final_ui,
                    g=final_vi,
                    b=final_li,
                    comments=comment)

            if self.debug:
                exrs[ds]["u"]["exr"][np.isnan(exrs[ds]["u"]["exr"])] = -1
                exrs[ds]["v"]["exr"][np.isnan(exrs[ds]["v"]["exr"])] = -1
                final_u = exrs[ds]["u"]["exr"]
                final_v = exrs[ds]["v"]["exr"]
                exr.save_exr(
                        "%s.nointerp.exr" % ds,
                        r=final_u,
                        g=final_v,
                        b=np.zeros_like(final_u),
                        comments=comment)


            #save the resulting geometry to the parameter server
            if self.update_parameter_server:
                geom = self.geom.to_geom_dict()
                dsc.set_geometry(geom)
                dsc.set_binary_exr(exrpath)
                rospy.loginfo("updated parameter server")
                subprocess.call(["rosnode", "kill", "/%s"%ds])
                rospy.loginfo("restarted server")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--calibration', type=str, action='append', required=True, nargs='*', help=\
        "bag file containing calibration messages")
    parser.add_argument(
        '--visualize', action='store_true', default=False, help=\
        "show plots")
    parser.add_argument(
        '--debug', action='store_true', default=False, help=\
        "save extra exr files for debugging")
    parser.add_argument(
        '--luminance', action='store_true', default=False, help=\
        "also blen luminance")
    parser.add_argument(
        '--update', action='store_true', default=False, help=\
        "update display server(s) with new exr data")
    parser.add_argument(
        '--reconstructor', type=str, help=\
        "path to a new flydra calibration (use the "\
        "new reconstructor to calculate the 3D position)")
    parser.add_argument(
        '--gamma', type=float, default=2.2, help=\
        "gamma for blending regious")
    parser.add_argument(
        '--blend-curve', type=float, default=1.0, help=\
        "shape of blend function in overlapping regious")
    parser.add_argument(
        '--smooth', type=float, help=\
        "amount to smooth by, see pcl.filter_msl, rviz the /calibratio/ topics",
        metavar="[0...1.5]")
    parser.add_argument(
        '--interpolation', type=str, default="linear", choices=["nearest","linear","cubic","none"], help=\
        "interpolation method in (see scipy.interpolate.griddata)")
    parser.add_argument(
        '--no-wait-display-server', action='store_true', default=False, help=\
        "dont wait for display server - use display server configuration from "
        "parameter server")

    # use argparse, but only after ROS did its thing
    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    rospy.init_node('calibration_generate_exr', anonymous=True)

    if args.visualize:
        cv2.startWindowThread()

    cal = Calibrator(
                visualize=args.visualize,
                debug=args.debug,
                new_reconstructor=args.reconstructor,
                mask_out=False,
                update_parameter_server=args.update)

    tmp=[]
    [tmp.extend(_) for _ in args.calibration]
    cal_files = []
    [cal_files.extend(glob.glob(_)) for _ in tmp]
    rospy.loginfo('cal_files: %r'%cal_files)
    if len(cal_files):
        #multiple bag files
        for c in cal_files:
            fn = decode_url(c)
            assert os.path.exists(fn)
            cal.load(fn, args.no_wait_display_server)
    else:
        rospy.logfatal('No data. Specify inputs with --calibration <file.bag>')
        rospy.signal_shutdown('no data')
        sys.exit(1)

    if args.smooth:
        cal.smooth(args.smooth)

    cal.do_exr(args.interpolation, args.luminance, args.gamma, args.blend_curve)

    if cal.visualize:
        plt.show()

    rospy.loginfo('finished')

    rospy.spin()

