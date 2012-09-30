#!/usr/bin/env python

import roslib
roslib.load_manifest('vros_display')
roslib.load_manifest('rosbag')
roslib.load_manifest('motmot_ros_utils')
import rosbag
import rospy

import simple_geom
import display_client
import calib.imgproc
from calib.visualization import create_pcd_file_from_points, create_point_cloud_message_publisher, show_pointcloud_3d_plot, create_cylinder_publisher, create_point_publisher
from calib.reconstruct import interpolate_pixel_cords
from calib.calibrationconstants import CALIB_MAPPING_TOPIC
from rosutils.io import decode_url
import flydra.reconstruct

import argparse
import os.path

import matplotlib.pyplot as plt
import numpy as np
import cv,cv2
import exr

X_INDEX = 0
Y_INDEX = 1
Z_INDEX = 2

class Calibrator:
    def __init__(self, visualize=True, new_reconstructor="", mask_out=False, update_parameter_server=True):
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
        self.update_parameter_server = update_parameter_server

        if new_reconstructor:
            self.flydra = flydra.reconstruct.Reconstructor(
                            cal_source=new_reconstructor)
        else:
            self.flydra = None

        self.filenames = []
        
    def load(self, b):
        with rosbag.Bag(b, 'r') as bag:
            rospy.loginfo("processing %s" % b)
            self.filenames.append(b)
            for topic, msg, t in bag.read_messages(topics=[CALIB_MAPPING_TOPIC]):
                key = msg.display_server
                try:
                    self.data[key]
                except KeyError:
                    self.data[key] = {}

                    dsc = display_client.DisplayServerProxy(key,wait=True)
                    mask = dsc.get_display_mask()
                    cvimg = dsc.new_image(color=255, mask=~mask, nchan=3, dtype=np.uint8)

                    self.dscs[key] = dsc
                    self.cvimgs[key] = cvimg
                    self.masks[key] = mask
                    if self.visualize:
                        cv2.namedWindow(key)

                finally:
                    xyz = np.array((msg.position.x,msg.position.y,msg.position.z))
                    pixel = np.array((msg.pixel_projector.x, msg.pixel_projector.y))
                    
                    pts = []
                    for projpt in msg.points:
                        pts.append( (projpt.camera,(projpt.pixel.x,projpt.pixel.y)) )

                    try:
                        self.data[msg.display_server][msg.vdisp].append( (xyz,pixel,pts) )
                    except KeyError:
                        self.data[msg.display_server][msg.vdisp] = [ (xyz,pixel,pts) ]

    def interpolate_points(self, xyz_arr, points_2d_arr, dsc, filt_method='linear'):
        #interpolation in XYZ to stop filter / wraparound effects

        x = xyz_arr[:,0]
        y = xyz_arr[:,1]
        z = xyz_arr[:,2]

        x0 = interpolate_pixel_cords(
                points_2d_arr, x,
                img_width=dsc.width,
                img_height=dsc.height,
                method=filt_method)
        y0 = interpolate_pixel_cords(
                points_2d_arr, y,
                img_width=dsc.width,
                img_height=dsc.height,
                method=filt_method)
        z0 = interpolate_pixel_cords(
                points_2d_arr, z,
                img_width=dsc.width,
                img_height=dsc.height,
                method=filt_method)

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

    def show_vdisp_points(self, arr, ds, vdisp):
        plt.figure()
        plt.subplot(211)
        plt.imshow(arr[:,:,X_INDEX])
        plt.colorbar()
        plt.title('%s/%s X' % (ds,vdisp))
        plt.subplot(212)
        plt.imshow(u0)
        plt.title('%s/%s U' % (ds,vdisp))
        plt.colorbar()

        plt.figure()
        plt.subplot(211)
        plt.imshow(arr[:,:,Y_INDEX])
        plt.colorbar()
        plt.title('%s/%s Y' % (ds,vdisp))
        plt.subplot(212)
        plt.imshow(v0)
        plt.colorbar()
        plt.title('%s/%s V' % (ds,vdisp))

    def do_exr(self):

        all_3d = []
        for ds in self.data:
            if self.visualize:
                cv2.namedWindow(ds)
            img = self.cvimgs[ds]
            
            #FIXME: change to using masked arrays...
            ds_u = np.zeros((768,1024))
            ds_u_mask = np.zeros((768,1024), dtype=np.bool)
            ds_vdisp_u_mask = np.ones((768,1024), dtype=np.bool)

            ds_v = np.zeros((768,1024))
            ds_v_mask = np.zeros((768,1024), dtype=np.bool)
            ds_vdisp_v_mask = np.ones((768,1024), dtype=np.bool)

            dsc = self.dscs[ds]

            ds_3d = []
            for vdisp in self.data[ds]:

                ds_vdisp_u_mask.fill(True)
                ds_vdisp_v_mask.fill(True)
                vdisp_3d = []
                vdisp_2d = []

                if self.visualize:
                    arr = np.zeros((768,1024,3))
                    arr.fill(np.nan)

                for xyz,pixel,pts in self.data[ds][vdisp]: #just do one vdisp
                    col = pixel[0]
                    row = pixel[1]
                    calib.imgproc.add_crosshairs_to_nparr(img, row=row, col=col, chan=2, sz=1)

                    if self.flydra:
                        xyz = self.flydra.find3d(pts,return_line_coords=False, undistort=True)
                    
                    vdisp_2d.append(pixel)
                    vdisp_3d.append(xyz)

                    if self.visualize:
                        arr[row-2:row+2,col-2:col+2,X_INDEX] = xyz[0]
                        arr[row-2:row+2,col-2:col+2,Y_INDEX] = xyz[1]
                        arr[row-2:row+2,col-2:col+2,Z_INDEX] = xyz[2]
                    
                u0,v0 = self.interpolate_points(
                            np.array(vdisp_3d, dtype=np.float),
                            np.array(vdisp_2d, dtype=np.float),
                            self.dscs[ds])

                ds_vdisp_u_mask[np.isnan(u0)] = False
                ds_u_mask |= ds_vdisp_u_mask 
                ds_vdisp_v_mask[np.isnan(v0)] = False
                ds_v_mask |= ds_vdisp_v_mask 

                ds_u += np.nan_to_num(u0)
                ds_v += np.nan_to_num(v0)

                ds_3d.extend(vdisp_3d)

                if self.visualize:
                    self.show_vdisp_points(arr, ds, visp)

            #save the exr file, -1 means no data, not NaN
            ds_u[~ds_u_mask] = -1
            ds_v[~ds_v_mask] = -1
            exrpath = decode_url('%s.exr' % ds)
            exr.save_exr(
                    exrpath,
                    r=ds_u, g=ds_v, b=np.zeros_like(ds_u),
                    comments=", ".join(self.filenames))

            #save the resulting geometry to the parameter server
            if self.update_parameter_server:
                geom = self.geom.to_geom_dict()
                dsc.set_geometry(geom)
                dsc.set_binary_exr(exrpath)

            if self.visualize:
                plt.figure()
                plt.imshow(ds_u)
                plt.colorbar()
                plt.title('%s U' % ds)

                plt.figure()
                plt.imshow(ds_v)
                plt.colorbar()
                plt.title('%s V' % ds)

            create_pcd_file_from_points(
                decode_url('/tmp/%sflydra3d.pcd' % ds),
                ds_3d)

            if self.visualize:
                cv2.imshow(ds, img)

            all_3d.extend(ds_3d)

        create_pcd_file_from_points(decode_url('/tmp/allflydra3d.pcd'),all_3d)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--calibration', type=str, action='append', help=\
        "bag file containing calibration messages")
    parser.add_argument(
        '--visualize', action='store_true', default=False, help=\
        "show plots")
    parser.add_argument(
        '--update', action='store_true', default=False, help=\
        "update display server(s) with new exr data")
    parser.add_argument(
        '--reconstructor', type=str, help=\
        "path to a new flydra calibration (use the "\
        "new reconstructor to calculate the 3D position)")
    args = parser.parse_args()

    rospy.init_node('calibration_generate_exr', anonymous=True)

    if args.visualize:
        cv2.startWindowThread()

#    ds0 = "CALIB20120907_222224.bag"
#    ds1 = "CALIB20120927_105939.bag" #new back 
#    ds3 = "CALIB20120908_175653.bag" #good middle
#    ds3 = "CALIB20120908_182035.bag" #sparse left mirror
#    C = decode_url("~/FLYDRA/vros-calibration/%s" % ds1)

    cal = Calibrator(
                visualize=args.visualize,
                new_reconstructor=args.reconstructor,
                mask_out=False,
                update_parameter_server=args.update)

    if args.calibration:
        #multiple bag files
        for c in args.calibration:
            fn = decode_url(c)
            assert os.path.exists(fn)
            cal.load(fn)

    cal.do_exr()
    if cal.visualize:
        plt.show()

    if not args.calibration:
        rospy.spin()

