#!/usr/bin/env python

import os.path

import numpy as np
import matplotlib.pyplot as plt
import argparse

import roslib;
roslib.load_manifest('freemoovr')
import rospy

import freemoovr.simple_geom as simple_geom

from calib.io import MultiCalSelfCam, AllPointPickle
from calib.visualization import create_pcd_file_from_points, \
    create_point_cloud_message_publisher, show_pointcloud_3d_plot, \
    create_cylinder_publisher, create_point_publisher, \
    show_pointcloud_2d_plots
from calib.reconstruct import interpolate_pixel_cords
from rosutils.io import decode_url
from rosutils.formats import camera_calibration_yaml_to_radfile

import flydra
import flydra.reconstruct
import flydra.align

import exr
import freemoovr.display_client as display_client

X_INDEX = 0
Y_INDEX = 1

class Calibrator(object):
    def __init__(self, flydra_calib, laser_pkl, visualize, inlier_dir):

        if inlier_dir is None:
            if not os.path.isdir(flydra_calib):
                raise RuntimeError('cannot guess inlier directory')
            self.inlier_dir = flydra_calib
        else:
            self.inlier_dir = inlier_dir

        self.flydra_calib = flydra_calib
        self.laser_pkl = laser_pkl
        self.visualize = visualize

        #publish the camera positions and the inlier set from the flydra calibraion
        MultiCalSelfCam.publish_calibration_points(self.flydra_calib,
                                                   topic_base='/flydra',
                                                   inlier_dir=self.inlier_dir)

        print '*'*80
        print 'loaded original calibration from', self.flydra_calib
        print '*'*80

        print '*'*80
        print 'loaded inliers from', self.inlier_dir
        print '*'*80

        self.fly = flydra.reconstruct.Reconstructor(cal_source=self.flydra_calib)

        self.laser = AllPointPickle()

        #load default laser points
        self.load_laser_points()

        if 1:
            # FIXME: we have hard-coded our geometry here.
            self.geom = simple_geom.Cylinder(
                base=dict(x=0,y=0,z=0),
                axis=dict(x=0,y=0,z=1),
                radius=0.5)
            print 'Using geometry -----------------------'
            print self.geom.to_geom_dict()
            print '--------------------------------------'
        if 0:
            # This is from John's original file
            cx,cy,cz,ax,ay,az,radius = [0.331529438495636, 0.5152832865715027, 0.2756080627441406, -0.7905913591384888, 0.24592067301273346, -7.272743225097656, 2.241607666015625]
            c.set_cylinder_model(cx,cy,cz,ax,ay,az,radius)

    def load_laser_points(self, path=None):
        if not path:
            path = self.laser_pkl

        #we need all the laser points for cylinder fitting
        self.laser.initilize_from_directory(self.laser_pkl)
        print "number of laser points", self.laser.num_points

        #get all points visible in 2 or more cameras
        #and use the flydra calibration to get the 3d coords
        pts = self.laser.get_points_in_cameras(self.fly.get_cam_ids())
        self.xyz = np.array([self.fly.find3d(pt,return_line_coords=False, undistort=True) for pt in pts])

        create_point_cloud_message_publisher(
            self.xyz,
            topic_name='/flydracalib/points',
            publish_now=True,
            latch=True)

        create_pcd_file_from_points(
                decode_url('package://flycave/calibration/pcd/flydracyl.pcd'),
                self.xyz)

        if self.visualize:
            plt.figure()
            show_pointcloud_3d_plot(self.xyz)
            fig = plt.figure()
            show_pointcloud_2d_plots(self.xyz,fig=fig)

    def generate_exrs(self, display_server_numbers, prefer_parameter_server_properties=False, filt_method='linear', mask_out=False, mask_fill_value=np.nan):
        #so, we can successfully recover a cylinder. Find the pixel coords of those points in the
        #projector cameras, and then, via the projector calibration, find the corresponding pixel
        #in the projector
        for dsnum in display_server_numbers:
            dsname = "ds%d" % dsnum

            #use the display server calibration to get the projector pixel coordinate
            ds = flydra.reconstruct.Reconstructor(
                    cal_source=decode_url('package://flycave/calibration/triplets/%s/result' % dsname))

            flydra_cams = self.fly.get_cam_ids()
            ds_cams = [c for c in ds.get_cam_ids() if not c.startswith('display_server')]

            flydra_points_3d = []
            ds_points_2d = []
            ds_points_3d = []

            arr = np.zeros((768,1024,2))
            arr.fill(np.nan)

            dsc = display_client.DisplayServerProxy("/display_server%d" % dsnum,
                        wait=False,
                        prefer_parameter_server_properties=prefer_parameter_server_properties)

            #all points visible in 2 or more cameras
            for pts in self.laser.get_points_in_cameras():
                flydra_ds_points_2d = []
                ds_ds_points_2d = []
                for pt in pts:
                    cam, _ = pt
                    if cam in flydra_cams:
                        flydra_ds_points_2d.append(pt)
                    if cam in ds_cams:
                        ds_ds_points_2d.append(pt)

                #need at least 2 of each for 3D reconstruction in each
                if len(flydra_ds_points_2d) >= 2 and len(ds_ds_points_2d) >= 2:
                    #get the projector coord
                    ds_3d = ds.find3d(ds_ds_points_2d,return_line_coords=False)
                    ds_2d = ds.find2d('display_server%d' % dsnum,ds_3d)

                    #check bounds are realistic
                    #FIXME: have I swapped u/v here????
                    #FIXME: can I make this check better???
                    u,v = ds_2d
                    if u > dsc.width or v > dsc.height or u < 0 or v < 0:
                        continue

                    try:
                        #get the real 3D coord
                        flydra_3d = self.fly.find3d(flydra_ds_points_2d,return_line_coords=False, undistort=True)
                        x,y,z = flydra_3d

                        #this is just a debug image for asessing coverage
                        arr[v-2:v+2,u-2:u+2,Y_INDEX] = y
                        arr[v-2:v+2,u-2:u+2,X_INDEX] = x

                        flydra_points_3d.append(flydra_3d)
                        ds_points_3d.append(ds_3d)

                        ds_points_2d.append(ds_2d)
                    except IndexError:
                        print "SHIT?"*10
                        pass

            create_pcd_file_from_points(
                decode_url('package://flycave/calibration/pcd/%sflydra3d.pcd' % dsname),
                flydra_points_3d)
            create_pcd_file_from_points(
                decode_url('package://flycave/calibration/pcd/%s3d.pcd' % dsname),
                ds_points_3d)

            flydra_points_3d_arr = np.array(flydra_points_3d)

            ds_points_2d_arr = np.array(ds_points_2d)
            ds_points_3d_arr = np.array(ds_points_3d)

            print "%s constructed from %d 2D coords mapping to %d 3D points" % (dsname, len(ds_points_2d), len(flydra_points_3d_arr))

            create_point_cloud_message_publisher(
                                flydra_points_3d_arr,
                                topic_name='/flydracalib/%spoints2' % dsname,
                                publish_now=True,
                                latch=True)

            uv = self.geom.worldcoord2texcoord(flydra_points_3d_arr)

            u0 = interpolate_pixel_cords(
                    ds_points_2d_arr, uv[:,0],
                    img_width=dsc.width,
                    img_height=dsc.height,
                    method=filt_method)
            v0 = interpolate_pixel_cords(
                    ds_points_2d_arr, uv[:,1],
                    img_width=dsc.width,
                    img_height=dsc.height,
                    method=filt_method)

            u0nonan = np.nan_to_num(u0)
            v0nonan = np.nan_to_num(v0)
            if u0nonan.max() > 1 or u0nonan.min() > 0:
                #the cubic interpolate function is sensitive to step changes, and introduces quite large
                #errors in smoothing. Crudely replace those values with invalid (nan)
                print 'replacing out of range errors in smoothed u'
                u0[u0>1] = np.nan
                u0[u0<0] = np.nan
            if v0nonan.max() > 1 or v0nonan.min() > 0:
                #the cubic interpolate function is sensitive to step changes, and introduces quite large
                #errors in smoothing. Crudely replace those values with invalid (nan)
                print 'replacing out of range errors in smoothed u'
                v0[v0>1] = np.nan
                v0[v0<0] = np.nan

            if mask_out:
                #mask out invalid parts if the convex hull doesnt work....
                #mask is usually used for *= with an image, but here we will use boolean indexing to fill
                #invalid areas with nan
                #... so fiddle the mask a little
                mask = dsc.get_virtual_display_mask('vdisp').squeeze()
                mask = ~mask

                u0[mask] = mask_fill_value
                v0[mask] = mask_fill_value

            if self.visualize:
                plt.figure()
                plt.subplot(211)
                plt.imshow(arr[:,:,X_INDEX])
                plt.colorbar()
                plt.title('%s X' % dsname)
                plt.subplot(212)
                plt.imshow(u0)
                plt.title('%s U' % dsname)
                plt.colorbar()

                plt.figure()
                plt.subplot(211)
                plt.imshow(arr[:,:,Y_INDEX])
                plt.colorbar()
                plt.title('%s Y' % dsname)
                plt.subplot(212)
                plt.imshow(v0)
                plt.colorbar()
                plt.title('%s V' % dsname)

            #save the exr file, -1 means no data, not NaN
            u0[np.isnan(u0)] = -1
            v0[np.isnan(v0)] = -1
            exrpath = decode_url('package://flycave/calibration/exr/%s.exr' % dsname)
            exr.save_exr(exrpath, r=u0, g=v0, b=np.zeros_like(u0))

            #save the resulting geometry to the parameter server
            geom = self.geom.to_geom_dict()
            dsc.set_geometry(geom)
            dsc.set_binary_exr(exrpath)

if __name__ == "__main__":
    rospy.init_node('calib_generate_exr', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--laser-pkldir', type=str, default='package://flycave/calibration/laser',
        help='path to dir containing result.pkl, resolution.pkl, etc')
    parser.add_argument(
        '--flydra-calib', type=str, default='package://flycave/calibration/flydra.aligned',
        help='path to flydra multicamselfcal result dir')
    parser.add_argument(
        '--inlier-dir', type=str, default=None,
        help='path to directory containing Xe.dat, Re.dat, Ce.dat')
    parser.add_argument(
        '--parameter-server-properties', default=False, action='store_true',
        help='get display server properties (height, width, etc) from the parameter server. '\
             'this means the display server does not need to be running.')
    parser.add_argument(
        '--display-server-numbers', type=str, default='0,1,3',
        help='comma separated list of display server numbers')
    parser.add_argument('--visualize', default=False, action='store_true')
    args = parser.parse_args()

    c = Calibrator(
            decode_url(args.flydra_calib),
            decode_url(args.laser_pkldir),
            args.visualize,
            args.inlier_dir)

    c.generate_exrs(
        [int(i) for i in args.display_server_numbers.split(',')],
        prefer_parameter_server_properties=args.parameter_server_properties,
        filt_method='linear',
        mask_out=False,
        mask_fill_value=np.nan)

    if args.visualize:
        plt.show()

    #print 'now spinning forever...'
    #rospy.spin()
