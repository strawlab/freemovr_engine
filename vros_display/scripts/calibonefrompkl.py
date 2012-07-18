import numpy as np
import matplotlib.pyplot as plt
import scipy.ndimage
import scipy.misc
import scipy.interpolate

import roslib;
roslib.load_manifest('vros_display')
roslib.load_manifest('motmot_ros_utils')
import rospy

from calib.io import AllPointPickle
from calib.visualization import create_pcd_file_from_points
from rosutils.io import decode_url

import flydra
import flydra.reconstruct
import flydra.align

import exr
import display_client

rospy.init_node('calibone', anonymous=True)

LASER_PKL       = decode_url('package://flycave/calibration/ds3laser')
FLYDRA_CALIB    = decode_url('package://flycave/calibration/flydra')
DS1_CALIB       = decode_url('package://flycave/calibration/triplets/ds1/result')

X_INDEX = 0
Y_INDEX = 1
FILT_METHOD = 'cubic'

#so, we can successfully recover a cylinder. Find the pixel coords of those points in the
#projector cameras, and then, via the projector calibration, find the corresponding pixel
#in the projector
if 1:
    #use the flydra calibration to get the 3d coords
    fly = flydra.reconstruct.Reconstructor(cal_source=FLYDRA_CALIB)

    #use the display server calibration to get the projector pixel coordinate
    ds1 = flydra.reconstruct.Reconstructor(cal_source=DS1_CALIB)

    laser = AllPointPickle()
    laser.initilize_from_directory(LASER_PKL)

    print "number of laser points",laser.num_points

    flydra_cams = fly.get_cam_ids()
    ds1_cams = [c for c in ds1.get_cam_ids() if not c.startswith('display_server')]

    flydra_points_3d = []
    ds1_points_2d = []
    ds1_points_3d = []

    arr = np.zeros((768,1024,2))
    arr.fill(np.nan)

    dsc = display_client.DisplayServerProxy("/display_server1", wait=False)

    #all points visible in 2 or more cameras
    for pts in laser.get_points_in_cameras():
        flydra_ds1_points_2d = []
        ds1_ds1_points_2d = []
        for pt in pts:
            cam, _ = pt
            if cam in flydra_cams:
                flydra_ds1_points_2d.append(pt)
            if cam in ds1_cams:
                ds1_ds1_points_2d.append(pt)

        #need at least 2 of each for 3D reconstruction in each
        if len(flydra_ds1_points_2d) >= 2 and len(ds1_ds1_points_2d) >= 2:
            #get the projector coord
            ds1_3d = ds1.find3d(ds1_ds1_points_2d,return_line_coords=False)
            ds1_2d = ds1.find2d('display_server1',ds1_3d)

            #check bounds are realistic
            #FIXME: have I swapped u/v here????
            u,v = ds1_2d
            if u > dsc.width or v > dsc.height or u < 0 or v < 0:
                continue

            try:
                #get the real 3D coord
                flydra_3d = fly.find3d(flydra_ds1_points_2d,return_line_coords=False)
                x,y,z = flydra_3d

                #this is just a debug image for asessing coverage
                #if the 2D point is outside the image bounds then it must be invalid. 
                #FIXME: we really could improve this check...
                arr[v-2:v+2,u-2:u+2,Y_INDEX] = y
                arr[v-2:v+2,u-2:u+2,X_INDEX] = x

                flydra_points_3d.append(flydra_3d)
                ds1_points_3d.append(ds1_3d)

                ds1_points_2d.append(ds1_2d)
            except IndexError:
                print "SHIT?"*10
                pass

    create_pcd_file_from_points('/tmp/flydra3d.pcd',flydra_points_3d)
    create_pcd_file_from_points('/tmp/ds13d.pcd',ds1_points_3d)

    flydra_points_3d_arr = np.array(flydra_points_3d)
    ds1_points_2d_arr = np.array(ds1_points_2d)
    ds1_points_3d_arr = np.array(ds1_points_3d)

    print "ds1 constructed from %d points" % (len(ds1_points_2d))

    if 1:
        #make EXR files via smoothing reprojected 3d points in the projecor view
        grid_y, grid_x = np.mgrid[0:dsc.height, 0:dsc.width]

        y0 = scipy.interpolate.griddata(
                ds1_points_2d_arr,
                flydra_points_3d_arr[:,Y_INDEX],
                (grid_x, grid_y),
                method=FILT_METHOD,
                fill_value=np.nan)
        x0 = scipy.interpolate.griddata(
                ds1_points_2d_arr,
                flydra_points_3d_arr[:,X_INDEX],
                (grid_x, grid_y),
                method=FILT_METHOD,
                fill_value=np.nan)

        if 0:
            #mask out invalid parts if the convex hull doesnt work....
            #mask is usually used for *= with an image, but here we will use boolean indexing to fill
            #invalid areas with nan
            #... so fiddle the mask a little
            mask = dsc.get_virtual_display_mask('vdisp').squeeze()
            mask = ~mask

            x0[mask] = np.nan
            y0[mask] = np.nan

        plt.figure()
        plt.subplot(211)
        plt.imshow(arr[:,:,Y_INDEX])
        plt.colorbar()
        plt.title('ds1 Original Y')
        plt.subplot(212)
        plt.imshow(y0)
        plt.colorbar()
        plt.title('Filtered %s' % FILT_METHOD)

        plt.figure()
        plt.subplot(211)
        plt.imshow(arr[:,:,X_INDEX])
        plt.colorbar()
        plt.title('ds1 Original X')
        plt.subplot(212)
        plt.imshow(x0)
        plt.colorbar()
        plt.title('Filtered %s' % FILT_METHOD)

        exr.save_exr( 'ds1.exr', r=y0, g=x0, b=np.zeros_like(x0))

        plt.show()

    if 0:
        s,R,T = flydra.align.estsimt(flydra_points_3d_arr.T,ds1_points_3d_arr.T)

#rospy.spin()

