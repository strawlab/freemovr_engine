import os.path

import numpy as np
import matplotlib.pyplot as plt
import scipy.ndimage
import scipy.misc
import scipy.interpolate
import pcl
import yaml

import roslib;
roslib.load_manifest('vros_display')
roslib.load_manifest('motmot_ros_utils')
import rospy


from calib.io import MultiCalSelfCam, AllPointPickle
from calib.visualization import create_pcd_file_from_points, create_point_cloud_message_publisher, show_pointcloud_3d_plot, create_cylinder_publisher
from calib.reconstruct import CylinderPointCloudTransformer, interpolate_pixel_cords
from rosutils.io import decode_url
from rosutils.formats import camera_calibration_yaml_to_radfile

import flydra
import flydra.reconstruct
import flydra.align

import exr
import display_client

rospy.init_node('calibone', anonymous=True)

LASER_PKL       = decode_url('package://flycave/calibration/laser')
FLYDRA_CALIB    = decode_url('package://flycave/calibration/flydra')

X_INDEX = 0
Y_INDEX = 1
FILT_METHOD = 'cubic'
FILL_VALUE = np.nan

#make sure the flydra cameras are intrinsically calibrated
name_map = MultiCalSelfCam.get_camera_names_map(FLYDRA_CALIB)
for c in MultiCalSelfCam.read_calibration_names(FLYDRA_CALIB):
    camera_calibration_yaml_to_radfile(
            decode_url('package://flycave/calibration/cameras/%s.yaml' % c),
            os.path.join(FLYDRA_CALIB,name_map[c]))

fly = flydra.reconstruct.Reconstructor(cal_source=FLYDRA_CALIB)

laser = AllPointPickle()
laser.initilize_from_directory(LASER_PKL)

print "number of laser points",laser.num_points

#get all points visible in 2 or more cameras
#and use the flydra calibration to get the 3d coords
pts = laser.get_points_in_cameras(fly.get_cam_ids())
xyz = np.array([fly.find3d(pt,return_line_coords=False) for pt in pts])
create_point_cloud_message_publisher(
        xyz,
        topic_name='/flydracalib/points',
        publish_now=True,
        latch=True)

create_pcd_file_from_points(
        decode_url('package://flycave/calibration/pcd/flydracyl.pcd'),
        xyz)

fig = plt.figure()
show_pointcloud_3d_plot(xyz)

##fit a cylinder to this cloud
#p = pcl.PointCloud()
#p.from_list(xyz)

##smooth = p.filter_mls(0.5)
##create_point_cloud_message_publisher(
##        xyz,
##        topic_name='/flydracalib/pointssmooth',
##        publish_now=True,
##        latch=True)

#seg = p.make_segmenter_normals(searchRadius=0.2)
#seg.set_optimize_coefficients (True);
#seg.set_model_type (pcl.SACMODEL_CYLINDER)
#seg.set_method_type (pcl.SAC_RANSAC)
#seg.set_normal_distance_weight (0.01)
#seg.set_max_iterations (10000)
#seg.set_distance_threshold (0.5)
#seg.set_radius_limits (1, 3)
#seg.set_axis(0.41,1.894,2.733)
#seg.set_eps_angle(0.1)

#indices, model = seg.segment()

#print model
#print len(indices)

#cx,cy,cz,ax,ay,az,radius = model 

#create_cylinder_publisher(
#    cx,cy,cz,ax,ay,az,radius,
#    topic_name='/flydracalib/cyl',
#    publish_now=True,
#    latch=True,
#    length=1,
#    color=(0,1,0,0.3))

cx,cy,cz,ax,ay,az,radius = [0.331529438495636, 0.5152832865715027, 0.2756080627441406, -0.7905913591384888, 0.24592067301273346, -7.272743225097656, 2.241607666015625]
recon = CylinderPointCloudTransformer(cx,cy,cz,ax,ay,az,radius,xyz)

create_point_cloud_message_publisher(
        recon.move_cloud(xyz),
        topic_name='/flydracalib/points2',
        publish_now=True,
        latch=True)

#so, we can successfully recover a cylinder. Find the pixel coords of those points in the
#projector cameras, and then, via the projector calibration, find the corresponding pixel
#in the projector
for dsnum in (0,):
    dsname = "ds%d" % dsnum

    #use the display server calibration to get the projector pixel coordinate
    ds = flydra.reconstruct.Reconstructor(
            cal_source=decode_url('package://flycave/calibration/triplets/%s/result' % dsname))

    flydra_cams = fly.get_cam_ids()
    ds_cams = [c for c in ds.get_cam_ids() if not c.startswith('display_server')]

    flydra_points_3d = []
    ds_points_2d = []
    ds_points_3d = []

    arr = np.zeros((768,1024,2))
    arr.fill(np.nan)

    dsc = display_client.DisplayServerProxy("/display_server%d" % dsnum, wait=False, prefer_parameter_server_properties=True)

    #all points visible in 2 or more cameras
    for pts in laser.get_points_in_cameras():
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
                flydra_3d = fly.find3d(flydra_ds_points_2d,return_line_coords=False)
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

    print "f3d shape", flydra_points_3d_arr

    ds_points_2d_arr = np.array(ds_points_2d)
    ds_points_3d_arr = np.array(ds_points_3d)

    print "ds2d shape", ds_points_2d_arr.shape

    print "%s constructed from %d 2D coords mapping to %d 3D points" % (dsname, len(ds_points_2d), len(flydra_points_3d_arr))

    if 1:
        #make EXR files via smoothing reprojected 3d points in the projecor view

        new = recon.move_cloud(flydra_points_3d_arr)

        create_point_cloud_message_publisher(
                            new,
                            topic_name='/flydracalib/%spoints2' % dsname,
                            publish_now=True,
                            latch=True)

        uv = recon.in_cylinder_coordinates(new)

        u0 = interpolate_pixel_cords(
                ds_points_2d_arr, uv[:,0],
                img_width=dsc.width,
                img_height=dsc.height,
                method=FILT_METHOD)
        v0 = interpolate_pixel_cords(
                ds_points_2d_arr, uv[:,1],
                img_width=dsc.width,
                img_height=dsc.height,
                method=FILT_METHOD)

        y0 = interpolate_pixel_cords(
                ds_points_2d_arr, flydra_points_3d_arr[:,Y_INDEX],
                img_width=dsc.width,
                img_height=dsc.height,
                method=FILT_METHOD)
        x0 = interpolate_pixel_cords(
                ds_points_2d_arr, flydra_points_3d_arr[:,X_INDEX],
                img_width=dsc.width,
                img_height=dsc.height,
                method=FILT_METHOD)

        if 0:
            #mask out invalid parts if the convex hull doesnt work....
            #mask is usually used for *= with an image, but here we will use boolean indexing to fill
            #invalid areas with nan
            #... so fiddle the mask a little
            mask = dsc.get_virtual_display_mask('vdisp').squeeze()
            mask = ~mask

            x0[mask] = FILL_VALUE
            y0[mask] = FILL_VALUE
            u0[mask] = FILL_VALUE
            v0[mask] = FILL_VALUE


        plt.figure()
        plt.subplot(211)
        plt.imshow(arr[:,:,Y_INDEX])
        plt.colorbar()
        plt.title('%s Original Y' % dsname)
        plt.subplot(212)
        plt.imshow(y0)
        plt.colorbar()
        plt.title('Filtered %s' % FILT_METHOD)

        plt.figure()
        plt.subplot(211)
        plt.imshow(arr[:,:,X_INDEX])
        plt.colorbar()
        plt.title('%s Original X' % dsname)
        plt.subplot(212)
        plt.imshow(x0)
        plt.colorbar()
        plt.title('Filtered %s' % FILT_METHOD)

        plt.figure()
        plt.imshow(u0)
        plt.colorbar()
        plt.title('%s u' % dsname)

        plt.figure()
        plt.imshow(v0)
        plt.colorbar()
        plt.title('%s v' % dsname)

        #save the exr file, -1 means no data, not NaN
        u0[np.isnan(u0)] = -1
        v0[np.isnan(v0)] = -1
        exrpath = '/tmp/%s.exr' % dsname
        exr.save_exr(exrpath, r=u0, g=v0, b=np.zeros_like(u0))

        #save the resulting geometry to the parameter server
        geom = recon.get_geom_dict()
        dsc.set_geometry(geom)
        dsc.set_binary_exr(exrpath)

    if 0:
        s,R,T = flydra.align.estsimt(flydra_points_3d_arr.T,ds_points_3d_arr.T)

plt.show()

#rospy.spin()

