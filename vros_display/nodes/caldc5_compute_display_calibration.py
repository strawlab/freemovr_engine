#!/usr/bin/env python

# Given camera/display correspondences (under a certain geometry), a
# camera calibration, and the description of the geometry, establish
# the "camera" calibration for the display.

# ROS imports
import roslib; roslib.load_manifest('vros_display')
import rospy
import camera_model
import tf
import tf.listener
import geometry_msgs
import sensor_msgs

# Standard Python stuff
import argparse
import pickle
import numpy as np
import sys

# in this ROS package
import simple_geom
import dlt

def compute_display_calibration(display_coords_filename, camera_bagfile, geometry_filename):
    camera = camera_model.load_camera_from_bagfile( camera_bagfile )
    camera_name = camera.get_name()

    print 'got camera name as "%s"'%(camera_name,)
    print 'cam center:',camera.get_camcenter()

    geom = simple_geom.Geometry(geometry_filename)

    fd = open(display_coords_filename,mode='r')
    data = pickle.load(fd)
    fd.close()

    for physical_display_id in data.keys():
        for virtual_display_id in data[physical_display_id].keys():

            loaded = data[physical_display_id][virtual_display_id]
            results = loaded['data']
            display_width_height = loaded['display_width_height']
            p2c_by_cam = loaded['p2c_by_cam']
            display_h, display_w = p2c_by_cam[camera_name]['x'].shape

            display_x_coords = results[camera_name][0]['address']
            display_y_coords = results[camera_name][0]['address']
            display_coords = np.concatenate( (np.expand_dims( display_x_coords, 2),
                                              np.expand_dims( display_y_coords, 2)),
                                             axis=2)

            cam_dims = display_x_coords.shape

            print physical_display_id, virtual_display_id, 'cam_dims',cam_dims
            geom_coords_3d = geom.compute_for_camera_view( camera )
            assert geom_coords_3d[:,:,0].shape == cam_dims

            geom_valid_cond = ~np.isnan(geom_coords_3d[:,:,0])
            print 'len(np.nonzero(geom_valid_cond)[0])',len(np.nonzero(geom_valid_cond)[0])

            display_valid_cond = display_x_coords >= 0.0
            print 'len(np.nonzero(display_valid_cond)[0])',len(np.nonzero(display_valid_cond)[0])
            joint_valid_cond = geom_valid_cond & display_valid_cond

            assert joint_valid_cond.shape ==cam_dims
            i,j = np.nonzero(joint_valid_cond)
            print '%d valid 2d/3d correspondences for projector and geometry'%(len(i),)

            X3d = geom_coords_3d[i,j]
            x2d = display_coords[i,j]

            if 0:
                idxs = range(len(i))
                import random
                random.shuffle(idxs)

                X3d = []; x2d = []
                for idx in idxs[:50]:
                    # (distorted) camera coords
                    cu,cv = i[idx], j[idx]
                    du,dv = display_coords[cu,cv]
                    Xx, Xy, Xz = geom_coords_3d[cu,cv]*100

                    print ' cam: (% 5d,% 5d)    display: (% 6.1f,% 6.1f)    3D: (% 6.1f, % 6.1f, % 6.1f)'%(
                        cu,cv,  du,dv,  Xx,Xy,Xz,
                        )
                    X3d.append( (Xx, Xy, Xz) )
                    x2d.append( (du, dv) )


            # if 1:
            #     #results = dlt.dlt( geom_coords_3d[i,j], display_coords[i,j])
            #     results = dlt.dlt( X3d, x2d, ransac=False)
            #     print physical_display_id, virtual_display_id,
            #     print 'center, mean_reprojection_error'
            #     print results['center'], results['mean_reprojection_error']
            #     print results['pmat']
            #     pmat = results['pmat']
            # else:
            #     import cal2
            #     results = cal2.cal2( X3d, x2d )

            if 1:
                results = dlt.dlt( X3d, x2d, ransac=True)
                dlt.print_summary(results)

            display_name = physical_display_id+'/'+virtual_display_id
            display_model = camera_model.load_camera_from_pmat( results['pmat'],
                                                                width=display_w,
                                                                height=display_h,
                                                                name=display_name )
            fname = display_name.replace('/','-')
            fname = 'display-model-'+fname+'.bag'
            display_model.save_to_bagfile(fname)
            print 'saved to',fname

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('display_coords_filename', type=str, help="pickle file with display/camera correspondence data")
    parser.add_argument('camera_bagfile', type=str, help="filename of camera-cambal.bag for calibration data")
    parser.add_argument('geometry_filename', type=str, help="JSON file with geometry description")
    args = parser.parse_args()

    compute_display_calibration(args.display_coords_filename,
                                args.camera_bagfile,
                                args.geometry_filename )
