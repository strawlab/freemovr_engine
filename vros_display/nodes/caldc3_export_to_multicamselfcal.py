#!/usr/bin/env python
import pickle
import os
import numpy
import argparse
import collections
import numpy as np
import warnings

cfg_file = """[Files]
Basename: cam
Image-Extension: jpg

[Images]
Subpix: 0.5

[Calibration]
Num-Cameras: {num_cameras}
Num-Projectors: 0
Nonlinear-Parameters: 50    0    1    0    0    0
Nonlinear-Update: 1   0   1   0   0   0
Initial-Tolerance: 10
Do-Global-Iterations: 0
Global-Iteration-Threshold: 0.5
Global-Iteration-Max: 5
Num-Cameras-Fill: 2
Do-Bundle-Adjustment: 1
Undo-Radial: 0
Min-Points-Value: 30
N-Tuples: 3
Square-Pixels: 1
Use-Nth-Frame: {use_nth_frame}
"""

def save_arr( fname, arr ):
    assert arr.ndim==2
    if arr.dtype==np.bool:
        arr = arr.astype( np.uint8 )
    fd = open(fname,mode='w')
    for row in arr:
        row_buf = ' '.join( map(repr,row) ) + '\n'
        fd.write(row_buf)
    fd.close()

def emit_multicamselfcal_dir(fname,out_dirname,use_nth_frame=1,visualize=False):
    os.mkdir(out_dirname)

    fd = open(fname,mode='r')
    data = pickle.load(fd)
    fd.close()

    n_displays = 0
    n_cameras = 0
    topic_prefixes = None
    camera_order = None # a MultiCamSelfCalCamera (the displays are last in the list
    displays = []

    Res = []

    for physical_display_id in data.keys():
        for virtual_display_id in data[physical_display_id].keys():
            n_displays+=1
            loaded = data[physical_display_id][virtual_display_id]
            results = loaded['data']
            display_width_height = loaded['display_width_height']
            p2c_by_cam = loaded['p2c_by_cam']
            displays.append( (physical_display_id, virtual_display_id) )

            if topic_prefixes is None:
                topic_prefixes = p2c_by_cam.keys()
                topic_prefixes.sort()
                camera_order = topic_prefixes[:] # copy
                n_cameras = len(topic_prefixes)
                for topic_prefix in topic_prefixes:
                    Res.append(  results[topic_prefix][0]['address'].shape[::-1] )
            else:
                tmp_topic_prefixes = p2c_by_cam.keys()
                tmp_topic_prefixes.sort()
                assert topic_prefixes==tmp_topic_prefixes

            camera_order.append( physical_display_id + '/' + virtual_display_id )
            Res.append( display_width_height )
    assert len(displays)==n_displays
    assert len(topic_prefixes)==n_cameras
    assert len(camera_order)==n_cameras+n_displays
    assert len(Res)==len(camera_order)
    Res = np.array(Res)

    points = []
    IdMat_all = []

    if visualize:
        import matplotlib.pyplot as plt
        import numpy.ma as ma

    for display_enum,(physical_display_id, virtual_display_id) in enumerate(displays):
        display_row_enum = n_cameras+display_enum
        loaded = data[physical_display_id][virtual_display_id]
        results = loaded['data']
        p2c_by_cam = loaded['p2c_by_cam']
        display_width_height = loaded['display_width_height']

        by_proj = collections.defaultdict(dict)

        w,h = display_width_height
        p2c_x_all = np.zeros( (h,w, n_cameras), dtype=np.float)
        p2c_y_all = np.zeros_like(p2c_x_all)

        for i,topic_prefix in enumerate(topic_prefixes):
            p2c_x_all[:,:,i] = p2c_by_cam[topic_prefix]['x']
            p2c_y_all[:,:,i] = p2c_by_cam[topic_prefix]['y']

        # need minimum of 3 points to be useful for multicamselfcal (2 cameras + display)
        # invalid values are -1
        valid_cond_all = (p2c_x_all >= 0.0) & (p2c_y_all >= 0.0)
        n_valid = np.sum( valid_cond_all, axis=2)
        valid_cond = n_valid >= 2

        if visualize:
            plt.figure()
            plt.imshow( valid_cond )
            plt.gca().set_title('valid: '+physical_display_id+'/'+virtual_display_id)
            plt.colorbar()

            for i in range(len(topic_prefixes)):
                arr1 = p2c_x_all[:,:,i]
                arr2 = ma.masked_array( arr1, mask=~valid_cond )
                plt.figure()
                plt.imshow( arr2 )
                plt.gca().set_title(physical_display_id+'/'+virtual_display_id+' X: '+topic_prefixes[i])
                plt.colorbar()

                arr1 = p2c_y_all[:,:,i]
                arr2 = ma.masked_array( arr1, mask=~valid_cond )
                plt.figure()
                plt.imshow( arr2 )
                plt.gca().set_title(physical_display_id+'/'+virtual_display_id+' Y: '+topic_prefixes[i])
                plt.colorbar()

        (valid_y, valid_x) = np.nonzero(valid_cond)
        for proj_y, proj_x in zip(valid_y,valid_x):
            cam_xs = p2c_x_all[proj_y,proj_x]
            cam_ys = p2c_y_all[proj_y,proj_x]
            cam_valid_cond = (cam_xs >= 0.0) & (cam_ys >= 0.0)
            display_valid_cond = np.zeros( (n_displays,), dtype=np.bool)
            display_valid_cond[display_enum] = 1

            idMat_row = np.concatenate( (cam_valid_cond, display_valid_cond) )
            IdMat_all.append(idMat_row)

            row = np.ones( (len(camera_order)*3,), dtype=np.float)
            row[0:n_cameras*3:3] = cam_xs
            row[1:n_cameras*3:3] = cam_ys
            row[display_row_enum*3+0] = proj_x
            row[display_row_enum*3+1] = proj_y
            points.append(row)

    if visualize:
        plt.show()

    points = np.array(points).T
    IdMat_all = np.array(IdMat_all).T

    print 'points.shape',points.shape
    print 'IdMat_all.shape',IdMat_all.shape
    print 'Res',Res

    camera_order_fd = open( os.path.join(out_dirname,'camera_order.txt'), mode='w' )
    for c in camera_order:
        camera_order_fd.write( c + '\n' )
    camera_order_fd.close()

    save_arr( os.path.join(out_dirname,'Res.dat'), Res )
    save_arr( os.path.join(out_dirname,'IdMat.dat'), IdMat_all )
    save_arr( os.path.join(out_dirname,'points.dat'), points )

    cfg_fd = open( os.path.join(out_dirname, 'multicamselfcal.cfg'), mode='w' )
    cfg_fd.write( cfg_file.format( num_cameras=len(Res),
                                   use_nth_frame=use_nth_frame ) )
    cfg_fd.close()

    return IdMat_all.shape[1]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', type=str)
    parser.add_argument('--use_nth_frame', default=1, type=int)
    parser.add_argument('--output_dir', type=str, default='mcam')
    parser.add_argument('--visualize', default=False, action='store_true')
    args = parser.parse_args()

    n_pts = emit_multicamselfcal_dir(args.filename,args.output_dir,use_nth_frame=args.use_nth_frame,visualize=args.visualize)
    print '%d points with 3 or more views'%n_pts
