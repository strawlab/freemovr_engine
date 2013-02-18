#!/usr/bin/env python
import numpy as np
import os
import yaml
import scipy

# ROS imports
import roslib; roslib.load_manifest('flyvr')
import rospkg
import flyvr.calib.pinhole_wizard as pw
import simple_geom

rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('flyvr')
data_fname = os.path.join( pkg_dir, 'data/calib_pinhole_sample/pinhole_wizard_sample.yaml')

def test_pinhole_wizard():
    buf = open(data_fname).read()
    data = yaml.load( buf )

    ui = pw.UI()

    ui._load_from_file(data_fname)
    ui.on_compute_intrinsics()

    method = 'extrinsic only'
    for vdisp in ['center','right','left']:
        ui.launch_calibration( method, vdisp )

        for row in ui.vdisp_store:
            if row[pw.VS_VDISP]==vdisp:
                row[pw.VS_SHOW_BEACHBALL]=True
                arr = ui.update_bg_image()
                row[pw.VS_SHOW_BEACHBALL]=False

                fname = 'beachball_%s.png'%vdisp
                scipy.misc.imsave(fname,arr)
                print 'saved',fname

