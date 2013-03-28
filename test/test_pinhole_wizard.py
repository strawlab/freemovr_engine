#!/usr/bin/env python
import numpy as np
import os
import yaml
import scipy
import tempfile

# ROS imports
import roslib; roslib.load_manifest('flyvr')
import rospkg
import flyvr.calib.pinhole.pinhole_wizard as pw

rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('flyvr')
data_fname = os.path.join( pkg_dir, 'data/calib_pinhole_sample/pinhole_wizard_sample.yaml')

def test_pinhole_wizard():
    for extrinsic_method in pw.EXTRINSIC_CALIBRATION_METHODS:
        yield check_pinhole_calibration, extrinsic_method

def check_pinhole_calibration( method ):
    ui = pw.UI()

    ui._load_from_file(data_fname)
    ui.on_compute_intrinsics()

    for vdisp in ['center','right','left']:
        ui.launch_calibration( method, vdisp )

        for row in ui.vdisp_store:
            if row[pw.VS_VDISP]==vdisp:
                row[pw.VS_SHOW_BEACHBALL]=True
                arr = ui.update_bg_image()
                row[pw.VS_SHOW_BEACHBALL]=False

                fname = 'beachball_%s_%s.png'%(vdisp,method.replace(' ','_'))
                scipy.misc.imsave(fname,arr)
                print 'saved',fname

def test_data_roundtrip():
    for to_buf in [True,False]:
        yield check_data_roundtrip, to_buf

def check_data_roundtrip(to_buf=False):
    buf1 = open(data_fname).read()
    data1 = yaml.load( buf1 )

    ui = pw.UI()

    ui._load_from_file(data_fname)
    if to_buf:
        fd = tempfile.TemporaryFile()
    else:
        fd = tempfile.mktemp()
        unlink_fd = fd
    ui._save_to_file( fd )

    if not to_buf:
        fd = open(fd)
    fd.seek(0)
    buf2 = fd.read()
    data2 = yaml.safe_load( buf2 )
    assert data1 == data2
    if not to_buf:
        os.unlink( unlink_fd )

def test_save_exr():
    ui = pw.UI()

    ui._load_from_file(data_fname)
    ui.on_compute_intrinsics()

    method = pw.EXTRINSIC_CALIBRATION_METHODS[0]

    ui.calibrate_all_vdisps(method)
    ui.save_calibration_exr('/tmp/pinhole.exr')
