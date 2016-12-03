#!/usr/bin/env python
import argparse
import numpy as np

import flydra.reconstruct

import roslib
roslib.load_manifest('freemoovr_engine')

from calib.visualization import create_pcd_file_from_points
from calib.io import AllPointPickle
from rosutils.io import decode_url

class Exporter(object):
    def __init__(self, flydra_calib, laser_pkl, undistort_flydra_points):
        self.flydra_calib = flydra_calib
        self.fly = flydra.reconstruct.Reconstructor(cal_source=self.flydra_calib)
        self.laser_pkl = laser_pkl
        self.undistort_flydra_points = undistort_flydra_points
        self.laser = AllPointPickle()
        self.laser.initilize_from_directory(self.laser_pkl)
        pts = self.laser.get_points_in_cameras(self.fly.get_cam_ids())
        self.xyz = np.array([self.fly.find3d(pt,return_line_coords=False, undistort=self.undistort_flydra_points) for pt in pts])

    def export(self,fname):
        print 'saving to',fname
        create_pcd_file_from_points(fname,
                self.xyz)
        print 'done saving'

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--laser-pkldir', type=str, default='package://flycave/calibration/laser',
        help='path to dir containing result.pkl, resolution.pkl, etc')
    parser.add_argument(
        '--flydra-calib', type=str, default='package://flycave/calibration/flydra',
        help='path to flydra multicamselfcal result dir')
    parser.add_argument(
        '--output', type=str, default=None)
    parser.add_argument('--undistort-radial', default=False, action='store_true')
    args = parser.parse_args()

    c = Exporter(
            decode_url(args.flydra_calib),
            decode_url(args.laser_pkldir),
            args.undistort_radial)

    if args.output is None:
        args.output = decode_url('package://flycave/calibration/pcd/flydracyl.pcd')

    c.export(args.output)
