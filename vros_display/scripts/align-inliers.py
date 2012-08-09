#!/usr/bin/env python
import argparse, json, os

import roslib;
roslib.load_manifest('vros_display')

from calib.io import MultiCalSelfCam
from calib.reconstruct import PointCloudTransformer

def align_inliers( align_json, in_dir, out_dir ):
    recon = PointCloudTransformer.from_json( align_json )
    Xe, Ce, Re = MultiCalSelfCam.read_calibration_result(in_dir)

    if not os.path.exists(out_dir):
        os.mkdir(out_dir)
    MultiCalSelfCam.transform_and_save( Xe, Ce, Re, out_dir, recon )

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--align-json', type=str, required=True)
    parser.add_argument(
        '--in-dir', type=str, required=True)
    parser.add_argument(
        '--out-dir', type=str, required=True)
    args = parser.parse_args()

    align_inliers( args.align_json, args.in_dir, args.out_dir )
