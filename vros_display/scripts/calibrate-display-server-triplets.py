import yaml
import argparse
import os.path

import roslib;
roslib.load_manifest('vros_display')
roslib.load_manifest('motmot_ros_utils')
import rospy

from calib.io import MultiCalSelfCam, AllPointPickle
from rosutils.io import decode_url

#FIXME: should use the single pickle file from the collect points part...
DS_PKL = decode_url('package://flycave/calibration/triplets/')

def calibrate(rerun_mcsc,calib_config,display_server_numbers):
    config = yaml.load(open(decode_url(calib_config)))

    for n in [int(i) for i in display_server_numbers]:
        src = DS_PKL + "/ds%d" % n
        ds = '/display_server%d' % n
        ids = [ds] + config['display_servers'][ds]

        a = AllPointPickle()
        a.initilize_from_directory(src)

        print "*"*20
        print src
        print "*"*20

        if rerun_mcsc:
            mcsc =  MultiCalSelfCam(src)
            mcsc.create_from_cams(
                            cam_ids=ids,
                            cam_resolutions=a.resolutions.copy(),
                            cam_points=a.results.copy(),
                            cam_calibrations={},
                            num_cameras_fill=0)
            dest = mcsc.execute(blocking=True, copy_files=True, silent=False)
        else:
            dest = src + "/result"

        MultiCalSelfCam.publish_calibration_points(
            dest,
            topic_base='/ds%d' % n)

if __name__ == "__main__":
    rospy.init_node('calibrate_display_server_triplets', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--rerun-mcsc', default=False, action='store_true')
    parser.add_argument(
        '--calib-config', type=str, default='package://flycave/conf/calib-all.yaml',
        help='path to calibration configuration yaml file')
    parser.add_argument(
        '--display-server-numbers', type=str, default='0,1,3',
        help='comma separated list of display server numbers')
    args = parser.parse_args()

    calibrate(
        args.rerun_mcsc,
        args.calib_config,
        args.display_server_numbers.split(','))

    rospy.spin()

