import roslib;
roslib.load_manifest('vros_display')
from calib.io import MultiCalSelfCam, load_ascii_matrix
import rospy

import numpy as np
import pickle

rospy.init_node('testmcsc', anonymous=True)

if 0:
    mcsc = MultiCalSelfCam('/home/stowers/Straw/MultiCamSelfCal/strawlab/test-data/DATA20100906_134124/')
    mcsc.publish_calibration_points()
    mcsc.save_to_pcd('/tmp/test.pcd')
    rospy.spin()

    idp = load_ascii_matrix(
            '/home/stowers/Straw/MultiCamSelfCal/strawlab/test-data/DATA20100906_134124/IdMat.dat')
    pts = load_ascii_matrix(
            '/home/stowers/Straw/MultiCamSelfCal/strawlab/test-data/DATA20100906_134124/points.dat')


mcsc = MultiCalSelfCam('allcalibresults')

dat = pickle.load(open('allcalibresults.pkl','r'))
res = pickle.load(open('allcalibresolution.pkl','r'))

mcsc.create_from_cams(cam_ids=dat.keys(), cam_resolutions=res, cam_points=dat)

