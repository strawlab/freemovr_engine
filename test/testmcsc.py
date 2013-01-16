import roslib;
roslib.load_manifest('flyvr')
from calib.io import MultiCalSelfCam, load_ascii_matrix
import rospy

import sys
import pickle

import numpy as np

d = '/home/stowers/vros-devel/vros/mcamall/'

rospy.init_node('testmcsc', anonymous=True)

mcsc = MultiCalSelfCam(d)
mcsc.publish_calibration_points()
mcsc.save_to_pcd('/tmp/test.pcd')
rospy.spin()

idp = load_ascii_matrix(d+'/IdMat.dat')
pts = load_ascii_matrix(d+'/points.dat')


mcsc = MultiCalSelfCam('allcalibresults')

dat = pickle.load(open('allcalibresults.pkl','r'))
res = pickle.load(open('allcalibresolution.pkl','r'))

mcsc.create_from_cams(cam_ids=dat.keys(), cam_resolutions=res, cam_points=dat)

