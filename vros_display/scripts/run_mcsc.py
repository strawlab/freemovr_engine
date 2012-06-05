import roslib;
roslib.load_manifest('vros_display')
from calib.io import MultiCalSelfCam
import rospy

import os.path
import sys

rospy.init_node('testmcsc', anonymous=True)

mcsc = MultiCalSelfCam(os.path.abspath(os.path.expanduser(sys.argv[1])))
dest = mcsc.execute(blocking=True)
print dest
pcd = dest+'/points.pcd'
mcsc.save_to_pcd(pcd, result_dir=dest)

