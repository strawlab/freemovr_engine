import os.path
import pickle
import sys
import json

import roslib;
roslib.load_manifest('flyvr')
roslib.load_manifest('std_msgs')
import rospy

from std_msgs.msg import UInt32, String

if __name__ == "__main__":
    rospy.init_node('showcalibpoints', anonymous=True)

    pub_pts = rospy.Publisher('/multicamselfcal_everything/points', String)
    pub_resolution = rospy.Publisher('/multicamselfcal_everything/resolution', String)

    d = os.path.abspath(os.path.expanduser(sys.argv[1]))

    with open(os.path.join(d,'results.pkl'),'r') as f:
        results = pickle.load(f)

    with open(os.path.join(d,'resolution.pkl'),'r') as f:
        resolution = pickle.load(f)

    while not rospy.is_shutdown():
        pub_pts.publish(json.dumps(results))
        pub_resolution.publish(json.dumps(resolution))
        rospy.sleep(0.5)

