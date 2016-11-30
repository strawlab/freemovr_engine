import json

ROS_PACKAGE_NAME='freemoovr'
import roslib; roslib.load_manifest(ROS_PACKAGE_NAME)

import sensor_msgs.msg

import freemoovr.rosmsg2json as rosmsg2json

def test_arrays():
    msg = sensor_msgs.msg.CameraInfo()
    d = rosmsg2json.rosmsg2dict(msg)
    json.dumps(d)

