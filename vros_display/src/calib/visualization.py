import roslib
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('geometry_msgs')
from sensor_msgs.msg import PointCloud2, PointField
try:
    from sensor_msgs.point_cloud2 import create_cloud_xyz32
except ImportError:
    from . _point_cloud2 import create_cloud_xyz32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import rospy

import numpy as np

def create_point_cloud(points, frame_id):
    header = rospy.Header(frame_id=frame_id)
    assert len(points) > 0
    assert len(points[0]) == 3
    return create_cloud_xyz32(header, points)    

def create_point_cloud_message_publisher(points, topic_name, publish_now=False, latch=False, frame_id='/'):
    pc = create_point_cloud(points, frame_id)
    publisher = rospy.Publisher(topic_name, PointCloud2, latch=latch)
    if publish_now:
        publisher.publish(pc)
    else:
        return publisher, pc

def create_camera_pose_message_publisher(ce, re, names, topic_name, publish_now=False, latch=False, frame_id='/'):
    #ported from MultiSelfCamCalib drawscene.m (MATLAB) and adjusted to be more pythonic...
    ncams = ce.shape[1]
    cams = ce.T.tolist()
    rmatrices = np.vsplit(re, ncams)

    assert len(names) == ncams

    markers = []
    for i in range(ncams):
        cmat = cams[i]
        rmat = rmatrices[i]
        axis_dir = -rmat[2,:]   #3rd row or rotation matrix
        axis_len = 2
        endcoord = cmat + axis_len*axis_dir

        startpoint = Point(x=cmat[0],y=cmat[1],z=cmat[2])
        endpoint = Point(x=endcoord[0],y=endcoord[1],z=endcoord[2])

        marker = Marker()
        marker.header.frame_id = '/'
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.1; #shaft radius
        marker.scale.y = 0.2; #head radius
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.points = [startpoint, endpoint]

        markers.append(marker)

    marray = MarkerArray(markers)
    publisher = rospy.Publisher(topic_name, MarkerArray, latch=latch)

    if publish_now:
        publisher.publish(marray)
    else:
        return publisher, marray

        
