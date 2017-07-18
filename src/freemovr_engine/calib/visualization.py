import math

import roslib
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('std_msgs')
roslib.load_manifest('tf')
from sensor_msgs.msg import PointCloud2, PointField
try:
    from sensor_msgs.point_cloud2 import create_cloud_xyz32
except ImportError:
    from . _point_cloud2 import create_cloud_xyz32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import ColorRGBA
import rospy
import tf

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d

import freemovr_engine.simple_geom as simple_geom

def _points_check(points, ensure_ndarray=False):
    if len(points):
        if len(points[0]) == 3:
            if ensure_ndarray and type(points) != np.ndarray:
                return np.array(points)
            return points
    raise ValueError("Points must be a list of (x,y,z) tuples")

def create_point_cloud(points, frame_id='/'):
    _points_check(points)
    header = rospy.Header(frame_id=frame_id)
    return create_cloud_xyz32(header, points)

def create_point_cloud_message_publisher(points, topic_name=None, publish_now=False, latch=False, frame_id='/'):
    publisher = rospy.Publisher(topic_name, PointCloud2, latch=latch)
    if publish_now:
        pc = create_point_cloud(points, frame_id)
        publisher.publish(pc)
        return publisher, pc
    else:
        return publisher

def create_camera_pose_message_publisher(ce, re, names, topic_name=None, publish_now=False, latch=False, frame_id='/'):
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
        marker.header.frame_id = frame_id
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

def create_cylinder(cx,cy,cz,ax,ay,az,radius, frame_id='/', obj_id=0, length=1, color=(0,1,0,0.3)):

    #The cylinder's axis if unrotated is originally pointing along the z axis of the referential
    #so that will be the up direction (0, 0 1).
    #
    #The cross product of the desired cylinder axis (normalized) and the up vector will give us
    #the axis of rotation, perpendicular to the plane defined by the cylinder axis and the up vector.
    #
    #The dot product of the cylinder axis and the up vector will provide the angle of rotation.

    axis = (ax, ay, az)
    rotation_axis = np.cross((0, 0, 1), axis)
    rotation_angle = simple_geom.angle_between_vectors(axis, (0, 0, 1))

    cyl = Marker()
    cyl.id = obj_id
    cyl.header.frame_id = frame_id
    cyl.type = Marker.CYLINDER
    cyl.action = Marker.ADD
    cyl.pose.position = Point(x=cx,y=cy,z=cz)
    cyl.pose.orientation = Quaternion(*tf.transformations.quaternion_about_axis(rotation_angle, axis))
    cyl.scale.x = 2*radius
    cyl.scale.y = 2*radius
    cyl.scale.z = length

    cyl.color = ColorRGBA(*color)

    return cyl

def create_cylinder_publisher(cx,cy,cz,ax,ay,az,radius,topic_name=None, publish_now=False, latch=False, frame_id='/', obj_id=0, length=1, color=(0,1,0,0.3)):
    publisher = rospy.Publisher(topic_name, Marker, latch=latch)
    if publish_now:
        cyl = create_cylinder(cx,cy,cz,ax,ay,az,radius, frame_id, obj_id, length, color)
        publisher.publish(cyl)
        return publisher, cyl
    else:
        return publisher

def create_point(cx,cy,cz,r,frame_id='/',obj_id=0, color=(1,0,0,0.8)):

    pt = Marker()
    pt.id = obj_id
    pt.header.frame_id = frame_id
    pt.type = Marker.SPHERE
    pt.action = Marker.ADD
    pt.pose.position = Point(x=cx,y=cy,z=cz)
    pt.scale.x = r
    pt.scale.y = r
    pt.scale.z = r
    pt.color = ColorRGBA(*color)

    return pt

def create_point_publisher(cx,cy,cz,r,topic_name=None, publish_now=False, latch=False, frame_id='/', obj_id=0, color=(1,0,0,0.8)):
    publisher = rospy.Publisher(topic_name, Marker, latch=latch)
    if publish_now:
        pt = create_point(cx,cy,cz,r,frame_id,obj_id,color)
        publisher.publish(pt)
        return publisher, pt
    else:
        return publisher

def create_pcd_file_from_points(fname, points, npts=None):
    HEADER = \
    "# .PCD v.7 - Point Cloud Data file format\n"\
    "VERSION .7\n"\
    "FIELDS x y z\n"\
    "SIZE 4 4 4\n"\
    "TYPE F F F\n"\
    "COUNT 1 1 1\n"\
    "WIDTH %(npoints)d\n"\
    "HEIGHT 1\n"\
    "VIEWPOINT 0 0 0 1 0 0 0\n"\
    "POINTS %(npoints)d\n"\
    "DATA ascii\n"

    _points_check(points)

    with open(fname, 'w') as fd:
        fd.write(HEADER % {"npoints":len(points)})
        for pt in points:
            fd.write("%f %f %f\n" % tuple(pt))

def show_pointcloud_3d_plot(points, ax=None):
    points = _points_check(points, ensure_ndarray=True)
    if not ax:
        ax = plt.gca(projection='3d')
    ax.plot(points[:,0],points[:,1],points[:,2],'o')

def show_pointcloud_2d_plots(points, fig=None):
    points = _points_check(points, ensure_ndarray=True)
    x = points[:,0]
    y = points[:,1]
    z = points[:,2]

    ax1 = fig.add_subplot(2,1,1)
    ax1.plot( x,z,'b.')
    ax1.set_ylabel('z')
    ax1.set_aspect('equal')

    ax2 = fig.add_subplot(2,1,2,sharex=ax1)
    ax2.plot( x,y,'b.')
    ax2.set_ylabel('y')
    ax2.set_xlabel('x')
    ax2.set_aspect('equal')
