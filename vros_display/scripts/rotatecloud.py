import os.path
import contextlib

import roslib;
roslib.load_manifest('vros_display')
roslib.load_manifest('motmot_ros_utils')
roslib.load_manifest('tf')
import rospy
import tf
import rosutils.io

import numpy as np

import calib.visualization
import simple_geom

import pcl

rospy.init_node("rotatecloud", anonymous=True, disable_signals=True )

pcd_file = rosutils.io.decode_url('package://flycave/calibration/pcd/flydracyl.smooth.pcd')

p = pcl.PointCloud()
p.from_file(pcd_file)

cx,cy,cz,ax,ay,az,radius = [0.331529438495636, 0.5152832865715027, 0.2756080627441406, -0.7905913591384888, 0.24592067301273346, -7.272743225097656, 2.241607666015625]

calib.visualization.create_point_cloud_message_publisher(
                            p.to_list(),
                            topic_name='/flydracalib/points',
                            publish_now=True,
                            latch=True)

calib.visualization.create_cylinder_publisher(
                        cx,cy,cz,ax,ay,az,radius,
                        topic_name='/flydracalib/cyl',
                        publish_now=True,
                        latch=True,
                        length=5,
                        color=(0,1,0,0.2))

#we need to create a rotation matrix to apply to the 3d points to align them on z axis
axis = (ax, ay, az)
rotation_axis = np.cross(axis, (0, 0, 1))
rotation_angle = simple_geom.angle_between_vectors((0, 0, 1), axis)

print rotation_angle
print rotation_axis

rotation_quaternion = tf.transformations.quaternion_about_axis(rotation_angle, axis)

Rt = tf.transformations.rotation_matrix(rotation_angle, rotation_axis)
print Rt
R = Rt[0:3,0:3]
print R

arr = np.array(p.to_list())

print arr.shape

new = np.dot(R,arr.T).T

#move this to the origin
cx,cy,_ = np.mean(new,axis=0)
_,_,zmin = np.min(new,axis=0)
_,_,zmax = np.min(new,axis=0)
new -= np.array([cx,cy,zmin])

p2 = pcl.PointCloud()
p2.from_array(new.astype(np.float32))

calib.visualization.create_point_cloud_message_publisher(
                            p2.to_list(),
                            topic_name='/flydracalib/points2',
                            publish_now=True,
                            latch=True)

rospy.spin()

