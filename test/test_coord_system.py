#!/usr/bin/env python
import numpy as np

# ROS imports
import roslib; roslib.load_manifest('freemoovr_engine')
import tf.transformations
from freemoovr_engine.coord_system import body_frame_to_camera_frame, \
     get_body_frame_to_camera_frame_mat

def test_coord_system_mat():
    pts = np.array([[ 10.0,  0,   0], # ahead
                    [  0.0,  0, 1.0], # up
                    [ 10.0,  0, 1.0], # ahead and up
                    [    0, -1,   0], # right
                    ]).T
    camera_mat = get_body_frame_to_camera_frame_mat()
    pts_camera_actual = np.dot( camera_mat, pts )

    pts_camera_expected = np.array([[0,  0, 10.0],    # ahead is in +Z
                                    [0, -1,    0],    # up is in -Y
                                    [0, -1, 10.0],
                                    [1,  0,    0],    # right is +X
                                    ]).T

    assert np.allclose( pts_camera_actual, pts_camera_expected )

def test_coord_system_quats():
    pts = np.array([[ 10.0,  0,   0], # ahead
                    [  0.0,  0, 1.0], # up
                    [ 10.0,  0, 1.0], # ahead and up
                    [    0, -1,   0], # right
                    ]).T
    body_frame_quat = tf.transformations.quaternion_about_axis(0.0, (0,0,0)) # no rotation

    camera_quat = body_frame_to_camera_frame( body_frame_quat )
    camera_matT = tf.transformations.quaternion_matrix(camera_quat)[:3,:3]
    camera_mat = camera_matT.T # (tf.transformations uses row-major matrices)
    pts_camera_actual = np.dot( camera_mat, pts )

    pts_camera_expected = np.array([[0,  0, 10.0],    # ahead is in +Z
                                    [0, -1,    0],    # up is in -Y
                                    [0, -1, 10.0],
                                    [1,  0,    0],    # right is +X
                                    ]).T

    assert np.allclose( pts_camera_actual, pts_camera_expected )
