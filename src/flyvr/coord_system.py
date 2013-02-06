"""Utility functions for converting coordinate systems.

The pose of the observer is given with +X forward, +Y left, and +Z
up. Our camera model (and that of ROS and OpenCV) has +Z forward, -Y
up, and +X right.

"""

import tf.transformations
import numpy as np

def get_body_frame_to_camera_frame_mat():
    rmat = np.array([[ 0,-1, 0],
                     [ 0, 0,-1],
                     [ 1, 0, 0]] )
    return rmat

def get_body_frame_to_camera_frame_quat():
    rmat = get_body_frame_to_camera_frame_mat()
    # Convert to 4x4 matrix. (tf.transformations works with 4x4 matrices)
    R = np.zeros((4,4))
    R[3,3]=1
    R[:3,:3]=rmat

    # Now make quaternion
    # (tf.transformations uses row-major matrices)
    q = tf.transformations.quaternion_from_matrix(R.T)
    return q

BF2CF = get_body_frame_to_camera_frame_quat()

def body_frame_to_camera_frame(in_rotation):
    """Convert body frame to camera frame.

    In our body frame coordinate system, observer looks at +X with +Y
    left and +Z up. In the camera frame, the look direction is +Z
    with +X right and +Y down.
    """
    out_rotation = tf.transformations.quaternion_multiply( in_rotation, BF2CF)
    return out_rotation
