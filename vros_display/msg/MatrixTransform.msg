# This represents the transform between two coordinate frames in free space.

# This takes the place of the geometry_msgs.Transform message type.

# It uses a matrix for the rotation because there are some valid
# rotation matrices that I haven't yet figured out how to same as a
# quaternion without losing information. 

# It would be good to solve this issue and use geometry_msgs.Transform
# instead of this class.

geometry_msgs/Vector3 translation
float64[9]  rotation # 3x3 row-major matrix
