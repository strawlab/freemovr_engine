import subprocess

import roslib
roslib.load_manifest('flyvr')

import geometry_msgs.msg
import std_msgs.msg

def build_move_node_message(name, x=0, y=0, z=0, scale=1, orientation_x=0, orientation_y=0, orientation_z=0):
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = name
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    # w is interpreted as the scale of the object in the OSG file. 0 scale
    # hides it, for example
    p.pose.orientation.w = scale
    p.pose.orientation.x = orientation_x
    p.pose.orientation.y = orientation_y
    p.pose.orientation.z = orientation_z
    return p

def parse_osg_file(name):
    """
    returns a list of named nodes and a list of animations present in the osg file
    """
    stdout = subprocess.check_output("rosrun flyvr parseosg %s" % name, shell=True)
    nodes = []
    animations = []
    for line in stdout.split('\n'):
        if line and ('=' in line):
            thing,name = line.split('=')
            if thing == 'MatrixTransformNode':
                nodes.append(name)
            elif thing == 'Animation':
                animations.append(name)
    return nodes, animations

class StimulusOSGController(object):
    pass


if __name__ == "__main__":
    import sys
    n,a = parse_osg_file(sys.argv[1])
    print "nodes:",",".join(n)
    print "animations:",",".join(a)
