import subprocess

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('std_msgs')
roslib.load_manifest('geometry_msgs')
import rospy
import std_msgs.msg
import geometry_msgs.msg

roslib.load_manifest('freemovr_engine')
import freemovr_engine.display_client as display_client


def build_move_node_message(name, x=0, y=0, z=0, scale=1, hidden=False, orientation_x=0, orientation_y=0, orientation_z=0):
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = name
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    # w is interpreted as the scale of the object in the OSG file. 0 scale
    # hides it, for example
    p.pose.orientation.w = 0 if hidden else scale
    p.pose.orientation.x = orientation_x
    p.pose.orientation.y = orientation_y
    p.pose.orientation.z = orientation_z
    return p

def parse_osg_file(name):
    """
    returns a list of named nodes and a list of animations present in the osg file
    """
    stdout = subprocess.check_output("rosrun freemovr_engine parseosg %s" % name, shell=True)
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

    def __init__(self):
        self.pub_filename = rospy.Publisher('osg_filename', std_msgs.msg.String, latch=True)
        self.pub_submodel = rospy.Publisher('osg_submodel_pose', geometry_msgs.msg.PoseStamped)
        self.pub_anim_start = rospy.Publisher('osg_animation_start', std_msgs.msg.String, latch=True)
        self.pub_anim_stop = rospy.Publisher('osg_animation_stop', std_msgs.msg.String, latch=True)

        display_client.DisplayServerProxy.set_stimulus_mode('StimulusOSG')

    @staticmethod
    def get_animations(path):
        return parse_osg_file(path)[1]

    @staticmethod
    def get_nodes(path):
        return parse_osg_file(path)[0]

    def load_osg(self, path):
        self.pub_filename.publish(path)

    def animation_start(self, name):
        self.pub_anim_start.publish(name)

    def animation_stop(self, name):
        self.pub_anim_stop.publish(name)

    def move_node(self, *args, **kwargs):
        msg = build_move_node_message(*args, **kwargs)
        self.pub_submodel.publish(msg)


class _OSG2FileHandle(object):

    def __init__(self, proxy, filename):
        self._p = proxy
        self._f = filename

    def move(self, *args, **kwargs):
        msg = build_move_node_message(self._f, *args, **kwargs)
        self._p._pub_submodel.publish(msg)

    def hide(self):
        msg = build_move_node_message(self._f, hidden=True)
        self._p._pub_submodel.publish(msg)

    def fade_in(self):
        self._p._pub_anim_fade_in.publish(self._f)

    def animation_start(self, name):
        self._p._pub_anim_start.publish("%s|%s" % (self._f, name))

    def animation_stop(self, name):
        self._p._pub_anim_stop.publish("%s|%s" % (self._f, name))

    def unload(self):
        self._p.load_osg(self, self._f + '.unload')


class StimulusOSG2Controller(object):

    def __init__(self):
        self._pub_filename = rospy.Publisher('osg2_filename', std_msgs.msg.String, latch=True)
        self._pub_anim_duration = rospy.Publisher('osg2_animation_duration', std_msgs.msg.Float32, latch=True)

        self._pub_anim_fade_in = rospy.Publisher('osg2_submodel_fade_in', std_msgs.msg.String)
        self._pub_submodel = rospy.Publisher('osg2_submodel_pose', geometry_msgs.msg.PoseStamped)
        self._pub_anim_start = rospy.Publisher('osg2_animation_start', std_msgs.msg.String, latch=True)
        self._pub_anim_stop = rospy.Publisher('osg2_animation_stop', std_msgs.msg.String, latch=True)

        display_client.DisplayServerProxy.set_stimulus_mode('StimulusOSG2')

    @staticmethod
    def get_animations(path):
        return parse_osg_file(path)[1]

    @staticmethod
    def get_nodes(path):
        return parse_osg_file(path)[0]

    def set_animation_duration(self, v):
        self._pub_anim_duration.publish(float(v))

    def unload_all(self):
        self._pub_filename.publish('/dev/null')

    def load_osg(self, path):
        self._pub_filename.publish(path)
        return _OSG2FileHandle(self, path)


if __name__ == "__main__":
    import sys
    n,a = parse_osg_file(sys.argv[1])
    print("nodes:",",".join(n))
    print("animations:",",".join(a))

