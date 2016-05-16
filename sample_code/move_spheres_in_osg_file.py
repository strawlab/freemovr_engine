import roslib
roslib.load_manifest('freemovr_engine')

import rospy
import geometry_msgs.msg
import std_msgs.msg
import freemovr_engine.display_client as display_client
import freemovr_engine.osg_utils as osg_utils

from math import sin, cos

rospy.init_node('moveobjects')

pub = rospy.Publisher('osg_submodel_pose', geometry_msgs.msg.PoseStamped)
pub_color = rospy.Publisher('osg_background_color', std_msgs.msg.ColorRGBA)
pub_filename = rospy.Publisher('osg_filename', std_msgs.msg.String)

rospy.sleep(1)
pub_color.publish(0,0,0,1) #black
pub_color.publish(1,1,1,1) #white
pub_filename.publish('spheres.osg')

rospy.sleep(1)

display_client.DisplayServerProxy.set_stimulus_mode('StimulusOSG')

i = 0
while not rospy.is_shutdown():
    i += 1

    p = osg_utils.build_move_node_message('s1')
    p.pose.position.x = sin(i*0.01)
    pub.publish(p)

    print p.pose.position

    p = osg_utils.build_move_node_message('s2')
    p.pose.position.y = cos(i*0.02)
    pub.publish(p)


    rospy.sleep(1/100.0)


