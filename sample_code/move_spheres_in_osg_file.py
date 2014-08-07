import roslib
roslib.load_manifest('flyvr')

import rospy
import geometry_msgs.msg
import std_msgs.msg
import flyvr.display_client as display_client

from math import sin, cos

rospy.init_node('moveobjects')

pub = rospy.Publisher('osg_submodel_pose', geometry_msgs.msg.PoseStamped)
pub_color = rospy.Publisher('osg_background_color', std_msgs.msg.ColorRGBA)
pub_filename = rospy.Publisher('osg_filename', std_msgs.msg.String)

rospy.sleep(1)
pub_color.publish(0,0,0,1) #black
pub_color.publish(1,1,1,1) #white
pub_filename.publish('spheres.osg')

def get_msg(frame):
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = frame
    p.pose.position.z = 0
    #w is interpreted as the scale of the object in the OSG file. 0 scale
    #hides it, for example
    p.pose.orientation.w = 1
    return p

rospy.sleep(1)

display_client.DisplayServerProxy.set_stimulus_mode('StimulusOSG')

i = 0
while not rospy.is_shutdown():
    i += 1

    p = get_msg('s1')
    p.pose.position.x = sin(i*0.01)
    pub.publish(p)

    print p.pose.position

    p = get_msg('s2')
    p.pose.position.y = cos(i*0.02)
    pub.publish(p)


    rospy.sleep(1/100.0)


