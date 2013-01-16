#!/usr/bin/env python

# Just grab our ROS config and stuff it into a json file, then call
# the real program (which does not depend on ROS).

# ROS imports
import roslib; roslib.load_manifest('vros_display')
import rospy
import sys, subprocess
import json

def get_physical_display_dict(display_num=0):
    # This will have to be updated to support multiple physical
    # displays.
    display_ids = rospy.get_param('/physical_displays',{}).keys()
    display_ids.sort()
    for i, display_id in enumerate(display_ids):
        rospy.loginfo( 'available display %d: %s'%(i,display_id) )
    if len(display_ids):
        display_id = display_ids[display_num]
        rospy.loginfo( 'choosing display %d: %s'%(display_num,display_id))
        name = '/physical_displays/'+display_id
        physical_display_dict = rospy.get_param(name)
    else:
        rospy.logwarn( 'no display parameters found' )
        physical_display_dict = {}
    return physical_display_dict

if __name__=='__main__':
    physical_display_dict = get_physical_display_dict(display_num=0)
    json_config = json.dumps(physical_display_dict)
    fname = '/tmp/caldc4.json'
    fd = open(fname,mode='w')
    fd.write( json_config )
    fd.close()
    sys.exit(subprocess.call("rosrun vros_display caldc4_manual_camera_calibration --cfg "+fname, shell=True))
