#!/usr/bin/env python

import argparse

# ROS imports ############################
import roslib; roslib.load_manifest('vros_display')
import rospy

import vros_display.srv
import display_client
##########################################
import os

def show_image(fname,white,black):
    rospy.init_node('show_image')

    dsc = display_client.DisplayServerProxy()
    dsc.enter_2dblit_mode()

    if black:
        dsc.show_pixels(dsc.new_image(dsc.IMAGE_COLOR_BLACK))
    elif white:
        dsc.show_pixels(dsc.new_image(dsc.IMAGE_COLOR_WHITE))
    else:
        dsc.show_image(fname)

def main():
    wd = roslib.packages.get_pkg_dir('vros_display')
    default_fname = os.path.join(wd,'vienna-morning.jpg')

    parser = argparse.ArgumentParser()
    parser.add_argument('fname',nargs='?',default=default_fname)
    parser.add_argument('--white', action='store_true', help='show a white screen')
    parser.add_argument('--black', action='store_true', help='show a white screen')
    
    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    show_image(args.fname, args.white, args.black)

if __name__=='__main__':
    main()
