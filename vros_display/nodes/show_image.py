#!/usr/bin/env python

import argparse

# ROS imports ############################
import roslib; roslib.load_manifest('vros_display')
import rospy

import vros_display.srv
import display_client
##########################################
import os

def show_image(fname=None):
    rospy.init_node('show_image')

    display_server = display_client.DisplayServerProxy()
    display_server.enter_standby_mode()
    display_server.set_mode('Stimulus2DBlit')

    # load image file
    image = vros_display.msg.VROSCompressedImage()
    image.format = os.path.splitext(fname)[-1]
    image.data = open(fname).read()


    # send image to server
    blit_compressed_image_proxy = rospy.ServiceProxy(display_server.get_fullname('blit_compressed_image'),
                                                     vros_display.srv.BlitCompressedImage)
    blit_compressed_image_proxy(image)

def main():
    wd = roslib.packages.get_pkg_dir('vros_display')
    default_fname = os.path.join(wd,'vienna-morning.jpg')

    parser = argparse.ArgumentParser()
    parser.add_argument('fname',nargs='?',default=default_fname)
    # use argparse, but only after ROS did its thing
    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    show_image(fname = args.fname)

if __name__=='__main__':
    main()
