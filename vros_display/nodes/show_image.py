#!/usr/bin/env python

import argparse

# ROS imports ############################
import roslib; roslib.load_manifest('vros_display')
import rospy

import numpy as np
import scipy.misc

import vros_display.srv
import display_client
##########################################
import os

def show_image(fname,white,black,viewport):
    rospy.init_node('show_image')

    dsc = display_client.DisplayServerProxy()
    dsc.enter_2dblit_mode()

    if viewport:
        mask = dsc.get_virtual_display_mask(viewport)
    else:
        mask = None

    if white:
        arr = dsc.new_image(dsc.IMAGE_COLOR_WHITE, mask)
    elif black:
        arr = dsc.new_image(dsc.IMAGE_COLOR_BLACK, mask)
    else:
        arr = scipy.misc.imread(fname)
        arr = arr[0:min(dsc.height,arr.shape[0]),0:min(dsc.width,arr.shape[1]),:]
        if mask != None:
            masks = np.dstack([mask for i in range(0,arr.shape[-1])])
            if arr.shape != masks.shape:
                arr = np.resize(arr, masks.shape)
            arr *= masks

    dsc.show_pixels(arr)

def main():
    wd = roslib.packages.get_pkg_dir('vros_display')
    default_fname = os.path.join(wd,'data','vienna-morning.jpg')

    parser = argparse.ArgumentParser()
    parser.add_argument('fname',nargs='?',default=default_fname)
    parser.add_argument('--white', action='store_true', help='show a white screen')
    parser.add_argument('--black', action='store_true', help='show a black screen')
    parser.add_argument('--viewport', type=str, help='only show on this viewport')
    
    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    show_image(args.fname, args.white, args.black, args.viewport)

if __name__=='__main__':
    main()
