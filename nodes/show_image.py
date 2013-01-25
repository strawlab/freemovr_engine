#!/usr/bin/env python

import roslib; roslib.load_manifest('flyvr')
import rospy

import os
import argparse

import numpy as np
import scipy.misc

import flyvr.srv
import display_client

def show_image(ds,viewport,fname,white,black,pixel):
    rospy.init_node('show_image')

    dsc = display_client.DisplayServerProxy(ds)
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
        if arr.shape!=(dsc.height,dsc.width):
            arr = arr[0:min(dsc.height,arr.shape[0]),0:min(dsc.width,arr.shape[1]),:]
        if mask != None:
            masks = np.dstack([mask for i in range(0,arr.shape[-1])])
            if arr.shape != masks.shape:
                arr = np.resize(arr, masks.shape)
            arr *= masks

    if pixel and (white or black):
        ptsize = 2
        col,row = map(int,pixel.split(','))
        arr[row-ptsize:row+ptsize,col-ptsize:col+ptsize,:3] =\
            dsc.IMAGE_COLOR_BLACK if white else dsc.IMAGE_COLOR_WHITE
        print col

    dsc.show_pixels(arr)

def main():
    wd = roslib.packages.get_pkg_dir('flyvr')
    default_fname = os.path.join(wd,'data','vienna-morning.jpg')

    parser = argparse.ArgumentParser()
    parser.add_argument('fname',nargs='?',default=default_fname)
    parser.add_argument('--white', action='store_true', help='show a white screen')
    parser.add_argument('--black', action='store_true', help='show a black screen')
    parser.add_argument('--viewport', type=str, help='only show on this viewport')
    parser.add_argument(
        '--display-server', type=str, metavar='/display_server', required=True, help=\
        'the path of the display server to configure')
    parser.add_argument('--pixel', type=str, help='light this pixel', metavar='x,y')
    
    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    show_image(args.display_server, args.viewport, args.fname, args.white, args.black, args.pixel)

if __name__=='__main__':
    main()
