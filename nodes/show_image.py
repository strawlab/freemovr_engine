#!/usr/bin/env python

import roslib; roslib.load_manifest('flyvr')
import rospy

import os
import argparse

import numpy as np
import scipy.misc

import flyvr.srv
import display_client

def show_image(ds,viewport,fname,white,black,rgb,pixel, ptsize, scale=False):
    rospy.init_node('show_image')

    dsc = display_client.DisplayServerProxy(ds,wait=True)
    dsc.enter_2dblit_mode()

    if viewport:
        mask = dsc.get_virtual_display_mask(viewport)
    else:
        mask = None

    if rgb != (-1,-1,-1):
        arr = dsc.new_image(rgb, mask)
    elif white:
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

    if pixel and (white or black or (rgb != (-1,-1,-1))):
        col,row = map(int,pixel.split(','))

        if white:
            arr = dsc.new_image(dsc.IMAGE_COLOR_BLACK, mask)
            rgb = (dsc.IMAGE_COLOR_WHITE, dsc.IMAGE_COLOR_WHITE, dsc.IMAGE_COLOR_WHITE)
        elif black:
            arr = dsc.new_image(dsc.IMAGE_COLOR_WHITE, mask)
            rgb = (dsc.IMAGE_COLOR_BLACK, dsc.IMAGE_COLOR_BLACK, dsc.IMAGE_COLOR_BLACK)
        else:
            arr = dsc.new_image(dsc.IMAGE_COLOR_BLACK, mask)

        for i,c in enumerate(rgb):
            arr[row-ptsize:row+ptsize,col-ptsize:col+ptsize,i] = c

    if scale:
        assert arr.ndim==3
        orig_aspect = arr.shape[1]/float(arr.shape[0]) # w/h
        native_aspect = dsc.width/float(dsc.height)
        if native_aspect >= orig_aspect:
            # display is wider than image
            new_shape_height_h = int(dsc.width/float(orig_aspect))
            new_shape_full = new_shape_height_h, dsc.width

        else:
            # display is taller than image

            new_shape_wide_w = int(orig_aspect*dsc.height)
            new_shape_full = dsc.height, new_shape_wide_w
        new_image = scipy.misc.imresize( arr, new_shape_full )
        arr = new_image[:dsc.height, :dsc.width]
    dsc.show_pixels(arr)

def main():
    wd = roslib.packages.get_pkg_dir('flyvr')
    default_fname = os.path.join(wd,'data','vienna-morning.jpg')

    parser = argparse.ArgumentParser()
    parser.add_argument('fname',nargs='?',default=default_fname)
    parser.add_argument('--rgb', help='RGB value r,g,b', default="-1,-1,-1")    
    parser.add_argument('--white', action='store_true', help='show a white screen')
    parser.add_argument('--black', action='store_true', help='show a black screen')
    parser.add_argument('--viewport', type=str, help='only show on this viewport')
    parser.add_argument(
        '--display-server', type=str, metavar='/display_server', required=True, help=\
        'the path of the display server to configure')
    parser.add_argument('--pixel', type=str, help='light this pixel', metavar='x,y')
    parser.add_argument('--scale', action='store_true', help='scale the image to fullscreen')
    parser.add_argument('--pxsize', type=int, default=2)
    
    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    show_image(args.display_server, args.viewport, args.fname,
            args.white,
            args.black,
            tuple(map(int,args.rgb.split(','))),
            args.pixel,
            args.pxsize,
            scale = args.scale,   
    )

if __name__=='__main__':
    main()
