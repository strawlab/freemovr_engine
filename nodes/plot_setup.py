#!/usr/bin/env python

# Given camera/display correspondences (under a certain geometry), a
# camera calibration, and the description of the geometry, establish
# the "camera" calibration for the display.

# ROS imports
import roslib; roslib.load_manifest('freemoovr_engine')
import rospy
import pymvg
import freemoovr_engine.simple_geom as simple_geom

import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from plot_utils import get_3d_verts, plot_camera

def plot_setup(geometry_filename,
               display_bagfiles):
    geom = simple_geom.Geometry(geometry_filename)
    displays = [pymvg.CameraModel.load_camera_from_bagfile(dbf) for dbf in display_bagfiles]

    print geometry_filename
    print display_bagfiles
    print [d.get_name() for d in displays]

    display = displays[0]

    tcs = np.zeros( (display.height,display.width,2))-1

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    verts = get_3d_verts(geom)
    ax.plot( verts[:,0], verts[:,1], verts[:,2], 'ko' )

    for display in displays:
        plot_camera(ax, display)

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

    if 1:
        fig = plt.figure()
        n = len(displays)
        for i in range(n):
            display = displays[i]
            ax = fig.add_subplot(n,1,i+1)
            pts = display.project_3d_to_pixel(verts)
            ax.plot(pts[:,0], pts[:,1], 'ko')
            ax.set_xlim([0,display.width])
            ax.set_ylim([0,display.height])
            ax.set_title(display.name)

    if 1:
        plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('geometry_filename', type=str, help="JSON file with geometry description")
    parser.add_argument('display_bagfiles', type=str, help="filename of display-model.bag for calibration data", nargs='+')
    args = parser.parse_args()

    plot_setup(args.geometry_filename,
               args.display_bagfiles)
