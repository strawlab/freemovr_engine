#!/usr/bin/env python

import os.path
import contextlib
import argparse

import numpy as np

import roslib;
roslib.load_manifest('flyvr')
roslib.load_manifest('motmot_ros_utils')
roslib.load_manifest('rosgobject')

import rospy
import rosgobject
import calib.visualization
import rosutils.io

import flydra.a2.core_analysis as core_analysis

import pcl

from gi.repository import Gtk, GObject, Gdk

GObject.threads_init()
Gdk.threads_init()

class Tune(object):
    def __init__(self, h5):

        self._ca = core_analysis.get_global_CachingAnalyzer()
        obj_ids, use_obj_ids, is_mat_file, data_file, extra = self._ca.initial_file_load(h5)

        x = []
        y = []
        z = []

        for obj_id_enum,obj_id in enumerate(use_obj_ids):
            rows = self._ca.load_data(obj_id,
                                      data_file,
                                      use_kalman_smoothing=False)
            verts = np.array( [rows['x'], rows['y'], rows['z']] ).T
            x.append( verts[:,0] )
            y.append( verts[:,1] )
            z.append( verts[:,2] )

        x = np.concatenate(x)
        y = np.concatenate(y)
        z = np.concatenate(z)

        coords = np.array([x,y,z]).T

        self.pubcyl = None
        self.pubpt = None
        self.p = pcl.PointCloud()
        self.p.from_array(coords)
        self.p.to_file('/tmp/fuckit.pcd')
        self.np = len(coords)

        self._cx,self._cy,self._cz,self._ax,self._ay,self._az,self._radius = (0,0,0,0,0,0,0)

        self.pubpts, pts = calib.visualization.create_point_cloud_message_publisher(
                                coords,
                                topic_name='/flydracalib/points',
                                publish_now=True,
                                latch=True)
        self.pubptsinliers = calib.visualization.create_point_cloud_message_publisher(
                                [(0,0,0)],
                                topic_name='/flydracalib/pointsinliers',
                                publish_now=False,
                                latch=True)

        self.pubcyl = calib.visualization.create_cylinder_publisher(
                            self._cx,self._cy,self._cz,self._ax,self._ay,self._az,self._radius,
                            topic_name='/flydracalib/cyl',
                            publish_now=False,
                            latch=True,
                            length=5,
                            color=(0,1,1,0.4))
        self.pubpt = calib.visualization.create_point_publisher(
                            self._cx,self._cy,self._cz,
                            r=0.1,
                            topic_name='/flydracalib/cylcenter',
                            publish_now=False,
                            latch=True,
                            color=(1,0,0,0.5))

        thisdir = os.path.dirname(os.path.abspath(__file__))
        self.ui = Gtk.Builder()
        self.ui.add_from_file(os.path.join(thisdir,"tune.ui"))

        self._ks = -1.0
        self.ui.get_object("ks_adjustment").props.value = self._ks
        self._sr = 0.5
        self.ui.get_object("sr_adjustment").props.value = self._sr
        self._nd = 0.5
        self.ui.get_object("nd_adjustment").props.value = self._nd
        self._dt = 0.10
        self.ui.get_object("dt_adjustment").props.value = self._dt
        self._minr = 0.5
        self.ui.get_object("minr_adjustment").props.value = self._minr
        self._maxr = 1.5
        self.ui.get_object("maxr_adjustment").props.value = self._maxr

        self._exbtn = self.ui.get_object("execute_btn")
        self._exbtn.connect("clicked", self._calculate)

        self._axadj = self.ui.get_object("ax_adjustment")
        self._ayadj = self.ui.get_object("ay_adjustment")
        self._azadj = self.ui.get_object("az_adjustment")
        self._cxadj = self.ui.get_object("cx_adjustment")
        self._cyadj = self.ui.get_object("cy_adjustment")
        self._czadj = self.ui.get_object("cz_adjustment")
        self._radj = self.ui.get_object("r_adjustment")
        self._in = self.ui.get_object("inliersentry")

        self.ui.connect_signals(self)
        w = self.ui.get_object("window1")
        w.connect("delete-event", rosgobject.main_quit)
        w.show_all()

    def on_maxr_changed(self, adj):
        self._maxr = adj.props.value

    def on_minr_changed(self, adj):
        self._minr = adj.props.value

    def on_sr_changed(self, adj):
        self._sr = adj.props.value

    def on_ks_changed(self, adj):
        self._ks = adj.props.value

    def on_nd_changed(self, adj):
        self._nd = adj.props.value

    def on_dt_changed(self, adj):
        self._dt = adj.props.value

    def on_ax_changed(self, adj):
        if adj.get_data("update"):
            self._ax = adj.props.value
            self._update_cyl()

    def on_ay_changed(self, adj):
        if adj.get_data("update"):
            self._ay = adj.props.value
            self._update_cyl()

    def on_az_changed(self, adj):
        if adj.get_data("update"):
            self._az = adj.props.value
            self._update_cyl()

    def on_cx_changed(self, adj):
        if adj.get_data("update"):
            self._cx = adj.props.value
            self._update_cyl()

    def on_cy_changed(self, adj):
        if adj.get_data("update"):
            self._cy = adj.props.value
            self._update_cyl()

    def on_cz_changed(self, adj):
        if adj.get_data("update"):
            self._cz = adj.props.value
            self._update_cyl()

    def on_r_changed(self, adj):
        if adj.get_data("update"):
            self._radius = adj.props.value
            self._update_cyl()

    def _calculate(self, *args):
        self._exbtn.set_sensitive(False)

        rl = self._minr, self._maxr

        seg = self.p.make_segmenter_normals(
                        searchRadius=self._sr,
                        ksearch=int(self._ks))
        print "SR",self._sr
        print "KS",self._ks

        seg.set_optimize_coefficients (True);
        seg.set_model_type (pcl.SACMODEL_CYLINDER)
        seg.set_method_type (pcl.SAC_RANSAC)

        seg.set_normal_distance_weight (self._nd)
        print "ND",self._nd

        seg.set_max_iterations (1000)
 
        seg.set_distance_threshold (self._dt)
        print "DT",self._dt

        seg.set_radius_limits (*rl)
        print "Radius Limits",rl

        #seg.set_axis(0.41,1.894,2.733)
        #seg.set_eps_angle(0.1)

        try:
            ind, model = seg.segment()
            nind = len(ind)
            self._in.set_text(
                        "%d (%.1f%%)" % (int(nind), 100*(float(nind)/self.np)))

            #publish the inlier set
            cloud = self.p.extract(ind, negative=False)
            pts = calib.visualization.create_point_cloud(
                                    cloud.to_list())
            self.pubptsinliers.publish(pts)

            self.cx,self.cy,self.cz,self.ax,self.ay,self.az,self.radius = model 
        finally:
            self._exbtn.set_sensitive(True)

        self._update_cyl()

    def _update_cyl(self):
        print "update cyl"
        cyl = calib.visualization.create_cylinder(
                self._cx,self._cy,self._cz,self._ax,self._ay,self._az,self._radius,
                length=5,
                color=(0,1,1,0.4))
        pt = calib.visualization.create_point(
                self._cx,self._cy,self._cz,
                r=0.1,
                color=(1,0,0,0.5))
        self.pubcyl.publish(cyl)
        self.pubpt.publish(pt)

    @contextlib.contextmanager
    def _freezer(self, adj):
        adj.set_data("update",False)
        yield adj
        adj.set_data("update",True)

    def set_ax(self,value):
        with self._freezer(self._axadj) as adj:
            adj.set_value(value)
        self._ax = value
    ax = property(fset=set_ax)

    def set_ay(self,value):
        with self._freezer(self._ayadj) as adj:
            adj.set_value(value)
        self._ay = value
    ay = property(fset=set_ay)

    def set_az(self,value):
        with self._freezer(self._azadj) as adj:
            adj.set_value(value)
        self._az = value
    az = property(fset=set_az)

    def set_cx(self,value):
        with self._freezer(self._cxadj) as adj:
            adj.set_value(value)
        self._cx = value
    cx = property(fset=set_cx)

    def set_cy(self,value):
        with self._freezer(self._cyadj) as adj:
            adj.set_value(value)
        self._cy = value
    cy = property(fset=set_cy)

    def set_cz(self,value):
        with self._freezer(self._czadj) as adj:
            adj.set_value(value)
        self._cz = value
    cz = property(fset=set_cz)

    def set_radius(self,value):
        with self._freezer(self._radj) as adj:
            adj.set_value(value)
        self._radius = value
    radius = property(fset=set_radius)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--h5',help='name of .pcd file', required=True)
    parser.add_argument('--stim-xml')

    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    rospy.init_node("cylfittune", anonymous=True, disable_signals=True )
    rosgobject.get_ros_thread()
    rosgobject.add_console_logger()

    t = Tune(args.h5)

    Gtk.main()

