#!/usr/bin/env python

import os.path
import contextlib

import roslib;
roslib.load_manifest('vros_display')
roslib.load_manifest('motmot_ros_utils')
import rospy
import calib.visualization
import rosutils.io

import pcl

from gi.repository import Gtk, GObject, Gdk

GObject.threads_init()
Gdk.threads_init()

class Tune(object):
    def __init__(self, pcd):
        self.pubcyl = None
        self.pubpt = None
        self.p = pcl.PointCloud()
        self.p.from_file(pcd)

        self.pubpts, pts = calib.visualization.create_point_cloud_message_publisher(
                                self.p.to_list(),
                                topic_name='/flydracalib/points',
                                publish_now=False,
                                latch=True)
        self.pubpts.publish(pts)
        self.pubptsinliers, pts = calib.visualization.create_point_cloud_message_publisher(
                                [(0,0,0)],
                                topic_name='/flydracalib/pointsinliers',
                                publish_now=False,
                                latch=True)
        self.pubptsinliers.publish(pts)

        thisdir = os.path.dirname(os.path.abspath(__file__))
        self.ui = Gtk.Builder()
        self.ui.add_from_file(os.path.join(thisdir,"tune.ui"))

        self._sr = 0.5
        self.ui.get_object("sr_adjustment").props.value = self._sr
        self._nd = 1.0
        self.ui.get_object("nd_adjustment").props.value = self._nd
        self._dt = 0.11
        self.ui.get_object("dt_adjustment").props.value = self._dt
        self._rl = (1.5,2.5)

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
        w.connect("delete-event", Gtk.main_quit)
        w.show_all()

    def on_sr_changed(self, adj):
        self._sr = adj.props.value
        self._calculate()

    def on_nd_changed(self, adj):
        self._nd = adj.props.value
        self._calculate()

    def on_dt_changed(self, adj):
        self._dt = adj.props.value
        self._calculate()

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

    def _calculate(self):
        seg = self.p.make_segmenter_normals(searchRadius=self._sr)
        seg.set_optimize_coefficients (True);
        seg.set_model_type (pcl.SACMODEL_CYLINDER)
        seg.set_method_type (pcl.SAC_RANSAC)
        seg.set_normal_distance_weight (self._nd)
        seg.set_max_iterations (10000)
        seg.set_distance_threshold (self._dt)
        seg.set_radius_limits (*self._rl)

        #seg.set_axis(0.41,1.894,2.733)
        #seg.set_eps_angle(0.1)

        print "SR",self._sr
        print "ND",self._nd
        print "DT",self._dt
        print "Radius Limits",self._rl

        ind, model = seg.segment()
        print model
        self._in.set_text(str(len(ind)))

        #publish the inlier set
        pts = calib.visualization.create_point_cloud(
                                self.p.extract(ind).to_list())
        self.pubptsinliers.publish(pts)

        self.cx,self.cy,self.cz,self.ax,self.ay,self.az,self.radius = model 
        self._update_cyl()

    def _update_cyl(self):
        print "update cyl"
        if not self.pubcyl:
            self.pubcyl, cyl = calib.visualization.create_cylinder_publisher(
                                self._cx,self._cy,self._cz,self._ax,self._ay,self._az,self._radius,
                                topic_name='/flydracalib/cyl',
                                publish_now=False,
                                latch=True,
                                length=5,
                                color=(0,1,0,0.2))
            self.pubpt, pt = calib.visualization.create_point_publisher(
                                self._cx,self._cy,self._cz,
                                r=0.1,
                                topic_name='/flydracalib/cylcenter',
                                publish_now=False,
                                latch=True,
                                color=(1,0,0,0.5))

        else:
            cyl = calib.visualization.create_cylinder(
                    self._cx,self._cy,self._cz,self._ax,self._ay,self._az,self._radius,
                    length=5,
                    color=(0,1,0,0.2))
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
    pcd_file = rosutils.io.decode_url('package://flycave/calibration/pcd/flydracyl.smooth.pcd')
    rospy.init_node("cylfittune", anonymous=True, disable_signals=True )
    t = Tune(pcd_file)
    Gtk.main()

