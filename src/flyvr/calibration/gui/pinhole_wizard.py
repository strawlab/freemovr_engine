#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil -*-
import os
import argparse
import yaml
import collections
import datetime
import tempfile

import numpy as np
import scipy.misc
import pkgutil

import roslib; roslib.load_manifest('flyvr')
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('camera_calibration')

import rospy

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import MarkerArray

import tf.broadcaster

from pymvg.camera_model import CameraModel

#import pymvg.extern.ros.rviz_utils as rviz_utils
import flyvr
import flyvr.simple_geom as simple_geom
#import flyvr.dlt as dlt
import flyvr.display_client as display_client
import flyvr.tools.fill_polygon as fill_polygon
import flyvr.exr as exr
#from flyvr.fit_extrinsics import fit_extrinsics, fit_extrinsics_iterative
#from flyvr.calib.pinhole.widgets import CellRendererButton

import rosgobject.core
import rosgobject.wrappers
from gi.repository import Gtk, GObject

from flyvr.calibration.gui.displayserver_comm import ProxyDisplayClient


from intrinsics_widget import get_intrinsics_grid

def nice_float_fmt(treeviewcolumn, cell, model, iter, column):
    float_in = model.get_value(iter, column)
    cell.set_property('text', '%g'%float_in )

DEFAULT_CHECKER_NROWS = 6
DEFAULT_CHECKER_NCOLS = 8
DEFAULT_CHECKER_SIZE = 0.015

EXTRINSIC_CALIBRATION_METHODS = [
    'extrinsic only',
    'iterative extrinsic only',
    'DLT',
    #'RANSAC DLT',
    ]
FULLSCREEN='FULL_SCREEN'

# columns in self.point_store
VDISP=0
TEXU=1
TEXV=2
DISPLAYX=3
DISPLAYY=4
SHOWPT=5
JOYLISTEN=6

# columns in self.vdisp_store
VS_VDISP = 0
VS_COUNT = 1
VS_CALIBRATION_RUNNING = 2
VS_CALIBRATION_PROGRESS = 3
VS_MRE = 4
VS_SHOW_BEACHBALL = 5
VS_CAMERA_OBJECT = 6
VS_PUBLISH_RVIZ = 7

def pretty_intrinsics_str(cam):
    K = cam.K
    d = cam.distortion
    dstr = ' '.join(['% 3g'%di for di in d])
    args = tuple(list(K.ravel()) + [dstr])
    result = \
"""K: % 10g % 10g % 10g
   % 10g % 10g % 10g
   % 10g % 10g % 10g
distortion: %s"""%args
    return result



class UI(object):
    def __init__(self):
        ui_file_contents = pkgutil.get_data('flyvr.calibration.gui','pinhole-wizard.ui')
        self._ui = Gtk.Builder()
        self._ui.add_from_string( ui_file_contents )
        #initilaize connect to display server widget sensitivity
        self.dsc = ProxyDisplayClient()
        self._build_ui()

        self.display_intrinsic_cam = None

        self.data_filename = None
        self.yamlfilter = Gtk.FileFilter()
        self.yamlfilter.set_name("YAML Files")
        self.yamlfilter.add_pattern("*.yaml")

        self.exr_filter = Gtk.FileFilter()
        self.exr_filter.set_name("EXR Files")
        self.exr_filter.add_pattern("*.exr")


        self.joy_mode='do points'

        self.intr_pub = {}
        self.frustum_pub = {}

        self.sub_joy = rosgobject.core.SubscriberGObject('joy_click_pose', Pose2D)
        self.sub_joy.connect('message', self.on_joy_callback)

        self.tf_b = tf.broadcaster.TransformBroadcaster()

        a1 = self._ui.get_object('main_window')
        a1.connect("delete-event", rosgobject.main_quit)
        a1.show_all()

        self._geom_dict = {}

        GObject.timeout_add(100, self.on_timer)

    def on_joy_callback(self, widget, msg):
        if self.joy_mode=='do CK':
            pt = [ msg.x,  msg.y ]
            self._current_checkerboard['points'].append(pt)
            npts = len(self._current_checkerboard['points'])
            self.add_CK_dialog.add_point(npts, pt)

        elif self.joy_mode=='do points':
            if not len(self.point_store):
                return

            for row in self.point_store:
                if row[JOYLISTEN]:
                    break

            if not row[JOYLISTEN]:
                # no row selected
                return

            row[DISPLAYX] = msg.x
            row[DISPLAYY] = msg.y
            self.update_bg_image()

    def _pulse_spinner(self, *args):
        if self.vdisp_store:
            for row in self.vdisp_store:
                row[VS_CALIBRATION_PROGRESS] += 1
        return True

    def _build_ui(self):
        # connection status ----------------------
        self._ds_connect_btn = self._ui.get_object('connect_to_display_server_button')
        self._ds_connect_btn.connect('clicked', self.on_connect_to_display_server)
        self._ds_disconnect_btn = self._ui.get_object('disconnect_from_display_server_button')
        self._ds_disconnect_btn.connect('clicked', self.on_disconnect_from_display_server)
        self._ds_status_lbl = self._ui.get_object('ds_connection_status_label')

        # save calibration EXR button -----------
        button = self._ui.get_object('save_calibration_exr_button')
        button.connect('clicked', self.on_save_calibration_exr)

        # build main window ----------------------

        window = self._ui.get_object('main_box')

        nb = Gtk.Notebook()
        window.add(nb)

        intrinsics_grid = get_intrinsics_grid(self.dsc, self.on_compute_intrinsics)
        # XXX: _icb = intrinsics_grid.on_joystick_button
        nb.append_page(intrinsics_grid, Gtk.Label(label='intrinsics'))

        nb.append_page(self._ui.get_object('virtual_display_layout_grid'),
                       Gtk.Label(label='virtual displays'))

        nb.append_page(self._ui.get_object('geom_grid'),
                       Gtk.Label(label='display geometry'))

        nb.append_page(self._ui.get_object('corresponding_points_grid'),
                       Gtk.Label(label='extrinsics'))

        self._ui.get_object('file_open_menu_item').connect(
            'activate', self.on_open)

        self._ui.get_object('file_save_menu_item').connect(
            'activate', self.on_save)

        self._ui.get_object('file_saveas_menu_item').connect(
            'activate', self.on_save_as)

        self._ui.get_object('file_quit_menu_item').connect(
            'activate', rosgobject.main_quit)

        self._ui.get_object('help_about_menu_item').connect(
            'activate', self.on_help_about)

        self._ui.get_object('view_mock_ds_item').connect(
            'activate', self.on_view_mock_ds)

        # setup help->about dialog -----------------
        self.help_about_dialog = Gtk.Dialog(title='About',
                                            parent=None,
                                            buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK))
        self.help_about_dialog.get_content_area().add(self._ui.get_object('help_about_dialog_grid'))
        version_str = getattr(flyvr,'__version__','0.0')
        self._ui.get_object('version_label').set_text(version_str)

        # setup vdisp combobox ----------------
        self.vdisp_store = Gtk.ListStore(str,int,bool,int,float,bool,object,bool)

        # create vdisp treeview -----------------------

        treeview = self._ui.get_object('vdisp_treeview')
        treeview.set_model( self.vdisp_store )

        renderer = Gtk.CellRendererText()
        column = Gtk.TreeViewColumn("virtual display", renderer, text=VS_VDISP)
        column.set_sort_column_id(VS_VDISP)
        treeview.append_column(column)

        renderer = Gtk.CellRendererText()
        column = Gtk.TreeViewColumn("count", renderer, text=VS_COUNT)
        column.set_sort_column_id(VS_COUNT)
        treeview.append_column(column)

        #renderer = CellRendererButton('Calibrate')
        #renderer.connect("clicked", self.on_trigger_cal)
        #column = Gtk.TreeViewColumn("calibration")
        #column.pack_start(renderer, False)
        #renderer = Gtk.CellRendererSpinner()
        #column.pack_start(renderer, False)
        #column.add_attribute(renderer, "active", VS_CALIBRATION_RUNNING)
        #column.add_attribute(renderer, "pulse", VS_CALIBRATION_PROGRESS)
        #treeview.append_column(column)
        #GObject.timeout_add(100, self._pulse_spinner)

        renderer = Gtk.CellRendererText()
        column = Gtk.TreeViewColumn("mean reproj. error", renderer,
                                    text=VS_MRE)
        column.set_sort_column_id(VS_MRE)
        treeview.append_column(column)

        renderer = Gtk.CellRendererToggle()
        renderer.connect("toggled", self.on_toggle_show_beachball)
        column = Gtk.TreeViewColumn("show beachball", renderer, active=VS_SHOW_BEACHBALL)
        treeview.append_column(column)

        renderer = Gtk.CellRendererToggle()
        renderer.connect("toggled", self.on_toggle_publish_rviz)
        column = Gtk.TreeViewColumn("publish RViz cam", renderer, active=VS_PUBLISH_RVIZ)
        treeview.append_column(column)

        # create point treeview -----------------------
#
        self.point_store = Gtk.ListStore(str, float, float, float, float, bool, bool)

        treeview = self._ui.get_object('treeview1')
        treeview.set_model( self.point_store )

        renderer_text = Gtk.CellRendererCombo(model=self.vdisp_store,
                                              text_column=VDISP,
                                              editable=True)
        renderer_text.connect("edited", self.on_edit_vdisp, VDISP)
        column = Gtk.TreeViewColumn("virtual display", renderer_text, text=VDISP)
        column.set_sort_column_id(VDISP)
        treeview.append_column(column)

        renderer_text = Gtk.CellRendererText(editable=True)
        renderer_text.connect("edited", self.on_edit_cell, TEXU)
        column = Gtk.TreeViewColumn("texture U", renderer_text, text=TEXU)
        column.set_cell_data_func(renderer_text, nice_float_fmt, func_data=TEXU)
        column.set_sort_column_id(TEXU)
        treeview.append_column(column)

        renderer_text = Gtk.CellRendererText(editable=True)
        renderer_text.connect("edited", self.on_edit_cell, TEXV)
        column = Gtk.TreeViewColumn("texture V", renderer_text, text=TEXV)
        column.set_cell_data_func(renderer_text, nice_float_fmt, func_data=TEXV)
        column.set_sort_column_id(TEXV)
        treeview.append_column(column)

        renderer_text = Gtk.CellRendererText(editable=True)
        renderer_text.connect("edited", self.on_edit_cell, DISPLAYX)
        column = Gtk.TreeViewColumn("display X", renderer_text, text=DISPLAYX)
        column.set_cell_data_func(renderer_text, nice_float_fmt, func_data=DISPLAYX)
        column.set_sort_column_id(DISPLAYX)
        treeview.append_column(column)

        renderer_text = Gtk.CellRendererText(editable=True)
        renderer_text.connect("edited", self.on_edit_cell, DISPLAYY)
        column = Gtk.TreeViewColumn("display Y", renderer_text, text=DISPLAYY)
        column.set_cell_data_func(renderer_text, nice_float_fmt, func_data=DISPLAYY)
        column.set_sort_column_id(DISPLAYY)
        treeview.append_column(column)

        renderer_toggle = Gtk.CellRendererToggle()
        renderer_toggle.connect("toggled", self.on_toggle_point_show)
        column = Gtk.TreeViewColumn("show", renderer_toggle, active=SHOWPT)
        column.set_sort_column_id(SHOWPT)
        treeview.append_column(column)

        renderer_pixbuf = Gtk.CellRendererToggle()
        renderer_pixbuf.set_radio(True)
        renderer_pixbuf.connect("toggled", self.on_do_point)
        column = Gtk.TreeViewColumn('Joystick select', renderer_pixbuf, active=JOYLISTEN)
        treeview.append_column(column)

        self.point_store.connect("row-changed",  self.on_points_updated)
        self.point_store.connect("row-inserted", self.on_points_updated)
        self.point_store.connect("row-deleted",  self.on_points_updated)

        # connect treeview buttons ---------------------------
        self._ui.get_object('UV_add_button').connect('clicked', self.on_add_UV)
        self._ui.get_object('UV_remove_button').connect('clicked', self.on_remove_UV)

        # self._ui.get_object('save_points_button').connect('clicked', self.on_save_to_yaml,
        #                                                   self.point_store_to_list)
        # self._ui.get_object('load_points_button').connect('clicked', self.on_load_points_button)

        self._ui.get_object('show_all_button').connect('clicked', self.on_show_all_button, True)
        self._ui.get_object('show_none_button').connect('clicked', self.on_show_all_button, False)

        # setup ComboBoxText
        cal_method_cbtext = self._ui.get_object('cal_method_cbtext')

        for method in EXTRINSIC_CALIBRATION_METHODS:
            cal_method_cbtext.append(method,method)
        cal_method_cbtext.set_active_id(EXTRINSIC_CALIBRATION_METHODS[0])

    # File menu ----------------------------------------------------
    def _load_from_file( self, fname ):
        with open(fname,mode='r') as fd:
            buf = fd.read()
        obj = yaml.safe_load(buf)
        self.load_display(obj,fname)
        self.load_geom(obj,fname)
        self.load_corresponding_points(obj)
        self.load_checkerboards(obj)
        self.data_filename = fname

    def _get_state_as_dict( self ):
        obj = self.checkerboard_store_to_list()
        d2 = self.point_store_to_list()
        obj.update( d2 )

        obj.update( self.display_to_list() )
        obj.update( self.geom_to_list() )
        return obj


    def _save_to_file( self, file ):
        do_close = False
        if hasattr(file,'write'):
            fd = file
            self.data_filename = None # clear the cached value
        else:
            fd = open(file,mode='w')
            do_close = True
            self.data_filename = file

        obj = self._get_state_as_dict()
        buf = yaml.safe_dump(obj)

        fd.write(buf)
        if do_close:
            fd.close()

    def on_open(self, button):
        filechooserdialog = Gtk.FileChooserDialog(title="FileChooserDialog",
                                                  parent=None,
                                                  buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK,
                                                           Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL))

        try:
            response = filechooserdialog.run()
            if response == Gtk.ResponseType.OK:
                self._load_from_file( filechooserdialog.get_filename() )
        finally:
            filechooserdialog.destroy()

    def on_save(self, *args):
        if self.data_filename is None:
            return self.on_save_as(*args)
        self._save_to_file( self.data_filename )

    def on_save_as(self, *args):
        filechooserdialog = Gtk.FileChooserDialog(title="FileChooserDialog",
                                                  parent=None,
                                                  action=Gtk.FileChooserAction.SAVE,
                                                  buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK,
                                                           Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL))
        filechooserdialog.add_filter( self.yamlfilter )
        filechooserdialog.set_do_overwrite_confirmation(True)

        try:
            response = filechooserdialog.run()

            if response == Gtk.ResponseType.OK:
                fname = filechooserdialog.get_filename()
                self._save_to_file( fname )
        finally:
            filechooserdialog.destroy()

    # ---------------- View menu -----------------------------
    def on_view_mock_ds(self,*args):
        self.dsc.proxy_show_mock()

    # ---------------- Help menu -----------------------------
    def on_help_about(self,*args):
        try:
            self.help_about_dialog.run()
        finally:
            self.help_about_dialog.hide()

    # ---------------- Connect to display server ------------
    def update_dsc_sensitivity(self, value):
        msg = 'connected' if value else 'not connected'
        self._ds_status_lbl.set_text(msg)
        self._ds_connect_btn.set_sensitive(not value)
        self._ds_disconnect_btn.set_sensitive(value)

    def on_connect_to_display_server(self,*args):
        e = self._ui.get_object('display_server_name_entry')
        ds_name = e.get_text()
        self.dsc.proxy_set_dsc(display_client.DisplayServerProxy(ds_name))
        self.update_dsc_sensitivity(True)
        self.update_bg_image()

        if self._ui.get_object('display_server_populate_cb').get_active():
            gi = self.dsc.get_geometry_info()
            di = self.dsc.get_display_info()
            fname = "running display server %s" % ds_name
            self.load_display({"display":di},fname)
            self.load_geom({"geom":gi},fname)

    def on_disconnect_from_display_server(self,*args):
        self.dsc.proxy_set_dsc(None)
        self.update_dsc_sensitivity(False)

    # ---------------- Save calibration EXR file -------------
    def save_calibration_exr(self,fname):
        state_dict = self._get_state_as_dict()
        obj = {'pinhole_wizard_input':state_dict}

        tcs = np.zeros( (self.dsc.height,self.dsc.width,2))-1
        dist = np.nan*np.ones( (self.dsc.height,self.dsc.width))
        angle = np.nan*np.ones( (self.dsc.height,self.dsc.width))
        allmask = np.zeros((self.dsc.height,self.dsc.width))

        di = self.dsc.get_display_info()

        for row in self.vdisp_store:
            vdisp = row[VS_VDISP]

            for d in di['virtualDisplays']:
                if d['id'] != vdisp:
                    continue
                else:
                    break

            assert d['id'] == vdisp

            polygon_verts = d['viewport']
            maskarr = np.zeros( allmask.shape, dtype=np.uint8 )
            fill_polygon.fill_polygon(polygon_verts, maskarr)
            if np.max(maskarr)==0: # no mask
                maskarr += 1

            allmask += maskarr
            mask = np.nonzero(maskarr)

            camera = row[VS_CAMERA_OBJECT]
            assert camera is not None

            this_tcs = self.geom.compute_for_camera_view(camera,
                                                         what='texture_coords')
            this_dist = self.geom.compute_for_camera_view(camera,
                                                          what='distance' )
            this_angle = self.geom.compute_for_camera_view(camera,
                                                           what='incidence_angle' )

            this_tcs[ np.isnan(this_tcs) ] = -1.0 # nan -> -1

            # copy the important parts to the full display image
            tcs[mask] = this_tcs[mask]
            dist[mask] = this_dist[mask]
            angle[mask] = this_angle[mask]
        r=tcs[:,:,0]
        g=tcs[:,:,1]
        if 0:
            # Replace this code with something that calculates a real
            # blending value here based on distance. Probably need to
            # normalize by the maximum distance.
            raise NotImplementedError("b = f(dist,angle)")
        else:
            b=np.ones_like(tcs[:,:,1])

        exr.save_exr( fname, r=r, g=g, b=b)

        if 1:
            # save low dynamic range .png image
            ri = np.array( r*255, dtype=np.uint8 )
            gi = np.array( g*255, dtype=np.uint8 )
            bi = np.array( b*255, dtype=np.uint8 )

            h,w = r.shape[:2]
            imi = np.empty( (h,w,3), dtype=np.uint8 )
            imi[:,:,0] = ri
            imi[:,:,1] = gi
            imi[:,:,2] = bi
            scipy.misc.imsave(fname+'.png', imi)

    def on_save_calibration_exr(self,*args):
        filechooserdialog = Gtk.FileChooserDialog(title="FileChooserDialog",
                                                  parent=None,
                                                  action=Gtk.FileChooserAction.SAVE,
                                                  buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK,
                                                           Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL))
        filechooserdialog.add_filter( self.exr_filter )
        filechooserdialog.set_do_overwrite_confirmation(True)

        try:
            response = filechooserdialog.run()

            if response == Gtk.ResponseType.OK:
                fname = filechooserdialog.get_filename()
                self.save_calibration_exr(fname)
        finally:
            filechooserdialog.destroy()

    # ---------------- Checkerboard & intrinsics -------------

    def checkerboard_store_to_list(self):
        result = [row[0] for row in self.checkerboard_store]
        return {'checkerboards':result}

    def load_checkerboards(self,indict):
        in_list = indict.get('checkerboards',[])
        self.checkerboard_store.clear()
        for rowdict in in_list:
            self.checkerboard_store.append( [rowdict] )

    def render_checkerboard_row(self, treeviewcolumn, cell, model, iter, attr):
        rowdict = model.get_value(iter,0)
        cell.set_property('text', str(rowdict[attr] ))

    def on_CK_add(self, *args):
        self.joy_mode='do CK'
        self._current_checkerboard = {'points':[]}
        try:
            #reset current number of collected points, and current point
            response = self.add_CK_dialog.run(self.dsc)
            if response == Gtk.ResponseType.OK:
                self._current_checkerboard['rows'] = self.add_CK_dialog.get_num_rows()
                self._current_checkerboard['columns'] = self.add_CK_dialog.get_num_cols()
                self._current_checkerboard['size'] = self.add_CK_dialog.get_size()
                nowstr = datetime.datetime.now().isoformat(' ')
                self._current_checkerboard['date_string'] = nowstr
                self.checkerboard_store.append( [self._current_checkerboard] )
            self.joy_mode=None
            self._current_checkerboard = None # delete current checkerboard
        finally:
            self.add_CK_dialog.hide()
            self.joy_mode='do points'

    def on_CK_remove(self, *args):
        treeview = self._ui.get_object('checkerboard_treeview')
        selection = treeview.get_selection()
        sel = selection.get_selected()
        if not sel[1] == None:
            self.checkerboard_store.remove( sel[1] )

    def on_compute_intrinsics(self, obj):
        cam = CameraModel.from_dict(obj, extrinsics_required=False)
        self.display_intrinsic_cam = cam
        self._ui.get_object('intrinsic_status_label').set_text(pretty_intrinsics_str(cam))

    # Virtual displays ----------------------------------------------

    def display_to_list(self):
        result = dict(width = self.dsc.width,
                      height = self.dsc.height,
                      )
        di = self.dsc.get_display_info()

        if 'virtualDisplays' in di:
            vdisps = [d['id'] for d in di['virtualDisplays']]
            result['virtualDisplays'] = []
        else:
            vdisps = []#FULLSCREEN]

        for vdisp in vdisps:
            count = 0
            for d in di['virtualDisplays']:
                if d['id'] != vdisp:
                    continue

                this_vdisp = dict(id = vdisp,
                                  viewport = d['viewport'],
                                  )
                if 'mirror' in d:
                    this_vdisp['mirror'] = d['mirror']
                result['virtualDisplays'].append( this_vdisp )
                count += 1
            assert count == 1
        return {'display':result}

    def load_display(self,obj,fname):
        display_dict = obj['display']

        self.dsc.proxy_set_display_info(display_dict)

        self._ui.get_object('virtual_display_yaml').get_buffer().set_text(
            yaml.safe_dump(display_dict) )
        self._ui.get_object('virtual_display_header_label').set_text(
            "Virtual displays loaded from %s" % os.path.basename(fname))

        self.vdisp_store.clear()

        di = self.dsc.get_display_info()

        if 'virtualDisplays' in di:
            vdisps = [d['id'] for d in di['virtualDisplays']]
        else:
            vdisps = [FULLSCREEN]

        for vdisp in vdisps:
            self.vdisp_store.append([vdisp,0,False,0,np.nan,0,None,0])
            self.intr_pub[vdisp] = rospy.Publisher(vdisp+'/camera_info',
                                                   CameraInfo, latch=True)
            self.frustum_pub[vdisp] = rospy.Publisher(vdisp+'/frustum',
                                                       MarkerArray)

    # Display geometry ----------------------------------------------

    def geom_to_list(self):
        return {'geom':self._geom_dict}

    def load_geom(self,obj,fname):
        self._geom_dict = obj['geom']
        self.dsc.proxy_set_geometry_info(self._geom_dict)
        self._ui.get_object('geometry_entry').get_buffer().set_text(
            yaml.safe_dump(self._geom_dict) )
        self._ui.get_object('geometry_header_label').set_text(
            "Geometry loaded from %s" % os.path.basename(fname))

        self.geom = simple_geom.Geometry(geom_dict=self._geom_dict)

    # ---------------- Point correspondence & extrinsics -------------

    def on_show_all_button(self,*args):
        val = args[-1]
        for row in self.point_store:
            row[SHOWPT] = val
        self.update_bg_image()

    def on_points_updated(self, *args):
        vdisps = collections.defaultdict(int)
        for row in self.point_store:
            vdisps[ row[VDISP] ] += 1

        for row in self.vdisp_store:
            vdisp = row[VS_VDISP]
            row[VS_COUNT] = vdisps[ vdisp ]


    def on_edit_vdisp(self,widget,path,textval,colnum):
        self.point_store[path][colnum] = textval
        self.update_bg_image()

    def on_edit_cell(self,widget,path,textval,colnum):
        value = float(textval)
        self.point_store[path][colnum] = value
        self.update_bg_image()

    def on_do_point(self, widget, path):
        selected_path = Gtk.TreePath(path)
        # perform mutually-exclusive radio button setting
        for row in self.point_store:
            row[JOYLISTEN] = (row.path == selected_path)

    def _get_default_vdisp(self):
        di = self.dsc.get_display_info()
        if 'virtualDisplays' in di:
            val = di['virtualDisplays'][0]['id']
        else:
            val = FULLSCREEN
        return val

    def point_store_to_list(self):
        result = []
        for row in self.point_store:
            rowdict = dict(
                virtual_display = row[VDISP],
                texture_u = row[TEXU],
                texture_v = row[TEXV],
                display_x = row[DISPLAYX],
                display_y = row[DISPLAYY],
                )
            result.append( rowdict )
        r = {'uv_display_points':result}
        return r

    def load_corresponding_points(self,indict):
        in_list = indict.get('uv_display_points',[])
        self.point_store.clear()
        for rowdict in in_list:
            self._add_pt_entry( rowdict['virtual_display'],
                                rowdict['texture_u'],
                                rowdict['texture_v'],
                                displayX=rowdict['display_x'],
                                displayY=rowdict['display_y'],
                                )
        self.update_bg_image()

    def on_add_UV(self, button):
        vdisp = self._get_default_vdisp()
        self._add_pt_entry(vdisp, np.nan, np.nan )
        self.update_bg_image()

    def on_remove_UV(self,button):
        treeview = self._ui.get_object('treeview1')
        selection = treeview.get_selection()
        sel = selection.get_selected()
        if not sel[1] == None:
            self.point_store.remove( sel[1] )

    def _add_pt_entry(self, vdisp, texU, texV,
                      displayX=float(np.nan), displayY=float(np.nan),
                      show_point=True, joy_listen=False):
        self.point_store.append([vdisp, texU, texV,
                               displayX, displayY,
                               show_point, joy_listen])

    def on_toggle_point_show(self, widget, path):
        self.point_store[path][SHOWPT] = not self.point_store[path][SHOWPT]
        self.update_bg_image()

    def on_toggle_show_beachball(self, widget, path):
        self.vdisp_store[path][VS_SHOW_BEACHBALL] = not self.vdisp_store[path][VS_SHOW_BEACHBALL]
        self.update_bg_image()

    def on_toggle_publish_rviz(self, widget, path):
        self.vdisp_store[path][VS_PUBLISH_RVIZ] = not self.vdisp_store[path][VS_PUBLISH_RVIZ]
        self.publish_rviz()

    def on_timer(self):
        self.publish_rviz()
        return True

    def publish_rviz(self):
        now = rospy.Time.now()
        future = now + rospy.Duration(0.010) # 10 msec in future

        #all_cam_id_base = 100300
        all_cam_id_base = 0
        for enum,row in enumerate(self.vdisp_store):
            vdisp = row[VS_VDISP]

            frame_id = '/'+vdisp

            cam = row[VS_CAMERA_OBJECT]
            cam_id_base = enum*100 + all_cam_id_base
            if cam is not None and row[VS_PUBLISH_RVIZ]:

                intrinsics = cam.get_intrinsics_as_bunch()
                intrinsic_msg = CameraInfo(**intrinsics.__dict__)
                intrinsic_msg.header.stamp = now
                intrinsic_msg.header.frame_id = frame_id
                self.intr_pub[vdisp].publish( intrinsic_msg )

                translation, rotation = cam.get_ROS_tf()
                self.tf_b.sendTransform( translation,
                                         rotation,
                                         future,
                                         frame_id,
                                         '/map',
                                         )

                #r = rviz_utils.get_frustum_markers( cam, id_base=cam_id_base, scale=1.0, stamp=now )
                #self.frustum_pub[vdisp].publish(r['markers'])

    def on_trigger_cal(self, widget, path):
        vdisp = self.vdisp_store[path][VS_VDISP]
        method = self._ui.get_object('cal_method_cbtext').get_active_text()
        self.vdisp_store[path][VS_CALIBRATION_RUNNING] = True
        self.launch_calibration( method, vdisp )
        self.vdisp_store[path][VS_CALIBRATION_RUNNING] = False

    def calibrate_all_vdisps(self,method):
        for row in self.vdisp_store:
            row[VS_CALIBRATION_RUNNING] = True
            self.launch_calibration( method, row[VS_VDISP] )
            row[VS_CALIBRATION_RUNNING] = False

    def launch_calibration(self, method, vdisp ):

        def _pump_ui():
            if os.environ.get('RUNNING_NOSE') != '1':
                Gtk.main_iteration_do(False) #run once, no block
                while Gtk.events_pending():  #run remaining
                    Gtk.main_iteration()

        orig_data = []
        for row in self.point_store:
            if row[VDISP]==vdisp:
                orig_data.append( [ row[TEXU], row[TEXV], row[DISPLAYX], row[DISPLAYY] ] )
        orig_data = np.array(orig_data)
        uv = orig_data[:,:2]
        XYZ = self.geom.model.texcoord2worldcoord(uv)
        xy = orig_data[:,2:4]

        assert method in EXTRINSIC_CALIBRATION_METHODS

        if method in ('DLT','RANSAC DLT'):
            ransac = method.startswith('RANSAC')
            #r = dlt.dlt(XYZ, xy, ransac=ransac )
            #c1 = CameraModel.load_camera_from_M( r['pmat'],
            #                                           width=self.dsc.width,
            #                                           height=self.dsc.height,
            #                                           )

            #_pump_ui()

            if 0:
                c2 = c1.get_flipped_camera()

                # slightly hacky way to find best camera direction
                obj = self.geom.model.get_center()

                d1 = np.sqrt( np.sum( (c1.get_lookat() - obj)**2 ))
                d2 = np.sqrt( np.sum( (c2.get_lookat() - obj)**2 ))
                if d1 < d2:
                    #print 'using normal camera'
                    camera = c1
                else:
                    print 'using flipped camera'
                    camera = c2
            elif 1:
                farr = self.geom.compute_for_camera_view( c1,
                                                          what='texture_coords' )

                _pump_ui()

                u = farr[:,:,0]
                good = ~np.isnan( u )
                npix=np.sum( np.nonzero( good ) )
                if npix==0:
                    print 'using flipped camera, otherwise npix = 0'
                    camera = c1.get_flipped_camera()
                else:
                    camera = c1
            else:
                camera = c1
        elif method in ['extrinsic only','iterative extrinsic only']:
            assert self.display_intrinsic_cam is not None, 'need intrinsic calibration'

            di = self.dsc.get_display_info()

            mirror = None
            if 'virtualDisplays' in di:
                found = False
                for d in di['virtualDisplays']:
                    if d['id'] == vdisp:
                        found = True
                        mirror = d.get('mirror',None)
                        break
                assert found


            if mirror is not None:
                cami = self.display_intrinsic_cam.get_mirror_camera(axis=mirror)
            else:
                cami = self.display_intrinsic_cam

            _pump_ui()

            if method == 'iterative extrinsic only':
                #result = fit_extrinsics_iterative(cami,XYZ,xy)
                pass
            else:
                #result = fit_extrinsics(cami,XYZ,xy)
                pass

            _pump_ui()

            c1 = result['cam']
            if 1:
                farr = self.geom.compute_for_camera_view( c1,
                                                          what='texture_coords' )

                _pump_ui()

                u = farr[:,:,0]
                good = ~np.isnan( u )
                npix=np.sum( np.nonzero( good ) )
                if npix==0:
                    print 'using flipped camera, otherwise npix = 0'
                    camera = c1.get_flipped_camera()
                else:
                    camera = c1
            else:
                camera = c1
            del result
        else:
            raise ValueError('unknown calibration method %r'%method)

        _pump_ui()

        projected_points = camera.project_3d_to_pixel( XYZ )
        reproj_error = np.sum( (projected_points - xy)**2, axis=1)
        mre = np.mean(reproj_error)

        for row in self.vdisp_store:
            if row[VS_VDISP]==vdisp:
                row[VS_MRE] = mre
                row[VS_CAMERA_OBJECT] = camera

        _pump_ui()

        self.update_bg_image()

    def update_bg_image(self):
        arr = np.zeros( (self.dsc.height,self.dsc.width,3), dtype=np.uint8 )
        di = self.dsc.get_display_info()
        # draw beachballs ----------------------

        for row in self.vdisp_store:
            if not row[VS_SHOW_BEACHBALL]:
                continue
            cam = row[VS_CAMERA_OBJECT]
            if cam is None:
                continue

            vdisp = row[VS_VDISP]
            for d in di['virtualDisplays']:
                if d['id'] != vdisp:
                    continue
                else:
                    break

            assert d['id'] == vdisp

            farr = self.geom.compute_for_camera_view( cam,
                                                      what='texture_coords' )

            u = farr[:,:,0]
            good = ~np.isnan( u )
            print 'showing beachball for %r'%row[VS_VDISP]
            print '  npix0',np.sum( np.nonzero( good ) )

            arr2 = simple_geom.tcs_to_beachball(farr)

            h,w = arr2.shape[:2]
            maskarr = np.zeros( (h,w), dtype=np.uint8 )
            polygon_verts = d['viewport']
            fill_polygon.fill_polygon(polygon_verts, maskarr)
            if np.max(maskarr)==0: # no mask
                maskarr += 1

            arr3 = maskarr[:,:,np.newaxis]*arr2

            print '  npix1',np.sum(np.nonzero(arr2))
            arr = arr+arr3

            print '  npix2',np.sum(np.nonzero(arr))

        # draw individual points ---------------

        showing = []
        for row in self.point_store:
            if not row[SHOWPT]:
                continue
            xf,yf = row[DISPLAYX], row[DISPLAYY]
            try:
                x = int(np.round(xf))
                y = int(np.round(yf))
            except ValueError:
                # Likely cannot convert nan to integer.
                continue
            if 0 <= x and x < self.dsc.width:
                if 0 <= y and y < self.dsc.height:
                    arr[y,x] = 255
                    showing.append( (x,y) )
        self.dsc.show_pixels(arr)
        return arr

if __name__ == "__main__":
    rospy.init_node("pinhole_wizard")
    rosgobject.get_ros_thread() #ensure ros is spinning
    rosgobject.add_console_logger()

    parser = argparse.ArgumentParser()

    parser.add_argument('--just-use-this-data', type=str, default=None,
                        help="If specified, .yaml file with data "
                        "to run non-interactively.")

    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    ui = UI()
    if args.just_use_this_data:
        ui._load_from_file(args.just_use_this_data)
    Gtk.main()