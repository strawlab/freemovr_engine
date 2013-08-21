#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil -*-
import os.path
import argparse
import yaml
import math
import collections
import datetime
import tempfile

import numpy as np
import scipy.misc
import pkgutil

import roslib; roslib.load_manifest('flyvr')
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('camera_calibration')
import camera_calibration.calibrator

import rospy

import flyvr.rviz_utils
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import CameraInfo
import tf.broadcaster
from visualization_msgs.msg import MarkerArray

import flyvr.simple_geom as simple_geom
import camera_model
import flyvr.dlt as dlt
import flyvr.display_client as display_client
import flyvr.fill_polygon as fill_polygon
from flyvr.exr import save_exr

import rosgobject.core
import rosgobject.wrappers
import cairo
from gi.repository import Gtk, GObject
from flyvr.fit_extrinsics import fit_extrinsics, fit_extrinsics_iterative

def nice_float_fmt(treeviewcolumn, cell, model, iter, column):
    float_in = model.get_value(iter, column)
    cell.set_property('text', '%g'%float_in )

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
VS_CAL_BUTTON = 2
VS_MRE = 3
VS_SHOW_BEACHBALL = 4
VS_CAMERA_OBJECT = 5
VS_PUBLISH_RVIZ = 6

def pretty_intrinsics_str(cam):
    K = cam.K
    d = cam.distortion
    dstr = ' '.join(['% 3g'%di for di in d[:,0]])
    args = tuple(list(K.ravel()) + [dstr])#str(d[:,0])])
    result = \
"""K: % 10g % 10g % 10g
   % 10g % 10g % 10g
   % 10g % 10g % 10g
distortion: %s"""%args
    return result

def get_camera_for_boards(rows,width=0,height=0):
    info_dict = {}
    for row in rows:
        r = row[0]
        info_str = '%d %d %f'%(r['rows'], r['columns'], r['size'])
        if info_str not in info_dict:
            # create entry
            info = camera_calibration.calibrator.ChessboardInfo()
            info.dim = r['size']
            info.n_cols = r['columns']
            info.n_rows = r['rows']
            info_dict[info_str] = {'info':info,
                                   'corners':[]}
        this_list = info_dict[info_str]['corners']
        this_corners = r['points']
        assert len(this_corners)==r['rows']*r['columns']
        this_list.append( this_corners )

    boards = []
    goodcorners = []
    for k in info_dict:
        info = info_dict[k]['info']
        for xys in info_dict[k]['corners']:
            goodcorners.append( (xys,info) )

    cal = camera_calibration.calibrator.MonoCalibrator(boards)
    cal.size = (width,height)
    r = cal.cal_fromcorners(goodcorners)
    msg = cal.as_message()

    buf = roslib.message.strify_message(msg)
    obj = yaml.safe_load(buf)
    cam = camera_model.CameraModel.from_dict(obj,
                                             extrinsics_required=False)
    return cam

class CheckerboardPlotWidget(Gtk.DrawingArea):

    SZ = 50

    def __init__(self):
        Gtk.DrawingArea.__init__(self)
        self._nrows = 2
        self._ncols = 2
        self._currpt = 0

    def set_next_point(self, n):
        self._currpt = n
        self.queue_draw()

    def set_board_size_num_corners(self, row, col):
        self.set_board_size(
                None if row is None else row+1,
                None if col is None else col+1)

    def set_board_size(self, row, col):
        self._nrows = self._nrows if row is None else row
        self._ncols = self._ncols if col is None else col
        self.set_size_request(
                1.2*self._ncols*self.SZ,
                1.2*self._nrows*self.SZ
        )
        self.queue_draw()

    def do_draw(self, cr):
        SZ,W,H = self.SZ,int(self._ncols),int(self._nrows)

        x0 = (self.get_allocation().width - (W*SZ)) // 2
        y0 = (self.get_allocation().height - (H*SZ)) // 2

        #the internal corner number
        ic = 0

        #column
        for n,_x in enumerate(range(0,W*SZ,SZ)):
            #row
            for m,_y in enumerate(range(0,H*SZ,SZ)):
                #ensure columns start with alternating colors
                black = ((m % 2) + n) % 2

                #centre the board
                x = _x+x0
                y = _y+y0

                cr.rectangle(x, y, SZ,  SZ)
                if black:
                    colour = (0, 0, 0)
                else:
                    colour = (1, 1, 1)

                cr.set_source_rgb(*colour)
                cr.fill()

                if (n > 0) and (n < self._ncols):
                    if (m > 0) and (m < self._nrows):
                        ic += 1
                        if ic == self._currpt:
                            cr.set_source_rgb(1,0,0)
                            #draw a circle at the corner
                            cr.arc(x,y,5,0,2*math.pi)
                            cr.fill()
                        #always draw text
                        #cr.move_to(x, y)
                        #cr.show_text("%d" % ic)


class AddCheckerboardDialog(Gtk.Dialog):
    def __init__(self, ui, **kwargs):
        Gtk.Dialog.__init__(self, title="Add checkerboard",
                                  buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK,
                                           Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL),
                                  **kwargs)
        self.get_content_area().add(ui.get_object('add_CK_dialog_grid'))
        self._npts_lbl = ui.get_object('N_CKB_points_label')
        self._checker = CheckerboardPlotWidget()
        ui.get_object('checkerboard_plot_box').pack_start(self._checker, True, True, 0)

        self._sze = ui.get_object('CK_size_entry')

        #connect the spinbuttons to change the checkerboard
        self._nr = ui.get_object('CK_n_rows_spinbutton')
        self._nr.connect('value-changed',
                    lambda sbr: self._checker.set_board_size_num_corners(sbr.get_value(), None))
        self._nc = ui.get_object('CK_n_cols_spinbutton')
        self._nc.connect('value-changed',
                    lambda sbc: self._checker.set_board_size_num_corners(None, sbc.get_value()))
        self._checker.set_board_size_num_corners(self._nr.get_value(),self._nc.get_value())

    def get_num_rows(self):
        return int(self._nr.get_value())
    def get_num_cols(self):
        return int(self._nc.get_value())
    def get_size(self):
        return float(self._sze.get_text())

    def add_point(self, n, pt):
        self._npts_lbl.set_text('%d' % n)
        self._checker.set_next_point(n+1)

    def run(self):
        self.show_all()
        return Gtk.Dialog.run(self)

class ProxyDisplayClient(object):
    def __init__(self):
        self._dsc = None
        self._di = None
        self._file = os.path.join(tempfile.mkdtemp(),"ds.png")
        self._w = Gtk.Window(title="Display Server Output")
        self._img = Gtk.Image()
        self._img.set_from_stock("gtk-missing-image", Gtk.IconSize.DIALOG)
        self._w.add(self._img)
        self._w.connect("delete-event", self._on_close)

    def _on_close(self, *args):
        self._w.hide()
        return True #stop signal

    def proxy_show_mock(self):
        self._w.show_all()

    def proxy_set_dsc(self, dsc):
        self._dsc = dsc
        if self._dsc is not None:
            self._dsc.enter_2dblit_mode()

    def proxy_set_display_info(self, di):
        self._di = di

    def __getattr__(self, name):
        return getattr(self._dsc, name)

    @property
    def height(self):
        if self._di is not None:
            return self._di['height']
        elif self._dsc is not None:
            return self._dsc.height

    @property
    def width(self):
        if self._di is not None:
            return self._di['width']
        elif self._dsc is not None:
            return self._dsc.width

    def get_display_info(self):
        if self._di is not None:
            return self._di
        elif self._dsc is not None:
            return self._dsc.get_display_info()

    def show_pixels(self,arr):
        scipy.misc.imsave(self._file, arr)
        self._img.set_from_file(self._file)
        if self._dsc is not None:
            self._dsc.show_pixels(arr)

class UI(object):
    def __init__(self):
        ui_file_contents = pkgutil.get_data('flyvr.calib.pinhole','pinhole-wizard.ui')
        self._ui = Gtk.Builder()
        self._ui.add_from_string( ui_file_contents )
        self._build_ui()

        self.display_intrinsic_cam = None

        self.data_filename = None
        self.yamlfilter = Gtk.FileFilter()
        self.yamlfilter.set_name("YAML Files")
        self.yamlfilter.add_pattern("*.yaml")

        self.exr_filter = Gtk.FileFilter()
        self.exr_filter.set_name("EXR Files")
        self.exr_filter.add_pattern("*.exr")

        #initilaize connect to display server widget sensitivity
        self.dsc = ProxyDisplayClient()

        self.joy_mode='do points'

        self.intr_pub = {}
        self.frustum_pub = {}

        self.sub_joy = rosgobject.core.SubscriberGObject('joy_click_pose', Pose2D)
        self.sub_joy.connect('message', self.on_joy_callback)

        self.tf_b = tf.broadcaster.TransformBroadcaster()

        a1 = self._ui.get_object('main_window')
        a1.connect("delete-event", rosgobject.main_quit)
        a1.show_all()

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

        nb.append_page(self._ui.get_object('checkerboard_grid'),
                       Gtk.Label(label='intrinsics'))

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

        # setup checkerboard treeview ----------------

        self.checkerboard_store = Gtk.ListStore(object)

        treeview = self._ui.get_object('checkerboard_treeview')
        treeview.set_model( self.checkerboard_store )

        renderer = Gtk.CellRendererText()
        column = Gtk.TreeViewColumn("rows", renderer)
        column.set_cell_data_func(renderer, self.render_checkerboard_row, func_data='rows')
        treeview.append_column(column)

        renderer = Gtk.CellRendererText()
        column = Gtk.TreeViewColumn("columns", renderer)
        column.set_cell_data_func(renderer, self.render_checkerboard_row, func_data='columns')
        treeview.append_column(column)

        renderer = Gtk.CellRendererText()
        column = Gtk.TreeViewColumn("size", renderer)
        column.set_cell_data_func(renderer, self.render_checkerboard_row, func_data='size')
        treeview.append_column(column)

        renderer = Gtk.CellRendererText()
        column = Gtk.TreeViewColumn("time", renderer)
        column.set_cell_data_func(renderer, self.render_checkerboard_row, func_data='date_string')
        treeview.append_column(column)

        self._ui.get_object('CK_add_button').connect('clicked', self.on_CK_add)
        self._ui.get_object('CK_remove_button').connect('clicked', self.on_CK_remove)

        self._ui.get_object('compute_intrinsics').connect('clicked', self.on_compute_intrinsics)

        # setup checkerboard dialog ----------------
        self.add_CK_dialog = AddCheckerboardDialog(self._ui)

        # setup help->about dialog -----------------
        self.help_about_dialog = Gtk.Dialog(title='About',
                                            parent=None,
                                            buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK))
        self.help_about_dialog.get_content_area().add(self._ui.get_object('help_about_dialog_grid'))
        version_str = getattr(flyvr,'__version__','0.0')
        self._ui.get_object('version_label').set_text(version_str)

        # setup vdisp combobox ----------------
        self.vdisp_store = Gtk.ListStore(str,int,bool,float,bool,object,bool)

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

        renderer = Gtk.CellRendererToggle()
        renderer.connect("toggled", self.on_trigger_cal)
        column = Gtk.TreeViewColumn("calibrate", renderer, active=VS_CAL_BUTTON)
        treeview.append_column(column)

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

        self.load_display(obj)
        self.load_corresponding_points(obj)
        self.load_checkerboards(obj)
        self.load_geom(obj)
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
        buf = yaml.dump(obj)

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
        self._ds_disconnect_btn.set_sensitive(not value)

    def on_connect_to_display_server(self,*args):
        e = self._ui.get_object('display_server_name_entry')
        ds_name = e.get_text()
        self.dsc.proxy_set_dsc(display_client.DisplayServerProxy(ds_name))
        self.update_dsc_sensitivity(True)
        self.update_bg_image() # send current image over

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
        save_exr( fname, r=r, g=g, b=b)

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
            self.add_CK_dialog.add_point(0, None)
            response = self.add_CK_dialog.run()
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

    def on_compute_intrinsics(self,*args):
        rows = [r for r in self.checkerboard_store]
        cam = get_camera_for_boards( rows,
                                     width=self.dsc.width,
                                     height=self.dsc.height )
        self.display_intrinsic_cam = cam
        self._ui.get_object('intrinsic_status_label').set_text(pretty_intrinsics_str(cam))

        # if 1:
        #     print 'all ------------------'
        #     print pretty_intrinsics_str(cam)

        #     for i in range(len(rows)):
        #         r2 = [r for (j,r) in enumerate(rows) if j!=i]

        #         cam2 = get_camera_for_boards( r2,
        #                                       width=self.dsc.width,
        #                                       height=self.dsc.height )

        #         print 'not %d (%s) ------------------'%(i, rows[i][0]['date_string'])
        #         print pretty_intrinsics_str(cam2)

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

    def load_display(self,obj):
        display_dict = obj['display']

        self.dsc.proxy_set_display_info(display_dict)

        self._ui.get_object('virtual_display_yaml').get_buffer().set_text(
            yaml.dump(display_dict) )

        self.vdisp_store.clear()

        di = self.dsc.get_display_info()

        if 'virtualDisplays' in di:
            vdisps = [d['id'] for d in di['virtualDisplays']]
        else:
            vdisps = [FULLSCREEN]

        for vdisp in vdisps:
            self.vdisp_store.append([vdisp,0,0,np.nan,0,None,0])
            self.intr_pub[vdisp] = rospy.Publisher(vdisp+'/camera_info',
                                                   CameraInfo, latch=True)
            self.frustum_pub[vdisp] = rospy.Publisher(vdisp+'/frustum',
                                                       MarkerArray)

    # Display geometry ----------------------------------------------

    def geom_to_list(self):
        return {'geom':self._geom_dict}

    def load_geom(self,obj):
        self._geom_dict = obj['geom']
        self._ui.get_object('geometry_entry').get_buffer().set_text(
            yaml.dump(self._geom_dict) )

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

                intrinsic_msg = cam.get_intrinsics_as_msg()
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

                r = flyvr.rviz_utils.get_frustum_markers( cam, id_base=cam_id_base, scale=1.0, stamp=now )
                self.frustum_pub[vdisp].publish(r['markers'])

    def on_trigger_cal(self, widget, path):
        vdisp = self.vdisp_store[path][0]

        method = self._ui.get_object('cal_method_cbtext').get_active_text()
        self.launch_calibration( method, vdisp )

    def calibrate_all_vdisps(self,method):
        for row in self.vdisp_store:
            vdisp = row[VS_VDISP]
            self.launch_calibration( method, vdisp )

    def launch_calibration(self, method, vdisp ):
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
            r = dlt.dlt(XYZ, xy, ransac=ransac )
            c1 = camera_model.CameraModel.load_camera_from_pmat( r['pmat'],
                                                     width=self.dsc.width,
                                                     height=self.dsc.height,
                                                     )

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

            if method == 'iterative extrinsic only':
                result = fit_extrinsics_iterative(cami,XYZ,xy)
            else:
                result = fit_extrinsics(cami,XYZ,xy)

            c1 = result['cam']
            if 1:
                farr = self.geom.compute_for_camera_view( c1,
                                                          what='texture_coords' )

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

        projected_points = camera.project_3d_to_pixel( XYZ )
        reproj_error = np.sum( (projected_points - xy)**2, axis=1)
        mre = np.mean(reproj_error)

        for row in self.vdisp_store:
            if row[VS_VDISP]==vdisp:
                row[VS_MRE] = mre
                row[VS_CAMERA_OBJECT] = camera
        self.update_bg_image()

    def update_bg_image(self):
        arr = np.zeros( (self.dsc.height,self.dsc.width,3), dtype=np.uint8 )

        # draw beachballs ----------------------

        # FIXME: add masks

        for row in self.vdisp_store:
            if not row[VS_SHOW_BEACHBALL]:
                continue
            cam = row[VS_CAMERA_OBJECT]
            if cam is None:
                continue

            farr = self.geom.compute_for_camera_view( cam,
                                                      what='texture_coords' )

            u = farr[:,:,0]
            good = ~np.isnan( u )
            print 'showing beachball for %r'%row[VS_VDISP]
            print '  npix0',np.sum( np.nonzero( good ) )

            arr2 = simple_geom.tcs_to_beachball(farr)
            print '  npix1',np.sum(np.nonzero(arr2))
            arr = arr+arr2 # FIXME: hacky OR operation assumes pixels are either 0 or final value

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
