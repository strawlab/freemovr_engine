#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil -*-
import os
import collections
import itertools

from gi.repository import Gtk

import numpy as np
import pkgutil
import warnings
try:
    import roslib
except ImportError:
    warnings.warn("Can't import roslib. Make sure that flyvr python package is in your PYTHONPATH.")
else:
    roslib.load_manifest("flyvr")
    roslib.load_manifest('visualization_msgs')
    roslib.load_manifest('camera_calibration')

import flyvr.simple_geom as simple_geom
import flyvr.tools.fill_polygon as fill_polygon
from flyvr.calibration.gui.extra_widgets import ClassStoreTreeViewGenerator
from flyvr.calibration.extrinsics import ExtrinsicsAlgorithms

from pymvg.camera_model import CameraModel

import termcolor

class ViewportExtrinsicCalibration(object):

    def __init__(self, viewport):
        self.vp = viewport
        self.p2p = []
        self.camera = None
        self.show_beachball = False

    @property
    def viewportname(self):
        return self.vp.name

    @property
    def calibrated(self):
        return self.camera is not None

    def calibrate(self, camera, method, xy, XYZ):
        print termcolor.colored("Calibrating using:", "red"), termcolor.colored(method, "red")
        method_dict = ExtrinsicsAlgorithms[method]
        calfunc = method_dict['function']
        intrinsics_required = method_dict['requires-intrinsics']
        R, T, _ = calfunc(camera, XYZ, xy, extrinsics_guess=None, params={})
        print termcolor.colored("got:", "green")
        print "R:", R
        print "T:", T
        # camera form ...
        # self.camera = CameraModel.load_camera_from_ROS_tf(translation=T, rotation=R)
        print termcolor.colored("calibrated", "green")

    def render_beachball(self, *args):
        pass

    @property
    def p2pcount(self):
        out = [cor for cor in self.p2p if not any(np.isnan(cor))]
        return len(out)

    def __str__(self):
        return self.vp.name

    def reprojection_error(self):
        pass

class ExtrinsicsWidget(Gtk.VBox):

    _VIEWPORTNAME = 0
    _TEXTURE_U = 1
    _TEXTURE_V = 2
    _DISPLAY_X = 3
    _DISPLAY_Y = 4
    _SHOW_POINT = 5

    def __init__(self):
        Gtk.VBox.__init__(self)

        ui_file_contents = pkgutil.get_data('flyvr.calibration.gui','pinhole-wizard.ui')
        ui = Gtk.Builder()
        ui.add_from_string( ui_file_contents )

        grid = ui.get_object('corresponding_points_grid')
        self.pack_start(grid, True, True, 0)

        self._dsc_connected = False
        self._dsc = None

        self._cam_intrinsics = False
        self._cam = None

        self._color = (255, 255, 255)

        # Setup viewport treeview
        treeview_generator = ClassStoreTreeViewGenerator()
        treeview_generator.add_column_text("Viewport", attr="viewportname", editable=False)
        treeview_generator.add_column_number("p2p", attr="p2pcount", fmt="%d", editable=False)
        treeview_generator.add_column_button("calibrate", "run", on_click=self.calibrate,
                                             show_waiting_indicator=True)
        treeview_generator.add_column_checkbox("calibrated", attr="calibrated",
                                               editable=False)
        treeview_generator.add_column_checkbox("show beachball", attr="render_beachball",
                                               editable=True, on_edit="render_beachball")

        # add viewport treeview and buttons
        hbox = ui.get_object('vdisp_treeview_box')

        self.vdisp_treeview = treeview_generator.get_treeview()
        self.vdisp_treeview.set_size_request(-1, 50)
        self.vdisp_store = treeview_generator.get_liststore()
        hbox.pack_start(self.vdisp_treeview, True, True, 0)

        self.calibration_cb = Gtk.ComboBoxText()
        for i, k in enumerate(sorted(ExtrinsicsAlgorithms)):
            self.calibration_cb.append(k, k)
            if i == 0:
                self.calibration_cb.set_active_id(k)
        hbox.pack_end(self.calibration_cb, False, False, 0)


        # configure point 2 point treeview
        self.point_store = Gtk.ListStore(str, float, float, float, float, bool)

        self.point_store.connect("row-changed",  self.on_points_updated)
        self.point_store.connect("row-inserted", self.on_points_updated)
        self.point_store.connect("row-deleted",  self.on_points_updated)

        self.p2p_treeview = p2p_treeview = ui.get_object('treeview1')
        p2p_treeview.set_model(self.point_store)

        renderer_viewport_name = Gtk.CellRendererCombo(model=self.vdisp_store, text_column=0, editable=True)
        renderer_viewport_name.connect("edited", self.on_edit_p2p_viewport_name, self._VIEWPORTNAME)
        column = Gtk.TreeViewColumn("viewport", renderer_viewport_name, text=self._VIEWPORTNAME)
        column.set_sort_column_id(self._VIEWPORTNAME)
        p2p_treeview.append_column(column)

        renderer_text = Gtk.CellRendererText(editable=True)
        renderer_text.connect("edited", self.on_edit_cell, self._TEXTURE_U)
        column = Gtk.TreeViewColumn("texture U", renderer_text, text=self._TEXTURE_U)
        column.set_cell_data_func(renderer_text, self._float_fmt, func_data=self._TEXTURE_U)
        column.set_sort_column_id(self._TEXTURE_U)
        p2p_treeview.append_column(column)

        renderer_text = Gtk.CellRendererText(editable=True)
        renderer_text.connect("edited", self.on_edit_cell, self._TEXTURE_V)
        column = Gtk.TreeViewColumn("texture V", renderer_text, text=self._TEXTURE_V)
        column.set_cell_data_func(renderer_text, self._float_fmt, func_data=self._TEXTURE_V)
        column.set_sort_column_id(self._TEXTURE_V)
        p2p_treeview.append_column(column)

        renderer_text = Gtk.CellRendererText(editable=True)
        renderer_text.connect("edited", self.on_edit_cell, self._DISPLAY_X)
        column = Gtk.TreeViewColumn("display X", renderer_text, text=self._DISPLAY_X)
        column.set_cell_data_func(renderer_text, self._float_fmt, func_data=self._DISPLAY_X)
        column.set_sort_column_id(self._DISPLAY_X)
        p2p_treeview.append_column(column)

        renderer_text = Gtk.CellRendererText(editable=True)
        renderer_text.connect("edited", self.on_edit_cell, self._DISPLAY_Y)
        column = Gtk.TreeViewColumn("display Y", renderer_text, text=self._DISPLAY_Y)
        column.set_cell_data_func(renderer_text, self._float_fmt, func_data=self._DISPLAY_Y)
        column.set_sort_column_id(self._DISPLAY_Y)
        p2p_treeview.append_column(column)


        renderer_toggle = Gtk.CellRendererToggle()
        renderer_toggle.connect("toggled", self.on_toggle_point_show)
        column = Gtk.TreeViewColumn("show", renderer_toggle, active=self._SHOW_POINT)
        column.set_sort_column_id(self._SHOW_POINT)
        p2p_treeview.append_column(column)

        # connect treeview buttons ---------------------------
        ui.get_object('UV_add_single').connect('clicked', self.on_add_UV_single)
        ui.get_object('UV_add_button').connect('clicked', self.on_add_UV)
        ui.get_object('UV_remove_button').connect('clicked', self.on_remove_UV)
        ui.get_object('show_all_button').connect('clicked', self.on_show_all_button, True)
        ui.get_object('show_none_button').connect('clicked', self.on_show_all_button, False)

        self.geom_cb = ui.get_object('geometry_combobox')
        self.geom_cb.append("Sphere", "Sphere")
        self.geom_cb.append("Cylinder", "Cylinder")

        self.add_dialog = Gtk.Dialog(title="Add texture UV grid",
                                  buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK,
                                           Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL),
                                  )
        adbox = self.add_dialog.get_content_area()
        adbox.add(ui.get_object("add_dialog_grid"))
        self.add_dialog_entries = {
                'u': {
                    'from': ui.get_object('sp_u_from'),
                    'to': ui.get_object('sp_u_to'),
                    'steps': ui.get_object('sp_u_steps'),
                    'ep': ui.get_object('cb_u_endpoint'),
                    },
                'v': {
                    'from': ui.get_object('sp_v_from'),
                    'to': ui.get_object('sp_v_to'),
                    'steps': ui.get_object('sp_v_steps'),
                    'ep': ui.get_object('cb_v_endpoint'),
                    }
                }
        self.add_dialog_entries['u']['from'].set_adjustment(Gtk.Adjustment(0.000, 0.000, 1.000, 0.001))
        self.add_dialog_entries['v']['from'].set_adjustment(Gtk.Adjustment(0.000, 0.000, 1.000, 0.001))
        self.add_dialog_entries['u']['to'].set_adjustment(Gtk.Adjustment(1.000, 0.000, 1.000, 0.001))
        self.add_dialog_entries['v']['to'].set_adjustment(Gtk.Adjustment(1.000, 0.000, 1.000, 0.001))
        self.add_dialog_entries['u']['steps'].set_adjustment(Gtk.Adjustment(1, 1, 100, 1))
        self.add_dialog_entries['v']['steps'].set_adjustment(Gtk.Adjustment(1, 1, 100, 1))

        self.cb_viewports = ui.get_object("cb_viewports")


    def on_edit_p2p_viewport_name(self, widget, path, textval, colnum):
        self.point_store[path][colnum] = textval
        self.update_bg_image()

    def _float_fmt(self, treeviewcolumn, cell, model, iter, column):
        float_in = model.get_value(iter, column)
        cell.set_property('text', '%g' % float_in)

    def on_edit_cell(self, widget, path, textval, colnum):
        value = float(textval)
        self.point_store[path][colnum] = value
        self.update_bg_image()

    def on_toggle_point_show(self, widget, path):
        self.point_store[path][self._SHOW_POINT] = not self.point_store[path][self._SHOW_POINT]
        self.update_bg_image()

    def on_show_all_button(self, *args):
        val = args[-1]
        for row in self.point_store:
            row[self._SHOW_POINT] = val
        self.update_bg_image()

    def _add_p2p(self, viewport, texU, texV, displayX, displayY, show_point=True):
        self.point_store.append([viewport, texU, texV, displayX, displayY, show_point])
        self.update_bg_image()

    def on_add_UV_single(self, button):
        vp = self.cb_viewports.get_active_text()
        self._add_p2p(vp, np.nan, np.nan, np.nan, np.nan)

    def on_add_UV(self, button):
        try:
            response = self.add_dialog.run()
            if response == Gtk.ResponseType.OK:
                u_from = self.add_dialog_entries['u']['from'].get_value()
                v_from = self.add_dialog_entries['v']['from'].get_value()
                u_to = self.add_dialog_entries['u']['to'].get_value()
                v_to = self.add_dialog_entries['v']['to'].get_value()
                u_steps = self.add_dialog_entries['u']['steps'].get_value()
                v_steps = self.add_dialog_entries['v']['steps'].get_value()
                u_ep = self.add_dialog_entries['u']['ep'].get_active()
                v_ep = self.add_dialog_entries['v']['ep'].get_active()
                Us = np.linspace(u_from, u_to, u_steps, endpoint=u_ep)
                Vs = np.linspace(v_from, v_to, v_steps, endpoint=v_ep)
                vp = self.cb_viewports.get_active_text()
                for u, v in itertools.product(Us, Vs):
                    self._add_p2p(vp, u, v, np.nan, np.nan)
        finally:
            self.add_dialog.hide()

    def on_remove_UV(self, button):
        selection = self.p2p_treeview.get_selection()
        sel = selection.get_selected()
        if not sel[1] == None:
            self.point_store.remove( sel[1] )

    def calibrate(self, viewportcal):
        try:
            # geom = self.geom_cb.get_active_text()
            P2P = self.get_p2p_from_viewportname(viewportcal.viewportname)
            p2parr = np.array(P2P)
            uv = p2parr[:,0:2]
            xy = p2parr[:,2:4]
            # GEOMFIX
            XYZ = self.geom.model.texcoord2worldcoord(uv)
            method = self.calibration_cb.get_active_text()
        except:
            print "Calibration ERROR"
        else:
            viewportcal.calibrate(self._cam, method, xy, XYZ)

    def on_viewports_saved(self, viewportwidget, viewports):
        self.vdisp_store.clear()
        self.cb_viewports.remove_all()
        for i, vp in enumerate([v for v in viewports if len(v.nodes) >= 3]):
            self.vdisp_store.append([ViewportExtrinsicCalibration(vp)])
            self.cb_viewports.append(vp.name, vp.name)
            if i == 0:
                self.cb_viewports.set_active(0)
        self.on_points_updated(update_display=False)

    def on_joystick_button(self, joywidget, button):
        if button == "accept":
            position = tuple(joywidget._position)
            selection = self.p2p_treeview.get_selection()
            _, paths = selection.get_selected_rows()
            try:
                path = paths[0]
            except IndexError:
                pass
            else:
                self.point_store[path][self._DISPLAY_X] = position[0]
                self.point_store[path][self._DISPLAY_Y] = position[1]
                idx = path.get_indices()[0]
                self.p2p_treeview.set_cursor(idx+1)
        elif button == "remove":
            selection = self.p2p_treeview.get_selection()
            _, paths = selection.get_selected_rows()
            try:
                path = paths[0]
            except IndexError:
                pass
            else:
                self.point_store[path][self._DISPLAY_X] = float('nan')
                self.point_store[path][self._DISPLAY_Y] = float('nan')
        elif button == "nextitem":
            selection = self.p2p_treeview.get_selection()
            _, paths = selection.get_selected_rows()
            try:
                path = paths[0]
            except IndexError:
                self.p2p_treeview.set_cursor(0)
            else:
                idx = path.get_indices()[0]
                self.p2p_treeview.set_cursor(idx+1)
        elif button == "previtem":
            selection = self.p2p_treeview.get_selection()
            _, paths = selection.get_selected_rows()
            try:
                path = paths[0]
            except IndexError:
                self.p2p_treeview.set_cursor(0)
            else:
                idx = path.get_indices()[0]
                self.p2p_treeview.set_cursor(max(0, idx-1))

    def on_points_updated(self, *args, **kwargs):
        VPS = { vp.viewportname:vp for (vp,) in self.vdisp_store }
        P2P = collections.defaultdict(list)
        for row in self.point_store:
            vpname, u, v, x, y, show = row
            P2P[vpname].append(((u,v,x,y)))
        for key, vpc in VPS.items():
            vpc.p2p = P2P[key]
        if kwargs.get('update_display', True):
            self.update_bg_image()


    def get_p2p_from_viewportname(self, viewportname):
        P2P = []
        for row in self.point_store:
            vpname, u, v, x, y, show = row
            if vpname == viewportname:
                if show:
                    P2P.append((u,v,x,y))
        return P2P

    def extrinsics_as_list(self):
        OUT = []
        for row in self.point_store:
            viewport, texU, texV, displayX, displayY, show_point = row
            point = {
                'viewport': viewport,
                'u': texU,
                'v': texV,
                'x': displayX,
                'y': displayY
               }
            OUT.append(point)
        return OUT

    def extrinsics_from_list(self, data):
        self.point_store.clear()
        for d in data:
            p = [d['viewport'], d['u'], d['v'], d['x'], d['y'], True]
            self.point_store.append(p)

    def on_displayclient_connect(self, calling_widget, dsc, load_config):
        self._dsc = calling_widget
        self._dsc_connected = True
        # ACTIVATE BUTTONS
        self.geom = simple_geom.Geometry(geom_dict=self._dsc.get_geometry_info())

    def on_displayclient_disconnect(self, *args):
        self._dsc_connected = False
        # DEACTIVATE BUTTONS

    def on_intrinsics_computed(self, intrinsicswidget, intrinsics_yaml):
        print "intrinsics:"
        # print intrinsics_yaml
        self._cam = CameraModel.from_dict(intrinsics_yaml, extrinsics_required=False)
        self._cam_intrinsics = True
        print self._cam

    def update_bg_image(self, *args):

        if not self._dsc_connected:
            return

        # PREPARE SCREEN
        self._dsc._draw_prepare((0, 0, 0))
        self._dsc_arr = self._dsc._draw_array

        # CHECK VIEWPORTS
        for row in self.vdisp_store:
            viewport = row[0]
            if viewport.calibrated and viewport.show_beachball:
                # IF BEACHBALL
                cam = viewport.camera
                # generate an array which contains the uv coordinates for each pixel
                uv_arr = self.geom.compute_for_camera_view(cam, what='texture_coords')
                # use this array to calculate the beachball coloring
                bb_arr = simple_geom.tcs_to_beachball(uv_arr)
                    #u = farr[:,:,0]
                    #good = ~np.isnan( u )
                    #arr2 = simple_geom.tcs_to_beachball(farr)
                mask = np.zeros(bb_arr.shape[:2], dtype=np.uint8)
                fill_polygon.fill_polygon(viewport.viewport.to_list(), mask)
                if np.max(mask)==0: # no mask
                    mask += 1

                self._dsc_arr += mask[:,:,np.newaxis] * bb_arr
            else:
                # IF NO BEACHBALL
                for (_, _, x, y) in self.get_p2p_from_viewportname(viewport.viewportname):
                    if not np.isnan(x) and not np.isnan(y):
                        self._dsc_arr[y-2:y+2,x-2:x+2,:] = self._color

        self._dsc.show_pixels(self._dsc_arr)

