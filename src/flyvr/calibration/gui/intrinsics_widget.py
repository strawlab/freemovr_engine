#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil -*-
import math
import pkgutil
import datetime
from gi.repository import Gtk

import roslib
roslib.load_manifest("flyvr")
from flyvr.calibration.intrinsics import (helper_make_corners_boards,
                                          intrinsics_from_checkerboards)

DEFAULT_CHECKER_NROWS = 6
DEFAULT_CHECKER_NCOLS = 8
DEFAULT_CHECKER_SIZE = 0.015



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

        #row
        for m,_y in enumerate(range(0,H*SZ,SZ)):
            #col
            for n,_x in enumerate(range(0,W*SZ,SZ)):
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
    def __init__(self, ui, nrows, ncols, size, **kwargs):
        Gtk.Dialog.__init__(self, title="Add checkerboard",
                                  buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK,
                                           Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL),
                                  **kwargs)
        self.get_content_area().add(ui.get_object('add_CK_dialog_grid'))
        self._npts_lbl = ui.get_object('N_CKB_points_label')
        self._checker = CheckerboardPlotWidget()
        ui.get_object('checkerboard_plot_box').pack_start(self._checker, True, True, 0)

        self._sze = ui.get_object('CK_size_entry')
        self._nr = ui.get_object('CK_n_rows_spinbutton')
        self._nc = ui.get_object('CK_n_cols_spinbutton')

        self.set_board_size(nrows,ncols,size)

        #connect the spinbuttons to change the checkerboard
        self._nr.connect('value-changed',
                lambda sbr: self._checker.set_board_size_num_corners(sbr.get_value(), None))
        self._nc.connect('value-changed',
                lambda sbc: self._checker.set_board_size_num_corners(None, sbc.get_value()))

        self._dsc = None
        self._dsc_pts = None

    def set_board_size(self, nrows, ncols, size):
        self._nr.set_value(nrows)
        self._nc.set_value(ncols)
        self._sze.set_text(str(size))
        self._checker.set_board_size_num_corners(nrows,ncols)

    def get_num_rows(self):
        return int(self._nr.get_value())
    def get_num_cols(self):
        return int(self._nc.get_value())
    def get_size(self):
        return float(self._sze.get_text())

    def add_point(self, n, pt):
        self._npts_lbl.set_text('%d' % n)
        self._checker.set_next_point(n+1)

        if pt is not None and self._dsc is not None:
            x,y = pt
            self._dsc_arr[y-2:y+2,x-2:x+2,:] = self._dsc.IMAGE_COLOR_WHITE
            self._dsc.show_pixels(self._dsc_arr)

    def run(self, dsc):
        self.add_point(0, None)
        self._dsc_pts = []

        if dsc.proxy_is_connected():
            self._dsc = dsc
            self._dsc_arr = self._dsc.new_image(self._dsc.IMAGE_COLOR_BLACK)
        else:
            self._dsc = None

        self.show_all()
        resp = Gtk.Dialog.run(self)

        if dsc.proxy_is_connected():
            self._dsc.show_pixels(self._dsc.new_image(self._dsc.IMAGE_COLOR_BLACK))

        return resp


def get_intrinsics_grid(displayclient, on_intrinsics_computed):

    # Get grid container from ui file
    ui_file_contents = pkgutil.get_data('flyvr.calibration.gui', 'pinhole-wizard.ui')
    ui = Gtk.Builder()
    ui.add_from_string(ui_file_contents)
    grid = ui.get_object('checkerboard_grid')

    # Get treeview for configuration
    checkerboard_store = Gtk.ListStore(object)
    treeview = ui.get_object('checkerboard_treeview')
    treeview.set_model(checkerboard_store)

    def render_checkerboard_row(self, treeviewcolumn, cell, model, iter, attr):
        # The row rendering function
        rowdict = model.get_value(iter, 0)
        cell.set_property('text', str(rowdict[attr]))

    # configure cell renderes
    renderer = Gtk.CellRendererText()
    column = Gtk.TreeViewColumn("rows", renderer)
    column.set_cell_data_func(renderer, render_checkerboard_row, func_data='rows')
    treeview.append_column(column)

    renderer = Gtk.CellRendererText()
    column = Gtk.TreeViewColumn("columns", renderer)
    column.set_cell_data_func(renderer, render_checkerboard_row, func_data='columns')
    treeview.append_column(column)

    renderer = Gtk.CellRendererText()
    column = Gtk.TreeViewColumn("size", renderer)
    column.set_cell_data_func(renderer, render_checkerboard_row, func_data='size')
    treeview.append_column(column)

    renderer = Gtk.CellRendererText()
    column = Gtk.TreeViewColumn("time", renderer)
    column.set_cell_data_func(renderer, render_checkerboard_row, func_data='date_string')
    treeview.append_column(column)

    # Remove entries function
    def on_CK_remove(*args):
        selection = treeview.get_selection()
        sel = selection.get_selected()
        if not sel[1] == None:
            checkerboard_store.remove(sel[1])

    ui.get_object('CK_remove_button').connect('clicked', on_CK_remove)

    # setup checkerboard dialog ----------------
    add_CK_dialog = AddCheckerboardDialog(ui,
                            nrows=DEFAULT_CHECKER_NROWS,
                            ncols=DEFAULT_CHECKER_NCOLS,
                            size=DEFAULT_CHECKER_SIZE)

    _current_checkerboard = {'points': []}

    def on_CK_add(*args):
        _current_checkerboard = {'points': []}
        try:
            #reset current number of collected points, and current point
            response = add_CK_dialog.run(displayclient)
            if response == Gtk.ResponseType.OK:
                _current_checkerboard['rows'] = add_CK_dialog.get_num_rows()
                _current_checkerboard['columns'] = add_CK_dialog.get_num_cols()
                _current_checkerboard['size'] = add_CK_dialog.get_size()
                nowstr = datetime.datetime.now().isoformat(' ')
                _current_checkerboard['date_string'] = nowstr
                checkerboard_store.append( [_current_checkerboard] )
        finally:
            _current_checkerboard = {'points': []}
            add_CK_dialog.hide()

    ui.get_object('CK_add_button').connect('clicked', on_CK_add)

    def on_joystick_button(buttons, axis, position):
        if add_CK_dialog.is_focus():
            if buttons["accept"]:
                _current_checkerboard["points"].append(position)
                npts = len(_current_checkerboard["points"])
                add_CK_dialog.add_point(npts, position)

    # Monkey patch grid
    grid.on_joystick_button = on_joystick_button

    def on_compute_intrinsics(*args):
        # We have to convert the rows to the right type for intrinsics calibration
        cornerss, rowss, colss, dims = [], [], [], []
        for row in checkerboard_store:
            r = row[0]
            rowss.append(r['rows'])
            colss.append(r['columns'])
            dims.append(r['size'])
            cornerss.append(list(r['points']))

        corners_boards = helper_make_corners_boards(cornerss, rowss, colss, dims)
        intrinsics_yaml = intrinsics_from_checkerboards(corners_boards, displayclient.width,
                                                                        displayclient.height)
        on_intrinsics_computed(intrinsics_yaml)


    ui.get_object('compute_intrinsics').connect('clicked', on_compute_intrinsics)


    return grid

