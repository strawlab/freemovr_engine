#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil -*-
import os
import collections

import numpy as np
import pkgutil

try:
    import roslib
except ImportError:
    warnings.warn("Can't import roslib. Make sure that flyvr python package is in your PYTHONPATH.")
else:
    roslib.load_manifest("flyvr")
    roslib.load_manifest('visualization_msgs')
    roslib.load_manifest('camera_calibration')

import flyvr.simple_geom as simple_geom
#import flyvr.dlt as dlt
import flyvr.tools.fill_polygon as fill_polygon
#from flyvr.fit_extrinsics import fit_extrinsics, fit_extrinsics_iterative

from gi.repository import Gtk, GObject

class CellRendererButton(Gtk.CellRenderer):

    __gsignals__ = {
        'clicked': (GObject.SignalFlags.RUN_FIRST,
                    None,
                    (GObject.TYPE_STRING,))
    }

    text = GObject.property(type=str, default='')

    def __init__(self, text=''):
        GObject.GObject.__init__(self)
        self._xpad = 6
        self._ypad = 2
        self.props.mode = Gtk.CellRendererMode.ACTIVATABLE
        self.props.text = text

    def do_get_size (self, widget, *args):
        context = widget.get_pango_context()
        metrics = context.get_metrics(widget.get_style().font_desc, 
                                      context.get_language())
        row_height = metrics.get_ascent() + metrics.get_descent()
        height = (row_height + 512 >> 10) + self._ypad * 2

        layout = widget.create_pango_layout(self.props.text)
        (row_width, layout_height) = layout.get_pixel_size()
        width = row_width + self._xpad * 2
        return (0, 0, width, height)

    def do_render(self, cr, widget, background_area, cell_area, flags):
        style = widget.get_style()
        state = widget.get_state()

        layout = widget.create_pango_layout(self.props.text)
        (layout_width, layout_height) = layout.get_pixel_size()
        layout_xoffset = (cell_area.width - layout_width) / 2 + cell_area.x
        layout_yoffset = (cell_area.height - layout_height) / 2 + cell_area.y

        Gtk.paint_box(style, cr, state, Gtk.ShadowType.OUT,
                               widget, 'button',
                               cell_area.x, cell_area.y,
                               cell_area.width, cell_area.height)
        Gtk.paint_layout(style, cr, state, True, 
                                  widget, "cellrenderertext", layout_xoffset,
                                  layout_yoffset, layout)

    def do_activate(self, event, widget, path, background_area, cell_area, flags):
        self.emit('clicked', path)

GObject.type_register(CellRendererButton)

EXTRINSIC_CALIBRATION_METHODS = [
    'extrinsic only',
    'iterative extrinsic only',
    'DLT',
    #'RANSAC DLT',
    ]
FULLSCREEN='FULL_SCREEN'




def nice_float_fmt(treeviewcolumn, cell, model, iter, column):
    float_in = model.get_value(iter, column)
    cell.set_property('text', '%g'%float_in )

def get_extrinsics_grid(dsc):
    ui_file_contents = pkgutil.get_data('flyvr.calibration.gui','pinhole-wizard.ui')
    _ui = Gtk.Builder()
    _ui.add_from_string( ui_file_contents )

    grid = _ui.get_object('corresponding_points_grid')

    # setup vdisp combobox ----------------
    vdisp_store = Gtk.ListStore(str,int,bool,int,float,bool,object,bool)

    # columns in self.vdisp_store
    VS_VDISP = 0
    VS_COUNT = 1
    VS_CALIBRATION_RUNNING = 2
    VS_CALIBRATION_PROGRESS = 3
    VS_MRE = 4
    VS_SHOW_BEACHBALL = 5
    VS_CAMERA_OBJECT = 6
    VS_PUBLISH_RVIZ = 7
    # create vdisp treeview -----------------------

    treeview = _ui.get_object('vdisp_treeview')
    treeview.set_model(vdisp_store)

    renderer = Gtk.CellRendererText()
    column = Gtk.TreeViewColumn("virtual display", renderer, text=VS_VDISP)
    column.set_sort_column_id(VS_VDISP)
    treeview.append_column(column)

    renderer = Gtk.CellRendererText()
    column = Gtk.TreeViewColumn("count", renderer, text=VS_COUNT)
    column.set_sort_column_id(VS_COUNT)
    treeview.append_column(column)

    def _pulse_spinner(*args):
        if vdisp_store:
            for row in vdisp_store:
                row[VS_CALIBRATION_PROGRESS] += 1
        return True

    def launch_calibration(method, vdisp):

        def _pump_ui():
            if os.environ.get('RUNNING_NOSE') != '1':
                Gtk.main_iteration_do(False) #run once, no block
                while Gtk.events_pending():  #run remaining
                    Gtk.main_iteration()

        orig_data = []
        for row in point_store:
            if row[VDISP]==vdisp:
                orig_data.append( [ row[TEXU], row[TEXV], row[DISPLAYX], row[DISPLAYY] ] )
        orig_data = np.array(orig_data)
        uv = orig_data[:,:2]
        # FIXME:
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

        for row in vdisp_store:
            if row[VS_VDISP]==vdisp:
                row[VS_MRE] = mre
                row[VS_CAMERA_OBJECT] = camera

        _pump_ui()
        update_bg_image()

    def on_trigger_cal(widget, path):
        vdisp = vdisp_store[path][VS_VDISP]
        method = _ui.get_object('cal_method_cbtext').get_active_text()
        vdisp_store[path][VS_CALIBRATION_RUNNING] = True
        launch_calibration( method, vdisp )
        vdisp_store[path][VS_CALIBRATION_RUNNING] = False

    renderer = CellRendererButton('Calibrate')
    renderer.connect("clicked", on_trigger_cal)
    column = Gtk.TreeViewColumn("calibration")
    column.pack_start(renderer, False)
    renderer = Gtk.CellRendererSpinner()
    column.pack_start(renderer, False)
    column.add_attribute(renderer, "active", VS_CALIBRATION_RUNNING)
    column.add_attribute(renderer, "pulse", VS_CALIBRATION_PROGRESS)
    treeview.append_column(column)
    GObject.timeout_add(100, _pulse_spinner)

    renderer = Gtk.CellRendererText()
    column = Gtk.TreeViewColumn("mean reproj. error", renderer, text=VS_MRE)
    column.set_sort_column_id(VS_MRE)
    treeview.append_column(column)

    def on_toggle_show_beachball(widget, path):
        vdisp_store[path][VS_SHOW_BEACHBALL] = not vdisp_store[path][VS_SHOW_BEACHBALL]
        update_bg_image()

    renderer = Gtk.CellRendererToggle()
    renderer.connect("toggled", on_toggle_show_beachball)
    column = Gtk.TreeViewColumn("show beachball", renderer, active=VS_SHOW_BEACHBALL)
    treeview.append_column(column)

    def on_toggle_publish_rviz(widget, path):
        vdisp_store[path][VS_PUBLISH_RVIZ] = not vdisp_store[path][VS_PUBLISH_RVIZ]
        # publish_rviz()

    renderer = Gtk.CellRendererToggle()
    renderer.connect("toggled", on_toggle_publish_rviz)
    column = Gtk.TreeViewColumn("publish RViz cam", renderer, active=VS_PUBLISH_RVIZ)
    treeview.append_column(column)

    # create point treeview -----------------------
#
    point_store = Gtk.ListStore(str, float, float, float, float, bool, bool)

    treeview = _ui.get_object('treeview1')
    treeview.set_model(point_store)

    # columns in self.point_store
    VDISP=0
    TEXU=1
    TEXV=2
    DISPLAYX=3
    DISPLAYY=4
    SHOWPT=5
    JOYLISTEN=6

    renderer_text = Gtk.CellRendererCombo(model=vdisp_store,
                                          text_column=VDISP,
                                          editable=True)

    def on_edit_vdisp(widget,path,textval,colnum):
        point_store[path][colnum] = textval
        update_bg_image()

    renderer_text.connect("edited", on_edit_vdisp, VDISP)
    column = Gtk.TreeViewColumn("virtual display", renderer_text, text=VDISP)
    column.set_sort_column_id(VDISP)
    treeview.append_column(column)


    def on_edit_cell(widget,path,textval,colnum):
        value = float(textval)
        point_store[path][colnum] = value
        update_bg_image()

    renderer_text = Gtk.CellRendererText(editable=True)
    renderer_text.connect("edited", on_edit_cell, TEXU)
    column = Gtk.TreeViewColumn("texture U", renderer_text, text=TEXU)
    column.set_cell_data_func(renderer_text, nice_float_fmt, func_data=TEXU)
    column.set_sort_column_id(TEXU)
    treeview.append_column(column)

    renderer_text = Gtk.CellRendererText(editable=True)
    renderer_text.connect("edited", on_edit_cell, TEXV)
    column = Gtk.TreeViewColumn("texture V", renderer_text, text=TEXV)
    column.set_cell_data_func(renderer_text, nice_float_fmt, func_data=TEXV)
    column.set_sort_column_id(TEXV)
    treeview.append_column(column)

    renderer_text = Gtk.CellRendererText(editable=True)
    renderer_text.connect("edited", on_edit_cell, DISPLAYX)
    column = Gtk.TreeViewColumn("display X", renderer_text, text=DISPLAYX)
    column.set_cell_data_func(renderer_text, nice_float_fmt, func_data=DISPLAYX)
    column.set_sort_column_id(DISPLAYX)
    treeview.append_column(column)

    renderer_text = Gtk.CellRendererText(editable=True)
    renderer_text.connect("edited", on_edit_cell, DISPLAYY)
    column = Gtk.TreeViewColumn("display Y", renderer_text, text=DISPLAYY)
    column.set_cell_data_func(renderer_text, nice_float_fmt, func_data=DISPLAYY)
    column.set_sort_column_id(DISPLAYY)
    treeview.append_column(column)

    def on_toggle_point_show(widget, path):
        point_store[path][SHOWPT] = not point_store[path][SHOWPT]
        update_bg_image()

    renderer_toggle = Gtk.CellRendererToggle()
    renderer_toggle.connect("toggled", on_toggle_point_show)
    column = Gtk.TreeViewColumn("show", renderer_toggle, active=SHOWPT)
    column.set_sort_column_id(SHOWPT)
    treeview.append_column(column)

    def on_do_point(widget, path):
        selected_path = Gtk.TreePath(path)
        # perform mutually-exclusive radio button setting
        for row in point_store:
            row[JOYLISTEN] = (row.path == selected_path)

    renderer_pixbuf = Gtk.CellRendererToggle()
    renderer_pixbuf.set_radio(True)
    renderer_pixbuf.connect("toggled", on_do_point)
    column = Gtk.TreeViewColumn('Joystick select', renderer_pixbuf, active=JOYLISTEN)
    treeview.append_column(column)

    def on_points_updated(*args):
        vdisps = collections.defaultdict(int)
        for row in point_store:
            vdisps[ row[VDISP] ] += 1

        for row in vdisp_store:
            vdisp = row[VS_VDISP]
            row[VS_COUNT] = vdisps[ vdisp ]

    point_store.connect("row-changed",  on_points_updated)
    point_store.connect("row-inserted", on_points_updated)
    point_store.connect("row-deleted",  on_points_updated)

    def _get_default_vdisp():
        di = dsc.get_display_info()
        if 'virtualDisplays' in di:
            val = di['virtualDisplays'][0]['id']
        else:
            val = FULLSCREEN
        return val

    def _add_pt_entry(vdisp, texU, texV,
                      displayX=float(np.nan), displayY=float(np.nan),
                      show_point=True, joy_listen=False):
        point_store.append([vdisp, texU, texV,
                               displayX, displayY,
                               show_point, joy_listen])
    def on_add_UV(button):
        vdisp = _get_default_vdisp()
        _add_pt_entry(vdisp, np.nan, np.nan )
        update_bg_image()

    def on_remove_UV(button):
        treeview = _ui.get_object('treeview1')
        selection = treeview.get_selection()
        sel = selection.get_selected()
        if not sel[1] == None:
            point_store.remove( sel[1] )

    # connect treeview buttons ---------------------------
    _ui.get_object('UV_add_button').connect('clicked', on_add_UV)
    _ui.get_object('UV_remove_button').connect('clicked', on_remove_UV)

    def on_show_all_button(*args):
        val = args[-1]
        for row in point_store:
            row[SHOWPT] = val
        update_bg_image()

    _ui.get_object('show_all_button').connect('clicked', on_show_all_button, True)
    _ui.get_object('show_none_button').connect('clicked', on_show_all_button, False)

    # setup ComboBoxText
    cal_method_cbtext = _ui.get_object('cal_method_cbtext')

    for method in EXTRINSIC_CALIBRATION_METHODS:
        cal_method_cbtext.append(method,method)
    cal_method_cbtext.set_active_id(EXTRINSIC_CALIBRATION_METHODS[0])


    def update_bg_image():
        arr = np.zeros( (dsc.height, dsc.width, 3), dtype=np.uint8 )
        di = dsc.get_display_info()
        # draw beachballs ----------------------

        for row in vdisp_store:
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

            # FIXME
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
        for row in point_store:
            if not row[SHOWPT]:
                continue
            xf,yf = row[DISPLAYX], row[DISPLAYY]
            try:
                x = int(np.round(xf))
                y = int(np.round(yf))
            except ValueError:
                # Likely cannot convert nan to integer.
                continue
            if 0 <= x and x < dsc.width:
                if 0 <= y and y < dsc.height:
                    arr[y,x] = 255
                    showing.append( (x,y) )
        dsc.show_pixels(arr)
        return arr

    return grid

