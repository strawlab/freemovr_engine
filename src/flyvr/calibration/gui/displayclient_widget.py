import os
import tempfile
from gi.repository import Gtk, GObject

import numpy as np
import scipy.misc

import roslib
roslib.load_manifest("flyvr")
import flyvr.tools.fill_polygon

import time

import platform
import traceback
import pkgutil

# Platform specific import switch
#=================================
# On windows we don't want to depend on ROS functionality.
#
# the required functionality on the DisplayServerProxy side is:
# 
# flyvr.display_client.DisplayServerProxy
#  def enter_2dblit_mode
#  def show_pixels(arr)
#  def get_display_info()
#  def get_geometry_info()

if platform.system() == "Windows":
    # FIXME !!!!
    raise NotImplementedError("Implement Windows ROSFreeDisplayServerProxy first!")
    # import flyvr.rosfree.display_client
    # DEFAULT_DISPLAY_SERVER_NAME = "localhost:1701"
    # def get_display_server_proxy(name):
    #    return flyvr.rosfree.display_client.ROSFreeDisplayServerProxy(name)
    #
    # 
else:
    import flyvr.display_client
    DEFAULT_DISPLAY_SERVER_NAME = "/display_server"
    def get_display_server_proxy(name):
        return flyvr.display_client.DisplayServerProxy(name)

    import rospy
    import geometry_msgs.msg
    import flyvr.msg

    sprite_pub = rospy.Publisher('sprite_image', flyvr.msg.FlyVRCompressedImage, latch=True)
    position_pub = rospy.Publisher('sprite_pose', geometry_msgs.msg.Pose2D)

    def set_sprite_position(x, y):
        pose2d = geometry_msgs.msg.Pose2D()
        pose2d.x = x
        pose2d.y = y
        position_pub.publish(pose2d)

    def set_sprite_visible(state):
        _flyvr_dir = roslib.packages.get_pkg_dir('flyvr')
        if bool(state):
            fname = os.path.join(_flyvr_dir, "data", "cursor.png")
        else:
            fname = os.path.join(_flyvr_dir, "data", "cursorblank.png")
        image = flyvr.msg.FlyVRCompressedImage()
        image.format = os.path.splitext(fname)[-1]
        image.data = open(fname).read()
        sprite_pub.publish(image)

            # > "displayclient-connect"
            # > "displayclient-disconnect"
            # < on_position_change                  OK

class DisplayClientWidget(Gtk.VBox):

    __gsignals__ = {
            "displayclient-connect": (GObject.SignalFlags.RUN_FIRST, None, [object, bool]),
            "displayclient-disconnect": (GObject.SignalFlags.RUN_FIRST, None, []),
    }

    def __init__(self, interval_ms=1000, default_displayserver="/display_server0"):
        """This class encapsulates the communication with the display server

        And allows the pinhole_wizard to be run without a running display server
        """
        Gtk.VBox.__init__(self)

        # embed the diplay grid
        ui_file_contents = pkgutil.get_data('flyvr.calibration.gui', 'pinhole-wizard.ui')
        ui = Gtk.Builder()
        ui.add_from_string(ui_file_contents)
        grid = ui.get_object('connection_status_grid')
        self.pack_start(grid, True, True, 0)

        # UI elements
        self._dsc_name_entry = ui.get_object('display_server_name_entry')
        self._dsc_connect_btn = ui.get_object('connect_to_display_server_button')
        self._dsc_disconnect_btn = ui.get_object('disconnect_from_display_server_button')
        self._dsc_populate_cb = ui.get_object('display_server_populate_cb')
        self._dsc_status_img = ui.get_object('display_server_status_img')

        self._dsc_connect_btn.connect("clicked", self.on_connect_to_display_server)
        self._dsc_disconnect_btn.connect("clicked", self.on_disconnect_from_display_server)
        self.update_dsc_sensitivity(False)

        # store the internal display server infos
        self._dsc = self._display_server_proxy = None
        self._display_info = None
        self._geometry_info = None

        # prepare the fake display server output window
        self._tmp_image_file = os.path.join(tempfile.mkdtemp(),"ds.png")
        self._fake_window = Gtk.Window(title="Display Server Output")
        self._fake_image = Gtk.Image()
        self._fake_image.set_from_stock("gtk-missing-image", Gtk.IconSize.DIALOG)
        self._fake_window.add(self._fake_image)
        self._fake_window.connect("delete-event", self._on_close)

        # internal variables for the draw loop
        self._draw_cursor = [0, 0]
        self._draw_cursor_speed = [0, 0]
        self._draw_polygons = []

        # Attach the draw loop
        GObject.timeout_add(interval_ms, self.on_background_update)

    def update_dsc_sensitivity(self, is_connected):
        self._dsc_status_img.set_from_stock(Gtk.STOCK_YES if is_connected else Gtk.STOCK_NO, Gtk.IconSize.BUTTON)
        self._dsc_connect_btn.set_sensitive(not is_connected)
        self._dsc_disconnect_btn.set_sensitive(is_connected)
        self._dsc_name_entry.set_sensitive(not is_connected)
        self._dsc_populate_cb.set_sensitive(not is_connected)

    def on_connect_to_display_server(self, *args):
        ds_name = self._dsc_name_entry.get_text()
        if ds_name is "" or ds_name is None:
            print "[INFO] DisplayClientWidget: no display server name specified"
            return
        else:
            try:
                self.proxy_set_display_server_proxy(ds_name)
            except:
                print "[ERROR] DisplayClientWidget: connecting to '%s' failed" % ds_name
                return
            else:
                self.update_dsc_sensitivity(True)
                self.on_background_update()
                import_display_and_geometry_info = self._dsc_populate_cb.get_active()
                self.emit('displayclient-connect', self._display_server_proxy, import_display_and_geometry_info)

    def connect_external(self, name):
        try:
            self.proxy_set_display_server_proxy(name)
        except:
            print "[INFO] DisplayClientWidget: connecting to '%s' failed" % name
            return
        else:
            self._dsc_name_entry.set_text(name)
            self.update_dsc_sensitivity(True)
            self.on_background_update()
            import_display_and_geometry_info = self._dsc_populate_cb.get_active()
            self.emit('displayclient-connect', self._display_server_proxy, import_display_and_geometry_info)

    def on_disconnect_from_display_server(self, *args):
        self.proxy_set_display_server_proxy(None)
        self.update_dsc_sensitivity(False)
        self.emit('displayclient-disconnect')

    def _on_close(self, *args):
        # hide the window
        self._fake_window.hide()
        return True  # stop signal

    def proxy_show_mock(self, *args):
        """displays the Gtk window, that shows the display_server output"""
        self._fake_window.show_all()

    def proxy_is_connected(self):
        """return True if connected"""
        return self._display_server_proxy is not None

    def proxy_set_display_server_proxy(self, display_server_name=None):
        """connect a DisplayServerProxy to the DisplayclientWidget
        disconnect with dsp=None
        """
        if display_server_name is None:
            self._dsc = self._display_server_proxy = None
        else:
            try:
                self._dsc = self._display_server_proxy = get_display_server_proxy(display_server_name)
                self._display_server_proxy.enter_2dblit_mode()
            except:
                raise

    def proxy_set_display_info(self, display_info):
        """set the display_info dict"""
        self._display_info = display_info

    def get_display_info(self):
        """get the display_info dict

        if connected to a display_server_proxy return form there.
        else return stored dict
        """
        if self._display_server_proxy is not None:
            try:
                return self._display_server_proxy.get_display_info()
            except:
                pass
        elif self._display_info is not None:
            return self._display_info

    def proxy_set_geometry_info(self, geometry_info):
        """set the geometry_info dict"""
        self._geometry_info = geometry_info

    def get_geometry_info(self):
        """get the geometry_info dict

        if connected to a display_server_proxy return form there.
        else return stored dict
        """
        if self._display_server_proxy is not None:
            return self._display_server_proxy.get_geometry_info()
        elif self._geometry_info is not None:
            return self._geometry_info

    @property
    def height(self):
        return self.get_display_info()['height']

    @property
    def width(self):
        return self.get_display_info()['width']

    def show_pixels(self, arr):
        """display an array on the display_server"""
        # save the array as a png and load it in the fake_window.
        scipy.misc.imsave(self._tmp_image_file, arr)
        self._fake_image.set_from_file(self._tmp_image_file)
        # if the display server proxy is connected display there
        if self._display_server_proxy is not None:
            self._display_server_proxy.show_pixels(arr)

    def on_position_change(self, _dummy, msg):
        """this function should be connected to the on-axis signal of a joystick"""
        x, y = msg
        di = self.get_display_info()
        if di is not None:
            height, width = di['height'], di['width']
            x = msg[0] % width
            y = msg[1] % height
        set_sprite_position(x, y)

    def on_background_update(self, *args):
        start = time.time()
        if not self._draw_prepare():
            return
        if self._draw_polygons:
            for polygon in self._draw_polygons:
                self._draw_polygon_func((255,0,0), polygon)
        self._draw_finish()
        print "[INFO] DisplayClientWidget: background update took %fsec" % (time.time() - start)

    def _draw_prepare(self, color=(0,0,0)):
        """prepare an empty array for drawing

        This function returns False if no display_info is available
        """
        color = np.array(color, dtype=np.uint8)
        di = self.get_display_info()
        if di is None:
            return False
        height, width = di['height'], di['width']
        arr = np.empty((height, width, 3), dtype=np.uint8)
        arr[:,:,:] = color
        self._draw_array = arr
        return True

    def _draw_polygon_func(self, color, polygon_vertices=None):
        """draw a polygon in the draw array

        if polygon_vertices is None, fill the whole array
        """
        color = np.array(color, dtype=np.uint8)
        if polygon_vertices is None:
            self._draw_array[:,:,:] = color
        else:
            mask = np.zeros_like(self._draw_array[:,:,0], dtype=np.uint8)
            flyvr.tools.fill_polygon.fill_polygon(polygon_vertices, mask)
            # check if viewport is not defined ???
            if np.max(mask) == 0:
                mask[:,:] = 1  # sets whole mask to True
            mask = mask.astype(np.bool)
            self._draw_array = ((~mask[:,:,np.newaxis]) * self._draw_array) + (mask[:,:,np.newaxis] * color)

    def _draw_finish(self):
        """convinience function for the draw loop"""
        self.show_pixels(self._draw_array)

    def set_cursor_draw(self, state):
        """sets if the cursor should be displayed or not"""
        set_sprite_visible(state)

    def set_polygons_draw(self, list_of_polygons=tuple()):
        """provide a list of polygons to draw on the screen"""
        self._draw_polygons = list_of_polygons

    def draw_viewports(self, viewports):
        """draws viewports"""
        self._draw_prepare()
        for vp in viewports:
            if not vp._display:
                continue
            R = int(vp._fill_color[-3], 16) * 16
            G = int(vp._fill_color[-2], 16) * 16
            B = int(vp._fill_color[-1], 16) * 16
            self._draw_polygon_func((R, G, B), vp.to_list())
        self._draw_finish()





