#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil -*-
import warnings
import pkgutil
import platform

try:
    import roslib
except ImportError:
    warnings.warn("Can't import roslib. Make sure that flyvr python package is in your PYTHONPATH.")
else:
    roslib.load_manifest("flyvr")
    roslib.load_manifest("rosgobject")
    import rosgobject


from flyvr.calibration.gui.displayclient_widget import DisplayClientWidget
from flyvr.calibration.gui.intrinsics_widget import IntrinsicsWidget
from flyvr.calibration.gui.viewport_widget import ViewportWidget
from flyvr.calibration.gui.joystick_widget import JoystickConfigureDialog, PygletJoystickGObject
from flyvr.calibration.gui.extrinsics_widget import ExtrinsicsWidget
from flyvr.calibration.gui.info_widget import InfoWidget
from flyvr.calibration.gui.file_widgets import YAMLFileLoadDialog, YAMLFileSaveDialog
#from flyvr.calibration.gui.debugplot_widget import DebugplotWidget


from gi.repository import Gtk


class PinholeWizard(object):

    def __init__(self):
        """The pinhole wizard container class"""
        # load the ui file
        ui_file_contents = pkgutil.get_data('flyvr.calibration.gui','pinhole-wizard.ui')
        ui = Gtk.Builder()
        ui.add_from_string( ui_file_contents )

        # Prepare main window
        self.main_window = ui.get_object('main_window')
        self.main_window.connect("delete-event", rosgobject.main_quit)

        # Get all "widgets"
        self.displayclient_widget = DisplayClientWidget(1000)
            # > "displayclient-connect"             OK
            # > "displayclient-disconnect"          OK
            # < on_position_change                  OK
        self.joystickinterface = PygletJoystickGObject()
            # > "on-button"                         OK
            # > "on-axis"                           OK
            # > "on-position-change"                OK
        self.intrinsics_widget = IntrinsicsWidget()
            # > "intrinsics-computed" : yaml-string OK
            # < on_joystick_button                  OK
            # < on_displayclient_connect            OK FIX: activate buttons
            # < on_displayclient_disconnect         OK FIX: deactivate buttons
        self.viewport_widget = ViewportWidget()
            # > "viewports-saved" : yaml-string     OK
            # < on_key_press                        OK
            # < on_key_release                      OK
            # < on_displayclient_connect            OK FIX: update dsc
            # < on_displayclient_disconnect         OK
        self.extrinsics_widget = ExtrinsicsWidget()
            # > "extrinsics-computed" : yaml-string
            # < on_viewports_saved
            # < on_joystick_button
            # < on_displayclient_connect
            # < on_displayclient_disconnect
        self.info_widget = InfoWidget()
            # < on_displayclient_connect
            # < on_displayclient_disconnect
            # < on_intrinsics_computed
            # < on_viewports_saved
            # < on_extrinisics_computed

        #self.debugplot_widget = DebugplotWidget()

        lb = ui.get_object('left_box')
        lb.pack_start(self.displayclient_widget, False, False, 0)
        lb.pack_start(self.info_widget, True, True, 0)

        # Add Notebook to main_box and append widget pages
        nb = Gtk.Notebook()
        ui.get_object('main_box').pack_start(nb, True, True, 0)
        nb.append_page(self.intrinsics_widget, Gtk.Label(label='intrinsics'))
        nb.append_page(self.viewport_widget, Gtk.Label(label='viewports'))
        nb.append_page(self.extrinsics_widget, Gtk.Label(label='extrinsics'))
        nb.show_all()

        # Add joystick controls
        self.joystickinterface.connect("on-position-change", self.displayclient_widget.on_position_change)
        self.joystickinterface.connect("on-button", self.intrinsics_widget.on_joystick_button)
        self.joystickinterface.connect("on-button", self.extrinsics_widget.on_joystick_button)
        # self.joystickinterface.connect("on-button", self.intrinsics_widget.on_button)
        # self.joystickinterface.connect("on-button", self.extrinsics_widget.on_button)

        # Add joystick configuration
        self.joystickdialog = JoystickConfigureDialog(self.joystickinterface)
        ui.get_object('configure_joystick').connect('activate', self.joystickdialog.configure)

        # Connect checklist widget
        self.intrinsics_widget.connect("intrinsics-computed", self.info_widget.on_intrinsics_computed)
        self.intrinsics_widget.connect("intrinsics-computed", self.extrinsics_widget.on_intrinsics_computed)
        self.viewport_widget.connect("viewports-saved", self.info_widget.on_viewports_saved)
        self.viewport_widget.connect("viewports-saved", self.extrinsics_widget.on_viewports_saved)
        #self.extrinsics_widget.connect("on-calibrated", self.checklist_widget.on_calibrated)
        #self.displayclient_widget.connect("on-connected", self.checklist_widget.on_connected)

        # connect displayclient
        self.displayclient_widget.connect("displayclient-connect", self.viewport_widget.on_displayclient_connect)
        self.displayclient_widget.connect("displayclient-connect", self.intrinsics_widget.on_displayclient_connect)
        self.displayclient_widget.connect("displayclient-connect", self.extrinsics_widget.on_displayclient_connect)
        self.displayclient_widget.connect("displayclient-disconnect", self.viewport_widget.on_displayclient_disconnect)
        self.displayclient_widget.connect("displayclient-disconnect", self.intrinsics_widget.on_displayclient_disconnect)
        self.displayclient_widget.connect("displayclient-disconnect", self.extrinsics_widget.on_displayclient_disconnect)

        self.displayclient_widget.connect("displayclient-connect", self.joystickinterface.on_reset_position)

        self.main_window.connect("key_press_event", self.viewport_widget.on_key_press)
        self.main_window.connect("key_release_event", self.viewport_widget.on_key_release)


        # Add menu items
        self.fname = None
        ui.get_object('file_load_config').connect('activate', self.on_load)
        ui.get_object('file_save_config').connect('activate', self.on_save, self.fname)
        ui.get_object('file_saveas_config').connect('activate', self.on_save_as, None)
        ui.get_object('file_quit_menu_item').connect('activate', rosgobject.main_quit)
        #ui.get_object('help_about_menu_item').connect('activate', self.on_help_about)

        # Configure the DisplayClientWidget
        self.displayclient_widget.set_cursor_draw(True)
        ui.get_object('view_mock_ds_item').connect('activate', self.displayclient_widget.proxy_show_mock)
        self.displayclient_widget.connect_external("/display_server0")

        # initializing done.
        self.main_window.show_all()
        print "[INFO] PinholeWizard: initialized"  # TODO: log.

    def on_load(self, *args):
        print args
        yaml_file_loader = YAMLFileLoadDialog()
        try:
            response = yaml_file_loader.run()
            if response == Gtk.ResponseType.OK:
                config = yaml_file_loader.load_yaml()
                if config.get('intrinsics', False):
                    self.intrinsics_widget.intrinsics_from_list(config['intrinsics'])
                if config.get('viewports', False):
                    self.viewport_widget.viewports_from_list(config['viewports'])
                if config.get('extrinsics', False):
                    self.extrinsics_widget.extrinsics_from_list(config['extrinsics'])
        finally:
            yaml_file_loader.destroy()

    def on_save(self, *args):
        print args
        intrinsics = self.intrinsics_widget.intrinsics_as_list()
        viewports = self.viewport_widget.viewports_as_list()
        extrinsics = self.extrinsics_widget.extrinsics_as_list()
        out_dict = {}
        if intrinsics:
            out_dict['intrinsics'] = intrinsics
        if viewports:
            out_dict['viewports'] = viewports
        if extrinsics:
            out_dict['extrinsics'] = extrinsics
        if out_dict:
            yaml_file_saver = YAMLFileSaveDialog()
            try:
                response = yaml_file_saver.run()
                if response == Gtk.ResponseType.OK:
                    yaml_file_saver.save_yaml(out_dict)
            finally:
                yaml_file_saver.destroy()
        else:
            pass  # TODO

    def on_save_as(self, *args):
        print args
        intrinsics = self.intrinsics_widget.intrinsics_as_list()
        viewports = self.viewport_widget.viewports_as_list()
        extrinsics = self.extrinsics_widget.extrinsics_as_list()
        out_dict = {}
        if intrinsics:
            out_dict['intrinsics'] = intrinsics
        if viewports:
            out_dict['viewports'] = viewports
        if extrinsics:
            out_dict['extrinsics'] = extrinsics
        if out_dict:
            yaml_file_saver = YAMLFileSaveDialog()
            try:
                response = yaml_file_saver.run()
                if response == Gtk.ResponseType.OK:
                    yaml_file_saver.save_yaml(out_dict)
            finally:
                yaml_file_saver.destroy()
        else:
            pass  # TODO





if __name__ == "__main__":

    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    import argparse
    import traceback

    try:
        try:
            import roslib
        except ImportError:
            phw = PinholeWizard()
            try:
                Gtk.main()
            except:
                pass
        else:
            rosgobject.init_node("pinhole_wizard")
            rosgobject.get_ros_thread() #ensure ros is spinning
            rosgobject.add_console_logger()

            try:
                phw = PinholeWizard()
            except:
                print "ERROR in PHW"
                traceback.print_exc()
                exit(1)
            try:
                rosgobject.spin()
            except:
                pass
    except:
        traceback.print_exc()
