#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil -*-
import warnings
import yaml

try:
    import roslib
except ImportError:
    warnings.warn("Can't import roslib. Make sure that flyvr python package is in your PYTHONPATH.")
else:
    roslib.load_manifest("flyvr")

from gi.repository import Gtk

VS_VDISP = 0
VS_CAMERA_OBJECT = 6

# TODO:
# Save calibration exr only requires the camera objects per virtual_display
#
class YAMLFileLoadDialog(Gtk.FileChooserDialog):

    def __init__(self):
        # Init parent class
        Gtk.FileChooserDialog.__init__(self, title="Load YAML file",
                                             parent=None,
                                             buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK,
                                                Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL))
        # Filter yaml files
        yaml_filter = Gtk.FileFilter()
        yaml_filter.set_name("YAML Files")
        yaml_filter.add_pattern("*.yaml")
        self.add_filter(yaml_filter)
        # force confirmation when overwriting
        self.set_do_overwrite_confirmation(True)

    def load_yaml(self):
        fname = self.get_filename()
        with open(fname, 'r') as f:
            return yaml.safe_load(f.read())


class YAMLFileSaveDialog(Gtk.FileChooserDialog):

    def __init__(self):
        # Init parent class
        Gtk.FileChooserDialog.__init__(self, title="Save YAML file",
                                             parent=None,
                                             action=Gtk.FileChooserAction.SAVE,
                                             buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK,
                                                Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL))
        # Filter yaml files
        yaml_filter = Gtk.FileFilter()
        yaml_filter.set_name("YAML Files")
        yaml_filter.add_pattern("*.yaml")
        self.add_filter(yaml_filter)
        # force confirmation when overwriting
        self.set_do_overwrite_confirmation(True)

    def save_yaml(self, data):
        fname = self.get_filename()
        with open(fname, 'w+') as f:
            f.write(yaml.safe_dump(data))


class EXRFileSaveDialog(Gtk.FileChooserDialog):

    def __init__(self, title):
        # Init parent class
        Gtk.FileChooserDialog.__init__(self, title=title,
                                             parent=None,
                                             action=Gtk.FileChooserAction.SAVE,
                                             buttons=(Gtk.STOCK_OK, Gtk.ResponseType.OK,
                                                Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL))
        # Filter exr files
        exr_filter = Gtk.FileFilter()
        exr_filter.set_name("EXR Files")
        exr_filter.add_pattern("*.exr")
        self.add_filter(exr_filter)
        # force confirmation when overwriting
        self.set_do_overwrite_confirmation(True)

