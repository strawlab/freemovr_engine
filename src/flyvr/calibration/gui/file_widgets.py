#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil -*-
import numpy as np
import scipy.misc
import warnings
import yaml

try:
    import roslib
except ImportError:
    warnings.warn("Can't import roslib. Make sure that flyvr python package is in your PYTHONPATH.")
else:
    roslib.load_manifest("flyvr")

import flyvr.exr as exr
import flyvr.simple_geom as simple_geom
import flyvr.tools.fill_polygon as fill_polygon

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

    def save_calibration_exr(self, displayclient, virtual_display_store, fname=None):

        if fname is None:
            fname = self.get_filename()

        tcs = np.zeros((displayclient.height, displayclient.width, 2)) - 1
        dist = np.nan*np.ones((displayclient.height, displayclient.width))
        angle = np.nan*np.ones((displayclient.height, displayclient.width))
        allmask = np.zeros((displayclient.height, displayclient.width))

        di = displayclient.get_display_info()
        gi = displayclient.get_geometry_info()

        geom = simple_geom.Geometry(geom_dict=gi)

        for row in virtual_display_store:
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

            this_tcs = geom.compute_for_camera_view(camera, what='texture_coords')
            this_dist = geom.compute_for_camera_view(camera, what='distance' )
            this_angle = geom.compute_for_camera_view(camera, what='incidence_angle' )

            this_tcs[ np.isnan(this_tcs) ] = -1.0 # nan -> -1

            # copy the important parts to the full display image
            tcs[mask] = this_tcs[mask]
            dist[mask] = this_dist[mask]
            angle[mask] = this_angle[mask]
        r=tcs[:,:,0]
        g=tcs[:,:,1]

        # Replace this code with something that calculates a real
        # blending value here based on distance. Probably need to
        # normalize by the maximum distance.
        warnings.warn("Add blending here...")
        b=np.ones_like(tcs[:,:,1])

        # Savr EXR FILE
        exr.save_exr( fname, r=r, g=g, b=b)

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

