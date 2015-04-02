
from gi.repository import Gtk, GObject
import warnings
try:
    import roslib
except ImportError:
    warnings.warn("Can't import roslib. Make sure that flyvr python package is in your PYTHONPATH.")
else:
    roslib.load_manifest("flyvr")

import pkgutil
import yaml
from pymvg.camera_model import CameraModel

class InfoWidget(Gtk.VBox):

    def __init__(self):
        Gtk.VBox.__init__(self)

        # Get grid container from ui file
        ui_file_contents = pkgutil.get_data('flyvr.calibration.gui', 'pinhole-wizard.ui')
        ui = Gtk.Builder()
        ui.add_from_string(ui_file_contents)
        grid = ui.get_object('info_grid')
        self.add(grid)

        # make indicator dict:
        self.indicator = {
                'intrinsics': ui.get_object('intrinsics_indicator'),
                'extrinsics': ui.get_object('extrinsics_indicator'),
                'viewports': ui.get_object('viewport_indicator'),
            }
        self.intrinsics_display = ui.get_object('intrinsics_display')
        self.viewports_display = ui.get_object('viewports_display')

        self.exr_button = ui.get_object("save_calibration_exr_button")
        self.exr_button.set_sensitive(False)

    def _pretty_intrinsics_str(self, cam):
        K = cam.K
        d = cam.distortion
        dstr = ' '.join(['% 5.2f' % di for di in d])
        args = tuple(list(K.ravel()) + [dstr])
        result = ("K: % 12.2f % 12.2f % 12.2f\n"
                  "   % 12.2f % 12.2f % 12.2f\n"
                  "   % 12.2f % 12.2f % 12.2f\n"
                  "distortion:\n   %s") % args
        return result

    def on_intrinsics_computed(self, intrinsics_widget, camera_dict):
        intrinsics = True
        cam = CameraModel.from_dict(camera_dict, extrinsics_required=False)
        text = self._pretty_intrinsics_str(cam)
        self.intrinsics_display.set_text(text)
        self.indicator['intrinsics'].set_from_stock(Gtk.STOCK_YES if intrinsics else Gtk.STOCK_NO, Gtk.IconSize.BUTTON)

    def on_viewports_saved(self, viewportwidget, viewports):
        VPI = False
        if viewports:
            VP = []
            for vp in viewports:
                if len(vp.nodes) >= 3:
                    VP.append(" - %s" % vp.name)
                    VPI = True
            self.viewports_display.set_text("\n".join(VP))
        self.indicator['viewports'].set_from_stock(Gtk.STOCK_YES if VPI else Gtk.STOCK_NO, Gtk.IconSize.BUTTON)

    def on_extrinisics_computed(self, *args):
        pass

    def on_displayclient_connect(self, *args):
        pass

    def on_displayclient_disconnect(self, *args):
        pass

