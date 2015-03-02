import os
import tempfile
from gi.repository import Gtk


class ProxyDisplayClient(object):
    def __init__(self):
        self._dsc = None
        self._di = None
        self._gi = None
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

    def proxy_set_geometry_info(self, gi):
        self._gi = gi

    def proxy_is_connected(self):
        return self._dsc is not None

    def __getattr__(self, name):
        return getattr(self._dsc, name)

    @property
    def height(self):
        if self._dsc is not None:
            return self._dsc.height
        elif self._di is not None:
            return self._di['height']

    @property
    def width(self):
        if self._dsc is not None:
            return self._dsc.width
        elif self._di is not None:
            return self._di['width']

# _dsc.enter_2dblit_mode
# _dsc.height
# _dsc.width




