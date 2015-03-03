
import collections
import pkgutil
import sys
import time
import warnings

from gi.repository import Gtk, GObject
GObject.threads_init()
import pyglet
import threading

import roslib
roslib.load_manifest('flyvr')

DEFAULT_JOYSTICK_SETTINGS = {
    "accept": 0,
    "remove": 1,
    "nextitem": 5,
    "previtem": 4,
    "lr_axis": 0,
    "ud_axis": 1,
    "lr_slow_axis": 3,
    "ud_slow_axis": 4,
    "inv_lr": False,
    "inv_ud": False,
    "move_speed": 1.0,
    "move_speed_slow": 0.1,
    "axis_cutoff": 0.2,
    }


class PygletJoystickGObject(GObject.GObject, threading.Thread):

    __gsignals__ =  {
            "on-axis-raw": (
                GObject.SignalFlags.RUN_LAST, None, [
                object]),
            "on-button-raw": (
                GObject.SignalFlags.RUN_LAST, None, [
                object]),
            "on-axis": (
                GObject.SignalFlags.RUN_LAST, None, [
                object]),
            "on-button": (
                GObject.SignalFlags.RUN_LAST, None, [
                object]),
            "on-position-change": (
                GObject.SignalFlags.RUN_LAST, None, [
                object]),
            }

    def __init__(self):
        GObject.GObject.__init__(self)
        threading.Thread.__init__(self)
        self.daemon = True

        self.joysticks = pyglet.input.get_joysticks()
        if not self.joysticks:
            warnings.warn("No Joysticks were found. Connect Joystick and restart.")
        for i, js in enumerate(self.joysticks):
            js.open()
            js.push_handlers(self)
        print "Joystick: found", len(self.joysticks)

        self.msgcls = collections.namedtuple('Joy', 'buttons axis')
        self._last = self.msgcls(buttons=(0,)*16, axis=(0,)*6)
        self.axis_idx = {
                'x': 0,
                'y': 1,
                'z': 2,
                'rx': 3,
                'ry': 4,
                'rz': 5
            }

        # Position integration
        self._position = [0, 0]
        self._speed = (0, 0)
        GObject.timeout_add(30, self._position_integrate)

        self._lock = threading.Lock()
        # FIXME: load from file if available
        self._all_settings = DEFAULT_JOYSTICK_SETTINGS
        self.apply_settings(self._all_settings)
        self.start()

    def on_joybutton_press(self, js, button):
        with self._lock:
            # newbuttons = list(self._last.buttons)
            # RETURNS ONLY THE PRESSED BUTTON
            newbuttons = [0,]*16
            newbuttons[button] = 1
            newbuttons = tuple(newbuttons)
            oldaxis = self._last.axis
            newmsg = self.msgcls(buttons=newbuttons, axis=oldaxis)
            self._last = newmsg
        GObject.idle_add(GObject.GObject.emit, self, "on-button-raw", newmsg)
        self.on_button_msg(button)

    def on_joybutton_release(self, js, button):
        with self._lock:
            newbuttons = list(self._last.buttons)
            newbuttons[button] = 0
            newbuttons = tuple(newbuttons)
            oldaxis = self._last.axis
            newmsg = self.msgcls(buttons=newbuttons, axis=oldaxis)
            self._last = newmsg
        GObject.idle_add(GObject.GObject.emit, self, "on-button-raw", newmsg)

    def on_joyaxis_motion(self, js, axis, value):
        with self._lock:
            oldbuttons = self._last.buttons
            newaxis = list(self._last.axis)
            newaxis[self.axis_idx[axis]] = value
            newaxis = tuple(newaxis)
            newmsg = self.msgcls(buttons=oldbuttons, axis=newaxis)
            self._last = newmsg
        GObject.idle_add(GObject.GObject.emit, self, "on-axis-raw", newmsg)
        self.on_axis_msg(newmsg)

    def on_joyhat_motion(self, js, hat_x, hat_y):
        # FIXME: we do not support hat for now...
        warnings.warn("Joystick: hat_x, hat_y is not used.")

    def run(self, *args):
        try:
            pyglet.app.run()
        finally:
            for js in self.joysticks:
                js.close()

    def stop(self):
        try:
            pyglet.app.exit()
        except:
            pass

    def on_button_msg(self, pressed_button_idx):
        msg = self.settings_button[pressed_button_idx]
        GObject.idle_add(GObject.GObject.emit, self, "on-button", msg)

    def cutoff(self, val):
        cutoff = float(self.settings_axis['axis_cutoff'])
        return 0 if abs(val) < cutoff else val

    def on_axis_msg(self, msg):
        v = float(self.settings_axis['move_speed'])
        v_slow = float(self.settings_axis['move_speed_slow'])
        inv_lr = -1 if bool(self.settings_axis['inv_lr']) else 1
        inv_ud = -1 if bool(self.settings_axis['inv_ud']) else 1
        lr = int(self.settings_axis['lr_axis'])
        ud = int(self.settings_axis['ud_axis'])
        lr_slow = int(self.settings_axis['lr_slow_axis'])
        ud_slow = int(self.settings_axis['ud_slow_axis'])

        x = inv_lr * (v * self.cutoff(msg.axis[lr])
                    + v_slow * self.cutoff(msg.axis[lr_slow]))
        y = inv_ud * (v * self.cutoff(msg.axis[ud])
                    + v_slow * self.cutoff(msg.axis[ud_slow]))
        GObject.idle_add(GObject.GObject.emit, self, "on-axis", (x, y))
        with self._lock:
            self._speed = [x, y]

    def _position_integrate(self, *args):
        with self._lock:
            vx, vy = self._speed
        if vx != 0 or vy != 0:
            self._position[0] += vx
            self._position[1] += vy
            print "on:", self._position
            GObject.idle_add(GObject.GObject.emit, self, "on-position-change", self._position)
        return True

    def apply_settings(self, settings):
        self.settings_button = {}
        self.settings_axis = {}
        for k, v in settings.items():
            if k in ["accept", "remove", "nextitem", "previtem"]:
                self.settings_button[int(v)] = k
            else:
                self.settings_axis[k] = settings[k]
        self._all_settings = settings

    def get_settings(self):
        return self._all_settings


class JoystickConfigureDialog(Gtk.Dialog):

    def __init__(self, joystickinterface):

        super(JoystickConfigureDialog, self).__init__("Joystick configuration")

        ui_file_contents = pkgutil.get_data('flyvr.calibration.gui', 'pinhole-wizard.ui')
        ui = Gtk.Builder()
        ui.add_from_string(ui_file_contents)
        jsgrid = ui.get_object('joystick_configure_grid')

        self.get_content_area().add(jsgrid)

        self.joystick_settings = collections.OrderedDict([
            ("accept", ui.get_object('JS_accept_button_entry')),
            ("remove", ui.get_object('JS_remove_button_entry')),
            ("nextitem", ui.get_object('JS_next_item_button_entry')),
            ("previtem", ui.get_object('JS_prev_item_button_entry')),
            ("lr_axis", ui.get_object('JS_lr_axis_entry')),
            ("ud_axis", ui.get_object('JS_ud_axis_entry')),
            ("lr_slow_axis", ui.get_object('JS_lr_slow_axis_entry')),
            ("ud_slow_axis", ui.get_object('JS_ud_slow_axis_entry')),
            ("inv_lr", ui.get_object('JS_invert_lr_switch')),
            ("inv_ud", ui.get_object('JS_invert_up_switch')),
            ("move_speed", ui.get_object('JS_move_speed_entry')),
            ("move_speed_slow", ui.get_object('JS_move_speed_slow_entry')),
            ("axis_cutoff", ui.get_object('JS_axis_cutoff_entry')),
        ])
        self.joystick_types = {
            "accept": "button",
            "remove": "button",
            "nextitem": "button",
            "previtem": "button",
            "lr_axis": "axis",
            "ud_axis": "axis",
            "lr_slow_axis": "axis",
            "ud_slow_axis": "axis",
            "inv_lr": "switch",
            "inv_ud": "switch",
            "move_speed": "value",
            "move_speed_slow": "value",
            "axis_cutoff": "value",
        }

        self.add_buttons(Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
                         Gtk.STOCK_SAVE, Gtk.ResponseType.OK)

        self.lastmsg = None
        self.joystickinterface = joystickinterface

    def on_button_press(self, widget, msg):
        if msg is None:
            return
        if self.lastmsg is not None:
            try:
                idx = msg.buttons.index(1)
            except ValueError:
                return
            for key, entry in reversed(self.joystick_settings.items()):
                if entry.is_focus():
                    if self.joystick_types[key] == "button":
                        entry.set_text(str(idx))
                        entry.get_toplevel().child_focus(Gtk.DirectionType.TAB_FORWARD)
        self.lastmsg = msg


    def on_axis_change(self, widget, msg):
        if msg is None:
            return
        if self.lastmsg is not None:
            a_rising_edge = [abs(na) >= 1.0 and abs(na) - abs(oa) > 0 for na, oa in zip(msg.axis, self.lastmsg.axis)]
            for key, entry in reversed(self.joystick_settings.items()):
                if entry.is_focus():
                    if self.joystick_types[key] == "axis":
                        for idx, pressed in enumerate(a_rising_edge):
                            if pressed:
                                entry.set_text(str(idx))
                                entry.get_toplevel().child_focus(Gtk.DirectionType.TAB_FORWARD)
        self.lastmsg = msg

    def get_settings(self):
        settings = {}
        for key, entry in self.joystick_settings.items():
            if self.joystick_types[key] in ["button", "axis"]:
                try:
                    idx = int(entry.get_text())
                except:
                    idx = None
            elif self.joystick_types[key] == "switch":
                idx = bool(entry.get_active())
            elif self.joystick_types[key] == "value":
                idx = float(entry.get_text())
            else:
                raise Exception("Should not happen.")
            settings[key] = idx
        return settings

    def apply_settings(self, settings):
        for key, entry in self.joystick_settings.items():
            value = settings[key]
            if self.joystick_types[key] in ["button", "axis", "value"]:
                entry.set_text(str(value))
            elif self.joystick_types[key] == "switch":
                entry.set_active(value)

    def configure(self, *args, **kwargs):
        current_settings = self.joystickinterface.get_settings()
        self.apply_settings(current_settings)
        connect_handle0 = self.joystickinterface.connect("on-axis-raw", self.on_axis_change)
        connect_handle1 = self.joystickinterface.connect("on-button-raw", self.on_button_press)
        try:
            retval = super(JoystickConfigureDialog, self).run()
            if retval == Gtk.ResponseType.OK:
                new_settings = self.get_settings()
                self.joystickinterface.apply_settings(new_settings)
            return retval
        finally:
            self.joystickinterface.disconnect(connect_handle0)
            self.joystickinterface.disconnect(connect_handle1)
            self.hide()
            print "Joystick: configured"








if __name__ == "__main__":

    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    jsinterface = PygletJoystickGObject()
    jsdialog = JoystickConfigureDialog(jsinterface)

    tt = threading.Thread(target=Gtk.main)
    tt.daemon = True
    tt.start()

    try:
        response = jsdialog.run()
    except KeyboardInterrupt:
        print "Exit on user request"

