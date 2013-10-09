#!/usr/bin/env python

import roslib; roslib.load_manifest('flycave')
import rospy

import threading

from gi.repository import Gtk, GObject, Gdk, GLib

GObject.threads_init()
Gdk.threads_init()

COLORS = ("red","green","magenta","blue","yellow","teal","brown")

def check_shutdown(rosthread):
    alive = rosthread.is_alive()
    if not alive:
        Gtk.main_quit()
        return False
    return True

class WindowOnScreen:
    def __init__(self, color, screen, display):
        w = Gtk.Window()
        w.connect("delete-event", Gtk.main_quit)
        b = Gtk.EventBox()
        l = Gtk.Label()

        width = screen.get_width()
        height = screen.get_height()

        label = 'Display %s\nScreen %d\n%d x %d' % (
            display.get_name(),
            screen.get_number(),
            width,
            height)
        rospy.loginfo("Screen %d resolution %d x %d" % (
            screen.get_number(),
            width,
            height))

        l.set_markup('<span font_desc="150">%s</span>' % label)
        b.add(l)

        bg = Gdk.RGBA()
        bg.parse(color)
        b.override_background_color(Gtk.StateFlags.NORMAL,bg)

        fg = Gdk.RGBA()
        fg.parse("white")
        l.override_color(Gtk.StateFlags.NORMAL,fg)

        w.add(b)
        w.set_default_size (width, height)
        w.fullscreen()
        w.set_screen(screen)
        w.show_all()

if __name__ == "__main__":
    rospy.init_node("show_screens", anonymous=True, disable_signals=True )

    rosthread = threading.Thread(name="ros spin thread", target=rospy.spin)
    rosthread.daemon = True
    rosthread.start()

    GLib.timeout_add(1000/10, check_shutdown, rosthread)

    d = Gdk.Display.get_default()
    nscreens = d.get_n_screens()

    rospy.loginfo("Display %s has %d screens" % (d.get_name(), nscreens))

    for i in range(0, nscreens):
        WindowOnScreen(
            COLORS[i % len(COLORS)],
            d.get_screen(i),
            d)
    Gtk.main()

    rospy.signal_shutdown('quit from gui')
