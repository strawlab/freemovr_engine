from gi.repository import Gtk, Gdk

COLORS = ("red","green","blue","magenta","yellow","teal","brown")

class WindowOnScreen:
    def __init__(self, label, color, screen):
        w = Gtk.Window()
        b = Gtk.EventBox()
        l = Gtk.Label()
        l.set_markup('<span size="xx-large">%s</span>' % label)
        b.add(l)

        bg = Gdk.RGBA()
        bg.parse(color)
        b.override_background_color(Gtk.StateFlags.NORMAL,bg)

        fg = Gdk.RGBA()
        fg.parse("white")
        l.override_color(Gtk.StateFlags.NORMAL,fg)

        w.add(b)
        w.set_default_size (screen.get_width(), screen.get_height())
        w.fullscreen()
        w.set_screen(screen)
        w.show_all()

if __name__ == "__main__":
    d = Gdk.Display.get_default()
    for i in range(0, d.get_n_screens()):
        WindowOnScreen(
            'Display %s Screen: %d' % (d.get_name(),i),
            COLORS[i],
            d.get_screen(i))
    Gtk.main()
