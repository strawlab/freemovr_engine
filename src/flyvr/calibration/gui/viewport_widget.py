#!/usr/bin/env python
from gi.repository import Gtk
from lib.graphics import Sprite, Scene


class Node(Sprite):
    def __init__(self, x, y):
        Sprite.__init__(self, x=x, y=y, interactive=True, draggable=True)
        self.graphics.set_line_style(width = 1)
        self.graphics.rectangle(-5, -5, 10, 10, 3)
        self.graphics.fill_stroke(fill="#E00", stroke="#000")

    def squared_distance_to_line_segment(self, node1, node2):
        """from http://stackoverflow.com/questions/849211"""
        x1, y1 = node1.x, node1.y
        x2, y2 = node2.x, node2.y
        x3, y3 = self.x, self.y
        px = x2-x1
        py = y2-y1
        u =  ((x3 - x1) * px + (y3 - y1) * py) / float(px*px + py*py)
        u = min(1, max(0, u))
        x = x1 + u * px
        y = y1 + u * py
        dx = x - x3
        dy = y - y3
        return (dx*dx + dy*dy)

    def closest_index_from_nodes(self, nodes):
        """returns the best index for inserting this node into the list"""
        N = len(nodes)
        if N < 3:
            return N
        dists = []
        for i in range(len(nodes)):
            n0 = nodes[i-1]
            n1 = nodes[i]
            dists.append(self.squared_distance_to_line_segment(n0, n1))
        val, idx = min((val, idx) for (idx, val) in enumerate(dists))
        return idx

class Canvas(Scene):
    def __init__(self):
        Scene.__init__(self)
        self.nodes = []
        self.segments = []

        self.connect("on-enter-frame", self.on_enter_frame)
        self.connect("on-click", self.on_mouse_click)
        self.connect("on-drag", self.on_node_drag)

        self.MODE = "ADD"

    def on_mouse_click(self, area, event, target):
        if not target and self.MODE == "ADD":
            node = Node(event.x, event.y)
            idx = node.closest_index_from_nodes(self.nodes)
            self.nodes.insert(idx, node)
            self.add_child(node)
            self.redraw()
        elif target and self.MODE == "DELETE":
            for i, node in enumerate(self.nodes):
                if target == node:
                    break
            self.nodes.pop(i)
            self.remove_child(target)
            self.redraw()

    def on_node_drag(self, scene, node, event):
        self.redraw()

    def on_enter_frame(self, scene, context):
        context.set_line_width(0.5)
        context.set_source_rgb(0xE0, 0x00, 0x00)
        if self.nodes:
            context.move_to(self.nodes[-1].x, self.nodes[-1].y)
        for i in range(len(self.nodes)):
            node = self.nodes[i]
            context.line_to(node.x, node.y)
        context.close_path()
        context.set_source_rgb(0xE0, 0x00, 0x00)
        context.fill_preserve()
        context.set_source_rgb(0x00, 0x00, 0x00)
        context.stroke()



class BasicWindow:
    def __init__(self):
        window = Gtk.Window()
        window.set_size_request(600, 500)
        window.connect("delete_event", lambda *args: Gtk.main_quit())

        window.connect("key_press_event", self.on_key_press)
        window.connect("key_release_event", self.on_key_release)

        self.w = window

        vbox = Gtk.VBox()
        window.add(vbox)

        box = Gtk.HBox()
        box.set_border_width(10)
        box.add(Gtk.Label("Click to add Points. Hold CTRL and click to delete points."))
        vbox.pack_start(box, False, False, 0)

        self.canvas = Canvas()
        vbox.add(self.canvas)

        box = Gtk.HBox(False, 4)
        box.set_border_width(10)

        vbox.pack_start(box, False, False, 0)

        button = Gtk.Button("Clear")
        def on_click(*args):
            self.canvas.nodes = []
            self.canvas.mouse_node, self.canvas.prev_mouse_node = None, None
            self.canvas.clear()
            self.canvas.redraw()

        button.connect("clicked", on_click)
        box.pack_end(button, False, False, 0)

        window.show_all()

    def on_key_press(self, widget, event):
        if event.keyval == 65507:
            self.canvas.MODE = "DELETE"
            print "DELETE mode"

    def on_key_release(self, widget, event):
        if event.keyval == 65507:
            self.canvas.MODE = "ADD"
            print "ADD mode"

if __name__ == "__main__":
    example = BasicWindow()
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL) # Gtk3 screws up ctrl+c
    Gtk.main()
