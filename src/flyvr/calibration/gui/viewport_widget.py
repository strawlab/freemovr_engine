#!/usr/bin/env python
from gi.repository import Gtk, GObject, Gdk
from viewport_widget_lib import Sprite, Scene, Graphics
import collections
import itertools

def darker_color(fill_color):
    assert len(fill_color) == 4
    stroke_color = "#"+"".join(map(lambda x :hex(int(x, 16)/2)[2], fill_color[1:]))
    return stroke_color

viewportnames = ("ViewPort%d" % i for i in itertools.count(0))
viewportcolors = (c for c in itertools.cycle(['#F00','#0F0','#00F','#FF0','#0FF','#F0F','#FFF']))


class Node(Sprite):

    def __init__(self, x, y, fill_color):
        """Draggable node for the viewport definer"""
        Sprite.__init__(self, x=x, y=y, interactive=True, draggable=True)
        stroke_color = darker_color(fill_color)
        self.draw_node(fill_color, stroke_color)

    def draw_node(self, fill_color, stroke_color):
        self.graphics.set_line_style(width = 2)
        self.graphics.rectangle(-5, -5, 10, 10, 3)
        self.graphics.fill_stroke(fill=fill_color, stroke=stroke_color)

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


class Viewport(object):

    def __init__(self, name, fill_color):
        self._fill_color = fill_color
        self._stroke_color = darker_color(fill_color)
        self.nodes = []
        self._is_active = False
        self._display = True
        self.name = name

    def add_node_from_event(self, event):
        node = Node(event.x, event.y, fill_color=self._fill_color)
        idx = node.closest_index_from_nodes(self.nodes)
        self.nodes.insert(idx, node)
        return node

    def remove_node(self, node):
        for i, tnode in enumerate(self.nodes):
            if tnode == node:
                break
        self.nodes.pop(i)

    def node_in_viewport(self, node):
        return node in self.nodes

    def is_active(self):
        return self._is_active

    def set_active(self, state):
        self._is_active = bool(state)

    def set_display(self, state):
        self._display = bool(state)
        for node in self.nodes:
            node.visible = self._display

    def draw_nodes(self, context):
        if self._display:
            c_graphics = Graphics(context)
            c_graphics.set_line_style(width=2)
            if self.nodes:
                c_graphics.move_to(self.nodes[-1].x, self.nodes[-1].y)
            for i in range(len(self.nodes)):
                node = self.nodes[i]
                c_graphics.line_to(node.x, node.y)
            c_graphics.close_path()
            c_graphics.fill_stroke(fill=self._fill_color, stroke=self._stroke_color)

    @classmethod
    def from_list(cls, points, name="NoNameViewPort", fill_color="#F00"):
        vp = cls(name, fill_color)
        stroke_color = darker_color(fill_color)
        for p in points:
            vp.nodes.append(Node(p[0], p[1],
                                 fill_color=fill_color, stroke_color=stroke_color))
        return vp

    def set_color(self, fill_color):
        stroke_color = darker_color(fill_color)
        for node in self.nodes:
            node.draw_node(fill_color, stroke_color)
        self._fill_color = fill_color
        self._stroke_color = stroke_color


class Canvas(Scene):
    def __init__(self):
        Scene.__init__(self, background_color="#000")

        self.connect("on-enter-frame", self.on_enter_frame)
        self.connect("on-click", self.on_mouse_click)
        self.connect("on-drag", self.on_node_drag)

        self.MODE = "ADD"

        self.viewport_getter = lambda : []

    @property
    def viewports(self):
        return self.viewport_getter()

    def on_mouse_click(self, area, event, target):
        for viewport in self.viewports:
            if viewport.is_active():
                if not target and self.MODE == "ADD":
                    node = viewport.add_node_from_event(event)
                    self.add_child(node)
                    self.redraw()
                elif target and self.MODE == "DELETE":
                    if viewport.node_in_viewport(target):
                        viewport.remove_node(target)
                        self.remove_child(target)
                        self.redraw()

    def on_node_drag(self, scene, node, event):
        self.redraw()

    def on_enter_frame(self, scene, context):
        for viewport in self.viewports:
            viewport.draw_nodes(context)

    def on_key_press(self, widget, event):
        if event.keyval == 65507:
            self.MODE = "DELETE"
            print "DELETE mode"

    def on_key_release(self, widget, event):
        if event.keyval == 65507:
            self.MODE = "ADD"
            print "ADD mode"


class BasicWindow:
    def __init__(self):
        window = Gtk.Window()
        window.connect("delete_event", lambda *args: Gtk.main_quit())

        vbox = Gtk.VBox()
        window.add(vbox)

        box = Gtk.HBox()
        box.set_border_width(10)
        box.add(Gtk.Label("Click to add Points. Hold CTRL and click to delete points."))
        vbox.pack_start(box, True, True, 0)

        self.canvas = Canvas()
        self.canvas.set_size_request(700, 400)
        vbox.add(self.canvas)
        window.connect("key_press_event", self.canvas.on_key_press)
        window.connect("key_release_event", self.canvas.on_key_release)

        box = Gtk.HBox(False, 4)
        box.set_border_width(10)

        vbox.pack_start(box, False, False, 0)

        # treeview #

        VD_NAME = 0
        VD_FCOL = 1
        VD_DISP = 2
        VD_SEL = 3
        VD_VPOB = 4

        vdisp_store = Gtk.ListStore(str, str, bool, bool, object)
        treeview = Gtk.TreeView()
        treeview.set_model(vdisp_store)

        def viewport_getter():
            return list(row[VD_VPOB] for row in vdisp_store)

        self.canvas.viewport_getter = viewport_getter

        def on_edit_name(widget, path, textval, colnum):
            name = textval.replace(" ","_")
            vdisp_store[path][colnum] = name
            vdisp_store[path][VD_VPOB].name = name

        renderer_text = Gtk.CellRendererText(editable=True)
        renderer_text.connect("edited", on_edit_name, VD_NAME)
        column = Gtk.TreeViewColumn("Viewport", renderer_text, text=VD_NAME)
        treeview.append_column(column)

        def on_edit_color(widget, path, textval, colnum):
            try:
                assert len(textval) == 4
                assert textval[0] == "#"
                int("0x"+textval[1:4], 16)
            except:
                textval = "#F00"
            vdisp_store[path][colnum] = textval
            vdisp_store[path][VD_VPOB].set_color(textval)
            self.canvas.redraw()

        renderer_text = Gtk.CellRendererText(editable=True)
        renderer_text.connect("edited", on_edit_color, VD_FCOL)
        column = Gtk.TreeViewColumn("fill color", renderer_text, text=VD_FCOL)
        treeview.append_column(column)

        def on_toggle_display(widget, path):
            vdisp_store[path][VD_DISP] = not vdisp_store[path][VD_DISP]
            vdisp_store[path][VD_VPOB].set_display(vdisp_store[path][VD_DISP])
            self.canvas.redraw()

        renderer = Gtk.CellRendererToggle()
        renderer.connect("toggled", on_toggle_display)
        column = Gtk.TreeViewColumn("display", renderer, active=VD_DISP)
        treeview.append_column(column)

        def on_select(widget, path):
            selected_path = Gtk.TreePath(path)
            # perform mutually-exclusive radio button setting
            for row in vdisp_store:
                state = row.path == selected_path
                row[VD_SEL] = state
                row[VD_VPOB].set_active(state)

        renderer_pixbuf = Gtk.CellRendererToggle()
        renderer_pixbuf.set_radio(True)
        renderer_pixbuf.connect("toggled", on_select)
        column = Gtk.TreeViewColumn('select', renderer_pixbuf, active=VD_SEL)
        treeview.append_column(column)

        def on_selection_changed(selection, *args):
            model, paths = selection.get_selected_rows()
            for selected_path in paths:
                # perform mutually-exclusive radio button setting
                for row in vdisp_store:
                    state = row.path == selected_path
                    row[VD_SEL] = state
                    row[VD_VPOB].set_active(state)
        treeview.get_selection().connect("changed", on_selection_changed)

        box.pack_start(treeview, False, False, 0)

        # Buttons #

        button = Gtk.Button("Clear")
        def on_click(*args):
            for viewport in self.canvas.viewports:
                if viewport.is_active():
                    self.canvas.mouse_node, self.canvas.prev_mouse_node = None, None
                    for node in viewport.nodes:
                        self.canvas.remove_child(node)
                    viewport.nodes = []
                    self.canvas.redraw()

        button.connect("clicked", on_click)
        box.pack_end(button, False, False, 0)

        button = Gtk.Button("Add viewport")
        def on_click(*args):
            vp = Viewport.from_list([], name=viewportnames.next(),
                                        fill_color=viewportcolors.next())
            vp.set_active(False)
            vdisp_store.append([vp.name, vp._fill_color, True, False, vp])

        button.connect("clicked", on_click)
        box.pack_end(button, False, False, 0)

        button = Gtk.Button("Remove viewport")
        def on_click(*args):
            selection = treeview.get_selection()
            sel = selection.get_selected()
            if not sel[1] == None:
                row = vdisp_store[sel[1]]
                for node in row[VD_VPOB].nodes:
                    self.canvas.remove_child(node)
                vdisp_store.remove(sel[1])
                self.canvas.redraw()

        button.connect("clicked", on_click)
        box.pack_end(button, False, False, 0)

        vbox.pack_start(box, False, False, 0)

        window.show_all()


if __name__ == "__main__":
    example = BasicWindow()
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL) # Gtk3 screws up ctrl+c
    Gtk.main()
