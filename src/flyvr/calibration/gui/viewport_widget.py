#!/usr/bin/env python
from gi.repository import Gtk

try:
    import roslib
except ImportError:
    warnings.warn("Can't import roslib. Make sure that flyvr python package is in your PYTHONPATH.")
else:
    roslib.load_manifest("flyvr")

from flyvr.calibration.gui.viewport_widget_lib import Sprite, Scene, Graphics
from flyvr.calibration.gui.extra_widgets import ClassStoreTreeViewGenerator

import itertools
import re


def darker_color(fill_color):
    assert len(fill_color) == 4
    stroke_color = "#"+"".join(map(lambda x :hex(int(x, 16)/2)[2], fill_color[1:]))
    return stroke_color

viewportnames = ("ViewPort%d" % i for i in itertools.count(0))
viewportcolors = (c for c in itertools.cycle(['#F00','#0F0','#00F',
                                              '#FF0','#0FF','#F0F','#FFF']))


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
        return self.add_node_at_pos(event.x, event.y)

    def add_node_at_pos(self, x, y):
        node = Node(x, y, fill_color=self._fill_color)
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
        for node in self.nodes:
            node.interactive = bool(state)

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
        Scene.__init__(self, scale=True, keep_aspect=False)

        self.connect("on-enter-frame", self.on_enter_frame)
        self.connect("on-click", self.on_mouse_click)
        self.connect("on-drag", self.on_node_drag)

        self.MODE = "ADD"

        self.viewport_getter = None
        self.viewport_size = False

    def set_viewport_size(self, width, height):
        self._original_width = width
        self._original_height = height
        self.viewport_size = True

    def _get_aspect_x_y(self):
        if not self.scale or self._original_width is None:
            return 1, 1
        if self.scale:
            aspect_x = self.width / float(self._original_width)
            aspect_y = self.height / float(self._original_height)
            if self.keep_aspect:
                aspect_x = aspect_y = min(aspect_x, aspect_y)
            return aspect_x, aspect_y

    @property
    def viewports(self):
        try:
            return self.viewport_getter()
        except:
            warnings.warn("No viewport_getter function provided.")
            return []

    def on_mouse_click(self, area, event, target):
        for viewport in self.viewports:
            if viewport.is_active():
                if not target and self.MODE == "ADD":
                    x, y = event.x, event.y
                    # Fixes the node coordinates on click
                    if self.scale == True:
                        ax, ay = self._get_aspect_x_y()
                        x /= ax
                        y /= ay
                    node = viewport.add_node_at_pos(x, y)
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
        if self.viewport_size:
            c_graphics = Graphics(context)
            c_graphics.rectangle(0,0,self._original_width, self._original_height, 0)
            c_graphics.fill("#000")
        for viewport in self.viewports:
            viewport.draw_nodes(context)

    def on_key_press(self, widget, event):
        if event.keyval == 65507:
            self.MODE = "DELETE"

    def on_key_release(self, widget, event):
        if event.keyval == 65507:
            self.MODE = "ADD"

class CanvasContainer(Gtk.VBox):

    TEXT_NO_VIEWPORT = "Viewport size not set. Canvas disabled."
    TEXT_VIEWPORT = "Click to add Points. Hold CTRL and click to delete points."

    def __init__(self, canvas):
        Gtk.VBox.__init__(self)
        box = Gtk.HBox()
        box.set_border_width(10)
        self.label_header = Gtk.Label(self.TEXT_NO_VIEWPORT)
        box.add(self.label_header)
        self.pack_start(box, False, False, 0)
        self.canvas = canvas
        self.canvas.set_sensitive(False)
        scrolled = Gtk.ScrolledWindow()
        scrolled.add_with_viewport(self.canvas)
        scrolled.set_min_content_width(300)
        scrolled.set_min_content_height(200)
        self.pack_start(scrolled, True, True, 0)

        hbox = Gtk.HBox()
        hbox.set_border_width(2)
        self.label_aspect = Gtk.Label("NA @ NA")
        canvas.connect("on-resize", self.set_aspect_text)
        self.label_pos = Gtk.Label(" pos: 0.0, 0.0")
        canvas.connect("on-mouse-move", self.set_position_text)
        hbox.pack_start(self.label_aspect, False, False, 0)
        hbox.pack_start(self.label_pos, False, False, 0)
        self.combo_zoom = Gtk.ComboBoxText.new_with_entry()
        self.combo_zoom.append_text("dynamic")
        self.combo_zoom.append_text("100%")
        self.combo_zoom.append_text("75%")
        self.combo_zoom.append_text("50%")
        self.combo_zoom.set_active(0)
        self.combo_zoom.connect("changed", self.set_zoom)
        hbox.pack_end(self.combo_zoom, False, False, 0)
        hbox.pack_end(Gtk.Label("zoom: "), False, False, 0)
        self.pack_start(hbox, False, False, 0)

    def set_position_text(self, canvas, mouse_event):
        try:
            x, y = mouse_event.x, mouse_event.y
            orig_x, orig_y = x / self._aspect[0], y / self._aspect[1]
            self.label_pos.set_text(" pos: %4.1f, %4.1f" % (orig_x, orig_y))
        except:
            pass

    def set_aspect_text(self, canvas, resize_event):
        try:
            self._aspect = canvas._get_aspect_x_y()
            vals = (self.canvas._original_width,
                    self.canvas._original_height,
                    int(self._aspect[0] * 100),
                    int(self._aspect[1] * 100))
            self.label_aspect.set_text("%d, %d @ %d%%, %d%%" % vals)
        except:
            pass

    def set_zoom(self, combo):
        selected = combo.get_active_text()
        combo_entry = combo.get_child()
        if selected == "dynamic":
            self.canvas.keep_aspect = False
            self.canvas.hexpand = True
            self.canvas.vexpand = True
            self.canvas.set_size_request(300, 200)
            combo_entry.set_icon_from_stock(Gtk.EntryIconPosition.PRIMARY, None)
        elif re.match("^[0-9]+%$", selected):
            zoom = int(selected[:-1])/100.
            self.canvas.keep_aspect = True
            self.canvas.hexpand = False
            self.canvas.vexpand = False
            self.canvas.set_size_request(self.canvas._original_width * zoom,
                                         self.canvas._original_height * zoom)
            combo_entry.set_icon_from_stock(Gtk.EntryIconPosition.PRIMARY, None)
        else:
            combo_entry.set_icon_from_stock(Gtk.EntryIconPosition.PRIMARY,
                                            Gtk.STOCK_DIALOG_WARNING)

    def set_viewport_size(self, width, height):
        self.label_header.set_text(self.TEXT_VIEWPORT)
        self.canvas.set_viewport_size(width, height)
        self.canvas.set_sensitive(True)


class ViewportWidget(Gtk.VBox):

    def __init__(self):
        Gtk.VBox.__init__(self)

        self.canvas = Canvas()
        self.canvascontainer = CanvasContainer(self.canvas)
        self.pack_start(self.canvascontainer, True, True, 0)

        hbox = Gtk.HBox(False, 4)
        hbox.set_border_width(10)
        self.pack_start(hbox, False, False, 0)

        treeview_gen = ClassStoreTreeViewGenerator()
        treeview_gen.add_column_text(
                "Viewport",
                attr="name",
                editable=True,
                on_edit="name")
        treeview_gen.add_column_text(
                "Color",
                attr="_fill_color",
                editable=True,
                on_edit="set_color")
        treeview_gen.add_column_number(
                "Points",
                attr=lambda vp: len(vp.nodes),
                fmt="%d",
                editable=False)
        treeview_gen.add_column_checkbox(
                "Display",
                attr="_display",
                editable=True,
                on_edit="set_display")
        treeview_gen.call_on_select_mutually_exclusive(
                "set_active",
                yes_args=(True,),
                no_args=(False,))
        self._treeview = treeview_gen.get_treeview()
        self._liststore = treeview_gen.get_liststore()

        hbox.pack_start(self._treeview, False, False, 0)

        # Associate Treeview with Canvas with Treeview
        self.canvas.viewport_getter = lambda: list(row[0] for row in self._liststore)
        self.canvas.connect("on-click", self.emit_row_changed_for_all_rows)
        self._liststore.connect("row-changed", lambda *x: self.canvas.redraw())

        # Buttons #
        button = Gtk.Button("Clear")
        button.connect("clicked", self.on_clear_viewport)
        hbox.pack_end(button, False, False, 0)

        button = Gtk.Button("Add viewport")
        button.connect("clicked", self.on_add_viewport)
        hbox.pack_end(button, False, False, 0)

        button = Gtk.Button("Remove viewport")
        button.connect("clicked", self.on_remove_viewport)
        hbox.pack_end(button, False, False, 0)

    def emit_row_changed_for_all_rows(self, *args):
        self._liststore.foreach(
                lambda store, path, iter, dummy: store.row_changed(path, iter),
                None)

    def on_remove_viewport(self, *args):
        selection = self._treeview.get_selection()
        sel = selection.get_selected()
        if not sel[1] == None:
            row = self._liststore[sel[1]]
            for node in row[0].nodes:
                self.canvas.remove_child(node)
            self._liststore.remove(sel[1])
            self.canvas.redraw()

    def on_add_viewport(self, *args):
        vp = Viewport.from_list([], name=viewportnames.next(),
                                    fill_color=viewportcolors.next())
        vp.set_active(False)
        self._liststore.append([vp])

    def on_clear_viewport(self, *args):
        for viewport in self.canvas.viewports:
            if viewport.is_active():
                self.canvas.mouse_node, self.canvas.prev_mouse_node = None, None
                for node in viewport.nodes:
                    self.canvas.remove_child(node)
                viewport.nodes = []
                self.canvas.redraw()
                self.emit_row_changed_for_all_rows()

    def on_key_press(self, *args, **kwargs):
        self.canvas.on_key_press(*args, **kwargs)

    def on_key_release(self, *args, **kwargs):
        self.canvas.on_key_release(*args, **kwargs)

    def set_viewport_size(self, width, height):
        self.canvascontainer.set_viewport_size(width, height)


class BasicWindow:
    def __init__(self):
        window = Gtk.Window()
        window.connect("delete_event", lambda *args: Gtk.main_quit())

        vpw = ViewportWidget()
        window.add(vpw)

        window.connect("key_press_event", vpw.on_key_press)
        window.connect("key_release_event", vpw.on_key_release)

        window.show_all()

        vpw.set_viewport_size(1024, 768)


if __name__ == "__main__":
    example = BasicWindow()
    #test = TestWindow()
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL) # Gtk3 screws up ctrl+c
    Gtk.main()
