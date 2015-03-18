#!/usr/bin/env python
from gi.repository import Gtk, GObject
GObject.threads_init()
import types
import functools
import re
import threading

_parse_dict = {'diu': int,
               'o': lambda x: int(x, 8),
               'xX': lambda x: int(x, 16),
               'eEfFgG': float}
_parse_regex = "(%[#]?0?[-]? ?[+]?[0-9]*[.]?[0-9]*[diuoxXeEfFgG])"

def _parse(fmt, numberstring):
    """assumes only one value format in string"""
    fmt = fmt.replace('%%','\x00\x00')
    start_string, fmt_string, stop_string = re.split(_parse_regex, fmt)
    start, stop = len(start_string), len(stop_string)
    N = len(numberstring)
    s = numberstring[start:N-stop]
    for k, v in _parse_dict.items():
        if fmt_string[-1] in k:
            return _parse_dict[k](s)


class CellRendererButton(Gtk.CellRenderer):

    __gsignals__ = {
        'clicked': (GObject.SignalFlags.RUN_FIRST,
                    None,
                    (GObject.TYPE_STRING,))
    }

    text = GObject.property(type=str, default='')

    def __init__(self, text=''):
        GObject.GObject.__init__(self)
        self._xpad = 6
        self._ypad = 2
        self.props.mode = Gtk.CellRendererMode.ACTIVATABLE
        self.props.text = text

    def do_get_size (self, widget, *args):
        context = widget.get_pango_context()
        metrics = context.get_metrics(widget.get_style().font_desc,
                                      context.get_language())
        row_height = metrics.get_ascent() + metrics.get_descent()
        height = (row_height + 512 >> 10) + self._ypad * 2

        layout = widget.create_pango_layout(self.props.text)
        (row_width, layout_height) = layout.get_pixel_size()
        width = row_width + self._xpad * 2
        return (0, 0, width, height)

    def do_render(self, cr, widget, background_area, cell_area, flags):
        style = widget.get_style()
        state = widget.get_state()

        layout = widget.create_pango_layout(self.props.text)
        (layout_width, layout_height) = layout.get_pixel_size()
        layout_xoffset = (cell_area.width - layout_width) / 2 + cell_area.x
        layout_yoffset = (cell_area.height - layout_height) / 2 + cell_area.y

        Gtk.paint_box(style, cr, state, Gtk.ShadowType.OUT,
                               widget, 'button',
                               cell_area.x, cell_area.y,
                               cell_area.width, cell_area.height)
        Gtk.paint_layout(style, cr, state, True,
                                  widget, "cellrenderertext", layout_xoffset,
                                  layout_yoffset, layout)

    def do_activate(self, event, widget, path, background_area, cell_area, flags):
        self.emit('clicked', path)

GObject.type_register(CellRendererButton)


class ClassStoreTreeViewGenerator(object):

    def __init__(self):
        self._liststore = Gtk.ListStore(object)
        self._treeview = Gtk.TreeView()
        self._treeview.set_model(self._liststore)

    def add_column_text(self, name, attr=str, attr_args=(),
                        editable=False, on_edit=None, on_edit_args=()):
        """Add a column that displays text.

        name:
                title of the column
        attr:
                attribute to display.
                if attr is a string:
                    if object.attr is a function:
                        display object.attr(*attr_args)
                    else:
                        display object.attr
                if attr is a function:
                    attr(object, *attr_args)
        attr_args:
                see attr
        editable:
                if cell is editable
        on_edit:
                does something when cell is edited:
                if on_edit is a string:
                    if object.on_edit is a function:
                        calls object.on_edit(edited_value, *on_edit_args)
                    else:
                        sets object.on_edit = edited_value
                if on_edit is a function:
                    on_edit(object, edited_value, *on_edit_args)
        on_edit_args:
                see on_edit
        """
        # Prepare renderer
        renderer = Gtk.CellRendererText(editable=editable)
        if editable:
            assert (on_edit is not None), "Need on_edit function if editable."
            on_edit_func = functools.partial(self._cell_on_edit_func_text, on_edit, on_edit_args)
            renderer.connect("edited", on_edit_func)
        column = Gtk.TreeViewColumn(name)
        column.pack_start(renderer, False)
        # Prepare data func
        data_func = functools.partial(self._cell_data_func_text, attr, attr_args)
        column.set_cell_data_func(renderer, data_func)
        self._treeview.append_column(column)

    def add_column_number(self, name, attr, attr_args=(), fmt="%f",
                          editable=True, on_edit=None, on_edit_args=()):
        """Add a column that displays numbers.

        name:
                title of the column
        attr:
                attribute to display.
                if attr is a string:
                    if object.attr is a function:
                        display object.attr(*attr_args)
                    else:
                        display object.attr
                if attr is a function:
                    attr(object, *attr_args)
        attr_args:
                see attr
        format:
                number format string, i.e. "%03d" or "%3.1f"
        editable:
                if cell is editable
        on_edit:
                does something when cell is edited:
                if on_edit is a string:
                    if object.on_edit is a function:
                        calls object.on_edit(edited_value, *on_edit_args)
                    else:
                        sets object.on_edit = edited_value
                if on_edit is a function:
                    on_edit(object, edited_value, *on_edit_args)
        on_edit_args:
                see on_edit
        """
        assert fmt.replace("%%","").count("%") == 1, "fmt string should be for ONE number only"
        # Prepare renderer
        renderer = Gtk.CellRendererText(editable=editable)
        if editable:
            assert (on_edit is not None), "Need on_edit function if editable."
            on_edit_func = functools.partial(self._cell_on_edit_func_number, on_edit, on_edit_args, fmt)
            renderer.connect("edited", on_edit_func)
        column = Gtk.TreeViewColumn(name)
        column.pack_start(renderer, False)
        # Prepare data func
        data_func = functools.partial(self._cell_data_func_number, attr, attr_args, fmt)
        column.set_cell_data_func(renderer, data_func)
        self._treeview.append_column(column)

    def add_column_checkbox(self, name, attr, attr_args=(),
                            editable=True, on_edit=None, on_edit_args=()):
        """Add a column that displays a boolean value as checkbox.

        name:
                title of the column
        attr:
                attribute to display.
                if attr is a string:
                    if object.attr is a function:
                        display object.attr(*attr_args)
                    else:
                        display object.attr
                if attr is a function:
                    attr(object, *attr_args)
        attr_args:
                see attr
        editable:
                if cell is editable
        on_edit:
                does something when cell is edited:
                if on_edit is a string:
                    if object.on_edit is a function:
                        calls object.on_edit(edited_value, *on_edit_args)
                    else:
                        sets object.on_edit = edited_value
                if on_edit is a function:
                    on_edit(object, edited_value, *on_edit_args)
        on_edit_args:
                see on_edit
        """
        # Prepare renderer
        renderer = Gtk.CellRendererToggle(activatable=editable)
        if editable:
            assert (on_edit is not None), "Need on_edit function if editable."
            on_edit_func = functools.partial(self._cell_on_edit_func_checkbox, on_edit, on_edit_args)
            # FIXME: this workaround is not so nice...
            #        but required, when connected to the "toggled" signal
            #        if we don't store the bolean value in the liststore.
            tmp_store = [None]
            renderer.connect("toggled", on_edit_func, tmp_store)
        column = Gtk.TreeViewColumn(name)
        column.pack_start(renderer, False)
        # Prepare data func
        data_func = functools.partial(self._cell_data_func_checkbox, attr, attr_args)
        column.set_cell_data_func(renderer, data_func)
        self._treeview.append_column(column)


    def add_column_button(self, name, label, on_click, on_click_args=(), show_waiting_indicator=False):
        """Add a column that displays a button which calls on_click.

        name:
                title of the column
        label:
                button label
        on_click:
                call on button click
                if on_click is a string:
                    call object.on_click(*on_click_args)
                if on_click is a function:
                    call on_click(object, *on_click_args)
        on_click_args:
                see on_click
        show_waiting_indicator:
                display a waiting indicator if True
        """
        # Prepare renderer
        renderer = CellRendererButton(label)

        if show_waiting_indicator:
            spinner = Gtk.CellRendererSpinner()
        else:
            spinner = None

        on_click_func = functools.partial(self._on_click_func, spinner, on_click, on_click_args)
        renderer.connect("clicked", on_click_func)

        column = Gtk.TreeViewColumn(name)
        column.pack_start(renderer, False)
        if show_waiting_indicator:
            column.pack_start(spinner, False)
        self._treeview.append_column(column)


    def _cell_data_func_text(self, attr, attr_args, column, cell, model, iter, *dummy):
        data = self._data_func(attr, attr_args, column, cell, model, iter, *dummy)
        text = str(data)
        cell.set_property('text', text)

    def _cell_on_edit_func_text(self, on_edit, on_edit_args, widget, path, textval):
        textval = str(textval)
        self._on_edit_func(on_edit, on_edit_args, widget, path, textval)


    def _cell_data_func_number(self, attr, attr_args, fmt, column, cell, model, iter, *dummy):
        data = self._data_func(attr, attr_args, column, cell, model, iter, *dummy)
        text = fmt % data
        cell.set_property('text', text)

    def _cell_on_edit_func_number(self, on_edit, on_edit_args, fmt, widget, path, textval):
        number = _parse(fmt, textval)
        self._on_edit_func(on_edit, on_edit_args, widget, path, number)


    def _cell_data_func_checkbox(self, attr, attr_args, column, cell, model, iter, *dummy):
        data = self._data_func(attr, attr_args, column, cell, model, iter, *dummy)
        text = bool(data)
        cell.set_property('active', text)

    def _cell_on_edit_func_checkbox(self, on_edit, on_edit_args, widget, path, tmp_store):
        # textval = bool(textval)
        if tmp_store[0] is None:
            tmp_store[0] = widget.get_property("active")
        tmp_store[0] = not tmp_store[0]
        self._on_edit_func(on_edit, on_edit_args, widget, path, tmp_store[0])


    def _data_func(self, attr, attr_args, column, cell, model, iter, *dummy):
        instance = model[iter][0]
        if hasattr(attr, "__call__"):
            data = attr(instance, *attr_args)
        else:
            data = getattr(instance, attr)
            if type(data) == types.MethodType:
                # data could be a method of the class
                data = data(*attr_args)
        return data

    def _on_edit_func(self, on_edit, on_edit_args, widget, path, textval):
        instance = self._liststore[path][0]
        if hasattr(on_edit, "__call__"):
            on_edit(instance, textval, *on_edit_args)
        else:
            attr = getattr(instance, on_edit)
            if type(attr) == types.MethodType:
                # data could be a method of the class
                attr(textval, *on_edit_args)
            else:
                setattr(instance, on_edit, textval)
        iter = self._liststore.get_iter(path)
        path = self._liststore.get_path(iter)
        self._liststore.row_changed(path, iter)

    def _on_click_func(self, spinner, on_click, on_click_args, widget, path):
        instance = self._liststore[path][0]
        if hasattr(on_click, "__call__"):
            fun, fun_args = on_click, (instance,) + tuple(on_click_args)
        else:
            attr = getattr(instance, on_click)
            assert type(attr) == types.MethodType, "on_click is not a class method?"
            fun, fun_args = attr, tuple(on_click_args)
        args = (path, spinner, fun, fun_args)
        print args
        t = threading.Thread(target=self._launch_threaded, args=args)
        t.daemon = True
        t.start()

    def _spinner_start(self, spinner):
        if spinner is not None:
            spinner.set_property("active", True)
            GObject.timeout_add(200, self._spinner_pulse, spinner)

    def _spinner_pulse(self, spinner):
        if spinner.get_property("active"):
            pulse = spinner.get_property("pulse")
            spinner.set_property("pulse", pulse + 1)
            return True
        else:
            return False

    def _spinner_stop(self, spinner):
        if spinner is not None:
            spinner.set_property("active", False)

    def _launch_threaded(self, path, spinner, fun, fun_args):
        print "start"
        #GObject.idle_add(self._spinner_start, spinner)
        # start spinner
        #self._spinner_start(spinner)
        # disable row
        # ...
        try:
            fun(*fun_args)
        finally:
            #GObject.idle_add(self._spinner_stop, spinner)
            # enable row
            # ...
            # stop spinner
            #self._spinner_stop(spinner)
            print "stop"



    def _on_selection_handler(self, func, extra_args, selection):
        model, paths = selection.get_selected_rows()
        for selected_path in paths:
            instance = model[selected_path][0]
            args = (instance,) + tuple(extra_args)
            func(*args)

    def _on_selection_handler_mutually_exclusive(self, func, yes_args, no_args, selection):
        model, paths = selection.get_selected_rows()
        for row in self._liststore:
            instance = row[0]
            for selected_path in paths:
                if row.path == selected_path:
                    args = (instance,) + tuple(yes_args)
                else:
                    args = (instance,) + tuple(no_args)
                if hasattr(func, "__call__"):
                    func(*args)
                else:
                    getattr(instance, func)(*args[1:])

    def call_on_select(self, func, extra_args=()):
        handler = functools.partial(self._on_selection_handler, func, extra_args)
        return self._treeview.get_selection().connect("changed", handler)

    def call_on_select_mutually_exclusive(self, func, yes_args, no_args):
        handler = functools.partial(self._on_selection_handler_mutually_exclusive, func,
                                                                            yes_args, no_args)
        return self._treeview.get_selection().connect("changed", handler)


    def get_treeview(self):
        return self._treeview

    def get_liststore(self):
        return self._liststore

if __name__ == "__main__":
    import time

    class TEST(object):

        def __init__(self, f, t, b):
            self.f = f
            self._t = t
            self.b = b

        @property
        def is_true(self):
            print "is_true", self.b
            return self.b

        def set_new_t(self, text):
            print "set_new_t:", text
            self._t = text

        def launch(self):
            print "sleep for %fsec" % self.f
            time.sleep(self.f)
            print "done sleeping"

    ctv = ClassStoreTreeViewGenerator()
    ctv.add_column_text("Instance")
    ctv.add_column_text("text", attr="_t", editable=True, on_edit="set_new_t")
    ctv.add_column_number("number", attr="f", fmt="%3.2f", editable=True, on_edit="f")
    ctv.add_column_button("calibrate", label="Run", on_click="launch", show_waiting_indicator=True)
    treeview = ctv.get_treeview()
    liststore = ctv.get_liststore()

    a = TEST(10.0, "haha", True)
    b = TEST(6.0, "huhu", True)
    c = TEST(2.0, "hihi", False)
    liststore.append([a])
    liststore.append([b])
    liststore.append([c])

    w = Gtk.Window()
    w.connect("delete-event", Gtk.main_quit)
    w.add(treeview)
    w.show_all()

    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    Gtk.main()

