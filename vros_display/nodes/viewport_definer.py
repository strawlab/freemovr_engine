#!/usr/bin/env python
# ROS imports
import roslib; roslib.load_manifest('vros_display')
import rospy

import vros_display.srv
import vros_display.msg
import display_client

import json
import argparse
import tempfile, os, sys

# Major library imports
import numpy as np
import scipy.misc
import mahotas.polygon

traits_version = None

try:
    # Enthought library imports
    import enthought.traits.api as traits
    traits_version = 3
except ImportError:
    # traits 4
    import traits.api as traits
    traits_version = 4

if traits_version==3:
    from enthought.enable.api import Component, ComponentEditor, Window
    from enthought.traits.api import HasTraits, Instance
    from enthought.traits.ui.api import Item, Group, View

    # Chaco imports
    from enthought.chaco.api import ArrayPlotData, Plot
    from enthought.chaco.tools.api import LineSegmentTool, PanTool, ZoomTool
elif traits_version==4:
    from enable.api import Component, ComponentEditor, Window
    from traits.api import HasTraits, Instance
    from traitsui.api import Item, Group, View

    # Chaco imports
    from chaco.api import ArrayPlotData, Plot
    from chaco.tools.api import LineSegmentTool, PanTool, ZoomTool
else:
    raise RuntimeError('could not identify traits')

def posint(x,maxval=np.inf):
    x = int(x)
    if x < 0:
        x = 0
    if x>maxval:
        return maxval
    return x

class ViewportDefiner(HasTraits):
    width = traits.Int
    height = traits.Int
    display_name = traits.String
    plot = Instance(Component)
    linedraw = Instance(LineSegmentTool)
    viewport_id = traits.String
    display_mode = traits.Trait('white on black', 'black on white')
    client = traits.Any
    blit_compressed_image_proxy = traits.Any
    display_info = traits.Any

    set_display_server_mode_proxy = traits.Any

    traits_view = View(
                    Group(
        Item('display_mode'),
        Item('display_name'),
        Item('viewport_id'),
                        Item('plot', editor=ComponentEditor(),
                             show_label=False),
                        orientation = "vertical"),
                    resizable=True,
                    )

    def __init__(self, *args, **kwargs):
        super(ViewportDefiner, self).__init__(*args, **kwargs)

        #find our index in the viewport list
        self.viewport_idx = -1
        for i,obj in enumerate(self.display_info['virtualDisplays']):
            if obj['id'] == self.viewport_id:
                self.viewport_idx = i

        if self.viewport_idx == -1:
            raise Exception("Could not find viewport")

        self._update_image()

        self.fqdn = self.display_name + '/display/virtualDisplays'
        self.this_virtual_display = self.display_info['virtualDisplays'][self.viewport_idx]

        all_points_ok = True
        # error check
        for (x,y) in self.this_virtual_display['viewport']:
            if (x >= self.width) or (y >= self.height):
                all_points_ok = False
                break
        if all_points_ok:
            self.linedraw.points = self.this_virtual_display['viewport']
        else:
            self.linedraw.points = []
            rospy.logwarn('invalid points')
        self._update_image()

    def _update_image(self):
        self._image = np.zeros( (self.height, self.width, 3), dtype=np.uint8)
        # draw polygon
        if len(self.linedraw.points)>=3:
            pts = [ (posint(y,self.height-1),posint(x,self.width-1)) for (x,y) in self.linedraw.points]
            mahotas.polygon.fill_polygon(pts, self._image[:,:,0])
            self._image[:,:,0] *= 255
            self._image[:,:,1] = self._image[:,:,0]
            self._image[:,:,2] = self._image[:,:,0]

        # draw red horizontal stripes
        for i in range(0,self.height,100):
            self._image[i:i+10,:,0] = 255

        # draw blue vertical stripes
        for i in range(0,self.width,100):
            self._image[:,i:i+10,2] = 255

        if hasattr(self,'_pd'):
            self._pd.set_data("imagedata", self._image)
        self.send_array()
        if len(self.linedraw.points) >= 3:
            self.update_ROS_params()

    def _plot_default(self):
        self._pd = ArrayPlotData()
        self._pd.set_data("imagedata", self._image)

        plot = Plot(self._pd, default_origin="top left")
        plot.x_axis.orientation = "top"
        img_plot = plot.img_plot("imagedata")[0]

        plot.bgcolor = "white"

        # Tweak some of the plot properties
        plot.title = "Click to add points, press Enter to clear selection"
        plot.padding = 50
        plot.line_width = 1

        # Attach some tools to the plot
        pan = PanTool(plot, drag_button="right", constrain_key="shift")
        plot.tools.append(pan)
        zoom = ZoomTool(component=plot, tool_mode="box", always_on=False)
        plot.overlays.append(zoom)

        return plot

    def _linedraw_default(self):
        linedraw = LineSegmentTool(self.plot,color=(0.5,0.5,0.9,1.0))
        self.plot.overlays.append(linedraw)
        linedraw.on_trait_change( self.points_changed, 'points[]')
        return linedraw

    def points_changed(self):
        self._update_image()

    @traits.on_trait_change('display_mode')
    def send_array(self):
        # create an array
        if self.display_mode.endswith(' on black'):
            bgcolor = (0,0,0,1)
        elif self.display_mode.endswith(' on white'):
            bgcolor = (1,1,1,1)

        if self.display_mode.startswith('black '):
            color = (0,0,0,1)
        elif self.display_mode.startswith('white '):
            color = (1,1,1,1)

        fname = tempfile.mktemp('.png')
        try:
            scipy.misc.imsave(fname, self._image )
            image = vros_display.msg.VROSCompressedImage()
            image.format = 'png'
            image.data = open(fname).read()
            self.blit_compressed_image_proxy(image)
        finally:
            os.unlink(fname)

    def get_viewport_verts(self):
        # convert to integers
        pts = [ (posint(x,self.width-1),posint(y,self.height-1)) for (x,y) in self.linedraw.points]
        # convert to list of lists for maximal json compatibility
        return [ list(x) for x in pts ]

    def update_ROS_params(self):
        viewport_verts = self.get_viewport_verts()
        self.this_virtual_display['viewport'] = viewport_verts
        self.display_info['virtualDisplays'][self.viewport_idx] = self.this_virtual_display
        rospy.set_param(self.fqdn, self.display_info['virtualDisplays'])

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--display_server', type=str, required=True, help=\
        'the path of the display server to configure')
    parser.add_argument(
        '--virtual_display_id', type=str, required=True, help=\
        'the id of the virtual display on the display server')
    # use argparse, but only after ROS did its thing
    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    rospy.init_node('viewport_definer')

    display_server = display_client.DisplayServerProxy(args.display_server)
    display_server.enter_standby_mode()
    display_server.set_mode('display2d')

    display_info = display_server.get_display_info()

    blit_compressed_image_proxy = rospy.ServiceProxy(display_server.get_fullname('blit_compressed_image'),
                                                     vros_display.srv.BlitCompressedImage)

    demo = ViewportDefiner(display_info=display_info,
                           width=display_info['width'],
                           height=display_info['height'],
                           display_name=args.display_server,
                           viewport_id = args.virtual_display_id,
                           blit_compressed_image_proxy = blit_compressed_image_proxy,
                           )

    tmp = demo.linedraw # trigger default value to be initialized. (XXX how else to do this?)
    demo.configure_traits()

if __name__ == "__main__":
    main()
