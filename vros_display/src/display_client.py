import rospy
import vros_display.srv
import vros_display.msg

import warnings
import tempfile
import time
import os.path
import xmlrpclib

import json
import numpy as np
import scipy.misc

import fill_polygon

class DisplayServerProxy(object):

    IMAGE_COLOR_BLACK = 0
    IMAGE_COLOR_WHITE = 255
    IMAGE_NCHAN = 3

    def __init__(self, display_server_node_name=None, wait=False, prefer_parameter_server_properties=False):
        if not display_server_node_name:
            self._server_node_name = rospy.resolve_name('display_server')
        else:
            self._server_node_name = rospy.resolve_name(display_server_node_name)

        self._info_cached = {}

        self._use_param_server = prefer_parameter_server_properties
        if self._use_param_server:
            rospy.logwarn('parameters will be feteched from the parameter server not the remote instance')

        rospy.loginfo('trying display server: %s' % self._server_node_name)
        if wait:
            rospy.loginfo('waiting for display server: %s' % self._server_node_name)
            rospy.wait_for_service(self.get_fullname('set_display_server_mode'))

        self.get_display_server_mode_proxy = rospy.ServiceProxy(self.get_fullname('get_display_server_mode'),
                                                                vros_display.srv.GetDisplayServerMode)
        self.set_display_server_mode_proxy = rospy.ServiceProxy(self.get_fullname('set_display_server_mode'),
                                                                vros_display.srv.SetDisplayServerMode)
        self.blit_compressed_image_proxy = rospy.ServiceProxy(self.get_fullname('blit_compressed_image'),
                                                                vros_display.srv.BlitCompressedImage)


    @property
    def name(self):
        return self._server_node_name

    @property
    def width(self):
        return self.get_display_info()['width']

    @property
    def height(self):
        return self.get_display_info()['height']

    def get_fullname(self,name):
        return self._server_node_name+'/'+name

    def _spin_wait(self,mode):
        done = False
        first_mode = None
        while not done:
            response = self.get_display_server_mode_proxy()
            if response.mode == mode:
                done = True
            elif mode=='rotate_forest' and response.mode == 'scene3d_metamode':
                # backwards compatibility
                done = True
            time.sleep(0.02) # wait 20 msec

    def enter_standby_mode(self):
        response = self.get_display_server_mode_proxy()

        # return to standby mode in server if needed
        if response.mode != 'standby':
            return_to_standby_proxy = rospy.ServiceProxy(self.get_fullname('return_to_standby'),
                                                         vros_display.srv.ReturnToStandby)

            return_to_standby_proxy()
            self._spin_wait('StimulusStandby') # wait until in standby mode

    def enter_2dblit_mode(self):
        self.set_mode('Stimulus2DBlit')

    def set_mode(self,mode):
        # put server in mode
        if mode == 'display2d':
            warnings.warn("translating stimulus name 'display2d'->'Stimulus2DBlit'",DeprecationWarning)
            mode = 'Stimulus2DBlit'

        self.set_display_server_mode_proxy(mode)
        if mode=='quit':
            return

        # Wait until in desired mode - important so published messages
        # get to the receiver.
        self._spin_wait(mode)

    def get_mode(self):
        return self.get_display_server_mode_proxy().mode

    def get_display_info(self, nocache=False):
        if nocache or not self._info_cached:
            if self._use_param_server:
                self._info_cached = rospy.get_param(self._server_node_name+'/display')
            try:
                get_display_info_proxy = rospy.ServiceProxy(self.get_fullname('get_display_info'),
                                                            vros_display.srv.GetDisplayInfo)
                result = get_display_info_proxy()
                self._info_cached = json.loads(result.info_json)
            except:
                pass
        return self._info_cached

    def _get_viewport_index(self, name):
        viewport_ids = []
        viewport_idx = -1
        for i,obj in enumerate(self.get_display_info()['virtualDisplays']):
            viewport_ids.append(obj['id'])
            if obj['id'] == name:
                viewport_idx = i
                break
        return viewport_idx

    def get_virtual_display_points(self, vdisp_name):
        viewport_idx = self._get_viewport_index(vdisp_name)
        virtual_display = self.get_display_info()['virtualDisplays'][viewport_idx]

        all_points_ok = True
        # error check
        for (x,y) in virtual_display['viewport']:
            if (x >= self.width) or (y >= self.height):
                all_points_ok = False
                break
        if all_points_ok:
            points = virtual_display['viewport']
        else:
            points = []
        return points

    def get_virtual_display_mask(self, vdisp_name):
        points = self.get_virtual_display_points(vdisp_name)
        image = np.zeros((self.height, self.width, 1), dtype=np.bool)
        fill_polygon.fill_polygon(points, image, 1)
        return image

    def show_image(self, fname, unlink=False):
        try:
            image = vros_display.msg.VROSCompressedImage()
            image.format = os.path.splitext(fname)[-1]
            image.data = open(fname).read()
        finally:
            if unlink:
                os.unlink(fname)
        self.blit_compressed_image_proxy(image)

    def show_pixels(self, arr):
        fname = tempfile.mktemp('.png')
        scipy.misc.imsave(fname,arr)
        self.show_image(fname, unlink=True)

    def new_image(self, color, mask=None, nchan=None, dtype=np.uint8):
        if nchan == None:
            nchan = self.IMAGE_NCHAN
        arr = np.zeros((self.height,self.width,nchan),dtype=dtype)
        arr.fill(color)
        if mask != None:
            arr *= mask
        #arr[:,:,3]=255
        return arr

    @property
    def geometry(self):
        return rospy.get_param(self._server_node_name+"/geom")

    def set_geometry(self, var):
        #FIXME: what else is compulsory?
        assert "model" in var
        rospy.set_param(self._server_node_name+"/geom", var)

    def set_binary_exr(self, path):
        print path,self._server_node_name+"/p2g"
        with open(path,'rb') as f:
            b = xmlrpclib.Binary(f.read())
            rospy.set_param(self._server_node_name+"/p2g", b)
            

