import rospy
import vros_display.srv
import vros_display.msg

import warnings
import tempfile
import time
import os.path

import json
import scipy.misc

class DisplayServerProxy(object):
    def __init__(self, display_server_node_name=None, wait=False):
        if not display_server_node_name:
            self._server_node_name = rospy.resolve_name('display_server')
        else:
            self._server_node_name = rospy.resolve_name(display_server_node_name)

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

    def get_display_info(self):
        get_display_info_proxy = rospy.ServiceProxy(self.get_fullname('get_display_info'),
                                                    vros_display.srv.GetDisplayInfo)
        result = get_display_info_proxy()
        result_dict = json.loads(result.info_json)
        return result_dict

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

