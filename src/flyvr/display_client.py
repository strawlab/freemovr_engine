import rospy
import std_msgs.msg
import geometry_msgs.msg
import flyvr.srv
import flyvr.msg

import warnings
import tempfile
import time
import os.path
import xmlrpclib

import json
import numpy as np
import scipy.misc

#re-export the fill polygon helper
import flyvr.tools.fill_polygon as fp
fill_polygon = fp.fill_polygon

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
            rospy.logwarn('parameters will be fetched from the parameter '
                          'server not the remote instance')

        rospy.loginfo('trying display server: %s' % self._server_node_name)
        if wait:
            rospy.loginfo('waiting for display server: %s' % self._server_node_name)
            rospy.wait_for_service(self.get_fullname('set_display_server_mode'))
            _ = self.get_display_info(nocache=True) # fill our cache

        self.set_display_server_mode_proxy = rospy.ServiceProxy(self.get_fullname('set_display_server_mode'),
                                                                flyvr.srv.SetDisplayServerMode)
        self.blit_compressed_image_proxy = rospy.ServiceProxy(self.get_fullname('blit_compressed_image'),
                                                                flyvr.srv.BlitCompressedImage)
    @property
    def name(self):
        return self._server_node_name

    @property
    def width(self):
        return self.get_display_info()['width']

    @property
    def height(self):
        return self.get_display_info()['height']

    @property
    def virtual_displays(self):
        return [i['id'] for i in self.get_display_info()['virtualDisplays']]

    @staticmethod
    def set_stimulus_mode(mode):
        publisher = rospy.Publisher('/stimulus_mode', std_msgs.msg.String, latch=True)
        publisher.publish(mode)
        return publisher

    def _spin_wait_for_mode(self,mode):
        done = [False, None]
        def cb(msg):
            if mode == None or msg.data == mode:
                done[0] = True
            done[1] = msg.data
        sub = rospy.Subscriber( self.get_fullname('stimulus_mode'),
                                std_msgs.msg.String, cb )
        r = rospy.Rate(10.0)
        while not done[0]:
             r.sleep()
        sub.unregister()

        return done[1]

    def get_fullname(self,name):
        return self._server_node_name+'/'+name

    def enter_standby_mode(self):
        mode = self._spin_wait_for_mode(None)
        # return to standby mode in server if needed
        if mode != 'standby':
            return_to_standby_proxy = rospy.ServiceProxy(self.get_fullname('return_to_standby'),
                                                         flyvr.srv.ReturnToStandby)

            return_to_standby_proxy()
            self._spin_wait_for_mode('StimulusStandby') # wait until in standby mode

    def enter_2dblit_mode(self):
        self.set_mode('Stimulus2DBlit')

    def set_mode(self,mode):
        """Set the stimulus_mode of this particular display server.

        This call blocks until the stimulus_mode is switched
        registered on the display server.

        NOTE: typically one wants to set the stimulus_mode of all
        display servers simultaneously. In that case, call
        `DisplayServerProxy.set_stimulus_mode(mode)`.
        """
        rospy.logwarn('Using deprecated API: `display_client.DisplayServerProxy().set_mode(mode)`. '
                      'Instead, use display_client.DisplayServerProxy.set_stimulus_mode(mode)`')
        return DisplayServerProxy.set_stimulus_mode(mode)

    def set_mode_old(self,mode):
        # put server in mode
        if mode == 'display2d':
            warnings.warn("translating stimulus name 'display2d'->'Stimulus2DBlit'",DeprecationWarning)
            mode = 'Stimulus2DBlit'

        self.set_display_server_mode_proxy(mode)
        if mode == 'quit':
            return

        # Wait until in desired mode - important so published messages
        # get to the receiver.
        self._spin_wait_for_mode(mode)

    def get_mode(self):
        return self._spin_wait_for_mode(None)

    def _get_cached_service_call(self, paramname, servicename, nocache):
        if nocache or (paramname not in self._info_cached):
            if self._use_param_server:
                self._info_cached[paramname] = rospy.get_param(self._server_node_name+'/'+paramname)
            try:
                get_info_proxy = rospy.ServiceProxy(self.get_fullname(servicename),
                                                            flyvr.srv.GetDisplayInfo)
                result = get_info_proxy()
                self._info_cached[paramname] = json.loads(result.info_json)
            except:
                pass
        return self._info_cached[paramname]

    def get_geometry_info(self, nocache=False):
        return self._get_cached_service_call('geom','get_geometry_info',nocache)

    def get_display_info(self, nocache=False):
        return self._get_cached_service_call('display','get_display_info',nocache)

    def _get_viewport_index(self, name):
        viewport_idx = -1
        for i,obj in enumerate(self.get_display_info()['virtualDisplays']):
            if obj['id'] == name:
                viewport_idx = i
                break
        return viewport_idx

    def get_virtual_displays(self):
        return self.get_display_info()['virtualDisplays']

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

    def get_virtual_display_mask(self, vdisp_name, squeeze=False, dtype=np.bool, fill=1):
        points = self.get_virtual_display_points(vdisp_name)
        image = np.zeros((self.height, self.width, 1), dtype=dtype)
        fp.fill_polygon(points, image, fill)
        if squeeze:
            return np.squeeze(image)
        else:
            return image

    def get_display_mask(self, squeeze=False):
        """ Gets the mask of all virtual displays (logical or) """
        image = np.zeros((self.height, self.width, 1), dtype=np.bool)
        for vdisp in self.get_display_info()['virtualDisplays']:
            image |= self.get_virtual_display_mask(vdisp["id"])
        if squeeze:
            return np.squeeze(image)
        else:
            return image

    def get_virtual_display_mirror(self,vdisp_name):
        viewport_idx = self._get_viewport_index(vdisp_name)
        virtual_display = self.get_display_info()['virtualDisplays'][viewport_idx]
        return virtual_display.get('mirror',None)

    def show_image(self, fname, unlink=False):
        try:
            image = flyvr.msg.FlyVRCompressedImage()
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
        if type(color) in (tuple, list):
            assert len(color) == self.IMAGE_NCHAN
            for i,c in enumerate(color):
                arr[:,:,i] = c
        else:
            arr.fill(color)
        if mask != None:
            arr *= mask
        return arr

    @property
    def geometry(self):
        return rospy.get_param(self._server_node_name+"/geom")

    def set_geometry(self, var):
        #FIXME: what else is compulsory?
        assert "model" in var
        rospy.set_param(self._server_node_name+"/geom", var)

    def set_binary_exr(self, path):
        with open(path,'rb') as f:
            b = xmlrpclib.Binary(f.read())
            rospy.set_param(self._server_node_name+"/p2g", b)

class RenderFrameSlave:

    dsc = None

    def __init__(self, dsc):
        self.dsc = dsc

        self.path_pub = rospy.Publisher(self.dsc.name+'/capture_frame_to_path',
                                        flyvr.msg.ROSPath,
                                        latch=True)
        self.cam_pub = rospy.Publisher(self.dsc.name+'/trackball_manipulator_state',
                                       flyvr.msg.TrackballManipulatorState,
                                       latch=True)

        self.pose_pub = rospy.Publisher(self.dsc.name+'/pose',
                                        geometry_msgs.msg.Pose,
                                        latch=True)

    def set_pose(self, msg=None,x=None,y=None,z=None):
        if msg is None:
            if None in (x,y,z):
                raise Exception("Must suppy pose msg, or x,y,z coordinates")
            msg = geometry_msgs.msg.Pose()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z
        self.pose_pub.publish(msg)
        time.sleep(0.01)

    def set_view(self, msg):
        self.cam_pub.publish(msg)
        time.sleep(0.05)

    def render_frame(self, frame, posemsg):
        if os.path.exists(frame):
            raise Exception("frame already rendered")

        self.path_pub.publish(frame)
        time.sleep(0.05)

        timeout_t = time.time() + 10.0 #10 seconds
        success = False
        while not rospy.is_shutdown() and not success and time.time() < timeout_t:
            time.sleep(0.1)
            # wait for new frame to be saved
            if os.path.exists(frame):
                # TODO: check that the image is actually valid and makes sense
                success = True
        if not success:
            raise ValueError('requested frame that never came: %s' % frame)

        return frame

class StimulusSlave(object):

    def __init__(self, dsc, stimulus):
        self.dsc = dsc
        self.dsc.set_mode(stimulus)

class OSGFileStimulusSlave(StimulusSlave):

    def __init__(self, dsc, stimulus='StimulusOSGFile'):
        StimulusSlave.__init__(self, dsc, stimulus)

        self.pub_stimulus_scale = rospy.Publisher(self.dsc.name+'/model_scale',
                                                  geometry_msgs.msg.Vector3,
                                                  latch=True)
        self.pub_stimulus_centre = rospy.Publisher(self.dsc.name+'/model_pose',
                                                  geometry_msgs.msg.Pose,
                                                  latch=True)
        self.pub_stimulus = rospy.Publisher(self.dsc.name+'/stimulus_filename',
                                            std_msgs.msg.String,
                                            latch=True)

    def set_model_filename(self, fname):
        self.pub_stimulus.publish(fname)

    def set_model_origin(self, xyz):
        if len(xyz) != 3:
            raise ValueError("orgin must be 3-tuple of x,y,z (m)")
        msg = geometry_msgs.msg.Pose()
        msg.position.x = xyz[0]
        msg.position.y = xyz[1]
        msg.position.z = xyz[2]
        msg.orientation.w = 1.0
        self.pub_stimulus_centre.publish(msg)

    def set_model_scale(self, xyz):
        if len(xyz) != 3:
            raise ValueError("scale must be 3-tuple of x,y,z (m)")
        msg = geometry_msgs.msg.Vector3(*xyz)
        self.pub_stimulus_scale.publish(msg)



