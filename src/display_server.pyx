# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil -*-

ROS_PACKAGE_NAME = 'flyvr'
import roslib
roslib.load_manifest(ROS_PACKAGE_NAME)
roslib.load_manifest('std_msgs')
import rospy

import flyvr.srv
import geometry_msgs.msg
import std_msgs.msg
from flyvr.msg import ROSPath
import flyvr.msg
from geometry_msgs.msg import Quaternion, Point

import flyvr.rosmsg2json as rosmsg2json

import sys
import time
import os
import warnings
import threading
import Queue
import tempfile
import json
import argparse
import xmlrpclib

import numpy as np
cimport numpy as np

from cython.operator cimport dereference as deref

# -------------------------------------------- std -----

cdef extern from "<vector>" namespace "std": # tested on Cython 0.14.1 (older may not work)
    cdef cppclass vector[T]:
        vector()
        void push_back(T&)
        int size()
        T& at(int)

cdef extern from "<string>" namespace "std":
    ctypedef char* const_char_ptr "const char*"
    cdef cppclass std_string "std::string":
        std_string()
        std_string(char*)
        const_char_ptr c_str()

cdef extern from "stdint.h":
    ctypedef int uint32_t

# -------------------------------------------- OSG -----

cdef extern from "osg/Vec2" namespace "osg":
    cdef cppclass Vec2:
        Vec2()
        Vec2(double,double)
        double& operator[](int)

cdef extern from "osg/Vec3" namespace "osg":
    cdef cppclass Vec3:
        Vec3()
        Vec3(double,double,double)
        double& operator[](int)

cdef extern from "osg/Quat" namespace "osg":
    cdef cppclass Quat:
        Quat()
        Quat(double,double,double,double) # x,y,z,w
        double& operator[](int)

cdef extern from "osg/Array" namespace "osg":
    cdef cppclass Vec2Array:
        Vec2Array()
        void push_back(Vec2)
    cdef cppclass Vec3Array:
        Vec3Array()
        void push_back(Vec3)

cdef extern from "osg/Group" namespace "osg":
    cdef cppclass Group:
        Group()

cdef extern from "osg/Geode" namespace "osg":
    cdef cppclass Geode:
        Geode()

# -------------------------------------------- DSOSG -----
cdef extern from "dsosg.h" namespace "dsosg":
    ctypedef struct TrackballManipulatorState:
        Quat rotation
        Vec3 center
        double distance

    cdef cppclass DSOSG:
        DSOSG(std_string flyvr_basepath,
              std_string mode,
              float observer_radius,
              std_string config_data_dir,
              int two_pass,
              int show_geom_coords,
              int tethered_mode,
              int slave
              ) nogil except +
        void setup_viewer(std_string viewer_window_name, std_string json_config, int pbuffer) nogil except +
        void update( double, Vec3, Quat )
        void frame() nogil except +
        int done() nogil except +

        vector[std_string] get_stimulus_plugin_names() nogil except +
        std_string get_current_stimulus_plugin_name() nogil except +
        void set_stimulus_plugin(std_string) nogil except +
        void stimulus_receive_json_message(std_string,std_string, std_string) nogil except +
        vector[std_string] stimulus_get_topic_names(std_string) nogil except +
        std_string stimulus_get_message_type(std_string, std_string) nogil except +

        int getXSize() nogil except +
        int getYSize() nogil except +

        float getFrameRate() nogil except +
        setCursorVisible(int visible) nogil except +
        void setCaptureImageFilename(std_string name) nogil except +
        void setCaptureOSGFilename(std_string name) nogil except +
        void setGamma(float gamma) nogil except +
        void setRedMax(int red_max) nogil except +

        TrackballManipulatorState getTrackballManipulatorState() nogil except +
        void setTrackballManipulatorState(TrackballManipulatorState s) nogil except +

# ================================================================

def _import_message_name( message_type_name ):
    message_type_name = message_type_name.replace('/','.') # split on '.' or '/'
    modules = message_type_name.split('.')

    packages = modules[:-1]
    name = modules[-1]
    packages.append('msg')
    dotpackages = '.'.join(packages)
    try:
        __import__( dotpackages)
    except ImportError:
        roslib.load_manifest(packages[0])
        try:
            __import__( dotpackages)
        except ImportError:
            dotpackages = '.'.join(packages[:-1])
            __import__( dotpackages)
    real_module = sys.modules[dotpackages]
    message_type = getattr(real_module,name)
    return message_type

def _get_verts_from_viewport(viewport):
    # allow (deprecated) backward compatibility for old specification of quadrilateral viewport
    if len(viewport)<3:
        raise ValueError('need at least 3 vertices to define viewport')

    # check that we have list of (x,y) tuples
    is_list_of_2d_verts = False
    for xy in viewport:
        is_list_of_2d_verts = hasattr(xy,'__len__')
        if not is_list_of_2d_verts:
            break
    verts = viewport
    if not is_list_of_2d_verts:
        if len(viewport)==4:
            warnings.warn('DeprecationWarning: old quad viewport specification being loaded')
            l,b,w,h = viewport
            verts = [ (l,b),
                      (l,b+h),
                      (l+w,b+h),
                      (l+w,b) ]
        else:
            raise ValueError('expected list of tuples')
    return verts

def fixup_config( orig_config_dict ):
    """fixup 'stimulus_plugins' list to lookup ROS package names

    For example, this part of a config file::

        stimulus_plugins:
            - path: $(find my_ros_package)/lib
              name: MyStimulus

    will be replaced with::

        stimulus_plugins:
            - path: /some/path/to_my_ros_package/lib
              name: MyStimulus

    This allows specifying stimuli relative to the ROS install rather
    than an absolute path. The syntax should be identical to the
    substitutions done in ROS launch files. (If it's not, it's a bug).
    """
    def fixup( plugin_dict ):
        pp = plugin_dict['path']
        pp2 = rosmsg2json.fixup_path( pp )
        plugin_dict['path'] = pp2
        return plugin_dict
    config_dict = orig_config_dict.copy()
    if 'p2c' in config_dict:
        config_dict['p2c'] = rosmsg2json.fixup_path( config_dict['p2c'] )
    if 'p2g' in config_dict:
        config_dict['p2g'] = rosmsg2json.fixup_path( config_dict['p2g'] )

    orig_plugins = config_dict.get('stimulus_plugins',[])
    plugins = [ fixup(p) for p in orig_plugins ]
    config_dict['stimulus_plugins'] = plugins
    config_file = '/tmp/%s.json' % rospy.get_name()
    with open(config_file,mode='w') as f:
        f.write(json.dumps(config_dict))

    return config_dict, config_file

cdef object osg_quat_to_msg_quat(Quat q):
    result = Quaternion( q[0], q[1], q[2], q[3] ) # x,y,z,w
    return result

cdef object osg_vec3_to_msg_point(Vec3 v):
    result = Point( v[0], v[1], v[2] ) # x,y,z
    return result

cdef class MyNode:
    cdef DSOSG* dsosg
    cdef object _commands
    cdef object _commands_lock
    cdef object _current_subscribers
    cdef object _pose_lock
    cdef object _mode_lock
    cdef object _mode_change
    cdef object _pub_fps
    cdef object _pub_mode
    cdef Vec3* _pose_position
    cdef Quat* _pose_orientation
    cdef object _subscription_mode
    cdef int _throttle
    cdef object _timer
    cdef object _gamma
    cdef object _red_max

    def __init__(self,ros_package_name):
        self._current_subscribers = []
        self._commands = Queue.Queue()
        self._commands_lock = threading.Lock()
        self._pose_lock = threading.Lock()
        self._mode_lock = threading.Lock()
        self._mode_change =  None
        self._pose_position = new Vec3(0,0,0)
        self._pose_orientation = new Quat(0,0,0,1)
        self._subscription_mode = 'always'

        parser = argparse.ArgumentParser(
            description="VR display generator",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)

        parser.add_argument('--mode',
                            choices=['virtual_world','cubemap','overview','geometry','geometry_texture','vr_display'],
                            default='vr_display')
        parser.add_argument('--observer_radius', default=0.01, type=float) # 1cm if units are meters
        parser.add_argument('--pbuffer', default=False, action='store_true')
        parser.add_argument('--two_pass', default=False, action='store_true')
        parser.add_argument('--throttle', default=False, action='store_true')
        parser.add_argument('--slave', default=False, action='store_true',
            help='In a multiprocess VR setup, this is a slave')
        parser.add_argument('--show_geom_coords', default=False, action='store_true')
        parser.add_argument('--config', type=str,
            help='JSON configuration file describing the setup. '\
                 'If specified configuration is taken from here, otherwise config is taken from '\
                 'ROS parameters under this node')
        parser.add_argument('--stimulus', type=str, default=None,
            help='The stimulus to start in (overrides any latched ROS topic')

        # use argparse, but only after ROS did its thing
        argv = rospy.myargv()
        args = parser.parse_args(argv[1:])

        rospy.init_node("display_server")

        rospy.loginfo("slave instance: %s" % args.slave)

        self._throttle = args.throttle
        rospy.loginfo("throttle framerate: %s" % self._throttle)

        config_dict = rospy.get_param('~',{})
        rospy.loginfo("starting display_server")
        if args.config and os.path.exists(args.config):
            config_file = args.config
            with open(config_file,'r') as f:
                try:
                    rospy.loginfo("using config file")
                    config_dict = json.load(f)
                except ValueError:
                    pass
        elif config_dict:
            rospy.loginfo("using ros config")
            #the exr file can be specified as a base64 string. In that case we decode it and write it
            #to a tmp file
            p2g = config_dict.get('p2g',None)
            if isinstance(p2g, xmlrpclib.Binary):
                exrfile = '/tmp/%s.exr' % rospy.get_name()
                with open(exrfile, 'wb') as exr:
                    exr.write(p2g.data)
                config_dict['p2g'] = exrfile
                rospy.loginfo("decoded exr file and saved to %s" % exrfile)

            config_dict, config_file = fixup_config( config_dict )
        else:
            rospy.loginfo("using default config")
            config_file = os.path.join(roslib.packages.get_pkg_dir(ros_package_name),'config','config.json')
            config_dict = json.load(open(config_file,'r'))

        rospy.loginfo("config_file = %s" % config_file)

        tethered_mode = config_dict.get('tethered_mode',True)
        rospy.loginfo("tethered_mode: %s" % tethered_mode)

        self._gamma = config_dict.get('gamma', 1.0)
        rospy.loginfo("gamma correction: %s" % self._gamma)

        self._red_max = config_dict.get('red_max', False)
        rospy.loginfo("red max: %s" % self._red_max)

        rospy.Subscriber("pose", geometry_msgs.msg.Pose, self.pose_callback)
        rospy.Subscriber("stimulus_mode", std_msgs.msg.String, self.mode_callback)

        rospy.Subscriber("~gamma", std_msgs.msg.Float32, self.gamma_callback)
        rospy.Subscriber("~red_max", std_msgs.msg.Bool, self.red_max_callback)

        #wait for 2 seconds in case an existing stimulus mode is already latched
        #(also gives a chance to get the initial pose)
        #
        #but, if the user specified a mode on the command line, give up on the
        #latched stimulus
        t0 = t1 = rospy.get_time()
        while (t1 - t0) < 2.0: #seconds
            rospy.sleep(0.1)
            t1 = rospy.get_time()
            with self._mode_lock:
                if self._mode_change is not None:
                    rospy.loginfo('got latched simulus mode %s' % self._mode_change)
                    break
        with self._mode_lock:
            if args.stimulus is not None:
                self._mode_change = args.stimulus
            elif self._mode_change is None:
                self._mode_change = 'Stimulus3DDemo'

                self._mode_change = 'Stimulus3DDemo'

        rospy.loginfo('selecting initial simulus mode %s' % self._mode_change)

        flyvr_basepath = roslib.packages.get_pkg_dir(ros_package_name)
        self.dsosg = new DSOSG(std_string(flyvr_basepath),
                               std_string(args.mode),
                               args.observer_radius,
                               std_string(config_file),
                               args.two_pass,
                               args.show_geom_coords,
                               tethered_mode,
                               args.slave
                               )
        #these subscribers access self.dsosg
        rospy.Subscriber("~capture_frame_to_path", ROSPath, self.capture_image_callback)
        rospy.Subscriber("~capture_osg_to_path", ROSPath, self.capture_osg_callback)
        rospy.Subscriber("~trackball_manipulator_state",
                         flyvr.msg.TrackballManipulatorState,
                         self.manipulator_callback)

        display_window_name = rospy.get_name();
        display_json_str = json.dumps(config_dict['display'])
        self.dsosg.setup_viewer(std_string(display_window_name),std_string(display_json_str),
                                args.pbuffer)

        rospy.Service('~get_display_info',
                      flyvr.srv.GetDisplayInfo,
                      self.handle_get_display_info)
        rospy.Service('~set_display_server_mode',
                      flyvr.srv.SetDisplayServerMode,
                      self.handle_set_display_server_mode)
        rospy.Service('~return_to_standby',
                      flyvr.srv.ReturnToStandby,
                      self.handle_return_to_standby)
        rospy.Service('~blit_compressed_image',
                      flyvr.srv.BlitCompressedImage,
                      self.handle_blit_compressed_image)
        rospy.Service('~get_trackball_manipulator_state',
                      flyvr.srv.GetTrackballManipulatorState,
                      self.handle_get_trackball_manipulator_state)

        self._pub_fps = rospy.Publisher('~framerate', std_msgs.msg.Float32)
        self._pub_fps.publish(0)
        self._pub_mode = rospy.Publisher('~stimulus_mode', std_msgs.msg.String, latch=True)
        self._pub_mode.publish(self._mode_change)

        plugin_names = self.dsosg.get_stimulus_plugin_names()
        for i in range( plugin_names.size() ):
            name = plugin_names.at(i).c_str()
            rospy.loginfo('loaded plugin: %s' % name)
            if self._subscription_mode == 'always':
                self.register_subscribers(name)

    def handle_get_display_info(self,request):
        # this is called in some callback thread by ROS
        result = {'id':rospy.get_name(),
                  'width':self.dsosg.getXSize(),
                  'height':self.dsosg.getYSize(),
                  'framerate':self.dsosg.getFrameRate(),
                  }

        try:
            virtualDisplays = rospy.get_param('~display/virtualDisplays')
        except KeyError:
            pass
        else:
            result['virtualDisplays'] = virtualDisplays

        response = flyvr.srv.GetDisplayInfoResponse()
        response.info_json = json.dumps(result)
        return response

    def handle_set_display_server_mode(self, request):
        with self._mode_lock:
            self._mode_change = request.mode
        return flyvr.srv.SetDisplayServerModeResponse()

    def handle_return_to_standby(self,request):
        with self._mode_lock:
            self._mode_change = 'StimulusStandby'
        return flyvr.srv.ReturnToStandbyResponse()

    def handle_blit_compressed_image(self,request):
        # this is called in some callback thread by ROS
        plugin = self.dsosg.get_current_stimulus_plugin_name().c_str()

        if plugin != b'Stimulus2DBlit':
            #change to image blit mode
            with self._mode_lock:
                self._mode_change = 'Stimulus2DBlit'

        # put on command queue for main thread.
        image = request.image
        json_image = rosmsg2json.rosmsg2json(image)
        with self._commands_lock:
            self._commands.put({'command':'send plugin message',
                                'plugin': plugin,
                                'topic_name': 'blit_images',
                                'msg_json': json_image})
        return flyvr.srv.BlitCompressedImageResponse()

    def handle_get_trackball_manipulator_state(self,request):
        # This is called in some callback thread by ROS.
        # (Should it be handled in draw thread?)
        cdef TrackballManipulatorState ts

        response = flyvr.srv.GetTrackballManipulatorStateResponse()
        result = response.data
        ts = self.dsosg.getTrackballManipulatorState()
        result.rotation = osg_quat_to_msg_quat(ts.rotation)
        result.center = osg_vec3_to_msg_point(ts.center)
        result.distance = ts.distance
        return response

    def pose_callback(self, msg):
        # this is called in some callback thread by ROS
        new_position = new Vec3(msg.position.x,
                                msg.position.y,
                                msg.position.z)
        new_orientation = new Quat(msg.orientation.x,
                                   msg.orientation.y,
                                   msg.orientation.z,
                                   msg.orientation.w)
        with self._pose_lock:
            del self._pose_position
            del self._pose_orientation
            self._pose_position = new_position
            self._pose_orientation = new_orientation

    def gamma_callback(self, msg):
        self._gamma = msg.data

    def red_max_callback(self, msg):
        self._red_max = msg.data

    def capture_image_callback(self, msg):
        d = rosmsg2json.rosmsg2dict(msg)
        fname = d['data']
        rospy.loginfo("will capture next image frame to filename: %r"%fname)
        self.dsosg.setCaptureImageFilename(std_string(fname))

    def capture_osg_callback(self, msg):
        d = rosmsg2json.rosmsg2dict(msg)
        fname = d['data']
        rospy.loginfo("will capture next osg frame to filename: %r"%fname)
        self.dsosg.setCaptureOSGFilename(std_string(fname))

    def manipulator_callback(self, msg):
        # This is called in some callback thread by ROS.
        # (Should it be handled in draw thread?)
        cdef TrackballManipulatorState ts
        r = msg.rotation
        v = msg.center
        ts.rotation = Quat( r.x, r.y, r.z, r.w )
        ts.center   = Vec3( v.x, v.y, v.z )
        ts.distance = msg.distance
        self.dsosg.setTrackballManipulatorState(ts)

    def mode_callback(self, msg):
        with self._mode_lock:
            self._mode_change = msg.data

    def register_subscribers(self, plugin):
        # establish new topic name listeners
        tmp = self.dsosg.stimulus_get_topic_names(std_string(plugin))
        new_topic_names = [tmp.at(i).c_str() for i in range(tmp.size())]

        for tn in new_topic_names:
            msg_type = self.dsosg.stimulus_get_message_type(std_string(plugin),std_string(tn))
            self.create_subscriber( plugin, tn, msg_type.c_str() )

    def stimulus_plugin_callback(self, msg, callback_args):
        # This is called in some callback thread by ROS. After
        # conversion to JSON string, put on command queue for main thread.
        plugin,topic_name = callback_args
        msg_json = rosmsg2json.rosmsg2json(msg)

        with self._commands_lock:
            self._commands.put({'command':'send plugin message',
                                                'plugin': plugin,
                                                'topic_name': topic_name,
                                                'msg_json': msg_json})

    def create_subscriber(self, plugin, topic_name, message_type_name ):
        message_type = _import_message_name( message_type_name )
        sub = rospy.Subscriber(topic_name, message_type,
                               callback=self.stimulus_plugin_callback, callback_args=(plugin,topic_name))
        self._current_subscribers.append(sub)

    def run(self):
        cdef int do_shutdown
        cdef Vec3 position
        cdef Quat orientation
        cdef double now
        cdef double last

        do_shutdown = 0
        last = rospy.get_time()
        while not rospy.is_shutdown():
            with self._commands_lock:
                while True:
                    try:
                        cmd_dict = self._commands.get_nowait()
                    except Queue.Empty:
                        break

                    self.dsosg.stimulus_receive_json_message(std_string(cmd_dict['plugin']),
                                                          std_string(cmd_dict['topic_name']),
                                                          std_string(cmd_dict['msg_json']))

            with self._mode_lock:
                if self._mode_change:
                    if self._subscription_mode == 'current_only':
                        # tear down old topic name listeners
                        while len(self._current_subscribers):
                            sub = self._current_subscribers.pop()
                            sub.unregister()

                    # activate plugin
                    rospy.loginfo("Setting stimulus plugin %s" % self._mode_change)
                    self.dsosg.set_stimulus_plugin(std_string(self._mode_change))

                    if self._subscription_mode == 'current_only':
                        plugin = self.dsosg.get_current_stimulus_plugin_name()
                        self.register_subscribers(plugin.c_str())

                    #publish the mode change
                    self._pub_mode.publish(self._mode_change)

                    self._mode_change = None


            with self._pose_lock:
                now = rospy.get_time()
                self.dsosg.update( now, deref(self._pose_position), deref(self._pose_orientation))

            if self._gamma is not None:
                self.dsosg.setGamma(self._gamma)
                self._gamma = None

            if self._red_max is not None:
                self.dsosg.setRedMax(self._red_max)
                self._red_max = None

            with nogil:
                self.dsosg.frame()
                if self.dsosg.done():
                    do_shutdown = 1
            if do_shutdown:
                rospy.signal_shutdown('dsosg was done')

            if (now - last) > 1.0:
                self._pub_fps.publish(self.dsosg.getFrameRate())
                last = now

            #time.sleep(0.001) # spin ROS listeners
            if self._throttle:
                time.sleep(0.1) # run at 10 fps

def main(ros_package_name):
    node = MyNode(ros_package_name)
    node.run()

if __name__=='__main__':
    main(ROS_PACKAGE_NAME)
