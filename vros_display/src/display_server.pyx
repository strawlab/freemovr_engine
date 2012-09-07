# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-

ROS_PACKAGE_NAME = 'vros_display'
import roslib
roslib.load_manifest(ROS_PACKAGE_NAME)
roslib.load_manifest('std_msgs')
import rospy

import vros_display.srv
import geometry_msgs.msg
import std_msgs.msg

import rosmsg2json

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

cdef extern from "osg/Vec3" namespace "osg":
    cdef cppclass Vec3:
        Vec3()
        Vec3(double,double,double)

cdef extern from "osg/Quat" namespace "osg":
    cdef cppclass Quat:
        Quat()
        Quat(double,double,double,double)

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
    cdef cppclass DSOSG:
        DSOSG(std_string vros_display_basepath,
              std_string mode,
              float observer_radius,
              std_string config_data_dir,
              int two_pass,
              int show_geom_coords,
              int tethered_mode,
              ) nogil except +
        void setup_viewer(std_string viewer_window_name, std_string json_config) nogil except +
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
    cdef Vec3* pose_position
    cdef Quat* pose_orientation
    cdef object subscription_mode
    cdef int throttle
    cdef object _timer

    def __init__(self,ros_package_name):
        self._current_subscribers = []
        self._commands = Queue.Queue()
        self._commands_lock = threading.Lock()
        self._pose_lock = threading.Lock()
        self._mode_lock = threading.Lock()
        with self._pose_lock:
            self.pose_position = new Vec3(0,0,0)
            self.pose_orientation = new Quat(0,0,0,1)

        self.subscription_mode = 'always'

        parser = argparse.ArgumentParser(
            description="VR display generator",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)

        parser.add_argument('--mode',
                            choices=['virtual_world','cubemap','overview','geometry','geometry_texture','vr_display'],
                            default='vr_display')
        parser.add_argument('--observer_radius', default=0.01, type=float) # 1cm if units are meters
        parser.add_argument('--two_pass', default=False, action='store_true')
        parser.add_argument('--throttle', default=False, action='store_true')
        parser.add_argument('--show_geom_coords', default=False, action='store_true')
        parser.add_argument('--config', type=str,
            help='JSON configuration file describing the setup. '\
                 'If specified configuration is taken from here, otherwise config is taken from '\
                 'ROS parameters under this node')
        parser.add_argument('--stimulus', type=str, default='Stimulus3DDemo', help='The stimulus to start in')

        # use argparse, but only after ROS did its thing
        argv = rospy.myargv()
        args = parser.parse_args(argv[1:])

        rospy.init_node("display_server")
        time.sleep(1.0) # give a second to join ROS network, else early rospy.log* messages are lost

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
            p2g = config_dict['p2g']
            if isinstance(p2g, xmlrpclib.Binary):
                exrfile = '/tmp/%s.exr' % rospy.get_name()
                with open(exrfile, 'wb') as exr:
                    exr.write(p2g.data)
                config_dict['p2g'] = exrfile
                rospy.loginfo("decoded exr file and saved to %s" % exrfile)
            config_file = '/tmp/%s.json' % rospy.get_name()
            with open(config_file,mode='w') as f:
                f.write(json.dumps(config_dict))
        else:
            rospy.loginfo("using default config")
            config_file = os.path.join(roslib.packages.get_pkg_dir(ros_package_name),'config','config.json')
            config_dict = json.load(open(config_file,'r'))

        rospy.loginfo("config_file = %s" % config_file)

        tethered_mode = config_dict.get('tethered_mode',True)
        rospy.loginfo('tethered_mode: %s'%tethered_mode)
        
        self._pub_fps = rospy.Publisher('~framerate', std_msgs.msg.Float32)
        self._pub_mode = rospy.Publisher('~stimulus_mode', std_msgs.msg.String, latch=True)

        with self._mode_lock:
            self._mode_change =  args.stimulus
            self._pub_mode.publish(self._mode_change)

        vros_display_basepath = roslib.packages.get_pkg_dir(ros_package_name)
        self.dsosg = new DSOSG(std_string(vros_display_basepath),
                               std_string(args.mode),
                               args.observer_radius,
                               std_string(config_file),
                               args.two_pass,
                               args.show_geom_coords,
                               tethered_mode,
                               )
        rospy.Subscriber("pose", geometry_msgs.msg.Pose, self.pose_callback)
        rospy.Subscriber("stimulus_mode", std_msgs.msg.String, self.mode_callback)

        display_window_name = rospy.get_name();
        display_json_str = json.dumps(config_dict['display'])
        self.dsosg.setup_viewer(std_string(display_window_name),std_string(display_json_str))

        rospy.Service('~get_display_info',
                      vros_display.srv.GetDisplayInfo,
                      self.handle_get_display_info)
        rospy.Service('~set_display_server_mode',
                      vros_display.srv.SetDisplayServerMode,
                      self.handle_set_display_server_mode)
        rospy.Service('~return_to_standby',
                      vros_display.srv.ReturnToStandby,
                      self.handle_return_to_standby)
        rospy.Service('~blit_compressed_image',
                      vros_display.srv.BlitCompressedImage,
                      self.handle_blit_compressed_image)

        plugin_names = self.dsosg.get_stimulus_plugin_names()
        for i in range( plugin_names.size() ):
            name = plugin_names.at(i).c_str()
            rospy.loginfo('loaded plugin: %s' % name)
            if self.subscription_mode == 'always':
                self.register_subscribers(name)

        self.throttle = args.throttle
        self._timer = rospy.Timer(rospy.Duration(60), # every 60 seconds
                                  self._on_heartbeat)

    def _on_heartbeat(self,event):
        rospy.loginfo('heartbeat at ' + str(event.current_real))

    def handle_get_display_info(self,request):
        # this is called in some callback thread by ROS
        result = {'id':rospy.get_name(),
                  'width':self.dsosg.getXSize(),
                  'height':self.dsosg.getYSize(),
                  'framerate':self.dsosg.getFrameRate(),
                  'virtualDisplays':rospy.get_param('~display/virtualDisplays')
                  }

        response = vros_display.srv.GetDisplayInfoResponse()
        response.info_json = json.dumps(result)
        return response

    def handle_set_display_server_mode(self, request):
        with self._mode_lock:
            self._mode_change = request.mode
        return vros_display.srv.SetDisplayServerModeResponse()

    def handle_return_to_standby(self,request):
        with self._mode_lock:
            self._mode_change = 'StimulusStandby'

    def handle_blit_compressed_image(self,request):
        # this is called in some callback thread by ROS
        plugin = self.dsosg.get_current_stimulus_plugin_name().c_str()

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
        return vros_display.srv.BlitCompressedImageResponse()

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
            del self.pose_position
            del self.pose_orientation
            self.pose_position = new_position
            self.pose_orientation = new_orientation

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
                    if self.subscription_mode == 'current_only':
                        # tear down old topic name listeners
                        while len(self._current_subscribers):
                            sub = self._current_subscribers.pop()
                            sub.unregister()

                    # activate plugin
                    rospy.loginfo("Setting stimulus plugin %s" % self._mode_change)
                    self.dsosg.set_stimulus_plugin(std_string(self._mode_change))

                    if self.subscription_mode == 'current_only':
                        plugin = self.dsosg.get_current_stimulus_plugin_name()
                        self.register_subscribers(plugin.c_str())

                    #publish the mode change
                    self._pub_mode.publish(self._mode_change)

                    self._mode_change = None


            with self._pose_lock:
                now = rospy.get_time()
                self.dsosg.update( now, deref(self.pose_position), deref(self.pose_orientation))

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
            if self.throttle:
                time.sleep(0.1) # run at 10 fps

def main(ros_package_name):
    node = MyNode(ros_package_name)
    node.run()

if __name__=='__main__':
    main(ROS_PACKAGE_NAME)
