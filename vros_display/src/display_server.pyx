# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-

# ROS stuff
ROS_PACKAGE_NAME='vros_display'
import roslib; roslib.load_manifest(ROS_PACKAGE_NAME)

import vros_display.srv
import rosmsg2json

import rospy
import geometry_msgs.msg
import roslib

# standard Python modules
import sys, time, os, warnings, threading, Queue, tempfile
import json
import argparse

import pprint

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
              ) nogil except +
        void setup_viewer(std_string json_config) nogil except +
        void update( double, Vec3, Quat )
        void frame() nogil except +
        int done() nogil except +

        vector[std_string] get_stimulus_plugin_names() nogil except +
        std_string get_current_stimulus_plugin_name() nogil except +
        void set_stimulus_plugin(std_string) nogil except +
        void stimulus_send_json_message(std_string,std_string, std_string) nogil except +
        vector[std_string] stimulus_get_topic_names(std_string) nogil except +
        std_string stimulus_get_message_type(std_string, std_string) nogil except +

        int getXSize() nogil except +
        int getYSize() nogil except +

# ================================================================

def _log(var, msg=""):
    print "--->%s: \n%s" % (msg, pprint.pformat(var,indent=4))

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

def get_physical_display_dict(physical_display_id=None):
    return rospy.get_param('/physical_displays',{}).get(physical_display_id,{})

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
    cdef object _current_subscribers
    cdef object physical_display_dict
    cdef object physical_display_id
    cdef object _pose_lock
    cdef Vec3* pose_position
    cdef Quat* pose_orientation
    cdef object subscription_mode

    def __init__(self,ros_package_name):
        self._current_subscribers = []
        self._commands = Queue.Queue()
        self._pose_lock = threading.Lock()
        with self._pose_lock:
            self.pose_position = new Vec3(0,0,0)
            self.pose_orientation = new Quat(0,0,0,1)

        self.subscription_mode = 'always'

        default_config = os.path.join(roslib.packages.get_pkg_dir(ros_package_name),'sample_data','config.xml')

        parser = argparse.ArgumentParser(
            description="VR display generator",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)

        parser.add_argument('--mode',
                            choices=['virtual_world','cubemap','overview','geometry_texture','vr_display'],
                            default='vr_display')

        parser.add_argument('--observer_radius', default=0.01, type=float) # 1cm if units are meters
        parser.add_argument('--two_pass', default=False, action='store_true')
        parser.add_argument('--show_geom_coords', default=False, action='store_true')
        parser.add_argument('--config', default=default_config, type=str, help='config file describing the setup')
        parser.add_argument('--physical_display_id', type=str, default=None)

        # use argparse, but only after ROS did its thing
        argv = rospy.myargv()
        args = parser.parse_args(argv[1:])

        self.physical_display_id = args.physical_display_id or "display_server"
        rospy.init_node(self.physical_display_id)

        self.physical_display_dict = get_physical_display_dict(self.physical_display_id)

        vros_display_basepath = roslib.packages.get_pkg_dir(ros_package_name)
        config_file = args.config
        print 'using config file',config_file

        self.dsosg = new DSOSG(std_string(vros_display_basepath),
                               std_string(args.mode),
                               args.observer_radius,
                               std_string(config_file),
                               args.two_pass,
                               args.show_geom_coords,
                               )
        rospy.Subscriber("pose", geometry_msgs.msg.Pose, self.pose_callback)

        json_config = json.dumps(self.physical_display_dict)
        fd = open('/tmp/%s.json' % self.physical_display_id,mode='w')
        fd.write( json_config )
        fd.close()

        self.dsosg.setup_viewer(std_string(json_config))

        rospy.Service('~get_display_info',
                      vros_display.srv.GetDisplayInfo,
                      self.handle_get_display_info)
        rospy.Service('~get_display_server_mode',
                      vros_display.srv.GetDisplayServerMode,
                      self.handle_get_display_server_mode)
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
            print 'plugin:',name
            if self.subscription_mode == 'always':
                self.register_subscribers(name)

        self._switch_to_stimulus_plugin('Stimulus3DDemo') # default stimulus

    def handle_get_display_server_mode(self,request):
        # this is called in some callback thread by ROS
        plugin = self.dsosg.get_current_stimulus_plugin_name().c_str()
        response = vros_display.srv.GetDisplayServerModeResponse()
        response.mode = plugin
        return response

    def handle_get_display_info(self,request):
        # this is called in some callback thread by ROS
        result = {'id':self.physical_display_id,
                  'width':self.dsosg.getXSize(),
                  'height':self.dsosg.getYSize(),
                  }

        response = vros_display.srv.GetDisplayInfoResponse()
        response.info_json = json.dumps(result)
        return response

    def handle_set_display_server_mode(self,request):
        # this is called in some callback thread by ROS
        request_mode = request.mode

        plugin = self.dsosg.get_current_stimulus_plugin_name().c_str()
        if plugin == request_mode:
            return vros_display.srv.SetDisplayServerModeResponse()

        # We're in a different thread than main draw loop, so put command into queue.
        self.call_pseudo_synchronous( cmd_dict={'command':'enter stimulus plugin',
                                                'name': request_mode},
                                      )
        plugin = self.dsosg.get_current_stimulus_plugin_name().c_str()

        if plugin == request_mode:
            return vros_display.srv.SetDisplayServerModeResponse()
        else:
            raise RuntimeError("Did not switch to requested mode %s."%(request_mode,))

    def handle_return_to_standby(self,request):
        # this is called in some callback thread by ROS
        warnings.warn( 'call to ROS service ~return_to_standby made', DeprecationWarning )
        new_request = vros_display.srv.SetDisplayServerMode()
        new_request.mode = 'StimulusStandby'
        self.handle_set_display_server_mode(new_request)
        return vros_display.srv.ReturnToStandbyResponse()

    def handle_blit_compressed_image(self,request):
        # this is called in some callback thread by ROS
        plugin = self.dsosg.get_current_stimulus_plugin_name().c_str()
        assert plugin == b'Stimulus2DBlit'

        # put on command queue for main thread.
        image = request.image
        json_image = rosmsg2json.rosmsg2json(image)
        self.call_pseudo_synchronous( cmd_dict={'command':'send plugin message',
                                                'plugin': plugin,
                                                'topic_name': 'blit_images',
                                                'msg_json': json_image},
                                      lock=False,
                                      )

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

    def call_pseudo_synchronous(self, cmd_dict=None,lock=True):
        if lock:
            condition = threading.Condition()
        else:
            condition = None
        self._commands.put( (condition,cmd_dict) )
        if lock:
            with condition: # acquire lock
                condition.wait() # release the lock, wait for notify(), and re-acquire the lock

    def _switch_to_stimulus_plugin(self,name):

        # This is done in the main thread between frame drawing.

        if self.subscription_mode == 'current_only':

            # tear down old topic name listeners
            while len(self._current_subscribers):
                sub = self._current_subscribers.pop()
                sub.unregister()

        # activate plugin
        self.dsosg.set_stimulus_plugin(std_string(name))

        if self.subscription_mode == 'current_only':
            plugin = self.dsosg.get_current_stimulus_plugin_name()
            self.register_subscribers(plugin.c_str())

    def register_subscribers(self, plugin):
        # establish new topic name listeners
        tmp = self.dsosg.stimulus_get_topic_names(std_string(plugin))
        new_topic_names = [tmp.at(i).c_str() for i in range(tmp.size())]

        for tn in new_topic_names:
            msg_type = self.dsosg.stimulus_get_message_type(std_string(plugin),std_string(tn))
            self._create_subscriber( plugin, tn, msg_type.c_str() )

    def _stimulus_plugin_callback(self, msg, callback_args):
        # This is called in some callback thread by ROS. After
        # conversion to JSON string, put on command queue for main thread.
        plugin,topic_name = callback_args
        msg_json = rosmsg2json.rosmsg2json(msg)

        self.call_pseudo_synchronous( cmd_dict={'command':'send plugin message',
                                                'plugin': plugin,
                                                'topic_name': topic_name,
                                                'msg_json': msg_json},
                                      lock=False,
                                      )


    def _create_subscriber(self, plugin, topic_name, message_type_name ):
        message_type = _import_message_name( message_type_name )
        sub = rospy.Subscriber(topic_name, message_type,
                               callback=self._stimulus_plugin_callback, callback_args=(plugin,topic_name))
        self._current_subscribers.append(sub)

    def run(self):
        cdef int do_shutdown
        cdef Vec3 position
        cdef Quat orientation
        cdef double now

        do_shutdown = 0
        while not rospy.is_shutdown():
            try:
                while 1:
                    cmd = self._commands.get_nowait()
                    (condition, cmd_dict) = cmd
                    if condition is not None:
                        with condition: # acquire lock
                            if cmd_dict['command'] == 'enter stimulus plugin':
                                name = cmd_dict['name']
                                self._switch_to_stimulus_plugin(name)
                                condition.notifyAll()
                            else:
                                raise ValueError('did not understand command "%s"'%cmd_dict['command'])
                    else:
                        if cmd_dict['command'] == 'send plugin message':
                            self.dsosg.stimulus_send_json_message(std_string(cmd_dict['plugin']),
                                                                  std_string(cmd_dict['topic_name']),
                                                                  std_string(cmd_dict['msg_json']))
                        else:
                            raise ValueError('did not understand command "%s"'%cmd_dict['command'])
            except Queue.Empty:
                pass

            with self._pose_lock:
                now = rospy.get_time()
                self.dsosg.update( now, deref(self.pose_position), deref(self.pose_orientation))

            with nogil:
                self.dsosg.frame()
                if self.dsosg.done():
                    do_shutdown = 1
            if do_shutdown:
                rospy.signal_shutdown('dsosg was done')
            #time.sleep(0.001) # spin ROS listeners

def main(ros_package_name):
    node = MyNode(ros_package_name)
    node.run()

if __name__=='__main__':
    main(ROS_PACKAGE_NAME)
