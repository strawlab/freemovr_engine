import roslib
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('dynamic_reconfigure')

import rospy
import sensor_msgs.msg
import dynamic_reconfigure.srv
import dynamic_reconfigure.encoding

import numpy as np
import time
import os.path
import queue

class CameraHandler(object):
    def __init__(self,topic_prefix='',debug=False,enable_dynamic_reconfigure=False):
        self.topic_prefix=topic_prefix
        self.debug = debug
        rospy.Subscriber( '%s/image_raw'%self.topic_prefix, sensor_msgs.msg.Image,
                          self.get_image_callback)
        self.pipeline_max_latency = 0.2
        self.last_image = None
        self.im_queue = None

        self.recon = None
        if enable_dynamic_reconfigure:
            self.recon = rospy.ServiceProxy('%s/set_parameters'%self.topic_prefix, dynamic_reconfigure.srv.Reconfigure)
            self.recon_cache = {}

    def reconfigure(self, **params):
        if self.recon is not None:
            changed = {}
            for k,v in list(params.items()):
                if k in self.recon_cache:
                    if self.recon_cache[k] != v:
                        changed[k] = v
                else:
                    changed[k] = v

            if changed:
                msg = dynamic_reconfigure.encoding.encode_config(params)
                self.recon_cache.update(changed)
                self.recon(msg)
                if self.im_queue is not None:
                    #clear the queue so we get a new image with the new settings
                    while True:
                        try:
                            self.im_queue.get_nowait()
                        except queue.Empty:
                            break

    def set_im_queue(self,q):
        self.im_queue = q

    def get_image_callback(self,msg):
        if self.im_queue is None:
            return
        try:
            if self.debug:
                print("%s got image: %f" % (self.topic_prefix, msg.header.stamp.to_sec()))
            self.im_queue.put_nowait((self.topic_prefix,msg))
        except queue.Full:
            if self.debug:
                print(self.topic_prefix,"full")

class _Runner(object):
    def __init__(self,cam_handlers,ros_latency=0.2,queue_depth=20):
        self.cam_handlers = cam_handlers
        self.im_queue = queue.Queue(len(cam_handlers)*queue_depth)
        for ch in self.cam_handlers:
            ch.set_im_queue(self.im_queue)
        self.ros_latency = ros_latency
        self.max_cam_latency = max( [ch.pipeline_max_latency for ch in self.cam_handlers ])
        self._result = {}

    @property
    def result(self):
        return self._result

    @property
    def result_as_nparray(self):
        res = {}
        for cam in self._result:
            nimgs = len(self._result[cam])
            tmpres = [0]*nimgs
            for i in range(nimgs):
                msg = self._result[cam][i]
                shape = (msg.height, msg.width)
                imarr = np.fromstring(msg.data,dtype=np.uint8)
                imarr.shape = (msg.height, msg.width)
                tmpres[i] = imarr
            #sad to use dstack here, IMO res[cam][:,:,i] = imarr
            #should have worked. 
            res[cam] = np.dstack(tmpres)
        return res

    def cycle_duration( self, dur ):
        tstart = time.time()
        while (time.time() - tstart) < dur:
            time.sleep(0.05) # wait 50 msec

    def clear_queue(self):
        q = self.im_queue
        while 1:
            try:
                q.get_nowait()
            except queue.Empty:
                break

    def _is_done(self,rdict,n_per_camera,verbose=False):
        done=True
        for topic_prefix in list(rdict.keys()):
            if verbose:
                rospy.loginfo('  _is_done() has %d frames for %r'%(len(rdict[topic_prefix]), topic_prefix))
            if len(rdict[topic_prefix]) < n_per_camera:
                done=False
        return done

class SimultaneousCameraRunner(_Runner):
    def __init__(self,cam_handlers,**kwargs):
        _Runner.__init__(self, cam_handlers,**kwargs)

    def get_images(self,n_per_camera, pre_func=None, pre_func_args=[], post_func=None, post_func_args=[], verbose=False):
        self._result.clear()
        for ch in self.cam_handlers:
            self._result[ch.topic_prefix] = []

        #clear the queue
        self.clear_queue()

        if pre_func: pre_func(*pre_func_args)
        t_latest = time.time() + (self.ros_latency + self.max_cam_latency)*n_per_camera

        #wait for the images to arrive
        while not self._is_done(self._result,n_per_camera,verbose=verbose):
            try:
                topic_prefix, msg = self.im_queue.get(1,10.0) # block, 10 second timeout
            except queue.Empty:
                continue
            t_image = msg.header.stamp.to_sec()
            if t_image > t_latest:
                rospy.logwarn("image from %s at t=%f was too slow (by %f)" % (topic_prefix, t_image, t_image - t_latest))
            self._result[topic_prefix].append( msg )

        if post_func: post_func(*post_func_args)

class SequentialCameraRunner(_Runner):
    def __init__(self,cam_handlers,**kwargs):
        _Runner.__init__(self, cam_handlers,**kwargs)
        self.wait_duration = kwargs.get("wait_duration", 0.1)
        self.check_earliest = False
        self.check_latest = False

    def get_images(self,n_per_camera,verbose=False):
        self._result.clear()
        for ch in self.cam_handlers:
            self._result[ch.topic_prefix] = []

        t_earliest = time.time()
        self.clear_queue()
        t_latest = t_earliest + (self.ros_latency + self.max_cam_latency)

        while not self._is_done(self._result,n_per_camera,verbose=verbose):
            try:
                topic_prefix, msg = self.im_queue.get(1,10.0) # block, 10 second timeout
            except queue.Empty:
                continue

            t_image = msg.header.stamp.to_sec()
            if self.check_latest and t_image > t_latest:
                rospy.logwarn("image from %s at t=%f was too slow (by %f)" % (topic_prefix, t_image, t_image - t_latest))
            if self.check_earliest and t_image < t_earliest:
                rospy.logwarn("image from %s at t=%f was too early (by %f)" % (topic_prefix, t_image, t_earliest - t_image))
                continue

            self._result[topic_prefix].append( msg )

