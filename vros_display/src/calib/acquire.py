import roslib; roslib.load_manifest('sensor_msgs')
import rospy
import sensor_msgs.msg

import numpy as np
import time
import os.path
import Queue

class CameraHandler(object):
    def __init__(self,topic_prefix='',debug=False):
        self.topic_prefix=topic_prefix
        self.debug = debug
        rospy.Subscriber( '%s/image_raw'%self.topic_prefix, sensor_msgs.msg.Image,
                          self.get_image_callback)
        self.pipeline_max_latency = 0.2
        self.last_image = None
        self.im_queue = None
    def set_im_queue(self,q):
        self.im_queue = q
    def get_image_callback(self,msg):
        if self.im_queue is None:
            return
        try:
            if self.debug:
                print "%s got image: %f" % (self.topic_prefix, msg.header.stamp.to_sec())
            self.im_queue.put_nowait((self.topic_prefix,msg))
        except Queue.Full:
            if self.debug:
                print self.topic_prefix,"full"

class _Runner(object):
    def __init__(self,cam_handlers,ros_latency=0.2):
        self.cam_handlers = cam_handlers
        self.im_queue = Queue.Queue(len(cam_handlers)*20)
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
            except Queue.Empty:
                break

    def _is_done(self,rdict,n_per_camera):
        done=True
        for topic_prefix in rdict.keys():
            if len(rdict[topic_prefix]) < n_per_camera:
                done=False
                break
        return done

class SimultainousCameraRunner(_Runner):
    def __init__(self,cam_handlers,**kwargs):
        _Runner.__init__(self, cam_handlers,**kwargs)

    def get_images(self,n_per_camera, pre_func=None, pre_func_args=[], post_func=None, post_func_args=[]):
        self._result.clear()
        for ch in self.cam_handlers:
            self._result[ch.topic_prefix] = []

        #clear the queue
        self.clear_queue()

        if pre_func: pre_func(*pre_func_args)
        t_latest = time.time() + (self.ros_latency + self.max_cam_latency)*n_per_camera

        #wait for the images to arrive
        while not self._is_done(self._result,n_per_camera):
            try:
                topic_prefix, msg = self.im_queue.get(1,10.0) # block, 10 second timeout
            except Queue.Empty:
                continue
            t_image = msg.header.stamp.to_sec()
            if t_image > t_latest:
                rospy.logwarn("image from %s at t=%f was too slow (by %f)" % (topic_prefix, t_image, t_image - t_latest))
            self._result[topic_prefix].append( msg )

        if post_func: post_func(*post_func_args)

class SequentialCameraRunner(object):
    def __init__(self,cam_handlers):
        _Runner.__init__(self, cam_handlers,**kwargs)

    def get_images(self,n_per_camera):
        self.clear_queue()
        tstart = time.time()
        self.cycle_duration( self.wait_duration )
        t_earliest = tstart + self.wait_duration
        self.clear_queue()
        self._result.clear()
        for ch in self.cam_handlers:
            self._result[ch.topic_prefix] = []
        self.clear_queue()
        while not self._is_done(self._result,n_per_camera):
            topic_prefix, msg = self.im_queue.get(1,10.0) # block, 10 second timeout
            t_image = msg.header.stamp.to_sec()
            t_diff = abs(t_image-t_earliest)
            if t_diff > 10.0:
                raise ValueError('image timestamps more than 10 seconds different (was %f)' % t_diff)
            if t_image < t_earliest:
                # image too old
                continue
            self._result[topic_prefix].append( msg )

