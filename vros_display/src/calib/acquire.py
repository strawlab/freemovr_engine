import roslib; roslib.load_manifest('sensor_msgs')
import rospy
import sensor_msgs.msg

import numpy as np
import time
import os.path
import Queue

class CameraHandler(object):
    def __init__(self,topic_prefix=''):
        self.topic_prefix=topic_prefix
        rospy.Subscriber( '%s/image_raw'%self.topic_prefix, sensor_msgs.msg.Image,
                          self.get_image_callback)
        self.pipeline_max_latency = 0.15 # 150 msec
        self.last_image = None
        self.im_queue = None
    def set_im_queue(self,q):
        self.im_queue = q
    def get_image_callback(self,msg):
        if self.im_queue is None:
            return
        self.im_queue.put((self.topic_prefix,msg))

class Runner(object):
    def __init__(self,cam_handlers):
        self.cam_handlers = cam_handlers

        max_cam_latency = max( [ch.pipeline_max_latency for ch in self.cam_handlers ])
        display_pipeline_max_latency = 0.2 # 200 msec
        self.wait_duration = max_cam_latency + display_pipeline_max_latency
        self.im_queue = Queue.Queue()
        for ch in self.cam_handlers:
            ch.set_im_queue(self.im_queue)

    def get_images(self,n_per_camera=1):
        self.clear_queue()
        tstart = time.time()
        self.cycle_duration( self.wait_duration )
        t_earliest = tstart + self.wait_duration
        self.clear_queue()
        result = {}
        for ch in self.cam_handlers:
            result[ch.topic_prefix] = []
        while not self._is_done(result,n_per_camera):
            topic_prefix, msg = self.im_queue.get(1,10.0) # block, 10 second timeout
            t_image = msg.header.stamp.to_sec()
            t_diff = abs(t_image-t_earliest)
            if t_diff > 10.0:
                raise ValueError('image timestamps more than 10 seconds different (was %f)' % t_diff)
            if t_image < t_earliest:
                # image too old
                continue
            result[topic_prefix].append( msg )
        return result

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


