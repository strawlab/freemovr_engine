#!/usr/bin/env python
import os
import time
import numpy as np

import roslib; roslib.load_manifest('flyvr')
import rospy
import flyvr.display_client
import flyvr.msg
from flyvr.msg import ROSPath
from geometry_msgs.msg import Pose

TAU = 2*np.pi

class RenderTrajectory:
    def __init__(self,output_dir,output_fname_fmt):
        rospy.init_node('render_trajectory')
        assert not os.path.exists(output_dir), \
               "won't render to preexisting dir %r"%(output_dir,)
        os.makedirs(output_dir)
        self.output_dir = output_dir
        self.output_fname_fmt = output_fname_fmt

        flyvr.display_client.DisplayServerProxy.set_stimulus_mode('Stimulus3DDemo')

        self.dscs = {}
        self.capture = {}
        for name,node in [('vr','/display_server'),
                          ('cubemap','/ds_cubemap')]:
            p=flyvr.display_client.DisplayServerProxy(display_server_node_name=node,
                                                      wait=True)
            self.dscs[name]=p
            pub=rospy.Publisher(p.name+'/capture_frame_to_path',
                                ROSPath,
                                latch=True)
            self.capture[name]=pub

        self.cam_pub = rospy.Publisher(self.dscs['vr'].name+'/trackball_manipulator_state',
                                       flyvr.msg.TrackballManipulatorState,
                                       latch=True)
        self.pose_pub = rospy.Publisher('pose', Pose, latch=True)

        if 1:
            # generate artificial trajectory
            t = np.linspace(0,3,200)
            self.poses = np.zeros( (len(t),7) ) # columns: x,y,z, qx,qy,qz,qw
            self.poses[:,0] = 0.5*np.cos(TAU*t*0.5)
            self.poses[:,2] = 0.5
            self.poses[:,6] = 1

        if 1:
            # setup camera position
              msg = flyvr.msg.TrackballManipulatorState()
              msg.rotation.x = 0.149473585075
              msg.rotation.y = 0.36104283051
              msg.rotation.z = 0.854972787701
              msg.rotation.w = 0.341067814652
              msg.distance = 1.5655520953
              self.cam_pub.publish(msg)
              time.sleep(0.5) # give display servers time to catch up

    def render_frame(self,current_frame):
        pose = self.poses[current_frame]
        msg = Pose()
        for i in range(3):
            setattr(msg.position,'xyz'[i], pose[i] )
        for i in range(4):
            setattr(msg.orientation,'xyzw'[i], pose[i+3])
        self.pose_pub.publish(msg)
        time.sleep(0.005) # give message a change to get to display server

        wait = []
        for name in self.capture:
            fname = self.output_fname_fmt%(name,current_frame)
            full_fname = os.path.join(self.output_dir,fname)
            assert not os.path.exists(full_fname)

            self.capture[name].publish(full_fname)
            wait.append(full_fname)

        timeout_t = time.time() + 10.0 #10 seconds
        for full_fname in wait:
            success = False
            while not rospy.is_shutdown() and not success and time.time() < timeout_t:
                time.sleep(0.1)
                # wait for new frame to be saved
                if os.path.exists(full_fname):
                    # TODO: check that the image is actually valid and makes sense
                    success = True
            if not success:
                raise ValueError('requested frame that never came')

    def run(self):
        for i in range(len(self.poses)):
            self.render_frame(i)

def main():
    node = RenderTrajectory(output_dir='tmp',output_fname_fmt='frame_%s_%05d.png')
    node.run()

if __name__=='__main__':
    main()
