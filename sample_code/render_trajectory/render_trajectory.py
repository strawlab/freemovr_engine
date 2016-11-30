#!/usr/bin/env python
import os
import time
import numpy as np
import tempfile

import roslib; roslib.load_manifest('freemoovr')
import rospy
import freemoovr.display_client
import freemoovr.msg
from freemoovr.msg import ROSPath
from geometry_msgs.msg import Pose

TAU = 2*np.pi

class RenderTrajectory:
    def __init__(self,output_dir,output_fname_fmt):
        rospy.init_node('render_trajectory')
        if output_dir.startswith("/tmp/"):
            if not os.path.isdir(output_dir):
                os.makedirs(output_dir)
        else:
            if not os.path.isdir(output_dir):
                raise Exception("won't render to preexisting dir %s" % output_dir)
            else:
                os.makedirs(output_dir)
        self.output_dir = output_dir
        self.output_fname_fmt = output_fname_fmt

        freemoovr.display_client.DisplayServerProxy.set_stimulus_mode('Stimulus3DDemo')

        self.dscs = {}
        self.capture = {}
        self.cam = {}
        for name,node in [('vr','/ds_virtual_world'),
                          ('cubemap','/ds_cubemap'),
                          ('geometry','/ds_geometry'),
                          ]:
            p=freemoovr.display_client.DisplayServerProxy(display_server_node_name=node,
                                                      wait=True)
            self.dscs[name]=p

            pathpub=rospy.Publisher(p.name+'/capture_frame_to_path',
                                ROSPath,
                                latch=True)
            self.capture[name]=pathpub

            if name != 'cubemap':
                cam_pub = rospy.Publisher(p.name+'/trackball_manipulator_state',
                                          freemoovr.msg.TrackballManipulatorState,
                                          latch=True)
                self.cam[name] = cam_pub
        self.pose_pub = rospy.Publisher('pose', Pose, latch=True)

        if 1:
            # generate artificial trajectory
            t = np.linspace(0,3,200)
            self.poses = np.zeros( (len(t),7) ) # columns: x,y,z, qx,qy,qz,qw
            self.poses[:,0] = 0.35*np.cos(TAU*t*0.5)
            self.poses[:,2] = 0.5
            self.poses[:,6] = 1

        if 1:
            # setup camera position
            for name in self.cam:
                # easiest way to get these:
                #   rosservice call /ds_geometry/get_trackball_manipulator_state
                if name=='vr':
                    msg = freemoovr.msg.TrackballManipulatorState()
                    msg.rotation.x = 0.340491356063
                    msg.rotation.y = 0.13693607086
                    msg.rotation.z = 0.313095377359
                    msg.rotation.w = 0.875948305335
                    msg.center.x = 1.89151716232
                    msg.center.y = -1.83406555653
                    msg.center.z = 2.26704955101
                    msg.distance = 1.5655520953
                elif name=='geometry':
                    msg = freemoovr.msg.TrackballManipulatorState()
                    msg.rotation.x = 0.122742295197
                    msg.rotation.y = 0.198753058426
                    msg.rotation.z = 0.873456803025
                    msg.rotation.w = 0.427205763051
                    msg.center.x = -0.187373220921
                    msg.center.y = -0.0946640968323
                    msg.center.z = 0.282709181309
                    msg.distance = 1.5655520953
                else:
                    raise ValueError('unknown name %r'%name)
                self.cam[name].publish(msg)
            time.sleep(0.5) # give display servers time to catch up

    def render_frame(self,current_frame):
        pose = self.poses[current_frame]
        msg = Pose()
        for i in range(3):
            setattr(msg.position,'xyz'[i], pose[i] )
        for i in range(4):
            setattr(msg.orientation,'xyzw'[i], pose[i+3])
        self.pose_pub.publish(msg)
        time.sleep(0.01) # give message a change to get to display server

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

        return wait

    def run(self):
        for i in range(len(self.poses)):
            print self.render_frame(i)

def main():
    tmpdir = tempfile.mkdtemp()
    node = RenderTrajectory(output_dir=tmpdir,output_fname_fmt='frame_%s_%05d.png')
    node.run()

if __name__=='__main__':
    main()
