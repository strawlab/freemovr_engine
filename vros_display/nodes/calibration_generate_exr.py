import roslib
roslib.load_manifest('vros_display')
roslib.load_manifest('rosbag')
roslib.load_manifest('motmot_ros_utils')
import rosbag
import rospy

import simple_geom
import display_client
import calib.imgproc
from calib.visualization import create_pcd_file_from_points, create_point_cloud_message_publisher, show_pointcloud_3d_plot, create_cylinder_publisher, create_point_publisher
from calib.reconstruct import CylinderPointCloudTransformer, interpolate_pixel_cords
from rosutils.io import decode_url
import flydra.reconstruct

import os.path
import json

import matplotlib.pyplot as plt
import scipy.interpolate
import numpy as np
import cv,cv2
import exr


CALIBMAPPING_TOPIC = '/calibration/mapping'

X_INDEX = 0
Y_INDEX = 1
Z_INDEX = 2

class Foo:
    def __init__(self, visualize=True, new_reconstructor="", mask_out=False, update_parameter_server=False):
        self.data    = {}
        self.masks   = {}
        self.cvimgs  = {}
        self.dscs    = {}
        
        self.geom = simple_geom.Cylinder(
                base=dict(x=0,y=0,z=0),
                axis=dict(x=0,y=0,z=1),
                radius=0.5)

        self.mask_out = mask_out
        self.visualize = visualize
        self.update_parameter_server = update_parameter_server

        if new_reconstructor:
            self.flydra = flydra.reconstruct.Reconstructor(
                            cal_source=new_reconstructor)
        else:
            self.flydra = None
        
    def load(self, b):
        data    = {}
        masks   = {}
        cvimgs  = {}
        dscs    = {}

        with rosbag.Bag(b, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=[CALIBMAPPING_TOPIC]):
                key = msg.display_server
                try:
                    data[key]
                except KeyError:
                    data[key] = {}

                    dsc = display_client.DisplayServerProxy(key,wait=True)
                    mask = dsc.get_display_mask()
                    cvimg = dsc.new_image(color=255, mask=~mask, nchan=3, dtype=np.uint8)

                    dscs[key] = dsc
                    cvimgs[key] = cvimg
                    masks[key] = mask
                    cv2.namedWindow(key)

                finally:
                    xyz = np.array((msg.position.x,msg.position.y,msg.position.z))
                    pixel = np.array((msg.pixel_projector.x, msg.pixel_projector.y))
                    
                    pts = []
                    for projpt in msg.points:
                        pts.append( (projpt.camera,(projpt.pixel.x,projpt.pixel.y)) )

                    try:
                        data[msg.display_server][msg.vdisp].append( (xyz,pixel,pts) )
                    except KeyError:
                        data[msg.display_server][msg.vdisp] = [ (xyz,pixel,pts) ]

        self.data = data
        self.masks = masks
        self.cvimgs = cvimgs
        self.dscs = dscs
        
    def make_exr(self, xyz_arr, points_2d_arr, dsc, filt_method='linear'):
        uv = self.geom.worldcoord2texcoord(xyz_arr)
        
        u0 = interpolate_pixel_cords(
                points_2d_arr, uv[:,0],
                img_width=dsc.width,
                img_height=dsc.height,
                method=filt_method)
        v0 = interpolate_pixel_cords(
                points_2d_arr, uv[:,1],
                img_width=dsc.width,
                img_height=dsc.height,
                method=filt_method)
                
        return u0, v0

    def fix_thing(self, u0, v0, u0nonan, v0nonan):
    
        if u0nonan.max() > 1 or u0nonan.min() > 0:
            #the cubic interpolate function is sensitive to step changes, and introduces quite large
            #errors in smoothing. Crudely replace those values with invalid (nan)
            print 'replacing out of range errors in smoothed u'
            u0[u0>1] = np.nan
            u0[u0<0] = np.nan
        if v0nonan.max() > 1 or v0nonan.min() > 0:
            #the cubic interpolate function is sensitive to step changes, and introduces quite large
            #errors in smoothing. Crudely replace those values with invalid (nan)
            print 'replacing out of range errors in smoothed u'
            v0[v0>1] = np.nan
            v0[v0<0] = np.nan
        if self.mask_out:
            #mask out invalid parts if the convex hull doesnt work....
            #mask is usually used for *= with an image, but here we will use boolean indexing to fill
            #invalid areas with nan
            #... so fiddle the mask a little
            mask = dsc.get_virtual_display_mask('vdisp').squeeze()
            mask = ~mask

            u0[mask] = mask_fill_value
            v0[mask] = mask_fill_value

    def do_exr(self):

        for ds in self.data:
            cv2.namedWindow(ds)
            img = self.cvimgs[ds]
            
            exrs_u = np.zeros((768,1024))
            exrs_v = np.zeros((768,1024))

            dsc = self.dscs[ds]

            all_3d = []
            for vdisp in self.data[ds]:
            
                vdisp_3d = []
                vdisp_2d = []
                arr = np.zeros((768,1024,3))
                arr.fill(np.nan)

                for xyz,pixel,pts in self.data[ds][vdisp]: #just do one vdisp
                    col = pixel[0]
                    row = pixel[1]
                    calib.imgproc.add_crosshairs_to_nparr(img, row=row, col=col, chan=2, sz=1)

                    if self.flydra:
                        xyz = self.flydra.find3d(pts,return_line_coords=False, undistort=True)
                    
                    vdisp_2d.append(pixel)
                    vdisp_3d.append(xyz)

                    all_3d.append(xyz)
                    
                    arr[row-2:row+2,col-2:col+2,X_INDEX] = xyz[0]
                    arr[row-2:row+2,col-2:col+2,Y_INDEX] = xyz[1]
                    arr[row-2:row+2,col-2:col+2,Z_INDEX] = xyz[2]
                    
                u0,v0 = self.make_exr(
                            np.array(vdisp_3d, dtype=np.float),
                            np.array(vdisp_2d, dtype=np.float),
                            self.dscs[ds])

                u0nonan = np.nan_to_num(u0)
                v0nonan = np.nan_to_num(v0)
                
                self.fix_thing(u0, v0, u0nonan, v0nonan)

                exrs_u += u0nonan
                exrs_v += v0nonan

                if self.visualize:
                    plt.figure()
                    plt.subplot(211)
                    plt.imshow(arr[:,:,X_INDEX])
                    plt.colorbar()
                    plt.title('%s/%s X' % (ds,vdisp))
                    plt.subplot(212)
                    plt.imshow(u0)
                    plt.title('%s/%s U' % (ds,vdisp))
                    plt.colorbar()

                    plt.figure()
                    plt.subplot(211)
                    plt.imshow(arr[:,:,Y_INDEX])
                    plt.colorbar()
                    plt.title('%s/%s Y' % (ds,vdisp))
                    plt.subplot(212)
                    plt.imshow(v0)
                    plt.colorbar()
                    plt.title('%s/%s V' % (ds,vdisp))

            u0nonan = np.nan_to_num(exrs_u)
            v0nonan = np.nan_to_num(exrs_v)
            self.fix_thing(exrs_u, exrs_v, u0nonan, v0nonan)

            #save the exr file, -1 means no data, not NaN
            exrs_u[np.isnan(exrs_u)] = -1
            exrs_v[np.isnan(exrs_v)] = -1
            exrpath = decode_url('%s.exr' % ds)
            exr.save_exr(exrpath, r=exrs_u, g=exrs_v, b=np.zeros_like(exrs_u))

            #save the resulting geometry to the parameter server
            if self.update_parameter_server:
                geom = self.geom.to_geom_dict()
                dsc.set_geometry(geom)
                dsc.set_binary_exr(exrpath)

            if self.visualize:
                plt.figure()
                plt.imshow(exrs_u)
                plt.colorbar()
                plt.title('%s U' % ds)

                plt.figure()
                plt.imshow(exrs_v)
                plt.colorbar()
                plt.title('%s V' % ds)

            create_pcd_file_from_points(
                decode_url('/tmp/%sflydra3d.pcd' % ds),
                all_3d)
            all_3d_arr = np.array(all_3d)

            cv2.imshow(ds, img)

if __name__ == "__main__":
    rospy.init_node('calibration_generate_exr')
    cv2.startWindowThread()

    C = decode_url("~/FLYDRA/vros-calibration/CALIB20120907_171639.bag")

    c = Foo()
    c.load(C)
    c.do_exr()
    
    if c.visualize:
        plt.show()
    else:
        rospy.spin()
