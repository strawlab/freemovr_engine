#!/usr/bin/env python
import pickle
import os
import numpy
import argparse
import collections
import numpy as np
import numpy.ma as ma
import warnings
from collections import defaultdict
import matplotlib.pyplot as plt

# ROS imports
import roslib; roslib.load_manifest('vros_display')
import rospy
import camera_model
import simple_geom

def my_subplot_rows_cols( n ):
    w = np.sqrt(n*1.5)
    w = int(np.round(w))
    h = int(np.ceil(n/float(w)))
    return h,w

class DisplayKeyPressHelper:
    def __init__(self, display, geom):
        self.display = display
        self.geom = geom
        self.addrx = {}
        self.addry = {}

        self.geom3d = self.geom.compute_for_camera_view( self.display )
        self.texcoord = self.geom.compute_for_camera_view( self.display, what='texture_coords')

        self.accum_3d = defaultdict(list)
        self.accum_2d = defaultdict(list)
        self.ax2token = {}

    def add_axes(self,name,ax_x,ax_y,addrx,addry):
        for ax in [ax_x, ax_y]:
            self.addrx[ax]=addrx
            self.addry[ax]=addry
            self.ax2token[ax] = name

    def on_key_press(self,event):
        #print 'received key',repr(event.key)
        if event.key=='x':
            if not event.inaxes:
                #print 'not in axes -- nothing to do'
                return

            ax = event.inaxes  # the axes instance
            token = self.ax2token[ax]

            v,u = event.xdata, event.ydata
            addrx = self.addrx[ax]
            addry = self.addry[ax]

            print 'display u,v',v,u # display backwards for (x,y) order
            u = int(np.round(u))
            v = int(np.round(v))
            if u < 0 or v < 0:
                print 'off image'
                return

            cam_addr = (addrx[u,v], addry[u,v])
            world = self.geom3d[u,v]
            texcoord = self.texcoord[u,v]

            print 'display u,v',v,u # display backwards for (x,y) order
            print 'world',world
            print 'texcoord',texcoord
            print 'cam_addr', cam_addr
            print

class KeyPressHelper:
    def __init__(self, camera, geom):
        self.camera = camera
        self.geom = geom
        self.addrx = {}
        self.addry = {}

        self.geom3d = self.geom.compute_for_camera_view( self.camera)
        self.texcoord = self.geom.compute_for_camera_view( self.camera, what='texture_coords')

        self.accum_3d = defaultdict(list)
        self.accum_2d = defaultdict(list)
        self.ax2token = {}
        self.width_height = {}

    def add_axes(self,name,ax_x,ax_y,addrx,addry,w,h):
        for ax in [ax_x, ax_y]:
            self.addrx[ax]=addrx
            self.addry[ax]=addry
            self.ax2token[ax] = name

        self.width_height[name] = w,h

    def on_key_press(self,event):
        #print 'received key',repr(event.key)
        if event.key in ['x','a']:
            if not event.inaxes:
                #print 'not in axes -- nothing to do'
                return

            ax = event.inaxes  # the axes instance
            token = self.ax2token[ax]

            v,u = event.xdata, event.ydata
            addrx = self.addrx[ax]
            addry = self.addry[ax]

            print 'cam u,v',v,u # display backwards for (x,y) order
            u = int(np.round(u))
            v = int(np.round(v))
            if u < 0 or v < 0:
                print 'off image'
                return

            proj = (addrx[u,v], addry[u,v])
            world = self.geom3d[u,v]
            texcoord = self.texcoord[u,v]

            print 'cam u,v',v,u # display backwards for (x,y) order
            print 'world',world
            print 'texcoord',texcoord
            print 'proj', proj

            if event.key == 'a':
                if proj[0] >= 0 and not np.isnan(world[0]):
                    self.accum_3d[token].append( world )
                    self.accum_2d[token].append( proj )
                    print 'accumulated %d points'%(len(self.accum_2d[token]),)
                    if len(self.accum_2d[token]) >= 6:
                        self.do_dlt(token)
            print
        elif event.key=='c':
            print 'cleared accum buffers'
            self.accum_3d = defaultdict(list)
            self.accum_2d = defaultdict(list)
        elif event.key=='b':
            ax = event.inaxes  # the axes instance
            token = self.ax2token[ax]
            self.do_dlt(token,save=True)
    def do_dlt(self,name,save=False):
        import dlt
        print '3d points -----------'
        print repr(self.accum_3d[name])
        print '2d points -----------'
        print repr(self.accum_2d[name])
        results = dlt.dlt( self.accum_3d[name], self.accum_2d[name], ransac=False)
        print 'display: ',name
        dlt.print_summary(results)
        if save:
            display_w, display_h = self.width_height[name]

            print 'saving with values:'
            print repr( dict(pmat=results['pmat'],
                             width=display_w,
                             height=display_h,
                             name=name ))
            display_model = camera_model.load_camera_from_pmat( results['pmat'],
                                                                width=display_w,
                                                                height=display_h,
                                                                name=name )

            fname = name.replace('/','-')
            fname = 'display-model-'+fname+'.bag'
            display_model.save_to_bagfile(fname)
            print 'saved to',fname

def get_verts( camera, geom):
    allw = []
    npts = 32
    for tc1 in [0,1]:
        tc = np.vstack( (
                np.linspace(0,1.,npts),
                tc1*np.ones( (npts,) ),
                )).T
        world = geom.model.texcoord2worldcoord(tc)
        allw.append(world)

    allw = np.concatenate(allw)

    uv = camera.project_3d_to_pixel( allw )
    return uv

global kphs
kphs = {}
def make_camera_views(data, camera=None, geom=None):
    global kphs
    topic_prefixes = None
    displays = []
    for physical_display_id in data.keys():
        for virtual_display_id in data[physical_display_id].keys():
            loaded = data[physical_display_id][virtual_display_id]
            results = loaded['data']
            displays.append( (physical_display_id, virtual_display_id) )

            if topic_prefixes is None:
                topic_prefixes = results.keys()
                topic_prefixes.sort()
            else:
                tmp_topic_prefixes = results.keys()
                tmp_topic_prefixes.sort()
                assert topic_prefixes==tmp_topic_prefixes

    for topic_prefix in topic_prefixes:
        # if camera is not None:
        #     if topic_prefix!=camera.name:
        #         continue
        # one figure for each camera
        fig = plt.figure()
        n_rows,n_cols = my_subplot_rows_cols( len(displays)*2 )
        ax_x = None
        for subplot_enum,(physical_display_id, virtual_display_id) in enumerate(displays):
            loaded = data[physical_display_id][virtual_display_id]
            results = loaded['data']

            ax_x = fig.add_subplot(n_rows,n_cols, subplot_enum*2+1, sharex=ax_x, sharey=ax_x)
            ax_y = fig.add_subplot(n_rows,n_cols, subplot_enum*2+2, sharex=ax_x, sharey=ax_x)
            if virtual_display_id is None:
                vdisplay = physical_display_id
            else:
                vdisplay = physical_display_id+'/'+virtual_display_id
            title = 'camera {camname}: {vdisplay} '.format( camname=topic_prefix,
                                                            vdisplay=vdisplay,
                                                            )
            ax_x.set_title(title+'X')
            ax_y.set_title(title+'Y')

            arr = results[topic_prefix][0]['address']
            arr = ma.masked_array(arr, mask=arr<0.0)
            mappable = ax_x.imshow( arr, interpolation='nearest' )
            #fig.colorbar(mappable)

            arr = results[topic_prefix][1]['address']
            arr = ma.masked_array(arr, mask=arr<0.0)
            ax_y.imshow( arr, interpolation='nearest' )


            if camera is not None and topic_prefix==camera.name:
                uv = get_verts( camera, geom )
                ax_x.plot( uv[:,0], uv[:,1], 'bo')
                ax_y.plot( uv[:,0], uv[:,1], 'bo')

                if fig not in kphs:
                    kph = KeyPressHelper(camera, geom)
                    fig.canvas.mpl_connect('key_press_event', kph.on_key_press)
                    kphs[fig] = kph
                else:
                    kph = kphs[fig]
                tmp_display_image = loaded['p2c_by_cam'][topic_prefix]['x']
                display_h, display_w = tmp_display_image.shape
                kph.add_axes( vdisplay, ax_x, ax_y,
                              results[topic_prefix][0]['address'],
                              results[topic_prefix][1]['address'],
                              display_w, display_h,
                              )

def make_display_views(data,display=None,geom=None):
    topic_prefixes = None
    displays = []
    for physical_display_id in data.keys():
        for virtual_display_id in data[physical_display_id].keys():
            loaded = data[physical_display_id][virtual_display_id]
            results = loaded['data']
            displays.append( (physical_display_id, virtual_display_id) )

            if topic_prefixes is None:
                topic_prefixes = results.keys()
                topic_prefixes.sort()
            else:
                tmp_topic_prefixes = results.keys()
                tmp_topic_prefixes.sort()
                assert topic_prefixes==tmp_topic_prefixes

    for (physical_display_id, virtual_display_id) in displays:
        # one figure for each virtual display
        fig = plt.figure()
        n_rows,n_cols = my_subplot_rows_cols( len(topic_prefixes)*2 )
        ax_x = None
        for subplot_enum,topic_prefix in enumerate(topic_prefixes):
            loaded = data[physical_display_id][virtual_display_id]
            p2c_by_cam = loaded['p2c_by_cam']

            ax_x = fig.add_subplot(n_rows,n_cols, subplot_enum*2+1, sharex=ax_x, sharey=ax_x)
            ax_y = fig.add_subplot(n_rows,n_cols, subplot_enum*2+2, sharex=ax_x, sharey=ax_x)
            if virtual_display_id is None:
                vdisplay = physical_display_id
            else:
                vdisplay = physical_display_id+'/'+virtual_display_id
            title = 'display {vdisplay}: {camname} '.format( camname=topic_prefix,
                                                             vdisplay=vdisplay,
                                                             )
            ax_x.set_title(title+'X')
            ax_y.set_title(title+'Y')

            arr = p2c_by_cam[topic_prefix]['x']
            arr = ma.masked_array(arr, mask=arr<0.0)
            ax_x.imshow( arr, interpolation='nearest' )

            arr = p2c_by_cam[topic_prefix]['y']
            arr = ma.masked_array(arr, mask=arr<0.0)
            ax_y.imshow( arr, interpolation='nearest' )

            fullname = physical_display_id+'/'+virtual_display_id
            if display is not None and fullname==display.name:
                uv = get_verts( display, geom )
                ax_x.plot( uv[:,0], uv[:,1], 'bo')
                ax_y.plot( uv[:,0], uv[:,1], 'bo')

                if fig not in kphs:
                    kph = DisplayKeyPressHelper(display, geom)
                    fig.canvas.mpl_connect('key_press_event', kph.on_key_press)
                    kphs[fig] = kph
                else:
                    kph = kphs[fig]

                tmp_display_image = loaded['p2c_by_cam'][topic_prefix]['x']
                display_h, display_w = tmp_display_image.shape
                kph.add_axes( vdisplay, ax_x, ax_y,
                              p2c_by_cam[topic_prefix]['x'],
                              p2c_by_cam[topic_prefix]['y'],
                              )


def visualize_dc_results(fname, camera_bagfile=None, display_bagfile=None, geometry_filename=None):
    camera=None
    display=None
    geom=None
    if camera_bagfile is not None:
        camera = camera_model.load_camera_from_bagfile( camera_bagfile )
    if display_bagfile is not None:
        display = camera_model.load_camera_from_bagfile( display_bagfile )
    if geometry_filename is not None:
        geom = simple_geom.Geometry(geometry_filename)

    fd = open(fname,mode='r')
    data = pickle.load(fd)
    fd.close()

    make_camera_views(data,camera=camera,geom=geom)
    #if camera is None:
    if 1:
        make_display_views(data,display=display,geom=geom)
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', type=str)
    parser.add_argument('--camera_bagfile', type=str, help="filename of camera-camcal.bag for calibration data")
    parser.add_argument('--display_bagfile', type=str, help="filename of display-model.bag for calibration data")
    parser.add_argument('--geometry_filename', type=str, help="JSON file with geometry description")
    args = parser.parse_args()

    visualize_dc_results(args.filename,
                         camera_bagfile=args.camera_bagfile,
                         display_bagfile=args.display_bagfile,
                         geometry_filename=args.geometry_filename )
