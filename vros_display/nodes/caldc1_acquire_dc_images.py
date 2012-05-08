#!/usr/bin/env python

# standard Python imports
import argparse
import Image as PIL
import numpy as np
import time
import os.path
import pickle
import math

# ROS imports
import roslib; roslib.load_manifest('vros_display')
import rospy

# local vros_display imports
import display_client
import fill_polygon
from graycode import graycode_str, graycode_arange
from calib.acquire import CameraHandler, SequentialCameraRunner

# constants
D2R = math.pi/180.0
R2D = 180.0/math.pi

X_AX=0
Y_AX=1

NCHAN=4

def get_mask(verts, width, height ):
    assert len(verts)>=3
    arr = np.zeros( (height, width, NCHAN), dtype=np.uint8)
    fill_polygon.fill_polygon( verts, arr, fill_value=1 )
    arr[:,:,3] = 1
    return arr

def localize_display( topic_prefixes=None, display_server=None, virtual_display_id=None, save_pngs=False, save_sample_masks=False ):
    assert len(topic_prefixes)>=1
    print 'topic_prefixes',topic_prefixes

    display_info = display_server.get_display_info()

    if 1:
        viewport_idx = -1

        if virtual_display_id is None:
            assert len(display_info['virtualDisplays'])==1
            viewport_idx = 0

        else:
            for i,obj in enumerate(display_info['virtualDisplays']):
                if obj['id'] == virtual_display_id:
                    viewport_idx = i

        if viewport_idx == -1:
            raise Exception("Could not find viewport")

        this_virtual_display = display_info['virtualDisplays'][viewport_idx]
        virtual_display_id = this_virtual_display['id']

    if virtual_display_id is not None:

        virtual_display = this_virtual_display
        print 'virtual_display',virtual_display

        viewport_verts=virtual_display['viewport']
        viewport_mask = get_mask( viewport_verts, display_info['width'], display_info['height'] )
    else:
        # no virtual display specified -- use entire display
        viewport_mask = np.ones( ( display_info['height'], display_info['width'], NCHAN), dtype=np.uint8 )
        print 'no virtual display specified -- using entire display'

    print 'virtual_display_id',virtual_display_id

    rospy.init_node('localize_display', anonymous=True)
    cam_handlers = [CameraHandler(prefix) for prefix in topic_prefixes]

    display_server.enter_standby_mode()
    display_server.set_mode('Stimulus2DBlit')

    width = display_info['width']
    height = display_info['height']

    physical_display_id = display_server.get_fullname('')
    physical_display_id = physical_display_id.strip('/') # eliminate trailing slash

    runner = SequentialCameraRunner(cam_handlers)
    runner.clear_queue()
    runner.cycle_duration(1.0) # give panda a chance to startup...
    runner.clear_queue()

    binary_coding='natural'
    assert binary_coding in ('gray','natural')

    if 1:
        result_images = []
        for axis in [X_AX,Y_AX]:
            arr = np.zeros((height,width,NCHAN),dtype=np.uint8)
            arr[:,:,3]=255
            if binary_coding=='natural':
                if axis==X_AX:
                    vals = np.arange(width, dtype=np.uint16)
                else:
                    vals = np.arange(height, dtype=np.uint16)
            elif binary_coding=='gray':
                if axis==X_AX:
                    vals = graycode_arange(width, dtype=np.uint16)
                else:
                    vals = graycode_arange(height, dtype=np.uint16)
            n_line_images = 30
            bitnos = range(-3,13) + list( 100+np.arange(n_line_images) )
            for bitno in bitnos:
                for flip in [0,1]:
                    if bitno==-3:
                        if flip==0: # invalid
                            continue
                        # all black for bitno -3
                        arr[:,:,:3]=0
                    elif bitno==-2:
                        if flip==0: # invalid
                            continue
                        # all white for bitno -2
                        arr[:,:,:3]=255
                    elif bitno==-1:
                        if flip==0: # invalid
                            continue
                        # all gray for bitno -1
                        arr[:,:,:3]=127
                    elif bitno < 100:
                        mask = 1 << bitno
                        tmp1 = np.bitwise_and(vals,mask)
                        bits = tmp1==0
                        if flip:
                            bits = ~bits
                        bits = 255*bits
                        for chan in range(3):
                            if axis==X_AX:
                                ovals_len = height
                            else:
                                ovals_len = width
                            for oval in range(ovals_len):
                                if axis==X_AX:
                                    arr[oval,:,chan] = bits
                                else:
                                    arr[:,oval,chan] = bits
                    else:
                        if flip==0: # invalid
                            continue
                        modval = bitno-100
                        if axis==X_AX:
                            vals2 = np.arange(width, dtype=np.uint16) % n_line_images
                        else:
                            vals2 = np.arange(height, dtype=np.uint16) % n_line_images

                        bits = vals2==modval
                        if not flip:
                            bits = ~bits
                        bits = 255*bits
                        for chan in range(3):
                            if axis==X_AX:
                                ovals_len = height
                            else:
                                ovals_len = width
                            for oval in range(ovals_len):
                                if axis==X_AX:
                                    arr[oval,:,chan] = bits
                                else:
                                    arr[:,oval,chan] = bits

                    arr = arr*viewport_mask
                    display_server.show_pixels(arr)
                    n_per_camera=10
                    print 'getting images for bitno %d'%bitno
                    imdict = runner.get_images(n_per_camera=n_per_camera)

                    if save_sample_masks and bitno == -2:
                        for cam in imdict:
                            fname = "%s_%s_%s_mask.sample.png" % (
                                display_server.name[1:],
                                virtual_display_id,
                                cam[1:])
                            save_image(fname, imdict[cam][1])
                            print "saved sample mask images: %s" % fname

                    if 0:
                        # don't average images at this stage, just save all images
                        for topic_prefix,msgs in imdict.iteritems():
                            for msg in msgs:

                                imarr = np.fromstring(msg.data,dtype=np.uint8)
                                imarr.shape = (msg.height, msg.width)

                                result_images.append( (axis,bitno,flip,topic_prefix,imarr) )
                    else:
                        # average images
                        for topic_prefix,msgs in imdict.iteritems():
                            cum_images = []
                            for msg in msgs:

                                imarr = np.fromstring(msg.data,dtype=np.uint8)[:msg.height*msg.width]
                                imarr.shape = (msg.height, msg.width)
                                cum_images.append( imarr )
                            imarr = np.mean( np.array(cum_images,dtype=np.float), axis=0)
                            imarr = np.round(imarr).astype(np.uint8)
                            result_images.append( (axis,bitno,flip,topic_prefix,imarr) )

                    if save_pngs:
                        save_images('localize_axis%s_bits%02d_%d'%(axis,bitno,flip),imdict)
        output_data = {'images':result_images,
                       'display_width_height': (display_info['width'],display_info['height']) ,
                       'physical_display_id': physical_display_id,
                       'virtual_display_id':virtual_display_id,
                       }
        fname = os.path.abspath('images-%s-%s.pkl'%(physical_display_id,virtual_display_id))
        fd = open(fname,mode='w')
        pickle.dump(output_data,fd)
        fd.close()
        print 'saved grey code images to %s' % fname

def save_image(fname, msg):
    arr = np.fromstring(msg.data,dtype=np.uint8)
    arr.shape = (msg.height, msg.width)

    pil_im = PIL.fromarray( arr[::-1] )
    pil_im.save(fname)


def save_images(imname,imdict):
    for topic_prefix in imdict.keys():
        for i, msg in enumerate(imdict[topic_prefix]):
            fname = '%s_%s_%03d.png'%(imname,topic_prefix,i)
            save_image(fname, msg)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        'topic_prefixes', type=str,
        help='camera topic prefix of the images used to view the projector (e.g. /camnode)',
        nargs='+')
    parser.add_argument(
        '--display-server', type=str, required=True, help=\
        'the path of the display server to configure')
    parser.add_argument(
        '--virtual-display-id', type=str)
    parser.add_argument(
        '--save-pngs', action='store_true', default=False)
    parser.add_argument(
        '--generate-sample-mask-images', action='store_true', default=False)

    # use argparse, but only after ROS did its thing
    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])
    # if len(args.topic_prefixes)==0:
    #     args.topic_prefixes.append('')

    display_server = display_client.DisplayServerProxy(args.display_server)
    display_server.enter_standby_mode()

    localize_display( topic_prefixes=args.topic_prefixes,
                      display_server = display_server,
                      virtual_display_id = args.virtual_display_id,
                      save_pngs = args.save_pngs,
                      save_sample_masks = args.generate_sample_mask_images)
