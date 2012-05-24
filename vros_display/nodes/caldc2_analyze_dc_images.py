#!/usr/bin/env python
import pickle
import matplotlib.pyplot as plt
import numpy as np
import numpy.ma as ma
import collections
import argparse
import time
import os.path
import scipy.ndimage
import scipy.stats
import scipy.misc.pilutil
import warnings
import Image as PIL

# ROS imports
import roslib; roslib.load_manifest('vros_display')

# local ROS package imports
from graycode import graycode_str, graycode_arange
from exr import save_exr
from calib.imgproc import load_mask_image

def savearr( fname, arr, is_int=True ):
    assert arr.ndim==2
    if is_int:
        savearr = arr.astype( np.uint16 )
        assert np.allclose(savearr, arr)
    else:
        savearr = arr

    fd = open(fname,mode='w')
    for row in savearr:
        rowbuf = ' '.join( [ str(ri) for ri in row ] )
        fd.write( rowbuf + '\n')
    fd.close()

def debug_array( name, d, opt_val=None):
    if 1:
        return
    arr = d[name]
    print '-----',name,
    if opt_val is not None:
        print opt_val
    else:
        print
    print arr[225:230,278:283]

def noise_free_sign(arr,n_sigma=1.0):
    n = np.array(arr,copy=True)
    s = np.std(n.flat)
    n[ abs(n) < s*n_sigma ] = 0
    return np.sign(n)

def find_display_in_images( input_data, visualize=False, min_lum_sig=20, MSB_min_lum_sig=100, no_subpixel=False,  mask_image_dir=None):
    """
    Parameters
    ----------
    visualize : bool
      Whether to plot results
    min_lum_sig : int
      The minimum luminance signal to find a location. Set higher to
      be more discriminating.
    MSB_min_lum_sig : int
      (same as above, but for MSBs)
    mask_image_dir : directory containing mask images for each camnode to restrict
      analysis
    """

    images = input_data['images']
    projector_width = input_data['display_width_height'][0]
    projector_height = input_data['display_width_height'][1]
    physical_display_id = input_data['physical_display_id']
    virtual_display_id = input_data['virtual_display_id']

    topic_prefix_rows = collections.defaultdict(list)
    sizes = {}

    binary_code='natural'
    assert binary_code in ('gray','natural')

    results = collections.defaultdict(dict)
    cam2projector = collections.defaultdict(list)

    for i,(axis,bitno,flip,topic_prefix,imarr) in enumerate(images):
        topic_prefix_rows[topic_prefix].append( i )

        shape = imarr.shape
        if topic_prefix in sizes:
            assert sizes[topic_prefix]==shape
        else:
            sizes[topic_prefix] = shape

    topic_prefix_rows = dict(topic_prefix_rows) # done with construction, normal dict
    data_by_cam = {}
    for do_topic_prefix in topic_prefix_rows.keys():
        height,width = sizes[do_topic_prefix]
        this_rows = topic_prefix_rows[do_topic_prefix]
        n_rows = len(this_rows)
        dtype=[('axis',np.uint8),
               ('bitno',np.int16),
               ('flip',np.uint8),
               ('topic_prefix','|S30'),
               ('imarr',np.uint8,(height,width))]
        zz = [ (dtup[0],i) for i,dtup in enumerate(dtype) ]

        # this is a real PITA to fill the structured array
        data=np.empty( (n_rows,), dtype=dtype)
        for i in range(n_rows):
            for colname,tupidx in zz:
                if tupidx==4:
                    data[i][colname][:,:] = images[ this_rows[i] ][tupidx]
                else:
                    data[i][colname] = images[ this_rows[i] ][tupidx]
        data_by_cam[do_topic_prefix] = data

    #load mask images if specified
    topic_prefixes = topic_prefix_rows.keys()
    baseline_valid = {}
    for do_topic_prefix in topic_prefix_rows.keys():
        height,width = sizes[do_topic_prefix]
        if mask_image_dir:
            mask_fname = '%s_%s_%s_mask.png' % (physical_display_id, virtual_display_id, do_topic_prefix[1:])
            print 'loading valid pixel mask from', mask_fname
            assert os.path.exists(mask_fname)
            arr = load_mask_image(mask_fname)
        else:
            print 'all valid by default for',do_topic_prefix
            arr = np.ones( (height,width), dtype=np.bool )
        baseline_valid[do_topic_prefix] = arr

    p2c_by_cam = {}


    for do_topic_prefix in topic_prefix_rows.keys():
        data = data_by_cam[do_topic_prefix]

        if 1:
            slopes = []

            for axis in (0,1): # x,y
                axis_cond = (data['axis']==axis)

                if 1:
                    bitno=-3
                    bitno_cond = (data['bitno']==bitno)
                    # any flip
                    # already did topic_prefix
                    this_cond = bitno_cond & axis_cond
                    noise_im_rows = data[this_cond]
                    noise_ims = noise_im_rows['imarr']

                    EM = np.mean( noise_ims, axis=0 )

                if 1:
                    bitno=-2
                    bitno_cond = (data['bitno']==bitno)
                    # any flip
                    # already did topic_prefix
                    this_cond = bitno_cond & axis_cond
                    noise_im_rows = data[this_cond]
                    noise_ims = noise_im_rows['imarr']

                    FM = np.mean( noise_ims, axis=0 )

                address_valid = FM > (EM + 10.0)

                bitno_images = []
                valid_cond_images = []
                max_bitno = 0
                bitno_mod = np.max(data['bitno'])-100+1
                for bitno in range( np.max(data['bitno'])+1 ):
                    bitno_cond = (data['bitno']==bitno)
                    if bitno >= 100:
                        continue
                    if np.sum(bitno_cond)==0:
                        continue
                    max_bitno = max(max_bitno,bitno)
                    flipim = {}
                    for flip in (0,1):
                        flip_cond = (data['flip']==flip)

                        this_cond = bitno_cond & axis_cond & flip_cond
                        this_ims = data[this_cond]['imarr']
                        flipim[flip] = np.mean( this_ims, axis=0 )

                    result_im = flipim[1] > flipim[0]
                    bitno_images.append( result_im )

                    debug_array( 'result_im', locals(), bitno )

                # get encoded value
                encoded_value = np.zeros( result_im.shape, dtype=np.uint16 )
                for bitno in range( max_bitno ):
                    encoded_value += (bitno_images[bitno] << bitno)
                if binary_code=='gray':
                    gray = graycode_arange( np.uint16(-1), dtype=np.uint16)
                    address = gray[encoded_value]
                else:
                    address = encoded_value

                debug_array( 'address', locals() )

                if 1:
                    # Limit addresses to valid.

                    # XXX FIXME TODO: Should we do this after we find
                    # subpixel positions so that we don't accidentally
                    # chop graycode values that could be useful for
                    # the subpixel section?

                    if axis==0:
                        maxval = projector_width
                    else:
                        assert axis==1
                        maxval = projector_height
                    address_ok = address < maxval
                    address_valid &= address_ok

                    address_valid &= baseline_valid[do_topic_prefix]

                address = address.astype( np.int16 )
                address[~address_valid]=-1 # clear invalid addresses to ensure non-use

                if 0:
                    # attempt to remove artifacts...
                    address = scipy.ndimage.filters.median_filter(address,size=5)

                line_peak_images = {}

                for bitno in range( 100, np.max(data['bitno'])+1 ):
                    if no_subpixel:
                        continue
                    bitno_cond = (data['bitno']==bitno)
                    max_projector_width = 10000
                    if bitno >= 100:
                        real_bitno = bitno-100
                        choices = real_bitno+np.arange(max_projector_width//bitno_mod)*bitno_mod
                        # if axis==0 and real_bitno==1:
                        #     print '%d of %d'%(real_bitno,bitno_mod)
                        #     print 'choices',choices
                        #     print
                        flip_cond = (data['flip']==1)

                        this_cond = bitno_cond & axis_cond & flip_cond
                        this_ims = data[this_cond]['imarr']
                        this_mean = np.mean( this_ims, axis=0 )
                        this_mean_v = ma.masked_array(this_mean, mask=~address_valid)
                        kernel = np.array( [1,1,1,1,0,-1,-1,-1,-1], dtype=np.float32)
                        if axis==0:
                            c = np.array([np.convolve( row, kernel, mode='same' ) for row in this_mean ])
                        else:
                            c = np.array([np.convolve( col, kernel, mode='same' ) for col in this_mean.T]).T
                        c_v = ma.masked_array(c, mask=~address_valid)

                        s = np.sign(c)
                        #s = noise_free_sign(c,n_sigma=0.7)
                        if axis==0:
                            p1=s[:,1:]-s[:,:-1]==-2
                            p = np.hstack( (np.zeros_like( p1[:,0,np.newaxis] ), p1) )
                        else:
                            p1=s[1:,:]-s[:-1,:]==-2
                            p = np.vstack( (np.zeros_like( p1[0,np.newaxis,:] ), p1 ) )
                        p_v = ma.masked_array(p, mask=~address_valid)
                        line_peak_images[ bitno-100 ] = p_v

                        debug_array( 'p_v', locals(), bitno-100 )

                        w2 = len(kernel)//2
                        row_idxs, col_idxs = np.nonzero(p_v)
                        #fig = plt.figure()
                        #ax = fig.add_subplot(1,1,1)
                        for ii,(i,j) in enumerate(zip(row_idxs,col_idxs)):
                            if axis==0:
                                s = c[i, j-w2:j+w2+1]
                                s2 = c[i, j-1:j+1]
                            else:
                                s = c[i-w2:i+w2+1,j]
                                s2 = c[i-1:i+1, j]
                            # if axis==0 and real_bitno==1:
                            #     print
                            #     print 'i,j,s',i,j,s
                            if len(s)!=len(kernel):
                                # at edge, skip
                                continue
                            assert len(s2)==2
                            #ax.plot(range(-w2,w2+1),s)
                            #ax.plot( [-1,0], s2, lw=2 )
                            center = float(np.interp( [0], s2[::-1], [0.0, -1.0] )[0])
                            slope = s2[1]-s2[0]
                            slopes.append(slope)
                            if axis==0:
                                cam_subpix_coord = j+center
                            else:
                                cam_subpix_coord = i+center
                            #ax.plot( [center], [0.0], 'ko' )
                            projector_graycode_coord = address[i,j]
                            projector_coord = (projector_graycode_coord//bitno_mod)*bitno_mod+real_bitno
                            # XXX TODO Fixme: Hmm, which rule to use to find best coordinate of choices?
                            #if not (projector_coord >= projector_graycode_coord):
                            if (projector_graycode_coord - projector_coord) > bitno_mod/2.0:
                                projector_coord += bitno_mod
                            # if axis==0 and real_bitno==1:
                            #     print 'projector_graycode_coord',projector_graycode_coord
                            #     print 'projector_coord',projector_coord
                            #     print 'cam_subpix_coord',cam_subpix_coord
                            cam2projector[ (i,j) ].append( (axis, cam_subpix_coord, projector_coord, slope) )
                            # if ii >= 100:
                            #     break
                        # plt.show()
                        # import sys
                        # sys.exit(0)

                        if bitno < 101 and visualize:
                            fig = plt.figure()
                            ax = fig.add_subplot(2,2,1)
                            ax.imshow( this_mean_v, interpolation='nearest' )
                            ax.set_title('%s, axis %d'%(do_topic_prefix,axis))

                            ax = fig.add_subplot(2,2,2)
                            ax.imshow( c_v, interpolation='nearest' )

                            ax = fig.add_subplot(2,2,3)
                            ax.imshow( p_v, interpolation='nearest' )



                # take MSB validity as entire image validity
                #address = ma.masked_array(address, mask=invalid_cond)
                results[do_topic_prefix][axis] = {'address':address, 'nmask':address_valid, 'FM':FM, 'EM':EM}

                address_ma = ma.masked_array(address, mask=~address_valid)
                if visualize:
                    fig = plt.figure()
                    plt.imshow( address_ma, interpolation='nearest' )
                    plt.gca().set_title('%s, axis %d'%(do_topic_prefix,axis))
                    plt.colorbar()


                if visualize and not no_subpixel:
                    lpi = np.sum( [v for v in line_peak_images.itervalues()], axis=0 )
                    lpi = lpi > 0
                    #lpi_v = ma.masked_array(lpi, mask=mask)
                    lpi_v = ma.masked_array(lpi, mask=~address_valid)
                    plt.figure()
                    plt.imshow( lpi_v, interpolation='nearest' )
                    plt.gca().set_title('coverage, axis %d'%(axis,))
                    plt.colorbar()

            if not no_subpixel:
                projector2cam = collections.defaultdict(list)
                slopes = np.array(slopes)
                max_slope = scipy.stats.scoreatpercentile(slopes, 90)

                if 0:
                    plt.figure()
                    plt.hist(slopes)
                    plt.show()
                for (cami, camj) in sorted(cam2projector.keys()):
                    result_list = cam2projector[ (cami, camj) ]
                    #print cami,camj,'-------------'
                    projxs = []
                    projys = []
                    for (axis, cam_subpix_coord, projector_coord, slope) in result_list:
                        #print axis, cam_subpix_coord, projector_coord, slope
                        if slope > max_slope:
                            # This line is not steep enough, and is probably not reliable.
                            continue
                        if axis==0:
                            projxs.append( (projector_coord, cam_subpix_coord) )
                        else:
                            projys.append( (projector_coord, cam_subpix_coord) )
                    for (projx, cam_subpix_x) in projxs:
                        for (projy, cam_subpix_y) in projys:
                            if projx >= projector_width:
                                continue
                            if projy >= projector_height:
                                continue
                            projector2cam[ (projy,projx) ].append( (cam_subpix_y, cam_subpix_x) )
                p2c_x = np.zeros( (projector_height, projector_width) )
                p2c_y = np.zeros( (projector_height, projector_width) )
                p2c_count = np.zeros( (projector_height, projector_width), dtype=np.int )
                cam_height,cam_width = sizes[do_topic_prefix]
                for (projy,projx), cam_coord_list in projector2cam.iteritems():
                    for cam_coords in cam_coord_list:
                        p2c_count[projy,projx] += 1
                        p2c_x[projy,projx] = cam_coords[1]#/float(cam_height)
                        p2c_y[projy,projx] = cam_coords[0]#/float(cam_width)

                p2c_x_v = ma.masked_array(p2c_x, mask=p2c_count==0)
                p2c_y_v = ma.masked_array(p2c_y, mask=p2c_count==0)

                # make unknown values invalid
                p2c_x[p2c_count==0] = -1.0
                p2c_y[p2c_count==0] = -1.0
            else:
                # no_subpixel

                #mode = 'last'
                mode = 'average'

                p2c_x = np.zeros( (projector_height, projector_width) )
                p2c_y = np.zeros( (projector_height, projector_width) )
                p2c_count = np.zeros( (projector_height, projector_width), dtype=np.int )

                proj_x = results[do_topic_prefix][0]['address']
                proj_y = results[do_topic_prefix][1]['address']

                valid_cam = (proj_x >= 0) & (proj_y >= 0)
                row_idxs, col_idxs = np.nonzero(valid_cam)
                for (i,j) in zip(row_idxs,col_idxs):
                    px = proj_x[i,j]
                    py = proj_y[i,j]
                    if (px < projector_width) and (py < projector_height):
                        if mode=='average':
                            p2c_x[py,px] += j
                            p2c_y[py,px] += i
                        elif mode=='last':
                            p2c_x[py,px] = j
                            p2c_y[py,px] = i
                        p2c_count[py,px] += 1

                if mode=='average':
                    valid = p2c_count>0
                    p2c_x[valid] = p2c_x[valid]/p2c_count[valid] # take average of all camera positions
                    p2c_y[valid] = p2c_y[valid]/p2c_count[valid] # take average of all camera positions
                p2c_x_v = ma.masked_array(p2c_x, mask=p2c_count==0)
                p2c_y_v = ma.masked_array(p2c_y, mask=p2c_count==0)

                # make unknown values invalid
                p2c_x[p2c_count==0] = -1.0
                p2c_y[p2c_count==0] = -1.0

            if visualize:
                plt.figure()
                plt.imshow( p2c_count, interpolation='nearest' )
                plt.gca().set_title('p2c_count')
                plt.colorbar()

                plt.figure()
                plt.imshow( p2c_x_v, interpolation='nearest' )
                plt.gca().set_title('p2c_x_v')
                plt.colorbar()

                plt.figure()
                plt.imshow( p2c_y_v, interpolation='nearest' )
                plt.gca().set_title('p2c_y_v')
                plt.colorbar()

                if 0:
                    ax = fig.add_subplot(2,3,5)
                    rows = [500, 501, 502, 503, 504]
                    colnum = np.arange( p2c_x_v.shape[1] )
                    for row in rows:
                        ax.plot( colnum, p2c_x_v[row], label=str(row) )
                    ax.legend()

                if 0:
                    fig = plt.figure()
                    ax = fig.add_subplot(2,1,1)
                    for i in range(218,222):
                        plt.plot( p2c_x_v[i,:] )

                    ax = fig.add_subplot(2,1,2)
                    for i in range(218,222):
                        plt.plot( p2c_y_v[i,:] )
        p2c_by_cam[do_topic_prefix] = {}
        p2c_by_cam[do_topic_prefix]['x'] = p2c_x
        p2c_by_cam[do_topic_prefix]['y'] = p2c_y

    if visualize:
        plt.show()
    data = dict(results)
    results = {'data':data,
               'display_width_height':input_data['display_width_height'],
               'p2c_by_cam':p2c_by_cam,
               'cam_sizes':sizes,
               }
    return results

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', type=str,nargs='+', metavar='images-FILE.pkl')
    parser.add_argument('--visualize',  action='store_true')
    parser.add_argument('--no-subpixel',  action='store_true')
    parser.add_argument('--mask-image-dir', type=str, help=\
        'path to directory containing mask images. The images are'\
        'named display_server_vdisp_id_image_topic_mask.png')
    parser.add_argument('--output-dir', type=str, help=\
        'path to save calibration result', default='./')

    args = parser.parse_args()

    if not os.path.isdir(args.output_dir):
        os.mkdir(args.output_dir)

    all_results = {}
    for filename in args.filename:
        with open(filename,mode='r') as fd:
            images = pickle.load(fd)
            physical_display_id = images['physical_display_id']
            virtual_display_id = images['virtual_display_id']
            if virtual_display_id is not None:
                display_id = physical_display_id+'/'+virtual_display_id
            else:
                display_id = physical_display_id
            print 'finding display ', display_id

        results = find_display_in_images(
                    images,
                    visualize=args.visualize,
                    no_subpixel=args.no_subpixel,
                    mask_image_dir=args.mask_image_dir)

        pd = all_results.setdefault(physical_display_id,{})
        assert virtual_display_id not in pd
        pd[virtual_display_id] = results
    fd = open(os.path.join(args.output_dir,'display_coords.pkl'),mode='w')
    pickle.dump(all_results,fd)
    fd.close()

    base_coord_dirname = os.path.join(args.output_dir,'display_coords_' + time.strftime( '%Y%m%d_%H%M%S' ))
    os.mkdir(base_coord_dirname)
    for physical_display_id in all_results:
        for virtual_display_id in all_results[physical_display_id]:
            if virtual_display_id is None:
                coord_dirname = os.path.join(base_coord_dirname,physical_display_id)
            else:
                coord_dirname = os.path.join(base_coord_dirname,physical_display_id,virtual_display_id)
            print 'saving data to ', coord_dirname
            os.makedirs(coord_dirname)

            results = all_results[physical_display_id][virtual_display_id]
            fname = os.path.join( coord_dirname, 'display_width_height' )
            fd = open(fname,mode='w')
            fd.write('%d %d\n'%results['display_width_height'])
            fd.close()

            camnames = results['data'].keys()
            for camname in camnames:
                camdir = os.path.join( coord_dirname, camname.lstrip('/') )
                os.mkdir( camdir )

                camdict = results['data'][camname]
                for axis,axname in [ (0,'x'),
                                     (1,'y') ]:

                    fname = os.path.join( camdir, axname + '_address' )
                    savearr( fname, camdict[axis]['address'], is_int=False )

                    fname = os.path.join( camdir, axname + '_mask' )
                    savearr( fname, camdict[axis]['nmask'] )

                fname = os.path.join( camdir, 'fm' )
                arr = camdict[axis]['FM']
                savearr( fname, arr, is_int=False )
                pil_im = PIL.fromarray( arr.astype(np.uint8) )
                pil_im.save(fname+'.png')

                fname = os.path.join( camdir, 'em' )
                arr = camdict[axis]['EM']
                savearr( fname, arr, is_int=False )
                pil_im = PIL.fromarray( arr.astype(np.uint8) )
                pil_im.save(fname+'.png')

                fname = os.path.join( camdir, 'p2c.exr' )
                x=results['p2c_by_cam'][camname]['x']
                y=results['p2c_by_cam'][camname]['y']
                cam_height,cam_width = results['cam_sizes'][camname]
                normx = x/cam_width
                normy = y/cam_height
                save_exr( fname, r=normx, g=normy, b=np.zeros_like(normy) )

                fname = os.path.join( camdir, 'p2c-unnormalised.exr' )
                save_exr( fname, r=x, g=y, b=np.zeros_like(y) )

                fname = os.path.join( camdir, 'c2p_graycode_only.exr' )
                graycode_x = camdict[0]['address']/float(results['display_width_height'][0])
                graycode_y = camdict[1]['address']/float(results['display_width_height'][1])
                graycode_x[~camdict[0]['nmask']]=-1.0
                graycode_y[~camdict[1]['nmask']]=-1.0
                save_exr( fname, r=graycode_x, g=graycode_y, b=np.zeros_like(graycode_y) )

