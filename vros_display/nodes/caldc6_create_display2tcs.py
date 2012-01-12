#!/usr/bin/env python

# Given camera/display correspondences (under a certain geometry), a
# camera calibration, and the description of the geometry, establish
# the "camera" calibration for the display.

# ROS imports
import roslib; roslib.load_manifest('vros_display')
import rospy
import camera_model
import simple_geom
from exr import save_exr

import argparse
import json
import scipy.misc
import numpy as np
import mahotas.polygon
import matplotlib.pyplot as plt

def load_params(physical_display_id, virtual_display_id):
    param_name = 'virtual_display_config_json_string'
    fqdn = '/virtual_displays/'+physical_display_id + '/' +  virtual_display_id
    fqpn = fqdn + '/' + param_name
    virtual_display_json_str = rospy.get_param(fqpn)
    return json.loads( virtual_display_json_str )

def get_verts( camera, geom):
    allw = []
    res_u = 32
    res_v = 5
    for tc1 in np.linspace(0,1,res_v):
        tc = np.vstack( (
                np.linspace(0,1.,res_u),
                tc1*np.ones( (res_u,) ),
                )).T
        world = geom.model.texcoord2worldcoord(tc)
        allw.append(world)

    allw = np.concatenate(allw)

    uv = camera.project_3d_to_pixel( allw )
    return uv

def plot_poly( ax, verts ):
    xs = []; ys = [];
    for x,y in verts:
        xs.append(x)
        ys.append(y)
    # repeat first vertex
    xs.append( verts[0][0] )
    ys.append( verts[0][1] )
    return ax.plot(xs,ys, 'k:')

def create_display2tcs(geometry_filename,
                       display_bagfiles,
                       output_filebase,
                       visualize=False):
    geom = simple_geom.Geometry(geometry_filename)
    displays = [camera_model.load_camera_from_bagfile(dbf) for dbf in display_bagfiles]

    print geometry_filename
    print display_bagfiles
    print [d.get_name() for d in displays]

    display_tuples = [ d.get_name().split('/') for d in displays] # [(physical_display_id, virtual_display_id),...]
    physical_display_ids = list(set(d[0] for d in display_tuples))
    if not len(physical_display_ids)==1:
        raise ValueError('need one, and only one, physical display. (You have %s)'%physical_display_ids)

    display_params = rospy.get_param('/physical_displays/'+physical_display_ids[0])
    tcs = np.zeros( (display_params['height'],display_params['width'],2))-1
    allmask = np.zeros( (display_params['height'],display_params['width']))
    EM = np.zeros( (display_params['height'],display_params['width']), dtype=np.uint8)
    FM = np.ones( (display_params['height'],display_params['width']), dtype=np.uint8)*255
    print display_params

    for display in displays:
        try:
            physical_display_id, virtual_display_id = display.get_name().split('/')
        except ValueError:
            physical_display_id = display.get_name()
            vdisp_params = {}
        else:
            vdisp_params = load_params(physical_display_id, virtual_display_id)

        maskarr = np.zeros( allmask.shape, dtype=np.uint8 )
        mahotas.polygon.fill_polygon([(y,x) for (x,y) in vdisp_params.get('viewport',[])], maskarr)
        if np.max(maskarr)==0: # no mask
            maskarr += 1

        allmask += maskarr
        mask = np.nonzero(maskarr)
        this_tcs = geom.compute_for_camera_view( display , what = 'texture_coords')
        if visualize:
            fig = plt.figure()
            uv = get_verts(display,geom)

            ax = fig.add_subplot(211)
            ax.imshow( this_tcs[:,:,0], vmin=0, vmax=1 )
            ax.plot( uv[:,0], uv[:,1], 'k.' )
            ax.set_title(display.get_name() + ', tc0')
            plot_poly( ax, vdisp_params.get('viewport') )


            ax = fig.add_subplot(212)
            ax.imshow( this_tcs[:,:,1], vmin=0, vmax=1 )
            ax.plot( uv[:,0], uv[:,1], 'k.' )
            ax.set_title(display.get_name() + ', tc1')
            plot_poly( ax, vdisp_params.get('viewport') )

        this_tcs[ np.isnan(this_tcs) ] = -1.0 # nan -> -1
        assert this_tcs.shape == tcs.shape

        tcs[mask] = this_tcs[mask]

    if output_filebase is None:
        output_filebase = 'display2tcs-'+physical_display_id
    scipy.misc.imsave(output_filebase+'-EM.png', EM)
    scipy.misc.imsave(output_filebase+'-FM.png', FM)
    if 1:
        # save texture coordinates as EXR file preserving floats
        r=tcs[:,:,0]
        g=tcs[:,:,1]
        b=np.zeros_like(tcs[:,:,1])
        save_exr( output_filebase+'.exr', r=r, g=g, b=b)

    if 1:

        # Save low-res (non-HDR normalized .png) version of texture
        # coord image.

        mmin = 0.0
        mmax = 1.0
        pngouttcs = (tcs/(mmax-mmin)-mmin)*255
        pngouttcs = np.clip(pngouttcs,0,255)
        pngouttcs = pngouttcs.astype(np.uint8)
        pngouttcs = np.concatenate( (pngouttcs, np.zeros_like(pngouttcs[:,:,0,np.newaxis])), axis=2)
        if 1:
            # draw viewport mask in gray
            i,j = np.nonzero(allmask)
            testvals = pngouttcs[i,j]
            cond = testvals==np.array((0,0,0))
            ivalid = i[cond[:,0]]
            jvalid = j[cond[:,0]]
            pngouttcs[ivalid,jvalid,:] = 127

        scipy.misc.imsave(output_filebase+'-tcs-lowres.png', pngouttcs)

    if visualize:
        plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('geometry_filename', type=str, help="JSON file with geometry description")
    parser.add_argument('display_bagfiles', type=str, help="filename of display-model.bag for calibration data", nargs='+')
    parser.add_argument('--output_filebase', type=str, help="basename of output files")
    parser.add_argument('--visualize', action='store_true', default=False, help="show plot")
    args = parser.parse_args()

    create_display2tcs(args.geometry_filename,
                       args.display_bagfiles,
                       args.output_filebase,
                       visualize=args.visualize)
