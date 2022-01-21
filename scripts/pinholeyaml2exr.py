#!/usr/bin/env python3

import yaml
import numpy as np
from scipy.spatial import ConvexHull
from scipy.interpolate import griddata
import matplotlib.pyplot as plt

# EXR stuff
import OpenEXR, Imath  # pip install OpenEXR
FREEMOVR_EXR_PIXEL_TYPE = Imath.PixelType(OpenEXR.FLOAT)

def save_exr( fname, r=None, g=None, b=None, comments=''):
    """saves exr file. Copied from freemovr_engine.exr"""
    r = np.array(r); assert r.ndim==2
    g = np.array(g); assert g.ndim==2; assert g.shape==r.shape
    b = np.array(b); assert b.ndim==2; assert b.shape==r.shape

    header = OpenEXR.Header(r.shape[1], r.shape[0])
    header['channels'] = {'R': Imath.Channel(FREEMOVR_EXR_PIXEL_TYPE),
                          'G': Imath.Channel(FREEMOVR_EXR_PIXEL_TYPE),
                          'B': Imath.Channel(FREEMOVR_EXR_PIXEL_TYPE),
                          }
    header['comments'] = comments
    out = OpenEXR.OutputFile(fname, header)
    data = {'R': r.astype(np.float32).tostring(),
            'G': g.astype(np.float32).tostring(),
            'B': b.astype(np.float32).tostring()}
    out.writePixels(data)
    out.close()


def extract_point_to_point_data_from_yaml(yaml_file):
    """Takes new-style pinhole_wizard yaml and returns Point2Point
    correspondance data"""
    data = yaml.safe_load(yaml_file)
    try:
        p2p = data['extrinsics']
    except KeyError:
        raise Exception("yaml file does not contain extrinsics field")

    VPDATA = {}
    for p in p2p:
        try:
            VPDATA.setdefault(p['viewport'], []).append((p['x'], p['y'], p['u'], p['v']))
        except KeyError:
            raise Exception("p2ps require these keys: 'viewport', 'x', 'y', 'u' and 'v'.")

    for k, v in VPDATA.items():
        arr = np.array(v, dtype=[('x', np.double),
                                 ('y', np.double),
                                 ('u', np.double),
                                 ('v', np.double),
                                ])
        mask = (~np.isnan(arr['x']) *
                ~np.isnan(arr['y']) *
                ~np.isnan(arr['u']) *
                ~np.isnan(arr['v']))
        arr = arr[mask]
        VPDATA[k] = {
                'data': arr,
                'cvhull': ConvexHull(np.vstack((arr['x'], arr['y'])).T),
                    }
    return VPDATA

def create_interpolated_array_uv_alpha_cylindrical(screensize, vpdata, method='linear'):
    sY, sX = screensize
    U, V = [], []
    for vp, vd in vpdata.items():

        x = vd['data']['x']
        y = vd['data']['y']
        u = vd['data']['u']
        v = vd['data']['v']
        if np.ptp(u) > 0.5:
            u[u > 0.5] = u[u > 0.5] - 1.0

        grid_y, grid_x = np.mgrid[0:(sY-1):(sY*1j), 0:(sX-1):(sX*1j)]

        grid_u = scipy.interpolate.griddata(np.vstack((x,y)).T, u,
                                            (grid_x, grid_y), method=method) % 1.0
        grid_v = scipy.interpolate.griddata(np.vstack((x,y)).T, v,
                                            (grid_x, grid_y), method=method)
        grid_a = np.ones_like(grid_u)

        U.append(grid_u)
        V.append(grid_v)

    mask_U = reduce(np.logical_or, (~np.isnan(gu) for gu in U))
    for gu in U:
        gu[np.isnan(gu)*mask_U] = 0.0
    comb_u = np.sum(U, axis=0)

    mask_V = reduce(np.logical_or, (~np.isnan(gv) for gv in V))
    for gv in V:
        gv[np.isnan(gv)*mask_V] = 0.0
    comb_v = np.sum(V, axis=0)

    grid_a = np.ones_like(comb_u)

    exr = np.dstack((comb_u, comb_v, grid_a)).astype(np.float32)
    exr[np.isnan(exr)] = -1.  # display_server fails if exr contains NANs
    return exr

def plot_viewports_points_and_exr(screensize, vpdata, exr):
    """displays viewport points"""
    sY, sX = screensize
    plt.figure()
    for vp, vd in vpdata.items():
        plt.plot(vd['data']['x'], vd['data']['y'], 'o', label=vp)
        for simplex in vd['cvhull'].simplices:
            plt.plot(vd['data']['x'][simplex], vd['data']['y'][simplex], 'k-')
    plt.xlim(0, (sX-1))
    plt.ylim(0, (sY-1))
    plt.legend()
    plt.figure()
    plt.imshow(exr)
    plt.show()


if __name__ == "__main__":

    import argparse
    import os.path
    import sys
    import time

    parser = argparse.ArgumentParser(description="create exr file from pinhole_wizard yaml")
    parser.add_argument('--show', action='store_true', help="display the data using matplotlib")
    parser.add_argument('--width', type=int, required=True, help='screen width in px')
    parser.add_argument('--height', type=int, required=True, help='screen height in px')
    parser.add_argument('--out', default=None, help="exr filename (default based on yaml filename)")
    parser.add_argument('--method', default="linear", help="interpolation method (default 'linear')")
    parser.add_argument('yamlfile', type=argparse.FileType(mode='r'), help="pinhole_wizard yaml file")

    args = parser.parse_args()

    vpd = extract_point_to_point_data_from_yaml(args.yamlfile)
    exr = create_interpolated_array_uv_alpha_cylindrical((args.height, args.width), vpd, method=args.method)

    if args.show:
        plot_viewports_points_and_exr((args.height, args.width), vpd, exr)

    if args.out is None:
        fnbase, _ = os.path.splitext(args.yamlfile.name)
        exrname = fnbase + ".exr"
    else:
        exrname = args.out

    comment = "Created: " + time.ctime() + " ".join(sys.argv)
    save_exr(exrname, exr[:,:,0], exr[:,:,1], exr[:,:,2], comments=comment)

