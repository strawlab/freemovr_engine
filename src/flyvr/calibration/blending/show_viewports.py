
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import itertools

import roslib
roslib.load_manifest('flyvr')
import flyvr.exr
import numpy as np
import scipy.ndimage.measurements
import scipy.signal
import os

viewportcolors = (c for c in itertools.cycle([(1.0, 0.0, 0.0, 0.5),
                                              (0.0, 1.0, 0.0, 0.5),
                                              (0.0, 0.0, 1.0, 0.5),
                                              (1.0, 1.0, 0.0, 0.5),
                                              (0.0, 1.0, 1.0, 0.5),
                                              (1.0, 0.0, 1.0, 0.5),
                                              (1.0, 0.5, 0.0, 0.5),
                                              (1.0, 0.0, 0.5, 0.5),
                                              (0.0, 1.0, 0.5, 0.5),
                                              (0.5, 1.0, 0.0, 0.5),
                                              (0.0, 0.5, 1.0, 0.5),
                                              (0.5, 0.0, 1.0, 0.5)]))


#TESTFILE = './optima-ml500-tethered2-2015-04-17-21-33.exr'
TESTFILE = './p2g.exr'


def exr_filename_to_coordinates(fname):
    """opens an exr file and returns a H,W,4 shaped array with the
    texture coordinates u,v and the pixel indices x,y
    """
    u, v, a = flyvr.exr.read_exr(fname)
    y, x = np.meshgrid(np.arange(u.shape[1]), np.arange(u.shape[0]))
    coordinates = np.dstack((u,v,x,y))
    return coordinates


def coordinate_array_to_uv_edge(coordinates):
    """takes a coordinate array and returns a list of the edges of all viewports"""
    u = coordinates[:,:,0]
    labeled_array, num_labels = scipy.ndimage.measurements.label(u >= 0.0)

    uv_OUT = []
    xy_OUT = []
    for idx in range(1, num_labels + 1):
        mask = np.array(labeled_array == idx, dtype=np.int)
        laplacian = np.array([[0,1,0],[1,-4,1],[0,1,0]], dtype=np.int)
        edge = scipy.signal.correlate2d(mask, laplacian, mode='same', boundary='symm')
        edge_mask = np.abs(edge) > 1  # due to the filtering some bg points are picked up
        #edge_mask = edge < 1  # due to the filtering some bg points are picked up
        edge_mask = edge_mask * (u >= 0)  # remove them here 
        edge_points = coordinates[edge_mask]
        # sort the coordinates in the projector coordinates
        xy = edge_points[:, 2:4]
        xy -= xy.mean(axis=0)
        phi = np.arctan2(xy[:,1], xy[:,0])
        sort_idx = np.argsort(phi)
        uv_edge = edge_points[sort_idx, 0:2]
        xy_edge = coordinates[edge_mask][sort_idx, 2:4]
        uv_OUT.append(uv_edge)
        xy_OUT.append(xy_edge)
    return labeled_array, xy_OUT, uv_OUT

def uv_edge_to_polygons(uv_edge):
    """takes a viewport edge in uv coordinates, removes wrapping
    artifacts along the u axis and returns a list of polygon
    vertices normally length 1, (2+ if a viewport wraps around u=0)
    """
    uv_edge = np.vstack((uv_edge, uv_edge[0:2,:]))

    # insert inf/nan points at the wrapping locations
    pts_x = []
    pts_y = []
    pts_x.append(uv_edge[0, 0])
    pts_y.append(uv_edge[0, 1])
    for p0, p1 in zip(uv_edge[:-1], uv_edge[1:]):
        u0 = p0[0]
        u1 = p1[0]
        if u0 - u1 < -0.5:  # wraps left
            x0, y0, x1, y1 = u0, p0[1], u1 - 1.0, p1[1]
            m = (y0 - y1) / (x0 - x1)
            t = y0 - m * x0
            pts_x.extend([0.0, float('inf'), 1.0])  # we can find inf later...
            pts_y.extend([t, float('nan'), t])
        elif u0 - u1 > 0.5:  # wraps right
            x0, y0, x1, y1 = u0 + 1.0, p0[1], u1, p1[1]
            m = (y0 - y1) / (x0 - x1)
            t = y0 - m * x0
            pts_x.extend([1.0, float('inf'), 0.0])
            pts_y.extend([m*1.0 + t, float('nan'), m*1.0 + t])
        else:
            pts_x.append(u1)
            pts_y.append(p1[1])

    X = np.array(pts_x)
    Y = np.array(pts_y)
    idx, = np.where(np.isinf(X))
    idx = idx.tolist()
    if len(idx) > 1:
        i = idx.pop(0)
        X = np.roll(X, -i)[1:]
        Y = np.roll(Y, -i)[1:]
    if len(idx) == 1:
        # TODO:
        i = idx.pop(0)
        X = np.roll(X, -i)[1:]
        Y = np.roll(Y, -i)[1:]
        X = np.hstack((np.array([0.0, 1.0]), X))
        if Y[0] > 0.5:  # FIXME
            Y = np.hstack((np.array([1.0, 1.0]), Y))
        else:
            Y = np.hstack((np.array([0.0, 0.0]), Y))


    polygons = []
    while True:
        try:
            _i = np.where(np.isinf(X))[0].tolist().pop(0)
        except IndexError:
            polygons.append(list(zip(X, Y)))
            break
        else:
            if len(X[:_i]) > 0:
                polygons.append(list(zip(X[:_i], Y[:_i])))
            X = X[_i+1:]
            Y = Y[_i+1:]

    polygons = [pp for pp in polygons if len(pp) > 3]  # remove artifacts from wrapping
    return polygons

def plot_exrs(*exr_filenames):

    axes_dict = {}
    for exrfn in exr_filenames:
        fig = plt.figure()
        fig.suptitle(os.path.split(exrfn)[1])
        axes_dict[exrfn] = plt.subplot2grid((1,1),(0,0))

    fig = plt.figure()
    fig.suptitle("texture coordinate system")
    ax = plt.subplot2grid((1,1),(0,0))

    for exrfn, ax_xy in axes_dict.items():
        coords = exr_filename_to_coordinates(exrfn)
        labeled_array, xy_edges, uv_edges = coordinate_array_to_uv_edge(coords)
        ax_xy.set_xlim(0, labeled_array.shape[1])
        ax_xy.set_ylim(labeled_array.shape[0], 0)

        for i, (xy_edge, uv_edge) in enumerate(zip(xy_edges, uv_edges)):
            color = viewportcolors.next()
            # xy
            ax_xy.add_patch(patches.Polygon(xy_edge[:,::-1].tolist(), lw=1, fc=color))
            # uv
            polygons = uv_edge_to_polygons(uv_edge)
            for pp in polygons:
                ax.add_patch(patches.Polygon(pp, lw=1, fc=color))

        ax_xy.set_xlabel("display coordinate x")
        ax_xy.set_ylabel("display coordinate y")

    ax.set_xlabel("texture coordinate u")
    ax.set_ylabel("texture coordinate v")
    plt.show()


def plot_exr_file(exrfn):
    coords = exr_filename_to_coordinates(exrfn)
    plt.figure()
    ax0 = plt.subplot2grid((1,2), (0,0))
    ax1 = plt.subplot2grid((1,2), (0,1))
    ax0.imshow(coords[:,:,0])
    ax1.imshow(coords[:,:,1])
    plt.show()

if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument('exrs', type=str, nargs='+',
                        help='exrs for displaying')
    args = parser.parse_args()
    plot_exrs(*args.exrs)

