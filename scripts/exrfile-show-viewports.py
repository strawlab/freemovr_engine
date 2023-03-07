#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import itertools

import roslib
roslib.load_manifest('freemovr_engine')
import freemovr_engine.exr
import numpy as np
import scipy.ndimage.measurements
import scipy.signal
import os

import cv2

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


def exr_filename_to_coordinates(fname):
    """opens an exr file and returns a H,W,4 shaped array with the
    texture coordinates u,v and the pixel indices x,y
    """
    u, v, a = freemovr_engine.exr.read_exr(fname)
    y, x = np.meshgrid(np.arange(u.shape[1]), np.arange(u.shape[0]))
    coordinates = np.dstack((u,v,x,y))
    return coordinates


def calculate_viewport_info(coordinates):
    """takes a coordinate array and returns a list of the edges of all viewports"""
    # the coordinates array contains four channels: u, v, x, y
    u = coordinates[:,:,0]
    labeled_array, num_labels = scipy.ndimage.measurements.label(u >= 0.0)

    # shapes
    arr_shape = u.shape
    padded_arr_shape = np.add(u.shape, [2, 2])  # 1px border

    # generate edges in both coordinates
    OUT = []
    for idx in range(1, num_labels + 1):
        # detect contour of viewport in display coordinates.
        # we assume that the viewport has no holes.
        viewport_mask = np.array(labeled_array == idx, dtype=np.uint8)
        padded_viewport_mask = np.zeros(padded_arr_shape, np.uint8)
        cv2.copyMakeBorder(viewport_mask, 1, 1, 1, 1,
                            cv2.BORDER_CONSTANT, padded_viewport_mask, 0)
        cnts, _ = cv2.findContours(padded_viewport_mask, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_NONE, offset=(-1, -1))
        assert len(cnts) == 1, "There should only be one contour per detected viewport."
        # draw the detected contour in a mask to select the viewport edge
        viewport_contour_mask = np.zeros(arr_shape, dtype=np.uint8)
        cv2.drawContours(viewport_contour_mask, cnts, -1, 1, 1)
        edge = coordinates[viewport_contour_mask > 0, :]
        # sort the edge by walking along the contour:
        idxs_y, idxs_x = np.where(viewport_contour_mask > 0)
        idx_order = [0]
        idx_max = len(idxs_x)
        directions = np.array([(-1, 0),
                               (-1, 1),
                               ( 0, 1),
                               ( 1, 1),
                               ( 1, 0),
                               ( 1,-1),
                               ( 0,-1),
                               (-1,-1)], dtype=np.int)
        while len(idx_order) < idx_max:
            cur_i = idx_order[-1]
            cur_y, cur_x = idxs_y[cur_i], idxs_x[cur_i]
            for dir_, (dy, dx) in enumerate(directions):
                new_i = np.where((idxs_x == (cur_x + dx)) & (idxs_y == (cur_y + dy)))[0]
                if len(new_i) > 0 and (new_i[0] not in idx_order):
                    idx_order.append(new_i[0])
                    directions = np.roll(directions, dir_, axis=0)
                    break
            else:
                # if we are stuck, but are missing only a few (10) pixels,
                # we ignore the missing ones...
                if len(idx_order) > idx_max - 10:  # FIXME: improve this...
                    break
        edge = edge[idx_order,:]  # sorted.
        uv_edge = edge[:, 0:2]
        uv_coords = coordinates[viewport_mask > 0, 0:2]
        xy_edge = edge[:, 2:4]
        xy_coords = coordinates[viewport_mask > 0, 2:4]
        # append viewport coordinates and detected edges for uv and xy
        OUT.append((uv_edge, uv_coords, xy_edge, xy_coords))
    return OUT

def generate_polygons(uv_edge, uv_coords):
    """takes a viewport edge in uv coordinates, removes wrapping
    artifacts along the u axis and returns a list of polygon
    vertices normally length 1, (2+ if a viewport wraps around u=0)
    """

    uv_edge = np.vstack((uv_edge, uv_edge[0:2,:]))
    _u, _v = uv_coords.T
    u_mean = np.arctan2(np.mean(np.sin(_u)), np.mean(np.cos(_u)))
    v_mean = np.mean(_v)

    # insert inf/nan points at the wrapping locations
    pts_x = []
    pts_y = []
    for p0, p1 in zip(uv_edge[:-1], uv_edge[1:]):
        u0 = p0[0]
        u1 = p1[0]
        if u0 - u1 < -0.5:  # wraps left
            x0, y0, x1, y1 = u0, p0[1], u1 - 1.0, p1[1]
            m = (y0 - y1) / (x0 - x1)
            t = y0 - m * x0
            pts_x.extend([0.0, float('inf'), 1.0])  # we can find inf later...
            pts_y.extend([t, float('inf'), t])
        elif u0 - u1 > 0.5:  # wraps right
            x0, y0, x1, y1 = u0 + 1.0, p0[1], u1, p1[1]
            m = (y0 - y1) / (x0 - x1)
            t = y0 - m * x0
            pts_x.extend([1.0, float('inf'), 0.0])
            pts_y.extend([m*1.0 + t, float('inf'), m*1.0 + t])
        else:
            pts_x.append(u1)
            pts_y.append(p1[1])

    X = np.array(pts_x)
    Y = np.array(pts_y)
    idx, = np.where(np.isinf(X))
    idx = idx.tolist()
    if len(idx) == 0:  # viewport does not wrap
        pass
    elif len(idx) % 2 == 0:  # viewport wraps
        i = idx.pop(0)
        X = np.roll(X, -i)[1:]
        Y = np.roll(Y, -i)[1:]
    elif len(idx) == 1:  # viewport wraps and is probably inverted
        i = idx.pop(0)
        X = np.roll(X, -i)[1:]
        Y = np.roll(Y, -i)[1:]
        i_closest = np.abs(X - u_mean).argmin()
        X = np.hstack((np.array([X[-1], X[0]]), X))
        if Y[i_closest] < v_mean:
            Y = np.hstack((np.array([1.0, 1.0]), Y))
        else:
            Y = np.hstack((np.array([0.0, 0.0]), Y))
    else:
        raise Exception("This wrapping case is not implemented yet.")

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
        viewport_info = calculate_viewport_info(coords)
        ax_xy.set_xlim(0, coords.shape[1])
        ax_xy.set_ylim(coords.shape[0], 0)

        for i, info in enumerate(viewport_info):
            uv_edge, uv_coords, xy_edge, xy_coords = info
            color = viewportcolors.next()
            # xy
            ax_xy.add_patch(patches.Polygon(xy_edge[:,::-1].tolist(), lw=1, fc=color))
            # uv
            polygons = generate_polygons(uv_edge, uv_coords)
            for pp in polygons:
                ax.add_patch(patches.Polygon(pp, lw=1, fc=color))

        ax_xy.set_xlabel("display coordinate x")
        ax_xy.set_ylabel("display coordinate y")

    ax.set_xlabel("texture coordinate u")
    ax.set_ylabel("texture coordinate v")
    ax.set_xlim(0, 1)
    ax.set_ylim(-0.1, 1.1)
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
    parser.add_argument('exrs', type=str, nargs='+', help='exrs for displaying')
    args = parser.parse_args()
    plot_exrs(*args.exrs)

