#!/usr/bin/env python3
import sys
import os.path
import collections

import numpy as np
import scipy.spatial
import scipy.ndimage as nd
import matplotlib.pyplot as plt
import yaml

from PIL import Image, ImageDraw

from ..exr import read_exr, save_exr

def convexHull (quv):
    hull = scipy.spatial.Delaunay(quv).convex_hull 
    # hull contains now line segments of the convex hull, meaning start and endpoint indices
    # remove double indices (each startpoint is another segments' endpoint'
    ps = set()
    for start_index, end_index in hull:
        ps.add(start_index)
        ps.add(end_index)
    ps = np.array(list(ps)) # ps now contains the indices of the convex hull

    # now sort the vertices of the convex hull clockwise
    center = np.mean(quv[ps], axis=0)
    A = quv[ps] - center
    idx = np.argsort(np.arctan2(A[:,1], A[:,0])) # idx now contains the ordered indices of the convex hull
    return ps[idx]
    
def mergedHull (q1, q2):
    hull1 = scipy.spatial.Delaunay(q1).convex_hull 
    hull2 = scipy.spatial.Delaunay(q2).convex_hull 
    # hull contains now line segments of the convex hull, meaning start and endpoint indices
    # remove double indices (each startpoint is another segments' endpoint'
    ps = set()
    for start_index, end_index in hull1:
        ps.add(start_index)
        ps.add(end_index)
    for start_index, end_index in hull2:
        ps.add(start_index)
        ps.add(end_index)
    ps = np.array(list(ps)) # ps now contains the indices of the convex hull

    # now sort the vertices of the convex hull clockwise
    center = np.mean(q1[ps], axis=0) # TODO: this is wrong, but not much, hopefully:
    # what I really should do is order the two hulls in their respective image spaces and then merge the ordered hulls
    A = q1[ps] - center
    idx = np.argsort(np.arctan2(A[:,1], A[:,0])) # idx now contains the ordered indices of the merged convex hulls
    return ps[idx]

def blendFunc(a, curve):
    return a

class Blender:
    def __init__(self, visualize, out_dir, debug_exr=True, exr_comments=''):
        self._visualize = visualize
        self._out_dir = out_dir
        self._debug_exr = debug_exr
        self._exr_comments = exr_comments

        #key: display_server_name
        self._dscs = collections.OrderedDict()
        self._u = collections.OrderedDict()
        self._v = collections.OrderedDict()
        self._ui = collections.OrderedDict()
        self._vi = collections.OrderedDict()

        self._uv_scale=[2400, 1133] # resolution of intermediary UV map
        self._uv_width = None
        self._uv_height = None

        #key: display_server_name/viewport_name (aka viewport_fq)
        self._masks = collections.OrderedDict()
        self._gradients = collections.OrderedDict()
        self._blended = collections.OrderedDict()

        #key: display_server_name
        self._output = collections.OrderedDict()

    def add_display_server(self, name, dsc, u, v, ui, vi):
        self._dscs[name] = dsc

        if len(set((u.shape,v.shape,ui.shape,vi.shape))) != 1:
            raise Exception("U & V images must be the same size")

        h,w = u.shape
        if self._uv_width is None:
            self._uv_width = w
            self._uv_height = h
        if (h != self._uv_height) or (w != self._uv_width):
            raise Exception("Display server images must be the same size")

        #XXX:GRR Toni has assumed -1 is the sentinal, which breaks things at XXX
        if np.isnan(u).sum() > 0:
            u[np.isnan(u)] = -1
            v[np.isnan(v)] = -1
        if np.isnan(ui).sum() > 0:
            ui[np.isnan(ui)] = -1
            vi[np.isnan(vi)] = -1

        self._u[name] = u
        self._v[name] = v
        self._ui[name] = ui
        self._vi[name] = vi

    def blend(self, gamma, blend_curve):
        img_count = 0

        for name in self._dscs:
            dsc = self._dscs[name]
            for viewport in dsc.virtual_displays: # loop over viewports
                viewport_fq = "%s/%s" % (name,viewport)

                #XXX: See XXX:GRRR
                mask_index = np.logical_and(
                                    self._u[name]>-0.99,
                                    dsc.get_virtual_display_mask(viewport,squeeze=True)
                )

                # coordinates of valid sample points
                YX = np.nonzero(mask_index)

                # extract valid sample values into vectors
                U = self._u[name][mask_index]
                V = self._v[name][mask_index]

                # test for wrap around of U coordinate (texture seam)
                has_wraparound = ((max(U)-min(U))>0.5)
                if has_wraparound:
                    L = (U>0.5)
                    U[L] -= 0.5
                    U[np.logical_not(L)] += 0.5

                q = np.transpose(YX)
                quv = np.transpose([V*self._uv_scale[1], U*self._uv_scale[0]])

                img_count += 1

                ch = mergedHull(q, quv)
                #ch=convexHull(q)
                h = q[ch]
                tp = tuple((x[1], x[0]) for x in h)

                #ch=convexHull(quv)
                huv = quv[ch]
                t = tuple((x[1], x[0]) for x in huv)

                # now generate binary viewport mask in projector space. Unlinke
                # the original viewport masks, this is the convex hull of observations
                # only
                mask = Image.new('F', (self._uv_width, self._uv_height), 0)
                drawMask=ImageDraw.Draw(mask)
                drawMask.polygon(tp, fill=1) # draw binary mask

                self._masks[viewport_fq] = np.array(mask)

                if self._debug_exr:
                    save_exr(
                        os.path.join(self._out_dir,"masks_%s.exr" % img_count),
                        r=mask, g=mask, b=mask, comments=self._exr_comments
                    )
                    masko = dsc.get_virtual_display_mask(viewport,squeeze=True)
                    save_exr(
                        os.path.join(self._out_dir,"maskso_%s.exr" % img_count),
                        r=masko, g=masko, b=masko, comments=self._exr_comments
                    )


                # now generate binary viewport mask in UV space
                img = Image.new('I', (self._uv_scale[0], self._uv_scale[1]), 0)
                draw=ImageDraw.Draw(img)
                draw.polygon(t, fill=1) # draw binary mask

                # calculate distance gradient in UV space
                p = np.array(img)
                pg = nd.distance_transform_edt(p)

                if has_wraparound:
                    pg = np.roll(pg, -np.shape(pg)[1]/2, axis=1)

                self._gradients[viewport_fq] = pg

                if self._debug_exr:
                    save_exr(
                        os.path.join(self._out_dir,"gradient_%s.exr" % img_count),
                        r=pg, g=p, b=pg, comments=self._exr_comments
                    )

        # sum over all distance gradients
        gradSum = np.zeros_like(pg)
        for gradient in list(self._gradients.values()):
            gradSum += gradient
        if self._debug_exr:
            save_exr(
                os.path.join(self._out_dir,"gradsum.exr"),
                r=gradSum, g=gradSum, b=gradSum, comments=self._exr_comments
            )

        #blend viewports in UV per viewport
        for i,(viewport_fq,gradient) in enumerate(self._gradients.items()):
            g = gradient > 0
            tr = np.zeros(np.shape(gradient))
            tr[g] = np.divide(gradient[g], gradSum[g])
            self._blended[viewport_fq] = tr
        if self._visualize:
            fig=plt.figure()
            fig.canvas.set_window_title('Blended Viewports in UV')
            for i,tr in enumerate(self._blended.values()):
                plt.subplot(3, 3, i+1)
                plt.imshow(Image.fromarray(tr*255), origin='lower')
        if self._debug_exr:
            for i,tr in enumerate(self._blended.values()):
                save_exr(
                    os.path.join(self._out_dir,"gradientV_%s.exr" % i),
                    r=tr, g=tr, b=tr, comments=self._exr_comments
                )

        if self._visualize:
            fig=plt.figure()

        for iarg,name in enumerate(self._dscs):
            dsc = self._dscs[name]

            #upscale and center
            #XXX: broken with NaNs
            U = (self._ui[name]*self._uv_scale[0]-0.5).astype(int)
            V = (self._vi[name]*self._uv_scale[1]-0.5).astype(int)

            # prepare output image
            I=np.zeros(np.shape(U))
            J=np.zeros(np.shape(self._blended[viewport_fq]))

            for viewport in dsc.virtual_displays:
                # loop over viewports
                viewport_fq = "%s/%s" % (name,viewport)
                # mask contains all pixels of the viewport
                mask = np.nonzero(self._masks[viewport_fq])
                # lookup into blended images on cylinder and apply gamma correction
                I[mask] = blendFunc(
                            self._blended[viewport_fq][(V[mask], U[mask])],
                            curve=blend_curve)**(1/gamma)

                J[(V[mask], U[mask])]+=1

            #the completed array
            self._output[name] = I

            if self._visualize:
                plt.subplot(1, 3, iarg+1)
                plt.imshow(Image.fromarray(I*255), origin='lower')
            if self._debug_exr:
                for i,tr in enumerate(self._blended.values()):
                    save_exr(
                        os.path.join(self._out_dir,"%s.blend.exr" % name),
                        r=self._ui[name], g=self._vi[name], b=I, comments=self._exr_comments
                    )
                    save_exr(
                        os.path.join(self._out_dir,"%s.forward.exr" % name),
                        r=J, g=J, b=J, comments=self._exr_comments
                    )

        return self._output

