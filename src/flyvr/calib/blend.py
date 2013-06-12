#!/usr/bin/env python
import sys
import os.path
import collections

import numpy as np
import scipy.spatial
import scipy.ndimage as nd
import matplotlib.pyplot as plt
import yaml

from PIL import Image, ImageDraw

try:
    from ..exr import read_exr, save_exr
    from ..display_client import DisplayServerProxy
except ValueError:
    from exr import read_exr, save_exr
    from display_client import fill_polygon,DisplayServerProxy


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

        self._u[name] = u
        self._v[name] = v
        self._ui[name] = ui
        self._vi[name] = vi

    def blend(self):
        img_count = 0

        for name in self._dscs:
            dsc = self._dscs[name]
            for viewport in dsc.virtual_displays: # loop over viewports
                viewport_fq = "%s/%s" % (name,viewport)

                #XXX: Should only get valid values here
                mask_index = np.logical_and(self._u[name]>-0.99, dsc.get_virtual_display_mask(viewport,squeeze=True))

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
                        r=mask, g=mask, b=mask, comments=''
                    )
                    masko = dsc.get_virtual_display_mask(viewport,squeeze=True)
                    save_exr(
                        os.path.join(self._out_dir,"maskso_%s.exr" % img_count),
                        r=masko, g=masko, b=masko, comments=''
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
                        r=pg, g=p, b=pg, comments=''
                    )

        # sum over all distance gradients
        gradSum = np.zeros_like(pg)
        for gradient in self._gradients.values():
            gradSum += gradient
        if self._debug_exr:
            save_exr(
                os.path.join(self._out_dir,"gradsum.exr"),
                r=gradSum, g=gradSum, b=gradSum, comments=''
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
                    r=tr, g=tr, b=tr, comments=''
                )

        if self._visualize:
            fig=plt.figure()

        for iarg,name in enumerate(self._dscs):
            dsc = self._dscs[name]

            # upscale and center
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
                # lookup into blended images on cylinder
                I[mask] = self._blended[viewport_fq][(V[mask], U[mask])]
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
                        r=self._ui[name], g=self._vi[name], b=self._output[name], comments=''
                    )

        #XXX
        plt.show()

        return self._output

def main2():

    DISPLAY_SERVERS = ("display_server0", "display_server1", "display_server3")

    in_directory=sys.argv[1] # directory, where input and output files are located 
    savePath=in_directory # path for debug output

    b = Blender(True, "hate/2")

    for name in DISPLAY_SERVERS:
        u,v,_ = read_exr(os.path.join(in_directory,"%s.nointerp.exr" % name))
        ui,vi,_ = read_exr(os.path.join(in_directory,"%s.exr" % name))
        b.add_display_server(
                name,
                DisplayServerProxy("/"+name, wait=False,prefer_parameter_server_properties=True),
                u,v,
                ui,vi,
        )

    b.blend()
    
def main():

    DISPLAY_SERVERS = ("display_server0", "display_server1", "display_server3")

    in_directory=sys.argv[1] # directory, where input and output files are located 
    savePath=in_directory # path for debug output

    UV_scale=[2400, 1133] # resolution of intermediary UV map
    masks=[]
    gradients=[]
    # generate blend masks in UV space
    blended=[]
    imgCount=0

    for iarg,name in enumerate(DISPLAY_SERVERS):
        in_file_name = os.path.join(in_directory,"%s.nointerp.exr" % name)
        print "reading: ", in_file_name
        # read OpenEXR file (M is an r,g,b) tuple
        M = read_exr(in_file_name)
        (channels, height, width) = np.shape(M)

        print "--",np.shape(M)

        dsc = DisplayServerProxy("/"+name, wait=False,prefer_parameter_server_properties=True)

        for viewport in dsc.virtual_displays: # loop over viewports

            mask_index = dsc.get_virtual_display_mask(viewport,squeeze=True)
            # valid samples are where M[0] > -1 and mask_index==i
            L=np.logical_and(M[0]>-0.99, mask_index)

            print (M[0]>-0.99).shape, mask_index.shape,L.shape

            # coordinates of valid sample points
            YX=np.nonzero(L)

            # extract valid sample values into vectors
            U=M[0][L]
            V=M[1][L]

            # test for wrap around of U coordinate (texture seam)
            has_wraparound=((max(U)-min(U))>0.5)
            if has_wraparound:
                L=(U>0.5)
                U[L]-=0.5
                U[np.logical_not(L)]+=0.5

            q=np.transpose(YX)
            quv=np.transpose([V*UV_scale[1], U*UV_scale[0]])
 
            imgCount+=1

            ch=mergedHull(q, quv)
            #ch=convexHull(q)
            h = q[ch]
            tp=tuple((x[1], x[0]) for x in h)

            #ch=convexHull(quv)
            huv = quv[ch]
            t=tuple((x[1], x[0]) for x in huv)

            # now generate binary viewport mask in projector space
            mask = Image.new('F', (width, height), 0)
            drawMask=ImageDraw.Draw(mask)
            drawMask.polygon(tp, fill=1) # draw binary mask
            save_exr( savePath+"masks_"+str(imgCount)+".exr", r=mask, g=mask, b=mask, comments='' )        
            masks.append(np.array(mask))

            # now generate binary viewport mask in UV space
            img = Image.new('I', (UV_scale[0], UV_scale[1]), 0)
            draw=ImageDraw.Draw(img)
            draw.polygon(t, fill=1) # draw binary mask
            
            # calculate distance gradient in UV space
            p = np.array(img)
            pg=nd.distance_transform_edt(p)
            #plt.subplot(2,3, 3+i)

            if has_wraparound:
                pg=np.roll(pg, -np.shape(pg)[1]/2, axis=1)

#            save_exr( savePath+"gradient_"+str(imgCount)+".exr", r=pg, g=p, b=pg, comments='' )
            gradients.append(pg) # save gradient to list

    plt.show(block=False)

    # sum over all distance gradients
    gradSum=np.copy(gradients[0])
    for i in range(1, len(gradients)): # loop over other gradients
        gradSum+=gradients[i]
        
    save_exr( savePath+"gradSum.exr", r=gradSum, g=gradSum, b=gradSum, comments='' )    

    fig=plt.figure()
    fig.canvas.set_window_title('Blended Viewports in UV')
    for i in range(0,len(gradients)): # loop over viewports
        g=gradients[i]>0
        tr=np.zeros(np.shape(gradients[i])) 
        tr[g]=np.divide(gradients[i][g], gradSum[g])
#        save_exr( savePath+"gradientV_"+str(i)+".exr", r=tr, g=tr, b=tr, comments='' )
        blended.append(tr) # save blend mask to list
        plt.subplot(3, 3, i+1)
        plt.imshow(Image.fromarray(tr*255), origin='lower') # show blended masks
        
    plt.show(block=False)    
    plt.figure()
    count=0    # count masks
    for iarg,name in enumerate(DISPLAY_SERVERS):
        in_file_name = os.path.join(in_directory,"%s.exr" % name)
#        print "reading: ", in_file_name
        # read interpolated OpenEXR file    
        M = read_exr(in_file_name)

        # extract channels
        U=(M[0]*UV_scale[0]-0.5).astype(int)
        V=(M[1]*UV_scale[1]-0.5).astype(int)

        # prepare output image
        I=np.zeros(np.shape(U))
        J=np.zeros(np.shape(blended[count]))
        for v in range(1,4): # loop over all viewports of one projector
            #mask=np.nonzero(np.logical_and(masks[count], np.logical_and( M[0]>-1, M[1]>-1)))  # mask contains all pixels of the viewport
            mask=np.nonzero(masks[count])  # mask contains all pixels of the viewport
        #    u=np.max(1, U[mask]);
        #    v=np.max(1, V[mask]);
            # lookup into blended images on cylinder
            I[mask]=blended[count][(V[mask], U[mask])]
            J[(V[mask], U[mask])]+=1
            #I[mask]=gradients[count][(V[mask], U[mask])]+1
            count += 1  # running index of masks

        out_file_name = os.path.join(in_directory,"%s.blend.exr" % name)
        print "writing: ", out_file_name
        save_exr( out_file_name, r=M[0], g=M[1], b=I, comments='blended masks' )
        #save_exr( savePath+"gradientUV_"+str(iarg)+".exr", r=I, g=I, b=I, comments='blended masks' )
        #save_exr( savePath+"forward_"+str(iarg)+".exr", r=J, g=J, b=J, comments='' )
        
#        import pdb; pdb.set_trace()
            
    # show illustration
        plt.subplot(1, 3, iarg+1)
        plt.imshow(Image.fromarray(I*255), origin='lower') # show blended masks
        #plt.imshow(Image.fromarray(J*50), origin='lower') # show blended masks
        plt.show(block=False)
        
    plt.show()

    
if __name__ == "__main__":
    main2()
