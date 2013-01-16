import time

import math
import numpy as np
import scipy.ndimage
import cv,cv2
import traceback

class DotBGFeatureDetector:
    WIN_TYPES = {
        "I":"img",
        "B":"bg",
        "D":"diff",
        "F":"features"
    }
    DETECT_METHODS = {
        "med":"median filter, peak fitting to sum per axis",
        "medbinary":"median filter, binarise, argmax",
        "morphbinary":"binarize, morphological filter, argmax"
    }
    def __init__(self, name, method="medbinary", show="DF", benchmark=False, diff_thresh=70, debug=False):
        assert method in self.DETECT_METHODS
        self._name = name
        self._method = method
        self._show = show
        self._thresh = diff_thresh
        self._debug = debug
        self._benchmark = False
        self._handles = {}
        for s in show:
            if s in self.WIN_TYPES:
                handle = self._name+"-"+self.WIN_TYPES[s]
                cv2.namedWindow(handle)
                self._handles[s] = handle
        self._bg = None
        self._imgsize = None
        self._shape = (-1,-1)
        self._mask = None

    @property
    def img_shape(self):
        return self._shape
    @property
    def img_width_px(self):
        return self._shape[1]   #swap from matrix semantics (row/col) to image coords
    @property
    def img_height_px(self):
        return self._shape[0]   #swap from matrix semantics (row/col) to image coords

    def _show_img(self, arr, win_type):
        if win_type in self._handles:
            if self._mask != None:
                img = arr * self._mask
            else:
                img = arr
            cv2.imshow(self._handles[win_type], img)

    def _show_features_and_diff(self, diff, dmax, features, sz=-1, scalediff=False,
                                                                   scalediffonlyfeatures=False,
                                                                   showdmax=True):
        if "F" in self._handles:
            if scalediff:
                if not scalediffonlyfeatures or (scaldiffonlyfeatures and features):
                    if dmax < 255:
                        diff /= (dmax / 255.0)
            
            b = np.zeros(diff.shape,dtype=np.uint8)
            g = np.zeros(diff.shape,dtype=np.uint8)
            rgb = np.dstack((b,g,diff))

            #dstack doesnt copy the memory in such a way that it works with
            #opencv.... sigh.
            img = rgb.copy()
            
            for f in features:
                rf,cf = int(f[0]),int(f[1])
                add_crosshairs_to_nparr(
                        arr=img,
                        row=rf, col=cf,
                        sz=sz, fill=255, chan=1)

            if showdmax:
                cv2.putText(img, "%d" % dmax, (20,20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0))

            cv2.imshow(self._handles["F"], img)

    def _argmax(self, arr):
        return np.unravel_index(arr.argmax(), arr.shape)

    def set_mask(self, arr, copy=True):
        if copy:
            #FIXME: DO I NEED TO MAKE THIS COPY IN ANY CASE???
            self._mask = arr.astype(np.bool).copy()
        else:
            self._mask = arr

    def clear_mask(self):
        self._mask = None

    def compute_bg(self, bgarr):
        shape = bgarr.shape
        assert len(shape) == 3
        self._shape = shape[0:2]
        self._bg = np.min(bgarr,2)
        self._show_img(self._bg, "B")

    def detect(self, imarr, thresh=None):
        """
        returns in matrix coordinates: row, col
        """
        if not thresh:
            thresh = self._thresh

        self._show_img(imarr, "I")

        t1 = time.time()

        if self._bg != None:
            #prevent wrap-around unsiged subtraction (is there a better way to do this??)
            diff = imarr.astype(np.int16) - self._bg
            diff = diff.clip(0,255).astype(np.uint8)
        else:
            diff = imarr.astype(np.uint8)

        if self._mask != None:
            diff *= self._mask

        dmax = diff.max()

        self._show_img(diff, "D")

        if self._debug:
            print "diff max: %d (thresh: %d) %s" % (dmax, thresh, self._name)

        if dmax < thresh:
            features = []
            feature_detector_vis_diff = diff
        elif self._method == "medbinary":
            scipy.ndimage.median_filter(diff,3,output=diff)
            binary = np.where(diff > thresh, 255, 0).astype(np.uint8)
            feature_detector_vis_diff = binary
            features = [self._argmax(binary)]
        elif self._method == "morphbinary":
            binary = np.where(diff > thresh, 255, 0)
            scipy.ndimage.binary_opening(binary, output=binary)
            feature_detector_vis_diff = (binary*255).astype(np.uint8)
            features = [self._argmax(binary)]
        elif self._method == "med":
            scipy.ndimage.median_filter(diff,3,output=diff)
            feature_detector_vis_diff = diff
            features = [self._argmax(diff)]
        else:
            raise Exception("Not Supported")

        t2 = time.time()
        if self._benchmark:
            print "%s (%s) = %.1fms" % (self._method, self._name, (t2-t1)*1000)

        self._show_features_and_diff(feature_detector_vis_diff, dmax, features)

        return features

def load_mask_image(mask_image_fname):
    """
    load the RGBA png image and return a numpy array of bools. Alpha
    transparent pixels map to True

    MAKE THE PARTS OF THE IMAGE YOU WISH TO ANALYSE TRANSPARENT
    """
    im = scipy.misc.imread( mask_image_fname )
    if len(im.shape) != 3:
        raise ValueError('mask image must have color channels (shape was %s)' % repr(im.shape))
    if im.shape[2] != 4:
        raise ValueError('mask image must have an alpha (4th) channel (shape was %s)' % repr(im.shape))
    alpha = im[:,:,3]
    mask = ~alpha.astype(np.bool)
    return mask

def add_crosshairs_to_nparr(arr, row, col, sz=-1, fill=255, chan=None):
    ri = arr.shape[0]
    ci = arr.shape[1]
    
    row = np.clip(math.floor(row),0,ri-1)
    col = np.clip(math.floor(col),0,ci-1)
    
    try:
        if sz < 0:
            if chan == None:
                arr[row,0:ci] = fill
                arr[0:ri,col] = fill
            else:
                arr[row,0:ci,chan] = fill
                arr[0:ri,col,chan] = fill
        elif sz == 0:
            if chan == None:
                arr[row,col] = fill
            else:
                arr[row,col,chan] = fill
        else:
            if chan == None:
                arr[max(0,row-sz):min(row+sz,ri),max(0,col-sz):min(col+sz,ci)] = fill
            else:
                arr[max(0,row-sz):min(row+sz,ri),max(0,col-sz):min(col+sz,ci),chan] = fill
    except:
        print "-----"*5
        print "ERROR PAINTING ARRAY",row,col,sz,fill,chan,arr.shape
        traceback.print_exc()
        print "-----"*5



