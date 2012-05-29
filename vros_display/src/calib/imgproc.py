import time

import numpy as np
import scipy.ndimage
import cv,cv2

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

    def _show_features_and_diff(self, diff, features, sz=-1):
        if "F" in self._handles:
            ri,ci = diff.shape
            b = np.zeros(diff.shape,dtype=np.uint8)
            g = np.zeros(diff.shape,dtype=np.uint8)

            for f in features:
                rf,cf = int(f[0]),int(f[1])
                if sz < 0:
                    g[rf,0:ci] = 255
                    g[0:ri,cf] = 255
                elif sz == 0:
                    g[rf,cf] = 255
                else:
                    g[max(0,rf-sz):min(rf+sz,ri),max(0,cf-sz):min(cf+sz,ci)] = 255

            rgb = np.dstack((b,g,diff))
            cv2.imshow(self._handles["F"], rgb.copy())

    def _argmax(self, arr):
        return np.unravel_index(arr.argmax(), arr.shape)

    def set_mask(self, arr):
        self._mask = arr.astype(np.bool).copy()

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

        #prevent wrap-around unsiged subtraction (is there a better way to do this??)
        diff = imarr.astype(np.int16) - self._bg
        diff = diff.clip(0,255).astype(np.uint8)

        if self._mask != None:
            diff *= self._mask

        self._show_img(diff, "D")

        if self._debug:
            print "diff max: %d (thresh: %d) %s" % (diff.max(), thresh, self._name)

        if diff.max() < thresh:
            features = []
            feature_detector_vis = diff
        elif self._method == "medbinary":
            scipy.ndimage.median_filter(diff,3,output=diff)
            binary = np.where(diff > thresh, 255, 0).astype(np.uint8)
            feature_detector_vis = binary
            features = [self._argmax(binary)]
        elif self._method == "morphbinary":
            binary = np.where(diff > thresh, 255, 0)
            scipy.ndimage.binary_opening(binary, output=binary)
            feature_detector_vis = (binary*255).astype(np.uint8)
            features = [self._argmax(binary)]
        elif self._method == "med":
            scipy.ndimage.median_filter(diff,3,output=diff)
            feature_detector_vis = diff
            features = [self._argmax(diff)]
        else:
            raise Exception("Not Supported")

        t2 = time.time()
        if self._benchmark:
            print "%s (%s) = %.1fms" % (self._method, self._name, (t2-t1)*1000)

        self._show_features_and_diff(feature_detector_vis, features)

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


