import time
import math
import numpy as np
import scipy.ndimage
import cv2
import traceback

class DotBGFeatureDetector:
    WIN_TYPES = {
        "I":"img",
        "B":"bg",
        "D":"diff",
        "F":"features"
    }
    DETECT_METHODS = {
        "med":"median filter, blob detect",
        "morphbinary":"binarize, morphological filter, blob detect"
    }
    def __init__(self, name, method="med", show="DF"):
        assert method in self.DETECT_METHODS
        self._name = name
        self._safe_name = self._name.replace('/','')
        self._method = method
        self._show = show
        self._thresh = None
        self._debug = False
        self._benchmark = False
        self._save_fmt = None
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
        self._n = 0
        self._debug_b = None

    @property
    def img_shape(self):
        return self._shape
    @property
    def img_width_px(self):
        return self._shape[1]   #swap from matrix semantics (row/col) to image coords
    @property
    def img_height_px(self):
        return self._shape[0]   #swap from matrix semantics (row/col) to image coords

    def _get_path(self, win_type):
        return self._save_fmt % {
                            "imgn":self._n,
                            "imgtype":win_type,
                            "name":self._safe_name,
                            "time":time.time()
        }

    def _show_img(self, arr, win_type):
        img = arr
        if win_type in self._handles:
            if self._mask != None:
                img = arr * self._mask
            cv2.imshow(self._handles[win_type], img)

        if self._save_fmt is not None:
            fname = self._get_path(win_type)
            #print 'for win_type %s, saving to %s'%(win_type,fname)
            cv2.imwrite(fname, arr)

    def _show_features_and_diff(self, diff, dmax, features, sz=-1):
        if "F" in self._handles:
            if self._debug_b is not None:
                b = self._debug_b
            else:
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

            cv2.putText(img, "%d" % dmax, (20,20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))
            cv2.imshow(self._handles["F"], img)

            if self._save_fmt is not None:
                cv2.imwrite(self._get_path("F"), img)
        else:
            if self._save_fmt is not None:
                cv2.imwrite(self._get_path("F"), diff)

    def _detect_blobs_and_luminance(self, imarr, diff, validmask, exact_luminance=False, use_argmax=False):
        #note: we modify diff in place here, but it has already been saved to
        #disk and shown to the user, so no problem.

        assert imarr.ndim == 2
        assert imarr.shape == diff.shape
        assert diff.shape == validmask.shape

        features = []

        if use_argmax:
            row, col = np.unravel_index(diff.argmax(), diff.shape)
            if exact_luminance:
                #return the mean of all pixels above the threshold
                lum = imarr[validmask].mean()
            else:
                lum = imarr[row,col]
            features.append( (row,col,lum) )
        else:
            #ndimage.label treats non-zero as valid, so set all pixels below
            #the thresh (invalid) to zero
            diff[~validmask] = 0
            lbls,maxlabel = scipy.ndimage.measurements.label(diff)
            slcs = scipy.ndimage.measurements.find_objects(lbls)

            #according to the implementation, the second return argument is actually
            #the maximum label(integer), and not necessarily the number of returned
            #labelled objects, although for that is assumed by other parts of the
            #ndimage code.
            for n in range(1,maxlabel+1): #0 is code for unlabelled
                row,col = list(map(int,scipy.ndimage.measurements.center_of_mass(diff,lbls,n)))
                if exact_luminance:
                    blob = imarr[slcs[n-1]]
                    lum = blob.sum()/np.count_nonzero(blob)
                else:
                    lum = imarr[row,col]

                features.append( (row,col,lum) )

        return features

    def enable_debug_detection(self):
        self._debug = True

    def enable_benchmark(self):
        self._benchmark = True

    def enable_debug_saveimages(self, fmt="%(imgn)d_%(name)s_%(imgtype)s.png"):
        self._save_fmt = fmt

    def add_debug_blue_channel(self, img):
        assert img.dtype == np.uint8
        self._debug_b = img

    def set_mask(self, arr, copy=True):
        if (arr is not None) and copy:
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

    def detect(self, imarr, thresh, exact_luminance=False):
        """
        returns in matrix coordinates: [row, col], dmax
        """
        self._n += 1

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
            print("diff max: %d (thresh: %d) %s" % (dmax, thresh, self._name))

        if dmax < thresh:
            features = []
            feature_detector_vis_diff = diff
        elif self._method == "morphbinary":
            valid = diff > thresh
            scipy.ndimage.binary_opening(valid, output=valid)
            feature_detector_vis_diff = (valid*255).astype(np.uint8)
            features = self._detect_blobs_and_luminance(imarr, valid, valid, exact_luminance)
        elif self._method == "med":
            scipy.ndimage.median_filter(diff,3,output=diff)
            valid = diff > thresh
            feature_detector_vis_diff = diff
            features = self._detect_blobs_and_luminance(imarr, diff, valid, exact_luminance)
        else:
            raise Exception("Not Supported")

        t2 = time.time()
        if self._benchmark:
            print("%s (%s) = %.1fms" % (self._method, self._name, (t2-t1)*1000))

        self._show_features_and_diff(feature_detector_vis_diff, dmax, features)

        return features,dmax

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
        print("-----"*5)
        print("ERROR PAINTING ARRAY",row,col,sz,fill,chan,arr.shape)
        traceback.print_exc()
        print("-----"*5)



