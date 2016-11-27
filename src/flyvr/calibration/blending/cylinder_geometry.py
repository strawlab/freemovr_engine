import numpy as np
from scipy import ndimage
import cv2

class cylinder_geometry:
    def calculate_distance(self, Ub, Vb, UV_scale, U, V):
        """calculate distances to given polygon on a cylindrical geometry
        takes seam along U-coordinate into account

        Args:
            Ub, Vb:     polygon as vector of UV coordinates on the cylinder (U=circumference, V=height)
            UV_scale:   scale of internal lookup table for UV
            U, V:       UV coordinates for which distances are going to be calculated

        Returns:
            distances to input U,V coords in the same order as the input
            distances are calculated from the inner contour of this polygon (outside=0)

        """
        has_wraparound=(max(Ub)-min(Ub)>0.5) # Ub coordinate wraps over u seam

        # transform U by 0.5 to avoid seam
        if (has_wraparound):
            L=(Ub>0.5)
            Ub[L]-=0.5
            Ub[np.logical_not(L)]+=0.5

        # scale UV space arbitrary to improve precision
        Ub=np.multiply(Ub, UV_scale[0]-1)
        Vb=np.multiply(Vb, UV_scale[1]-1)
        img=np.zeros(UV_scale)
        UV=np.array([np.array((Vb,Ub))]).transpose(2,0,1).astype(np.int32) # astype(int) throws on 32bit system

        # fill beam path in UV space with 
        cv2.drawContours(img, [UV], 0, 1, -1)
        img=ndimage.distance_transform_edt(img)

        # transform result back over u seam
        if has_wraparound:
            img=np.roll(img, np.shape(img)[0]/2, axis=0)

        return img[((U*(UV_scale[0]-1)).astype(int),(V*(UV_scale[1]-1)).astype(int))]
