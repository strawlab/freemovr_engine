from libcpp.string cimport string
import flyvr
import sys
import flyvr.simple_geom

import numpy as np
cimport numpy as np

from DisplaySurfaceArbitraryGeometry_wrap cimport DisplaySurfaceArbitraryGeometry as cpp_DisplaySurfaceArbitraryGeometry

cdef class DisplaySurfaceArbitraryGeometry:
    cdef cpp_DisplaySurfaceArbitraryGeometry *thisptr
    def __cinit__(self, string filename):
        self.thisptr = new cpp_DisplaySurfaceArbitraryGeometry(filename)
    def __dealloc__(self):
        del self.thisptr
    def texcoord2worldcoord(self, np.ndarray[np.float_t] u, np.ndarray[np.float_t] v):
        cdef np.ndarray[np.float_t] x = np.zeros( (u.shape[0],), dtype=np.float)
        cdef np.ndarray[np.float_t] y = np.zeros( (u.shape[0],), dtype=np.float)
        cdef np.ndarray[np.float_t] z = np.zeros( (u.shape[0],), dtype=np.float)
        cdef double xi=0, yi=0, zi=0
        for i in range( u.shape[0] ):
            err = self.thisptr.texcoord2worldcoord( u[i], v[i], xi, yi, zi )
            if err:
                raise RuntimeError(
                    'failed: self.thisptr.texcoord2worldcoord()=%d'%err)
            x[i] = xi
            y[i] = yi
            z[i] = zi
        return x,y,z
    def worldcoord2texcoord(self, np.ndarray[np.float_t] x, np.ndarray[np.float_t] y, np.ndarray[np.float_t] z):
        cdef np.ndarray[np.float_t] u = np.zeros( (x.shape[0],), dtype=np.float)
        cdef np.ndarray[np.float_t] v = np.zeros( (x.shape[0],), dtype=np.float)
        cdef double ui=0, vi=0
        for i in range( x.shape[0] ):
            err = self.thisptr.worldcoord2texcoord( x[i], y[i], z[i], ui, vi )
            if err:
                raise RuntimeError(
                    'failed: self.thisptr.worldcoord2texcoord()=%d'%err)
            u[i] = ui
            v[i] = vi
        return u,v

class ArbitraryGeometry(flyvr.simple_geom.ModelBase):
    def __init__(self, string filename):
        self.geom = DisplaySurfaceArbitraryGeometry(filename)
        super(ArbitraryGeometry,self).__init__()

    def texcoord2worldcoord(self,tc):
        # Parse inputs
        tc = np.array(tc,copy=False)
        assert tc.ndim==2
        assert tc.shape[1]==2
        tc = tc.T

        u, v = tc
        x,y,z = self.geom.texcoord2worldcoord(u,v)
        result = np.array( [x,y,z] ).T
        return result

    def worldcoord2texcoord(self,wc):
        # Parse inputs
        wc = np.array(wc,copy=False)
        assert wc.ndim==2
        assert wc.shape[1]==3
        wc = wc.T

        x,y,z = wc
        u,v = self.geom.worldcoord2texcoord(x,y,z)
        result = np.array( [u,v] ).T
        return result
