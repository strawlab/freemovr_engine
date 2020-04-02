# distutils: language=c++
# distutils: sources = src/DisplaySurfaceGeometry.cpp src/DisplaySurfaceArbitraryGeometry.cpp
# distutils: language_level: 2

from libcpp.string cimport string
import freemovr_engine
import sys
import freemovr_engine.simple_geom

import numpy as np
cimport numpy as np

from DisplaySurfaceArbitraryGeometry_wrap cimport DisplaySurfaceArbitraryGeometry as cpp_DisplaySurfaceArbitraryGeometry

cdef class DisplaySurfaceArbitraryGeometry:
    cdef cpp_DisplaySurfaceArbitraryGeometry *thisptr
    def __cinit__(self, string filename, double precision):
        self.thisptr = new cpp_DisplaySurfaceArbitraryGeometry(filename,precision)
    def __dealloc__(self):
        del self.thisptr
    def texcoord2worldcoord(self, np.ndarray[np.float_t] u, np.ndarray[np.float_t] v):
        cdef np.ndarray[np.float_t] x = np.zeros( (u.shape[0],), dtype=np.float)
        cdef np.ndarray[np.float_t] y = np.zeros( (u.shape[0],), dtype=np.float)
        cdef np.ndarray[np.float_t] z = np.zeros( (u.shape[0],), dtype=np.float)
        cdef double xi=0, yi=0, zi=0
        cdef int err
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
        cdef int err
        for i in range( x.shape[0] ):
            err = self.thisptr.worldcoord2texcoord( x[i], y[i], z[i], ui, vi )
            if err:
                raise RuntimeError(
                    'failed: self.thisptr.worldcoord2texcoord()=%d'%err)
            u[i] = ui
            v[i] = vi
        return u,v
    def get_first_surface(self,
                          np.ndarray[np.float_t] ax,
                          np.ndarray[np.float_t] ay,
                          np.ndarray[np.float_t] az,
                          np.ndarray[np.float_t] bx,
                          np.ndarray[np.float_t] by,
                          np.ndarray[np.float_t] bz):
        cdef np.ndarray[np.float_t] sx = np.zeros( (ax.shape[0],), dtype=np.float)
        cdef np.ndarray[np.float_t] sy = np.zeros( (ax.shape[0],), dtype=np.float)
        cdef np.ndarray[np.float_t] sz = np.zeros( (ax.shape[0],), dtype=np.float)
        cdef double sxi=0, syi=0, szi=0
        cdef int err
        for i in range( ax.shape[0] ):
            err = self.thisptr.get_first_surface( ax[i], ay[i], az[i],
                                                  bx[i], by[i], bz[i],
                                                  sxi, syi, szi)
            if err:
                raise RuntimeError(
                    'failed: self.thisptr.get_first_surface()=%d'%err)
            sx[i] = sxi
            sy[i] = syi
            sz[i] = szi
        return sx,sy,sz

class ArbitraryGeometry(freemovr_engine.simple_geom.ModelBase):
    def __init__(self, string filename, double precision):
        self._filename = filename
        self._precision = precision
        self.geom = DisplaySurfaceArbitraryGeometry(filename,precision)

        u = np.expand_dims(np.linspace(0.0,1.0,20.),1)
        v = np.expand_dims(np.linspace(0.0,1.0,20.),0)
        U, V = np.broadcast_arrays(u,v)
        tcs = np.vstack((U.flatten(),V.flatten())).T
        wcs = self.texcoord2worldcoord(tcs)
        self.center_arr = np.mean(wcs,axis=0)
        super(ArbitraryGeometry,self).__init__()

    def __repr__(self):
        return '<PyDisplaySurfaceArbitraryGeometry.ArbitraryGeometry filename=%r precision=%s>'%(
            self._filename,self._precision)

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

    def get_first_surface(self,a,b):
        def split3(arr):
            # Parse inputs
            arr = np.array(arr,copy=False)
            assert arr.ndim==2
            assert arr.shape[1]==3
            return arr.T

        ax,ay,az = split3(a)
        bx,by,bz = split3(b)
        sx,sy,sz = self.geom.get_first_surface(ax,ay,az, bx,by,bz)
        result = np.array( [sx,sy,sz] ).T
        return result

    def get_relative_distance_to_first_surface(self,a,b):
        s = self.get_first_surface(a,b)
        bdist = np.sqrt(np.sum((b-a)**2,axis=1))
        sdist = np.sqrt(np.sum((s-a)**2,axis=1))
        return sdist/bdist
