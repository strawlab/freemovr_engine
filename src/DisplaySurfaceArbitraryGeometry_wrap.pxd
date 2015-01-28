from libcpp.string cimport string

cdef extern from "DisplaySurfaceArbitraryGeometry.h" namespace "flyvr":
    cdef cppclass DisplaySurfaceArbitraryGeometry:
        DisplaySurfaceArbitraryGeometry(string filename) nogil except +
        int texcoord2worldcoord( double u, double v, double &x, double &y, double &z )
        int worldcoord2texcoord( double x, double y, double z, double &u, double &v)
