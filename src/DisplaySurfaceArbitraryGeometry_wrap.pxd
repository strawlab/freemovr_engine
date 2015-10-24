from libcpp.string cimport string

cdef extern from "DisplaySurfaceArbitraryGeometry.h" namespace "flyvr":
    cdef cppclass DisplaySurfaceArbitraryGeometry:
        DisplaySurfaceArbitraryGeometry(string filename, double eps) nogil except +
        int texcoord2worldcoord( double u, double v, double &x, double &y, double &z )
        int worldcoord2texcoord( double x, double y, double z, double &u, double &v)
        int get_first_surface( double ax, double ay, double az,
                               double bx, double by, double bz,
                               double &sx, double &sy, double &sz )
