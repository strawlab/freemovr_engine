import OpenEXR, Imath # openexr package (from pypi-install OpenEXR )
import numpy as np

def save_exr( fname, r=None, g=None, b=None ):
    r = np.array(r); assert r.ndim==2
    g = np.array(g); assert g.ndim==2; assert g.shape==r.shape
    b = np.array(b); assert b.ndim==2; assert b.shape==r.shape

    header = OpenEXR.Header(r.shape[1], r.shape[0])
    header['channels'] = {'R': Imath.Channel(Imath.PixelType(OpenEXR.FLOAT)),
                          'G': Imath.Channel(Imath.PixelType(OpenEXR.FLOAT)),
                          'B': Imath.Channel(Imath.PixelType(OpenEXR.FLOAT)),
                          }
    out = OpenEXR.OutputFile(fname, header)
    data = {'R': r.astype(np.float32).tostring(),
            'G': g.astype(np.float32).tostring(),
            'B': b.astype(np.float32).tostring()}
    out.writePixels(data)
    out.close()

