import OpenEXR, Imath # openexr package (from pypi-install OpenEXR )
import numpy as np

PIXEL_TYPE = Imath.PixelType(OpenEXR.FLOAT)

def save_exr( fname, r=None, g=None, b=None ):
    r = np.array(r); assert r.ndim==2
    g = np.array(g); assert g.ndim==2; assert g.shape==r.shape
    b = np.array(b); assert b.ndim==2; assert b.shape==r.shape

    header = OpenEXR.Header(r.shape[1], r.shape[0])
    header['channels'] = {'R': Imath.Channel(PIXEL_TYPE),
                          'G': Imath.Channel(PIXEL_TYPE),
                          'B': Imath.Channel(PIXEL_TYPE),
                          }
    out = OpenEXR.OutputFile(fname, header)
    data = {'R': r.astype(np.float32).tostring(),
            'G': g.astype(np.float32).tostring(),
            'B': b.astype(np.float32).tostring()}
    out.writePixels(data)
    out.close()

def read_exr(file):
    f = OpenEXR.InputFile(file)
    dw = f.header()['dataWindow']
    size = (dw.max.x - dw.min.x + 1, dw.max.y - dw.min.y + 1)
    pt = Imath.PixelType(Imath.PixelType.FLOAT)

    def read_chan(name):
        datastr = f.channel(name, pt)
        data = np.fromstring(datastr, dtype = np.float32)
        data.shape = (size[1], size[0]) # Numpy arrays are (row, col)
        return data

    r = read_chan('R')
    g = read_chan('G')
    b = read_chan('B')
    f.close()

    return r,g,b

