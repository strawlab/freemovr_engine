import os.path
import numpy as np

_cfg_file = """[Files]
Basename: {basename}
Image-Extension: jpg

[Images]
Subpix: 0.5

[Calibration]
Num-Cameras: {num_cameras}
Num-Projectors: 0
Nonlinear-Parameters: 50    0    1    0    0    0
Nonlinear-Update: 1   0   1   0   0   0
Initial-Tolerance: 10
Do-Global-Iterations: 0
Global-Iteration-Threshold: 0.5
Global-Iteration-Max: 5
Num-Cameras-Fill: 2
Do-Bundle-Adjustment: 1
Undo-Radial: {undo_radial}
Min-Points-Value: 30
N-Tuples: 3
Square-Pixels: {square_pixels}
Use-Nth-Frame: {use_nth_frame}
"""

def save_ascii_matrix_FIXME_OTHER(M,fd,isint=False):
    #XXJOHN: from flydra
    def fmt(f):
        if isint:
            return '%d'%f
        else:
            return '%8e'%f
    A = np.asarray(M)
    if len(A.shape) == 1:
        A=np.reshape(A, (1,A.shape[0]) )

    close_file = False
    if type(fd) == str:
        fd = open(fd,mode='wb')
        close_file = True

    for i in range(A.shape[0]):
        buf = ' '.join( map( fmt, A[i,:] ) )
        fd.write( buf )
        fd.write( '\n' )
    if close_file:
        fd.close()

def save_ascii_matrix(arr,fd,isint=False):
    assert arr.ndim==2
    if arr.dtype==np.bool:
        arr = arr.astype( np.uint8 )

    close_file = False
    if type(fd) == str:
        fd = open(fd,mode='wb')
        close_file = True

    for row in arr:
        row_buf = ' '.join( map(repr,row) )
        fd.write(row_buf)
        fd.write('\n')

    if close_file:
        fd.close()

class MultiCalSelfCam:
    def __init__(self, out_dirname, basename='cam', use_nth_frame=1):
        out_dirname = os.path.abspath(os.path.expanduser(out_dirname))
        if not os.path.isdir(out_dirname):
            os.mkdir(out_dirname)

        self.out_dirname = out_dirname
        self.basename = basename
        self.use_nth_frame = use_nth_frame

    def create_calibration_directory(self, IdMat=None, points=None, Res=None, cam_ids=[], radial_distortion=0, square_pixels=1, cam_calibrations=None):
        assert len(Res) == len(cam_ids)
        if cam_calibrations != None:
            assert len(cam_ids) == len(cam_calibrations)

        print 'points.shape',points.shape
        print 'IdMat.shape',IdMat.shape
        print 'Res',Res

        with open(os.path.join(self.out_dirname,'camera_order.txt'),'w') as f:
            for i,camid in enumerate(cam_ids):
                f.write("%s\n"%camid)

            if cam_calibrations:
                print 'write cam calib to rad file', cam_calibrations[i]

        save_ascii_matrix(Res, os.path.join(self.out_dirname,'Res.dat'), isint=True)
        save_ascii_matrix(IdMat, os.path.join(self.out_dirname,'IdMat.dat'), isint=True)
        save_ascii_matrix(points, os.path.join(self.out_dirname,'points.dat'))

        var = dict(
            basename = self.basename,
            num_cameras = len(cam_ids),
            undo_radial = int(radial_distortion),
            square_pixels = int(square_pixels),
            use_nth_frame = self.use_nth_frame
            )

        with open(os.path.join(self.out_dirname, 'multicamselfcal.cfg'), mode='w') as f:
            f.write(_cfg_file.format(**var))

    def cmd_string(self):
        return "octave gocal.m --config=%s" % os.path.join(self.out_dirname, "multicamselfcal.cfg")

