import os.path
import numpy as np

from calib.visualization import create_point_cloud_message_publisher, create_camera_pose_message_publisher

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

def load_ascii_matrix(filename):
    fd=open(filename,mode='rb')
    buf = fd.read()
    lines = buf.split('\n')[:-1]
    return np.array([map(float,line.split()) for line in lines])

def save_ascii_matrix(arr,fd,isint=False):
    """
    write a np.ndarray with 2 dims
    """
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

    def _write_cam_ids(self, cam_ids):
        with open(os.path.join(self.out_dirname,'camera_order.txt'),'w') as f:
            for i,camid in enumerate(cam_ids):
                f.write("%s\n"%camid)

    def create_from_cams(self, cam_ids=[], cam_resolutions={}, cam_points={}, **kwargs):

        self._write_cam_ids(cam_ids)

        resfd = open(os.path.join(self.out_dirname,'Res.dat'), 'w')
        foundfd = open(os.path.join(self.out_dirname,'IdMat.dat'), 'w')
        pointsfd = open(os.path.join(self.out_dirname,'points.dat'), 'w')

        for cam in cam_ids:
            points = np.array(cam_points[cam])
            assert points.shape[1] == 2
            npts = points.shape[0]

            #add colum of 1s (homogenous coords, Z)
            points = np.hstack((points, np.ones((npts,1))))
            #multicamselfcal expects points rowwise (as multiple cams per file)
            points = points.T

            #detected points are those non-nan (just choose one axis, there is no
            #possible scenario where one axis is a valid coord and the other is nan
            #in my feature detection scheme
            found = points[0,:]
            #replace nans with 0 and numbers with 1
            found = np.nan_to_num(found).clip(max=1)

            res = np.array(cam_resolutions[cam])

            save_ascii_matrix(res.reshape((1,2)), resfd, isint=True)
            save_ascii_matrix(found.reshape((1,npts)), foundfd, isint=True)
            save_ascii_matrix(points, pointsfd)

        resfd.close()
        foundfd.close()
        pointsfd.close()

    def create_calibration_directory(self, cam_ids, IdMat, points, Res, cam_calibrations={}, radial_distortion=0, square_pixels=1):
        assert len(Res) == len(cam_ids)
        if cam_calibrations != None:
            assert len(cam_ids) == len(cam_calibrations)

        print 'points.shape',points.shape
        print 'IdMat.shape',IdMat.shape
        print 'Res',Res

        self._write_cam_ids(cam_ids)

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


    def reshape_calibrated_points(self, xe):
        return xe[0:3,:].T.tolist()

    def read_calibration_result(self):
        Xe = load_ascii_matrix(os.path.join(self.out_dirname,'Xe.dat'))
        Ce = load_ascii_matrix(os.path.join(self.out_dirname,'Ce.dat'))
        Re = load_ascii_matrix(os.path.join(self.out_dirname,'Re.dat'))

        return Xe,Ce,Re

    def publish_calibration_points(self, topic_base=''):
        xe,ce,re = self.read_calibration_result()

        create_point_cloud_message_publisher(
            self.reshape_calibrated_points(xe),
            topic_base+'/points',
            publish_now=True,
            latch=True)

        create_camera_pose_message_publisher(
            ce,re,['a','b','c','d'],
            topic_base+'/cameras',
            publish_now=True,
            latch=True)

    def save_to_pcd(self, fname):
        HEADER = \
        "# .PCD v.7 - Point Cloud Data file format\n"\
        "VERSION .7\n"\
        "FIELDS x y z\n"\
        "SIZE 4 4 4\n"\
        "TYPE F F F\n"\
        "COUNT 1 1 1\n"\
        "WIDTH %(npoints)d\n"\
        "HEIGHT 1\n"\
        "VIEWPOINT 0 0 0 1 0 0 0\n"\
        "POINTS %(npoints)d\n"\
        "DATA ascii"

        with open(fname, 'w') as fd:
            xe,ce,re = self.read_calibration_result()
            fd.write(HEADER % {"npoints":xe.shape[1]-1})
            for row in self.reshape_calibrated_points(xe):
                fd.write("%f %f %f\n" % tuple(row))

