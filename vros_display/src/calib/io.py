import shutil
import os.path
import logging

import numpy as np

import roslib
roslib.load_manifest('motmot_ros_utils')
from rosutils.formats import camera_calibration_yaml_to_radfile

from calib.visualization import create_point_cloud_message_publisher, \
                                create_camera_pose_message_publisher

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
    lines = []
    for line in fd.readlines():
        if line[0] == "#":
            continue #comment
        lines.append(line.strip())
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

    LOG = logging.getLogger('mcsc')

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
                if camid[0] == "/":
                    camid=camid[1:]
                f.write("%s\n"%camid)

    def _write_cfg(self, cam_ids, radial_distortion, square_pixels):
        var = dict(
            basename = self.basename,
            num_cameras = len(cam_ids),
            undo_radial = int(radial_distortion),
            square_pixels = int(square_pixels),
            use_nth_frame = self.use_nth_frame
            )

        with open(os.path.join(self.out_dirname, 'multicamselfcal.cfg'), mode='w') as f:
            f.write(_cfg_file.format(**var))

    @property
    def cmd_string(self):
        return "octave gocal.m --config=%s" % os.path.join(self.out_dirname, "multicamselfcal.cfg")

    def create_from_cams(self, cam_ids=[], cam_resolutions={}, cam_points={}, cam_calibrations={}, **kwargs):

        #remove cameras with no points
        cams_to_remove = []
        for cam in cam_ids:
            nvalid = np.count_nonzero(np.nan_to_num(np.array(cam_points[cam])))
            if nvalid == 0:
                cams_to_remove.append(cam)
                self.LOG.warn("removing cam %s - no points detected" % cam)
        
        map(cam_ids.remove, cams_to_remove)

        self._write_cam_ids(cam_ids)

        resfd = open(os.path.join(self.out_dirname,'Res.dat'), 'w')
        foundfd = open(os.path.join(self.out_dirname,'IdMat.dat'), 'w')
        pointsfd = open(os.path.join(self.out_dirname,'points.dat'), 'w')

        for i,cam in enumerate(cam_ids):
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

            #write camera rad files if supplied
            if cam in cam_calibrations:
                url = cam_calibrations[cam]
                assert os.path.isfile(url)
                #i+1 because mcsc expects matlab numbering...
                dest = "%s/%s%d.rad" % (self.out_dirname, self.basename, i+1)
                if url.endswith('.yaml'):
                    camera_calibration_yaml_to_radfile(
                        url,
                        dest)
                elif url.endswith('.rad'):
                    shutil.copy(url,dest)
                else:
                    raise Exception("Calibration format %s not supported" % url)

        resfd.close()
        foundfd.close()
        pointsfd.close()

        self._write_cfg(cam_ids,
                #TODO: is this required??? Only undo radial distortion if all cameras are
                #calibrated
                radial_distortion=all([cam in cam_calibrations for cam in cam_ids]),
                square_pixels=True)

        print "wrote camera calibration directory: %s" % self.out_dirname

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

        self._write_cfg(cam_ids, radial_distortion, square_pixels)

    def reshape_calibrated_points(self, xe):
        return xe[0:3,:].T.tolist()

    def _read_calibration_result(self):
        Xe = load_ascii_matrix(os.path.join(self.out_dirname,'Xe.dat'))
        Ce = load_ascii_matrix(os.path.join(self.out_dirname,'Ce.dat'))
        Re = load_ascii_matrix(os.path.join(self.out_dirname,'Re.dat'))

        return Xe,Ce,Re

    def _read_calibration_names(self):
        f = os.path.join(self.out_dirname,'camera_order.txt')
        if os.path.isfile(f):
            return [l.strip() for l in open(f,'r').readlines()]
        else:
            print "WARNING: could not find camera_order.txt"
            return []

    def publish_calibration_points(self, topic_base=''):
        xe,ce,re = self._read_calibration_result()
        names = self._read_calibration_names()

        create_point_cloud_message_publisher(
            self.reshape_calibrated_points(xe),
            topic_base+'/points',
            publish_now=True,
            latch=True)

        create_camera_pose_message_publisher(
            ce,re,names,
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
            xe,ce,re = self._read_calibration_result()
            fd.write(HEADER % {"npoints":xe.shape[1]-1})
            for row in self.reshape_calibrated_points(xe):
                fd.write("%f %f %f\n" % tuple(row))

