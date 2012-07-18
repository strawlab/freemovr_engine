import shutil
import os.path
import logging
import tempfile
import shutil
import pickle

import numpy as np
import scipy.io

import roslib
roslib.load_manifest('motmot_ros_utils')
from rosutils.formats import camera_calibration_yaml_to_radfile

from calib.visualization import create_point_cloud_message_publisher, \
                                create_camera_pose_message_publisher, \
                                create_pcd_file_from_points

from . commandwrapper import WrapCommand as ThreadedCommand

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
Num-Cameras-Fill: {num_cameras_fill}
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

class _Calibrator:
    def __init__(self, out_dirname, **kwargs):
        if out_dirname:
            out_dirname = os.path.abspath(os.path.expanduser(out_dirname))
            if not os.path.isdir(out_dirname):
                os.mkdir(out_dirname)
        else:
            out_dirname = tempfile.mkdtemp(prefix=self.__class__.__name__)

        self.octave = '/usr/bin/octave'
        self.matlab = '/opt/matlab/R2011a/bin/matlab'
        self.out_dirname = out_dirname

    def create_from_cams(self, cam_ids=[], cam_resolutions={}, cam_points={}, cam_calibrations={}, **kwargs):
        raise NotImplementedError

class VincentSFM(_Calibrator):

    LOG = logging.getLogger('vsfm')

    def __init__(self, out_dirname):
        _Calibrator.__init__(self, out_dirname)

    def create_from_cams(self, cam_ids=[], cam_resolutions={}, cam_points={}, cam_calibrations={}, **kwargs):
        #remove cameras with no points
        cams_to_remove = []
        for cam in cam_ids:
            nvalid = np.count_nonzero(np.nan_to_num(np.array(cam_points[cam])))
            if nvalid == 0:
                cams_to_remove.append(cam)
                self.LOG.warn("removing cam %s - no points detected" % cam)
        map(cam_ids.remove, cams_to_remove)



        for i,cam in enumerate(cam_ids):
            points = np.array(cam_points[cam])
            assert points.shape[1] == 2
            npts = points.shape[0]

        W = np.dstack([np.array(cam_points[c]).T for c in cam_ids])

        dest = self.out_dirname+'/points.mat'
        scipy.io.savemat(dest,{'W':W})
        print dest

class AllPointPickle:

    LOG = logging.getLogger('allpoints')

    def __init__(self, directory=None):
        self._directory = directory
        self.results = {}
        self.results_mode = {}
        self.resolutions = {}
        self.num_results = 0

    def _load_previous_calibration_pickle(self, path):
        try:
            with open(path,'r') as f:
                results = pickle.load(f)
                self.LOG.info("loaded previous calibration data: %s" % path)
        except Exception, e:
            self.LOG.warn("error loading: %s" % e)
        return results

    def load_previous_calibration(self, path):
        results = self._load_previous_calibration_pickle(os.path.join(path,'results.pkl'))
        results_mode = self._load_previous_calibration_pickle(os.path.join(path,'results_mode.pkl'))
        resolutions = self._load_previous_calibration_pickle(os.path.join(path,'resolution.pkl'))
        
        #check the results arrays are all the same size
        num_results = 0
        if results:
            num_results = len(results[results.keys()[0]])
            for r in results:
                if num_results != len(results[r]):
                    raise Exception("Not all calibratable elements have the same number of points")
        num_results = num_results

        return results,results_mode,resolutions,num_results

    def initilize_from_directory(self, directory=None):
        if not directory:
            directory = self._directory
        if not directory:
            raise ValueError("Directory must be specified")

        self.results,self.results_mode,self.resolutions,self.num_results = \
                self.load_previous_calibration(directory)

        if all([c[0] == '/' for c in self.results.keys()]):
            self._cameras_strip_slash = True

    def save_calibration(self, directory=None, **kwargs):
        if not directory:
            directory = self._directory
        if not directory:
            raise ValueError("Directory must be specified")

        results = kwargs.get("results",self.results)
        results_mode = kwargs.get("results_mode", self.results_mode)
        resolutions = kwargs.get("resolutions",self.resolutions)

        with open(directory+'/results.pkl','w') as f:
            pickle.dump(results,f)
        with open(directory+'/results_mode.pkl','w') as f:
            pickle.dump(results_mode,f)
        with open(directory+'/resolution.pkl','w') as f:
            pickle.dump(resolutions,f)

    def get_points_in_cameras(self, *camera_ids, **kwargs):
        """
        Return all points visible in all cameras supplied in camera_ids.

        If camera_ids is not supplied, only points visible in all will be returned.

        The result can be returned in two ways.
        1. result_format=dict
            A dict where the key is the camera_id and the value is a list of points
        2. result_format=list
            A list of 2-tuples, the first element is the camera id, the second is the point
            (this format is designed to be consumed by flydra.reconstructor)

        A point is always a 2-tuple (x,y)
        """
        result_format = kwargs.get("result_format",dict)
        if result_format not in (list,dict):
            raise ValueError("Result format must be a list or a dict")
        if not camera_ids:
            camera_ids = a.cameras

        #enforce the API
        if any([c[0] == '/' for c in camera_ids]):
            raise ValueError("Camera IDs can not start with /")

        #internally work around my lazyness
        if self._cameras_strip_slash:
            camera_ids = ['/%s'%c for c in camera_ids]

        result = result_format()
        for i in range(self.num_results):
            #check points are in ALL requested cameras
            for c in camera_ids:
                pt = self.results[c][i]
                if np.isnan(pt).any():
                    #not visible in 1 camera, reject point
                    break

            #point was visible in all cameras.
            for c in camera_ids:
                pt = self.results[c][i]
                if self._cameras_strip_slash:
                    c = c[1:]
                if result_format == dict:
                    try:
                        result[c].append(pt)
                    except KeyError:
                        result[c] = [pt]
                else:
                    result.append( (c,pt) )

        return result

    @property
    def cameras(self):
        if self._cameras_strip_slash:
            return [c[1:] for c in self.results.keys()]
        else:
            return self.results.keys()

class MultiCalSelfCam(_Calibrator):

    LOG = logging.getLogger('mcsc')
    INPUT = ("camera_order.txt","IdMat.dat","points.dat","Res.dat","multicamselfcal.cfg")

    def __init__(self, out_dirname, basename='cam', use_nth_frame=1, mcscdir='/opt/MultiCamSelfCal/MultiCamSelfCal/'):
        _Calibrator.__init__(self, out_dirname)
        self.mcscdir = mcscdir
        self.basename = basename
        self.use_nth_frame = use_nth_frame
        
        if not os.path.exists(os.path.join(self.mcscdir,'gocal.m')):
            self.LOG.warn("could not find MultiCamSelfCal gocal.m in %s" % self.mcscdir)

    def _write_cam_ids(self, cam_ids):
        with open(os.path.join(self.out_dirname,'camera_order.txt'),'w') as f:
            for i,camid in enumerate(cam_ids):
                if camid[0] == "/":
                    camid=camid[1:]
                f.write("%s\n"%camid)

    def _write_cfg(self, cam_ids, radial_distortion, square_pixels, num_cameras_fill):
        if num_cameras_fill < 0 or num_cameras_fill > len(cam_ids):
            num_cameras_fill = len(cam_ids)

        var = dict(
            basename = self.basename,
            num_cameras = len(cam_ids),
            num_cameras_fill = int(num_cameras_fill),
            undo_radial = int(radial_distortion),
            square_pixels = int(square_pixels),
            use_nth_frame = self.use_nth_frame
            )

        with open(os.path.join(self.out_dirname, 'multicamselfcal.cfg'), mode='w') as f:
            f.write(_cfg_file.format(**var))
            
        print "calibrate cams: %s" % ','.join(cam_ids)
        print "undo radial: ", radial_distortion
        print "num_cameras_fill: ", num_cameras_fill
        print "wrote camera calibration directory: %s" % self.out_dirname


    @property
    def cmd_string(self):
        return "octave gocal.m --config=%s" % os.path.join(self.out_dirname, "multicamselfcal.cfg")

    def execute(self, blocking=True, cb=None, use_matlab=False, dest=None, silent=True, copy_files=True):
        """
        if dest is specified then all files are copied there unless copy is false. If dest is not
        specified then it is in a subdir of out_dirname called result        

        @returns: dest (or nothing if blocking is false). In that case cb is called when complete
        and is passed the dest argument
        """
        if not dest:
            dest = os.path.join(self.out_dirname,'result')
            if not os.path.isdir(dest):
                os.makedirs(dest)

        if silent:
            stdout = open(os.path.join(dest,'STDOUT'),'w')
            stderr = open(os.path.join(dest,'STDERR'),'w')
        else:
            stdout = stderr = None

        for f in self.INPUT:
            src = os.path.join(self.out_dirname,f)
            if copy_files:
                shutil.copy(src, dest)
            else:
                if not os.path.isfile(src):
                    raise ValueError("Could not find %s" % src)

        cfg = os.path.abspath(os.path.join(dest, "multicamselfcal.cfg"))

        if use_matlab:
            cmds = '%s -nodesktop -nosplash -r "cd(\'%s\'); gocal_func(\'%s\'); exit"' % (
                        self.matlab, self.mcscdir, cfg)
            cwd = None
        else:
            cmds = '%s gocal.m --config=%s' % (
                        self.octave, cfg)
            cwd = self.mcscdir

        cmd = ThreadedCommand(cmds,cwd=cwd,stdout=stdout,stderr=stderr)
        cmd.set_finished_cb(cb,dest)
        cmd.start()

        if blocking:
            cmd.join()
            return dest
            
    def create_from_cams(self, cam_ids=[], cam_resolutions={}, cam_points={}, cam_calibrations={}, num_cameras_fill=-1, **kwargs):
        #num_cameras_fill = -1 means use all cameras (= len(cam_ids))
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

        undo_radial = all([cam in cam_calibrations for cam in cam_ids])
        self._write_cfg(cam_ids,
                        undo_radial,
                        True,
                        num_cameras_fill)
        print "dropped cams: %s" % ','.join(cams_to_remove)

    def create_calibration_directory(self, cam_ids, IdMat, points, Res, cam_calibrations={}, radial_distortion=0, square_pixels=1, num_cameras_fill=-1):
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

        self._write_cfg(cam_ids, radial_distortion, square_pixels, num_cameras_fill)

    @staticmethod
    def reshape_calibrated_points(xe):
        return xe[0:3,:].T.tolist()

    @staticmethod
    def read_calibration_result(out_dirname):
        Xe = load_ascii_matrix(os.path.join(out_dirname,'Xe.dat'))
        Ce = load_ascii_matrix(os.path.join(out_dirname,'Ce.dat'))
        Re = load_ascii_matrix(os.path.join(out_dirname,'Re.dat'))

        return Xe,Ce,Re

    @staticmethod
    def read_calibration_names(out_dirname):
        f = os.path.join(out_dirname,'camera_order.txt')
        if os.path.isfile(f):
            return [l.strip() for l in open(f,'r').readlines()]
        else:
            print "WARNING: could not find camera_order.txt"
            return []

    @staticmethod
    def publish_calibration_points(dirname, topic_base=''):
        xe,ce,re = MultiCalSelfCam.read_calibration_result(dirname)
        names = MultiCalSelfCam.read_calibration_names(dirname)

        create_point_cloud_message_publisher(
            MultiCalSelfCam.reshape_calibrated_points(xe),
            topic_base+'/points',
            publish_now=True,
            latch=True)

        create_camera_pose_message_publisher(
            ce,re,names,
            topic_base+'/cameras',
            publish_now=True,
            latch=True)

    @staticmethod
    def save_to_pcd(dirname, fname):
        xe,ce,re = MultiCalSelfCam.read_calibration_result(dirname)
        points = MultiCalSelfCam.reshape_calibrated_points(xe)
        create_pcd_file_from_points(fname,points)

