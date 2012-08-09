import tempfile

import roslib;
roslib.load_manifest('vros_display')
roslib.load_manifest('motmot_ros_utils')
import rospy

from calib.io import MultiCalSelfCam, AllPointPickle
from calib.visualization import create_pcd_file_from_points
from rosutils.io import decode_url

import flydra
import flydra.reconstruct
import flydra.align

rospy.init_node('calibone', anonymous=True)

DS1_PKL         = decode_url('package://flycave/calibration/triplets/ds1')
DS1_CALIB       = decode_url('package://flycave/calibration/triplets/ds1/result')
LASER_PKL       = decode_url('package://flycave/calibration/laser')
FLYDRA_CALIB    = decode_url('package://flycave/calibration/flydra')

#test mcsc wrapper
if 0:
    a = AllPointPickle()
    a.initilize_from_directory(DS1_PKL)

    mcsc =  MultiCalSelfCam(DS1_CALIB)
    mcsc.create_from_cams(
            cam_ids=[],
            cam_resolutions=a.resolutions.copy(),
            cam_points=a.results.copy(),
            cam_calibrations={},
            num_cameras_fill=0)
    dest = mcsc.execute(blocking=True, dest=tempfile.mkdtemp(prefix='mcsc'), silent=False)
    MultiCalSelfCam.publish_calibration_points(dest)
    MultiCalSelfCam.save_to_pcd(dest, "/tmp/ds1.pcd")

#test roundtrip 2d -> 3d -> 2d
if 0:
    r = flydra.reconstruct.Reconstructor(cal_source=DS1_CALIB)

    cams = r.get_cam_ids()
    print "cameras",cams

    pts = a.get_points_in_cameras(r.get_cam_ids())

    pt = pts[0]
    print "2d points",pt

    X = r.find3d(pt,return_line_coords=False)
    print "3d point",X

    pt = r.find2d(cams[0],X)
    print "2d point in %s" % cams[0],pt

    print "2d points",[r.find2d(c,X) for c in cams]

#for all laser points, find their 3D coords, check it makes a cylinder
if 0:
    #use the flydra calibration to get the 3d coords
    r = flydra.reconstruct.Reconstructor(cal_source=FLYDRA_CALIB)

    laser = AllPointPickle()
    laser.initilize_from_directory(LASER_PKL)

    print "number of laser points",laser.num_points

    #all points visible in 2 or more cameras
    pts = laser.get_points_in_cameras(r.get_cam_ids())

    create_pcd_file_from_points(
            '/tmp/cyl.pcd',
            [r.find3d(pt,return_line_coords=False) for pt in pts])

