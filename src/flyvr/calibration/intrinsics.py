
import roslib
import roslib.message
roslib.load_manifest('camera_calibration')
from camera_calibration.calibrator import MonoCalibrator, ChessboardInfo
import yaml

def helper_make_corners_boards(cornerss, rowss, colss, dims):
    """helper function to create the required input

    """
    corners_boards = []
    for corners, rows, cols, dim in zip(cornerss, rowss, colss, dims):
        board = ChessboardInfo(n_cols=cols, n_rows=rows, dim=dim)
        corners_boards.append((corners, board))
    return corners_boards


def intrinsics_from_checkerboards(corners_boards, width_px, height_px):
    """Return calibration info from checkerboard input

    """
    boards = [b for _, b in corners_boards]
    # instanciate the monocalibrator class with type [ChessboardInfo]
    calibrator = MonoCalibrator(boards)
    # the size has to be set manually 
    # (see: image_pipeline/camera_calibration/src/camera_calibration/calibrator.py:187)
    calibrator.size = (width_px, height_px)

    # calibrate with type: [(corners, ChessboardInfo)]
    calibrator.cal_fromcorners(corners_boards)

    # return format needs annoying transform:
    out_yaml_str = roslib.message.strify_message(calibrator.as_message())
    return yaml.safe_load(out_yaml_str)
