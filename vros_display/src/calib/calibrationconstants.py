CHAN_R = 2
CHAN_G = 1
CHAN_B = 0

CALIB_MAPPING_TOPIC = '/calibration/mapping'

CALIB_MODE_SLEEP = "sleep"
CALIB_MODE_MANUAL_TRACKING = "manual_tracking"
CALIB_MODE_MANUAL_PROJECTOR = "manual_projector"
CALIB_MODE_MANUAL_CLICKED = "manual_clicked"
CALIB_MODE_DISPLAY_SERVER = "display_server"
CALIB_MODE_DISPLAY_SERVER_STOP = "display_server_stop"
CALIB_MODE_DISPLAY_SERVER_VDISP = "display_server_vdisp"
CALIB_MODE_DISPLAY_SERVER_HOME = "display_server+home"
CALIB_MODE_DISPLAY_SERVER_LASER = "display_server+laser"
CALIB_MODE_DISPLAY_SERVER_PROJECTOR = "display_server+projector"
CALIB_MODE_RESTORE = "restore"
CALIB_MODE_SET_BACKGROUND = "set_background"
CALIB_MODE_CLEAR_BACKGROUND = "clear_background"
CALIB_MODE_FINISHED = "finish"

CALIB_MODE_SRV_COMMAND = {
    CALIB_MODE_FINISHED:("","","",""),
    CALIB_MODE_MANUAL_TRACKING:("","","",""),
    CALIB_MODE_MANUAL_PROJECTOR:("display_server/vdisp","col","row",""),
    CALIB_MODE_MANUAL_CLICKED:("","","",""),
    CALIB_MODE_DISPLAY_SERVER:("display_server/vdisp","point space","",""),
    CALIB_MODE_DISPLAY_SERVER_VDISP:("display_server/vdisp","col","row","")
}
