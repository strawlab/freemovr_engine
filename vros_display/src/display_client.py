import rospy
import time
import vros_display.srv
import json
import warnings

class DisplayServerProxy(object):
    def __init__(self):
        self._server_node_name=rospy.resolve_name('display_server')
        if self._server_node_name=='/display_server':
            rospy.logwarn('seeking display server with default name "/display_server". '\
                              'If display server has been started with a non default name, this will fail. '\
                              '(Hint remap name with display_server:=new_name )')
        self.get_display_server_mode_proxy = rospy.ServiceProxy(self.get_fullname('get_display_server_mode'),
                                                                vros_display.srv.GetDisplayServerMode)
        self.set_display_server_mode_proxy = rospy.ServiceProxy(self.get_fullname('set_display_server_mode'),
                                                                vros_display.srv.SetDisplayServerMode)

    def get_fullname(self,name):
        return self._server_node_name+'/'+name

    def _spin_wait(self,mode):
        done = False
        first_mode = None
        while not done:
            response = self.get_display_server_mode_proxy()
            if response.mode == mode:
                done = True
            elif mode=='rotate_forest' and response.mode == 'scene3d_metamode':
                # backwards compatibility
                done = True
            time.sleep(0.02) # wait 20 msec

    def enter_standby_mode(self):
        response = self.get_display_server_mode_proxy()

        # return to standby mode in server if needed
        if response.mode != 'standby':
            return_to_standby_proxy = rospy.ServiceProxy(self.get_fullname('return_to_standby'),
                                                         vros_display.srv.ReturnToStandby)

            return_to_standby_proxy()
            self._spin_wait('StimulusStandby') # wait until in standby mode

    def set_mode(self,mode):
        # put server in mode
        if mode == 'display2d':
            warnings.warn("translating stimulus name 'display2d'->'Stimulus2DBlit'",DeprecationWarning)
            mode = 'Stimulus2DBlit'

        self.set_display_server_mode_proxy(mode)
        if mode=='quit':
            return

        # Wait until in desired mode - important so published messages
        # get to the receiver.
        self._spin_wait(mode)

    def get_display_info(self):
        get_display_info_proxy = rospy.ServiceProxy(self.get_fullname('get_display_info'),
                                                    vros_display.srv.GetDisplayInfo)
        result = get_display_info_proxy()
        result_dict = json.loads(result.info_json)
        return result_dict
