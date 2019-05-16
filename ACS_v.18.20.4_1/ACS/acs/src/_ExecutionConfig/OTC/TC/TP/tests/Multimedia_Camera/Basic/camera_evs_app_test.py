# coding: UTF-8
'''
Created on Dec 4, 2017

@author: Li Zixi
'''
import sys
import time

from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_switch_camera_helper import MultiMediaSwitchCameraHelper
from testlib.camera.cameratestbase import CameraTestBase
from testlib.util.config import TestConfig

class CameraTest(CameraTestBase):
    """
    @summary: This test used to test camera function
    """
    config = TestConfig()

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(CameraTest, self).setUp()
        self._test_name = __name__
        self.logger.debug("[Setup]: %s" % self._test_name)

        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        self.rvc_camera = MultiMediaSwitchCameraHelper().camera

        self.rvc_camera.backHome()

        self.rvc_camera.init_camera_c_with_canbox()

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(CameraTest, self).tearDown()
        self.logger.debug("[Teardown]: %s" % self._test_name)

        self.rvc_camera.change_camera_c_status("NONE")
        g_common_obj.stop_exp_handle()

    def evs_app_check_0_event_main_test(self, signal_list):
        self.case_name = sys._getframe().f_back.f_code.co_name

        for t_signal in signal_list:
            self.rvc_camera.change_camera_0_status(t_signal)
            time.sleep(1)

    def evs_app_check_c_event_main_test(self, signal_list):
        self.case_name = sys._getframe().f_back.f_code.co_name
        self.rvc_camera.change_camera_c_status("LEFT", 0)
        self.rvc_camera.change_camera_c_status("NONE", 0)

        for t_signal in signal_list:
            self.rvc_camera.change_camera_c_status(t_signal)
            time.sleep(1)

    def Test_Camera_EVS_APP_Gear_AllInactive_Runtime(self):
        self.evs_app_check_0_event_main_test(["GEAR_NEUTRAL", "GEAR_DRIVE", "GEAR_LOW"])

    def Test_Camera_EVS_APP_Gear_Park_Runtime(self):
        self.evs_app_check_0_event_main_test(["GEAR_PARK"])

    def Test_Camera_EVS_APP_Gear_Reverse_Runtime(self):
        self.evs_app_check_0_event_main_test(["GEAR_REVERSE"])

    def Test_Camera_EVS_APP_Drive_LeftTurnSig(self):
        self.evs_app_check_c_event_main_test(["LEFT"])

    def Test_Camera_EVS_APP_Drive_NoneSig_EmergencySig(self):
        self.evs_app_check_c_event_main_test(["NONE", "EMERGENCY"])

    def Test_Camera_EVS_APP_Drive_RightTurnSig(self):
        self.evs_app_check_c_event_main_test(["RIGHT"])

    def Test_Camera_EVS_APP_TurnSig_interative(self):
        self.evs_app_check_c_event_main_test(["RIGHT", "LEFT"] * 20)
