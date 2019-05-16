# coding: UTF-8
'''
Created on Nov 29, 2017

@author: Li Zixi
'''
import sys
import re

from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.CameraCommon import CameraCommon
from testlib.util.config import TestConfig
from testlib.graphics.common import adb32

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
        self.camera_common = CameraCommon()
        self.evs_vts_test_config = self.config.read(self.camera_common.DEFAULT_CONFIG_FILE, "evs_vts_test_config")
        self.multimedia_setting = MultiMediaSetting(self.camera_common.DEFAULT_CONFIG_FILE)
        adb32.adb_disable_verity()

        self.prepare_env()

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(CameraTest, self).tearDown()
        self.logger.debug("[Teardown]: %s" % self._test_name)
        g_common_obj.stop_exp_handle()

    def prepare_env(self):
        self.bit_32_file_src_path = self.evs_vts_test_config.get("32bit_file_src_path")
        self.bit_32_file_dst_path = self.evs_vts_test_config.get("32bit_file_dst_path")
        self.bit_64_file_src_path = self.evs_vts_test_config.get("64bit_file_src_path")
        self.bit_64_file_dst_path = self.evs_vts_test_config.get("64bit_file_dst_path")
        t_result = g_common_obj.adb_cmd_capture_msg("ls %s" % self.bit_32_file_dst_path)
        print t_result
        if self.bit_32_file_dst_path not in t_result:
            self.bit_32_file_dst_path = self.multimedia_setting.push_file_new(self.bit_32_file_src_path, self.bit_32_file_dst_path)
            g_common_obj.adb_cmd_capture_msg("chmod 777 %s" % self.bit_32_file_dst_path)
        t_result = g_common_obj.adb_cmd_capture_msg("ls %s" % self.bit_64_file_dst_path)
        if self.bit_64_file_dst_path not in t_result:
            self.bit_64_file_dst_path = self.multimedia_setting.push_file_new(self.bit_64_file_src_path, self.bit_64_file_dst_path)
            g_common_obj.adb_cmd_capture_msg("chmod 777 %s" % self.bit_64_file_dst_path)

    def EVS_VTS_main_test(self):
        self.case_name = sys._getframe().f_back.f_code.co_name
        case_name_split_list = self.case_name.split("_")
        check_case_name, check_bit = case_name_split_list[-2], case_name_split_list[-1]
        exec_file = self.bit_32_file_dst_path if "32" in check_bit else self.bit_64_file_dst_path
        cmd = "shell %s" % exec_file
        t_result = self.multimedia_setting.execute_adb_command(cmd)
        self.logger.debug("t_result=%s" % t_result)
        get_test_case_result_parttern = re.compile(r"\[(.*)\].*%s" % check_case_name)
        test_case_result_list = get_test_case_result_parttern.findall(t_result)
        for test_case_result in test_case_result_list:
            if "OK" in test_case_result:
                self.logger.debug("%s passed! test_case_result_list=%s" % (self.case_name, test_case_result_list))
                return
        assert 0, "%s failed! test_case_result_list=%s" % (self.case_name, test_case_result_list)

    def Test_Camera_EVS_VTS_CameraOpenAggressive_32bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_CameraOpenAggressive_64bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_CameraOpenClean_32bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_CameraOpenClean_64bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_CameraStreamBuffering_32bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_CameraStreamBuffering_64bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_CameraStreamPerformance_32bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_CameraStreamPerformance_64bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_CameraToDisplayRoundTrip_32bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_CameraToDisplayRoundTrip_64bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_DisplayOpen_32bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_DisplayOpen_64bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_DisplayStates_32bit(self):
        self.EVS_VTS_main_test()

    def Test_Camera_EVS_VTS_DisplayStates_64bit(self):
        self.EVS_VTS_main_test()