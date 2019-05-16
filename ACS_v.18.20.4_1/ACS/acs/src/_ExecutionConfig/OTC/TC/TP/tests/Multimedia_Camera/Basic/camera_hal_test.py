# coding: UTF-8
'''
Created on Nov 30, 2017

@author: Li Zixi
'''
import sys

from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.AOSPCamera import AOSPCamera
from testlib.graphics.common import adb32
from testlib.util.config import TestConfig
from testlib.multimedia.multimedia_scale_test_helper import MultiMediaScaleTestHelper

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
        self.host_path = self.camera_common.getTmpDir()
        self.camera_common.removeDeivceFile()
        self.camera_common.removeFile(self.host_path + "/*")
        self.multimedia_setting = MultiMediaSetting(self.camera_common.DEFAULT_CONFIG_FILE)
        self.multimedia_scale_test_helper = MultiMediaScaleTestHelper(self.host_path)
        self.aosp_camera = AOSPCamera()
        self.aosp_camera.cleanMediaFiles()

        self.multimedia_scale_test_helper.remount_device()
        self.multimedia_scale_test_helper.backup_file("o_image_media_xml_file_dst_path")
        self.multimedia_scale_test_helper.replace_file("o_image_media_xml_file_src_path", "o_image_media_xml_file_dst_path")
        self.multimedia_scale_test_helper.backup_file("o_image_camera3_xml_file_dst_path")[0]
        self.multimedia_scale_test_helper.restore_file("o_image_camera3_xml_file_dst_path", "_hdmi")
        adb32._adb_reboot()

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(CameraTest, self).tearDown()
        self.logger.debug("[Teardown]: %s" % self._test_name)
        g_common_obj.stop_exp_handle()
        self.camera_common.removeDeivceFile()
        self.camera_common.removeFile(self.host_path + "/*")
        self.aosp_camera.cleanMediaFiles()

        self.multimedia_scale_test_helper.remount_device()
        self.multimedia_scale_test_helper.restore_file("o_image_media_xml_file_dst_path")
        self.multimedia_scale_test_helper.restore_file("o_image_camera3_xml_file_dst_path")
        adb32._adb_reboot()

    def hal_main_test(self):
        self.case_name = sys._getframe().f_back.f_code.co_name
        case_name_split_list = self.case_name.split("_")
        video_resolution, picture_count = case_name_split_list[-2], case_name_split_list[-1]
        video_resolution = video_resolution[:-1]
        if "Iterative" in picture_count:
            t_loop = 20
            picture_count = 3
        else:
            t_loop = 1
            picture_count = int(picture_count[:-3])
        check_file_count = 0
        self.aosp_camera.startCameraApp()
        self.aosp_camera.selectMode("Video")
        self.aosp_camera.setVideoResolution(video_resolution, "Back")
        for i in range(1, t_loop + 1):
            self.logger.debug("----loop %d----" % i)
            self.aosp_camera.snapShotDuringVideo(1, 5, picture_count)
            check_file_count += (picture_count + 1)
        self.camera_common.checkFileCorrupt(check_file_count)
        self.aosp_camera.reviewPhotoAndVideo(check_file_count)
        self.aosp_camera.stopCameraApp()

    def test_Camera_Snapshot_1080p_10Pic(self):
        self.hal_main_test()

    def test_Camera_Snapshot_1080p_1Pic(self):
        self.hal_main_test()

    def test_Camera_Snapshot_1080p_Iterative(self):
        self.hal_main_test()

    def test_Camera_Snapshot_480p_10Pic(self):
        self.hal_main_test()

    def test_Camera_Snapshot_480p_1Pic(self):
        self.hal_main_test()

    def test_Camera_Snapshot_480p_Iterative(self):
        self.hal_main_test()

    def test_Camera_Snapshot_720p_10Pic(self):
        self.hal_main_test()

    def test_Camera_Snapshot_720P_1Pic(self):
        self.hal_main_test()

    def test_Camera_Snapshot_720p_Iterative(self):
        self.hal_main_test()
