# coding: UTF-8
'''
Created on Dec 11, 2017

@author: Li Zixi
'''
import sys

from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.AOSPCamera import AOSPCamera
from testlib.graphics.common import AdbExtension
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
        self.camera_common = CameraCommon()
        self.host_path = self.camera_common.getTmpDir()
        self.camera_common.removeDeivceFile()
        self.camera_common.removeFile(self.host_path + "/*")
        self.multimedia_setting = MultiMediaSetting(self.camera_common.DEFAULT_CONFIG_FILE)
        self.aosp_camera = AOSPCamera()

        self.adb_extension = AdbExtension()

        self.aosp_camera.cleanMediaFiles()

        self.adb_extension._adb_reboot()

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

    def aosp_check_video_record_test(self, video_size):
        self.aosp_camera.startCameraApp()
        self.aosp_camera.selectMode("Video")
        self.aosp_camera.setVideoResolution(video_size, "")
        self.aosp_camera.recordVideo(1, 10)
        self.camera_common.checkFileCorrupt(1)
        self.aosp_camera.reviewPhotoAndVideo(1)
        self.aosp_camera.stopCameraApp()

    def test_Camera_IVI_AOSP_VideoMode_MaximumSize(self):
        self.aosp_check_video_record_test("480")

    def test_Camera_IVI_AOSP_VideoMode_MinimumSize(self):
        self.aosp_check_video_record_test("QVGA")

    def test_Camera_IVI_AOSP_VideoMode_Rear_Preview(self):
        self.aosp_camera.startCameraApp()
        self.aosp_camera.selectMode("Video")
        self.aosp_camera.setVideoResolution("480", "")
        self.aosp_camera.recordVideo(1, 10)
        self.aosp_camera.stopCameraApp()

    def test_Camera_IVI_AOSP_VideoMode_Rear_RecordVideo(self):
        self.aosp_camera.startCameraApp()
        self.aosp_camera.selectMode("Video")
        self.aosp_camera.setVideoResolution("480", "")
        self.aosp_camera.recordVideo(1, 10)
        self.camera_common.checkFileCorrupt(1)
        self.aosp_camera.reviewPhotoAndVideo(1)
        self.aosp_camera.stopCameraApp()

    def test_Camera_IVI_AOSP_VideoMode_Rear_CaptureVideo_AllSize(self):
        self.aosp_camera.startCameraApp()
        self.aosp_camera.selectMode("Video")
        resolution_dict = self.aosp_camera.getAllVideoResolutions("")[0]
        resolution_count = len(resolution_dict)
        for t_key in resolution_dict.keys():
            t_resolution = resolution_dict[t_key]
            self.logger.debug("t_resolution: %s" % t_resolution)
            self.aosp_camera.setVideoResolution(t_resolution, "")
            self.aosp_camera.recordVideo(1, 10)
        self.camera_common.checkFileCorrupt(resolution_count)
        self.aosp_camera.reviewPhotoAndVideo(resolution_count)
        self.aosp_camera.stopCameraApp()

    def test_Camera_IVI_AOSP_Preview_Rear_CaptureMode(self):
        self.aosp_camera.startCameraApp()
        self.aosp_camera.selectMode("Camera")
        self.aosp_camera.capturePhoto(5)
        self.camera_common.checkFileCorrupt(5)
        self.aosp_camera.stopCameraApp()

    def test_Camera_IVI_AOSP_Rear_CapturePhoto(self):
        self.aosp_camera.startCameraApp()
        self.aosp_camera.selectMode("Camera")
        self.aosp_camera.capturePhoto(5)
        self.camera_common.checkFileCorrupt(5)
        self.aosp_camera.reviewPhotoAndVideo(5)
        self.aosp_camera.stopCameraApp()