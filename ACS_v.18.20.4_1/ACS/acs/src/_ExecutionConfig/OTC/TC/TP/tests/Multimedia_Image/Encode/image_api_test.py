# coding: utf-8
import os
import time
from testlib.camera.mum_camera_impl import CameraImpl
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.multimedia.multimedia_camera_helper import MultiMediaCameraHelper

class ImageAPITest(UIATestBase):
    """
    @summary: Test Video PlayBack
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(ImageAPITest, self).setUp()
        self.d = g_common_obj.get_device()
        self._test_name = __name__
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]
        print "[Setup]: %s" % self._test_name
        g_common_obj.stop_app_am("com.google.android.GoogleCamera")
        g_common_obj.stop_app_am("com.intel.otc.instrument.otcphotos")
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(ImageAPITest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()
        time.sleep(3)
        g_common_obj.stop_app_am("com.google.android.GoogleCamera")
        g_common_obj.stop_app_am("com.intel.otc.instrument.otcphotos")

    def launchPhotoAPP(self):
        g_common_obj.launch_app_am("com.intel.otc.instrument.otcphotos", \
                                   "com.intel.otc.instrument.otcphotos.MainActivity")
        time.sleep(3)
        assert self.d(textContains="/").exists, "launch photo app failed!"

    def enterPhotoPath(self, path):
        push_str = path.strip("\"").split("/")
        if push_str[0] == "" and push_str[1] == "sdcard":
            push_str = push_str[2:]
        print push_str
        for t_str in push_str:
            if t_str != "":
                self.d(text=t_str).click()
                time.sleep(2)

    def checkPictureExist(self, file_name):
        assert self.d(text=file_name).exists, file_name + " not find!"

    def appPrepare(self, case_name, model=1):
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_image.conf')
        self.video = CameraImpl(\
            self.config.read(cfg_file, case_name))

        self.multimedia_setting = MultiMediaSetting(cfg_file)
        self.multimedia_setting.install_apk("photo_apk")
        self.video.set_orientation_n()
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def imageViewCaptureJPEG(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_camera_helper = MultiMediaCameraHelper()
        self.multimedia_camera_helper.camera.startCameraApp()
        self.multimedia_camera_helper.camera.selectMode("Camera")
        # bxt-p do not support set Front camera
        if g_common_obj.adb_cmd_capture_msg("getprop ro.hardware") not in ('bxtp_abl', 'gordon_peak'):
            self.multimedia_camera_helper.camera.switchRearOrFront()
        self.multimedia_camera_helper.camera.capturePhoto()
        self.multimedia_camera_helper.camera.reviewPhotoAndVideo()
        print "case " + str(case_name) + " is pass"

    def imageViewCheck(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.multimedia_setting.push_file(self.video.cfg.get("push_picture"), self.video.cfg.get("datapath"))
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        self.checkPictureExist(os.path.split(path)[1])
        assert self.d(text=self.video.cfg.get("picture_size")).exists, "Orientation is wrong!"
        print "case " + str(case_name) + " is pass"

    def testImage_Capture_JPEG(self):
        """
        This test used to test Image
        The test case spec is following:
        1. Launch Camera to take pictures.
        """
        self.imageViewCaptureJPEG("test_API_image_003")

    def testImage_Capture_Portrait_Orientation(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheck("test_API_image_038")