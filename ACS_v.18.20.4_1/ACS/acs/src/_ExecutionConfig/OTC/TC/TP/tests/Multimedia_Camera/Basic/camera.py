# coding: utf-8
import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.checkImage import CheckImage
from testlib.camera.checkVideo import CheckVideo
from testlib.camera.CameraCommon import CameraCommon
from testlib.audio.audio_impl import AudioImpl
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.camera.cameratestbase import CameraTestBase
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.camera.checkIQ import CheckZoom
from testlib.camera.DeviceControl import DeviceControl

class CameraTest(CameraTestBase):
    """
    @summary: This test used to test camera function
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(CameraTest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(CameraTest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()
        time.sleep(3)

    def appPrepare(self):
        self.camera = CameraCommon().switchPlatform()
#         self.camera = RefCam2Camera()
        self.multimedia_setting = MultiMediaSetting(CameraCommon.DEFAULT_CONFIG_FILE)
        self.checkImage = CheckImage()
        self.video = CheckVideo()
        self.host_path = CameraCommon().getTmpDir()
        self.makefileTime = CameraCommon().makefileTime
        self.camera_dir = CameraCommon().camera_dir
        CameraCommon().removeDeivceFile()
        CameraCommon().removeFile(self.host_path + "/*")
        self.camera.cleanMediaFiles()
        CameraCommon().setOrientationToVertical()
        self.logger.debug("app prepare successfully")

    def checkFileCorrupt(self, mediaFileCount=1):
        return CameraCommon().checkFileCorrupt(mediaFileCount)

    def audioAppPrepare(self):
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.testplan.camera.conf')
        self.audio = AudioImpl(\
            self.config.read(cfg_file, "multimedia_camera"))
        g_common_obj.adb_cmd_capture_msg(self.audio.cfg.get("remove_audio"))
        self.multimedia_setting.push_file(self.audio.cfg.get("push_audio"), self.audio.cfg.get("datapath"))
        g_common_obj.adb_cmd_capture_msg(self.audio.cfg.get("refresh_sd"))
        self.audio_name = self.audio.cfg.get("audio_name")
        self.audio.set_orientation_n()
        self.audio.cleanUpData()

    def checkVideoInfo(self, image, image_name):
        error_msg = ""
        file_name = image.get("File Name")
        self.logger.debug("File Name: %s" % file_name)
        if file_name != image_name:
            error_msg += "check file name fail,actual=%s,expected=%s; " % (file_name, image_name)

        image_width = image.get("Image Width")
        image_height = image.get("Image Height")
        size = image.get("Image Size").split('x')
        self.logger.debug("Image Width: %s" % image_width)
        self.logger.debug("Image Height: %s" % image_height)
        self.logger.debug("Image Size: %s" % size)
        if image_width == None or image_width != size[0]:
            error_msg += "check width fail,actual=%s,expected=%s; " % (image_width, size[0])
        if image_height == None or image_height != size[1]:
            error_msg += "check height fail,actual=%s,expected=%s; " % (image_height, size[1])

        frame_rate = image.get("Video Frame Rate")
        if frame_rate <= 0:
            error_msg += "check video frame rate fail,actual=%s,expected > 0; " % frame_rate

        if error_msg != "":
            os.system("cp " + self.host_path + "/" + file_name + " " + g_common_obj.get_user_log_dir())
            self.assertTrue(False, error_msg)

    def storeLocationTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Back")
            self.camera.recordVideo(1, 5)
            CameraCommon().checkCameraCrash()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            self.camera.capturePhoto()
            time.sleep(2)
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def disableAndEnableCameraTest(self):
        try:
            self.appPrepare()
            if not self.camera.disableOrEnableCamera("Disable"):
                assert False, "camera not disable"
            if not self.camera.disableOrEnableCamera("Enable"):
                assert False, "camera not enable"
            self.camera.startCameraApp()
            CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.camera.disableOrEnableCamera("Enable")
            self.assertTrue(False, e)

    def launchCameraFromLockScreenTest(self):
        try:
            self.appPrepare()
            CameraCommon().lockScreen()
            time.sleep(2)
            CameraCommon().pressPower()
            CameraCommon().checkScreenLock()
            time.sleep(2)
            CameraCommon().pressPower()
            time.sleep(2)
            self.camera.openCameraFromLockScreenIcon(self.camera)
            CameraCommon().clickScreenCenter()
            self.camera.capturePhoto()
            self.checkFileCorrupt()
            CameraCommon().lockScreen(False)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            CameraCommon().lockScreen(False)
            self.assertTrue(False, e)

    def cameraPreviewCaptureModeTest(self, mode, lens, sleep_time):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            self.camera.switchRearOrFront(lens)
            time.sleep(int(sleep_time))
            CameraCommon().unlockScreen()
            CameraCommon().checkCameraCrash()
            self.camera.capturePhoto(3)
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            CameraCommon().lockScreen(False)
            self.assertTrue(False, e)

    def cameraClearDataForceCloseTest(self):
        try:
            self.appPrepare()
            self.camera.cleanMediaFiles()
            time.sleep(2)
            self.camera.startCameraApp()
            CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def videoMultiSettingsTest(self):
        try:
            self.appPrepare()
            CameraCommon().launchGpsApp()
            network_longitude, network_latitude = CameraCommon().getGPSLocation()
            self.camera.openLocation()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Back")
            self.camera.setVideoResolution("480", "Back")
            self.camera.setGrid("on")
            self.camera.openGEO()
            self.camera.snapShotDuringVideo()
            info = self.checkFileCorrupt(1)[0]
            if info.get("GPS Latitude"):
                latitude = info.get("GPS Latitude").split(' ')
                longitude = info.get("GPS Longitude").split(' ')
            else:
                self.assertTrue(False, "Can't get gps information from picture or video")
            self.logger.debug(str(latitude[0]) + ", " + str(longitude[0]) + ", " + str(network_latitude) + ", " + str(network_longitude))
            assert (network_longitude == longitude[0]) and (network_latitude == latitude[0]), "gps location error"
            time.sleep(2)
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Front")
            self.camera.setGrid("off")
            self.camera.recordVideo(1, 5)
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo()
        except Exception as e:
            self.assertTrue(False, e)

    def manualExposureInteractRearFrontTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            self.camera.openManualExposure()
            self.camera.setExposure("+2")
            CameraCommon().clickScreenCenter()
            self.camera.capturePhoto()
            pic_b1 = self.checkFileCorrupt()[1]
            CameraCommon().removeDeivceFile()
            self.camera.switchRearOrFront("Front")
            CameraCommon().clickScreenCenter()
            self.camera.setExposure("-2")
            self.camera.capturePhoto()
            pic_f1 = self.checkFileCorrupt()[1]
            CameraCommon().removeDeivceFile()
            self.camera.switchRearOrFront("Back")
            CameraCommon().clickScreenCenter()
            self.camera.capturePhoto()
            pic_b2 = self.checkFileCorrupt()[1]
            CameraCommon().removeDeivceFile()
            self.camera.switchRearOrFront("Front")
            CameraCommon().clickScreenCenter()
            self.camera.capturePhoto()
            pic_f2 = self.checkFileCorrupt()[1]
            pic_b1 = self.host_path + "/" + pic_b1
            pic_f1 = self.host_path + "/" + pic_f1
            pic_b2 = self.host_path + "/" + pic_b2
            pic_f2 = self.host_path + "/" + pic_f2
            pic_b1_b = self.checkImage.brightness(pic_b1)
            pic_b2_b = self.checkImage.brightness(pic_b2)
            pic_f1_b = self.checkImage.brightness(pic_f1)
            pic_f2_b = self.checkImage.brightness(pic_f2)
            self.logger.debug("picture name:" + str(pic_b1) + ", " + str(pic_f1) + ", " + str(pic_b2) + ", " + str(pic_f2))
            self.error_msg = ""
            if not self.checkImage.compare_images_brightness(pic_f1, pic_b1):
                self.error_msg = self.error_msg + "front -2 compare back +2 exposure check fail," + \
                    "-2(" + pic_f1 + ") brightness=" + str(pic_f1_b) + ", +2(" + pic_b1 + ") brightness=" + str(pic_b1_b) + "; "
                os.system("cp " + pic_f1 + " " + g_common_obj.get_user_log_dir())
                os.system("cp " + pic_b1 + " " + g_common_obj.get_user_log_dir())
            if not self.checkImage.compare_images_brightness(pic_f2, pic_b2):
                self.error_msg = self.error_msg + "back camera +2 compare front -2 exposure check fail," + \
                    "+2(" + pic_b2 + ") brightness=" + str(pic_b2_b) + ", -2(" + pic_f2 + ") brightness=" + str(pic_f2_b) + "; "
                os.system("cp " + pic_f2 + " " + g_common_obj.get_user_log_dir())
                os.system("cp " + pic_b2 + " " + g_common_obj.get_user_log_dir())
            m_brightness_big = pic_b1_b
            m_brightness_small = pic_b2_b
            if m_brightness_big - m_brightness_small < 0:
                m_brightness_big = pic_b2_b
                m_brightness_small = pic_b1_b
            self.logger.debug("brightness percent = " + str((m_brightness_big - m_brightness_small) / m_brightness_big))
            if (m_brightness_big - m_brightness_small) / m_brightness_big > 0.05 :
                self.error_msg = self.error_msg + "first back camera +2 compare second back camera +2 exposure check fail," + \
                    "+2(" + pic_b1 + ") brightness=" + str(pic_b1_b) + ", +2(" + pic_b2 + ") brightness=" + str(pic_b2_b) + "; "
                os.system("cp " + pic_b1 + " " + g_common_obj.get_user_log_dir())
                os.system("cp " + pic_b2 + " " + g_common_obj.get_user_log_dir())
            if self.error_msg == "":
                    self.logger.debug("exposure check successful")
            else:
                self.assertTrue(False, self.error_msg)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def compareExposureEffectIncreaseOrDecreaseTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            self.camera.openManualExposure()
            for i in range(2):
                self.camera.setExposure("-2")
                self.camera.capturePhoto()
                pic_n2 = self.checkFileCorrupt()[1]
                CameraCommon().removeDeivceFile()
                self.camera.setExposure("-1")
                self.camera.capturePhoto()
                pic_n1 = self.checkFileCorrupt()[1]
                CameraCommon().removeDeivceFile()
                self.camera.setExposure("0")
                self.camera.capturePhoto()
                pic_0 = self.checkFileCorrupt()[1]
                CameraCommon().removeDeivceFile()
                self.camera.setExposure("+1")
                self.camera.capturePhoto()
                pic_p1 = self.checkFileCorrupt()[1]
                CameraCommon().removeDeivceFile()
                self.camera.setExposure("+2")
                self.camera.capturePhoto()
                pic_p2 = self.checkFileCorrupt()[1]
                CameraCommon().removeDeivceFile()
                pic_n2 = self.host_path + "/" + pic_n2
                pic_n1 = self.host_path + "/" + pic_n1
                pic_0 = self.host_path + "/" + pic_0
                pic_p1 = self.host_path + "/" + pic_p1
                pic_p2 = self.host_path + "/" + pic_p2
                self.logger.debug(str(pic_n2) + ", " + str(pic_n1) + ", " + str(pic_0) + ", " + str(pic_p1) + ", " + str(pic_p2))
                pic_n2_b = self.checkImage.brightness(pic_n2)
                pic_n1_b = self.checkImage.brightness(pic_n1)
                pic_0_b = self.checkImage.brightness(pic_0)
                pic_p1_b = self.checkImage.brightness(pic_p1)
                pic_p2_b = self.checkImage.brightness(pic_p2)
                self.error_msg = ""
                if not self.checkImage.compare_images_brightness(pic_n2, pic_n1):
                    self.error_msg = self.error_msg + "-2 compare -1 exposure check fail," + \
                    "-2(" + pic_n2 + ") brightness=" + str(pic_n2_b) + ", -1(" + pic_n1 + ") brightness=" + str(pic_n1_b) + "; "
                    os.system("cp " + pic_n2 + " " + g_common_obj.get_user_log_dir())
                    os.system("cp " + pic_n1 + " " + g_common_obj.get_user_log_dir())
                if not self.checkImage.compare_images_brightness(pic_n1, pic_0):
                    self.error_msg = self.error_msg + "-1 compare 0 exposure check fail," + \
                    "-1(" + pic_n1 + ") brightness=" + str(pic_n1_b) + ", 0(" + pic_0 + ") brightness=" + str(pic_0_b) + "; "
                    os.system("cp " + pic_n1 + " " + g_common_obj.get_user_log_dir())
                    os.system("cp " + pic_0 + " " + g_common_obj.get_user_log_dir())
                if not self.checkImage.compare_images_brightness(pic_0, pic_p1):
                    self.error_msg = self.error_msg + "0 compare +1 exposure check fail," + \
                    "0(" + pic_0 + ") brightness=" + str(pic_0_b) + ", +1(" + pic_p1 + ") brightness=" + str(pic_p1_b) + "; "
                    os.system("cp " + pic_0 + " " + g_common_obj.get_user_log_dir())
                    os.system("cp " + pic_p1 + " " + g_common_obj.get_user_log_dir())
                if not self.checkImage.compare_images_brightness(pic_p1, pic_p2):
                    self.error_msg = self.error_msg + "+1 compare +2 exposure check fail." + \
                    "+1(" + pic_p1 + ") brightness=" + str(pic_p1_b) + ", +2(" + pic_p2 + ") brightness=" + str(pic_p2_b) + "."
                    os.system("cp " + pic_p1 + " " + g_common_obj.get_user_log_dir())
                    os.system("cp " + pic_p2 + " " + g_common_obj.get_user_log_dir())
                if self.error_msg == "":
                    self.logger.debug("exposure check successful")
                else:
                    self.assertTrue(False, self.error_msg)
                self.camera.switchRearOrFront("Front")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraGPSLocationRecordModeTest(self):
        try:
            self.appPrepare()
            CameraCommon().launchGpsApp()
            network_longitude, network_latitude = CameraCommon().getGPSLocation()
            self.camera.openLocation()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Back")
            self.camera.openGEO()
            self.camera.recordVideo(1, 5)
            info, picName = self.checkFileCorrupt()
            if info.get("GPS Latitude"):
                latitude = info.get("GPS Latitude").split(' ')
                longitude = info.get("GPS Longitude").split(' ')
            else:
                os.system("cp " + self.host_path + "/" + picName + " " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "Can't get gps information from picture or video")
            self.logger.debug(str(latitude[0]) + ", " + str(longitude[0]) + ", " + str(network_latitude) + ", " + str(network_longitude))
            assert (network_longitude == longitude[0]) and (network_latitude == latitude[0]), \
                    "gps location error"
            self.camera.selectMode("Camera")
            self.camera.capturePhoto()
            info, picName = self.checkFileCorrupt()
            if info.get("GPS Latitude"):
                latitude = info.get("GPS Latitude").split(' ')
                longitude = info.get("GPS Longitude").split(' ')
            else:
                os.system("cp " + self.host_path + "/" + picName + " " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "Can't get gps information from picture or video")
            self.logger.debug(str(latitude[0]) + ", " + str(longitude[0]) + ", " + str(network_latitude) + ", " + str(network_longitude))
            assert (network_longitude == longitude[0]) and (network_latitude == latitude[0]), \
                    "gps location error"
            CameraCommon().removeDeivceFile()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Front")
            self.camera.recordVideo(1, 5)
            info, picName = self.checkFileCorrupt()
            if info.get("GPS Latitude"):
                latitude = info.get("GPS Latitude").split(' ')
                longitude = info.get("GPS Longitude").split(' ')
            else:
                os.system("cp " + self.host_path + "/" + picName + " " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "Can't get gps information from picture or video")
            self.logger.debug(str(latitude[0]) + ", " + str(longitude[0]) + ", " + str(network_latitude) + ", " + str(network_longitude))
            assert (network_longitude == longitude[0]) and (network_latitude == latitude[0]), \
                    "gps location error"
            self.camera.selectMode("Camera")
            self.camera.capturePhoto()
            info, picName = self.checkFileCorrupt()
            if info.get("GPS Latitude"):
                latitude = info.get("GPS Latitude").split(' ')
                longitude = info.get("GPS Longitude").split(' ')
            else:
                os.system("cp " + self.host_path + "/" + picName + " " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "Can't get gps information from picture or video")
            self.logger.debug(str(latitude[0]) + ", " + str(longitude[0]) + ", " + str(network_latitude) + ", " + str(network_longitude))
            assert (network_longitude == longitude[0]) and (network_latitude == latitude[0]), \
                    "gps location error"
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraGPSLocationCaptureModeTest(self):
        try:
            self.appPrepare()
            CameraCommon().launchGpsApp()
            network_longitude, network_latitude = CameraCommon().getGPSLocation()
            self.camera.openLocation()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            self.camera.openGEO()
            self.camera.capturePhoto()
            info, picName = self.checkFileCorrupt()
            if info.get("GPS Latitude"):
                latitude = info.get("GPS Latitude").split(' ')
                longitude = info.get("GPS Longitude").split(' ')
            else:
                os.system("cp " + self.host_path + "/" + picName + " " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "Can't get gps information from picture or video")
            self.logger.debug(str(latitude[0]) + ", " + str(longitude[0]) + ", " + str(network_latitude) + ", " + str(network_longitude))
            assert (network_longitude == longitude[0]) and (network_latitude == latitude[0]), \
                    "gps location error"
            CameraCommon().removeDeivceFile()
            self.camera.switchRearOrFront("Front")
            self.camera.capturePhoto()
            info, picName = self.checkFileCorrupt()
            if info.get("GPS Latitude"):
                latitude = info.get("GPS Latitude").split(' ')
                longitude = info.get("GPS Longitude").split(' ')
            else:
                os.system("cp " + self.host_path + "/" + picName + " " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "Can't get gps information from picture or video")
            self.logger.debug(str(latitude[0]) + ", " + str(longitude[0]) + ", " + str(network_latitude) + ", " + str(network_longitude))
            assert (network_longitude == longitude[0]) and (network_latitude == latitude[0]), \
                    "gps location error"
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def closeCameraByReturnToHomeScreen(self):
        """
        This test used to test close camera by return to home screen
        The test case spec is following:
        1. Launch camera
        2. Close camera by return to home screen
        """
        try:
            self.appPrepare()
            for i in range(50):
                self.logger.debug("***** Loop " + str(i + 1) + " *****")
                self.camera.startCameraApp()
                CameraCommon().pressHome()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def destoryCameraInTaskManager(self):
        """
        This test used to test destory camera in task manager
        The test case spec is following:
        1. Destory camera in task manager
        """
        try:
            self.appPrepare()
            CameraCommon().removeAllAppFromRecentTask()
            for i in range(50):
                self.logger.debug("*****Loop " + str(i + 1) + " *****")
                self.camera.startCameraApp()
                CameraCommon().removeRecentApp("Camera")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def launchCameraInTaskManager(self):
        """
        This test used to test launch camera in task manager
        The test case spec is following:
        1. Launch camera in task manager
        """
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            for i in range(50):
                self.logger.debug("***** Loop " + str(i + 1) + " *****")
                CameraCommon().pressBack()
                CameraCommon().enterAppFromRecentTask()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def waitPreviewUI(self):
        while self.camera.GoogleDefaultCamera().camera_mode_options_overlay(False).exists() == False:
            time.sleep(0.5)

    def goThroughAllRadioButtons(self, radio_button_list, desp):
        for i in range(10):
            self.waitPreviewUI()
            self.camera.click_camera_menu_setting_vert()
            self.camera.enter_camera_setting_video_quality()
            if desp == "Image Quality":
                if self.camera.GoogleDefaultCamera().camera_Reso_quality_setting_page_front_image_quality(False).exists():
                    self.camera.GoogleDefaultCamera().camera_Reso_quality_setting_page_front_image_quality().click()
            elif desp == "Panorama Resolution":
                if self.camera.GoogleDefaultCamera().camera_Reso_quality_setting_page_panorama_resolution(False).exists():
                    self.camera.GoogleDefaultCamera().camera_Reso_quality_setting_page_panorama_resolution().click()
            if not self.camera.GoogleDefaultCamera().camera_setting_alert_page_cancel(False).exists():
                assert False, "Enter " + desp + " Setting page failed,not found " + desp + " in camera settings"
            if i == 0:
                if len(radio_button_list) == 0:
                    radio_button_list = self.camera.getRadioButtonsOnCurrentView()
            else:
                last_node = radio_button_list[i - 1]
                assert last_node.info["checked"], desp + ": " + last_node.info["text"] + "is not checked"
            if i < len(radio_button_list):
                current_node = radio_button_list[i]
                current_node.click()
            self.camera.press_back_loop(2)
            if i == len(radio_button_list):
                self.camera.press_back_loop(1)
                break

    def cameraRecodingVideoSizeSwitchTest(self, max_or_min):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Front")
            if max_or_min == "max":
                mResulotion = self.camera.getAllVideoResolutions("Front")[1]
            elif max_or_min == "min":
                mResulotion = self.camera.getAllVideoResolutions("Front")[2]
            else:
                self.assertTrue(False, "This resolution is not found")
            self.camera.setVideoResolution(mResulotion, "Front")
            self.camera.recordVideo(1, 3)
            self.camera.switchRearOrFront("Back")
            if max_or_min == "max":
                mResulotion = self.camera.getAllVideoResolutions("Back")[1]
            elif max_or_min == "min":
                mResulotion = self.camera.getAllVideoResolutions("Back")[2]
            else:
                self.assertTrue(False, "This resolution is not found")
            self.camera.setVideoResolution(mResulotion, "Back")
            self.camera.recordVideo(1, 3)
            for i in range(3):
                self.camera.switchRearOrFront("Front")
                self.camera.switchRearOrFront("Back")
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraCaptureSwitchTest(self, max_or_min):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            if max_or_min == "max":
                mResulotion = self.camera.getAllPhotoResolutions("Back")[1]
            elif max_or_min == "min":
                mResulotion = self.camera.getAllPhotoResolutions("Back")[2]
            else:
                self.assertTrue(False, "This resolution is not found")
            self.camera.setPhotoResolution(mResulotion, "Back")
            self.camera.capturePhoto()
            self.camera.switchRearOrFront("Front")
            if max_or_min == "max":
                mResulotion = self.camera.getAllPhotoResolutions("Front")[1]
            elif max_or_min == "min":
                mResulotion = self.camera.getAllPhotoResolutions("Front")[2]
            else:
                self.assertTrue(False, "This resolution is not found")
            self.camera.setPhotoResolution(mResulotion, "Front")
            self.camera.capturePhoto()
            for i in range(3):
                self.camera.switchRearOrFront("Back")
                self.camera.switchRearOrFront("Front")
            CameraCommon().checkCameraCrash()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def launchCloseCamera(self):
        """
        This test used to test launch camera
        The test case spec is following:
        1. Launch camera
        """
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            for i in range(50):
                self.logger.debug("***** Loop " + str(i + 1) + " *****")
                CameraCommon().pressBack()
                self.camera.startCameraApp()
                self.camera.swipeScreen("left")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def test_Camera_Switch_Between_AvailableModes(self):
        """
        This test used to test switch to camera options
        The test case spec is following:
        1. Launch camera
        2. Switch between 5 modes if the modes are available for 5 times
        """
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.reviewAvailableModes()
            CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def videoRecording(self, mode, lens, resolution, duration):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            self.camera.switchRearOrFront(lens)
            self.camera.setVideoResolution(resolution, lens)
            self.camera.recordVideo(1, duration)
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def homeKeyCapturePhotoWhenRelaunchTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            CameraCommon().pressHome()
            self.camera.startCameraApp()
            time.sleep(3)
            self.camera.capturePhoto()
            CameraCommon().checkCameraCrash()
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def killCameraCapturePhotoWhenRelaunchTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            CameraCommon().removeRecentApp()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.capturePhoto()
            CameraCommon().checkCameraCrash()
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraCaptureModeLockAndUnlockTest(self, loop):
        try:
            self.appPrepare()
            self.multimedia_setting.install_apk("unlock_app")
            CameraCommon().lockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.setTimer()
            for i in range(int(loop)):
                self.logger.debug("=====camera capture mode lock and unlock test loop %d =====" % (i + 1))
                self.camera.switchRearOrFront("Front")
                CameraCommon().clickScreenCenter()
                CameraCommon().pressPower()
                CameraCommon().getScreenshotAndPullToHost("sc1.png", self.host_path)
                time.sleep(2)
                self.logger.debug(str(self.checkImage.brightness(self.host_path + "/sc1.png")))
                if self.checkImage.brightness(self.host_path + "/sc1.png") > 30.0:
                    assert False, "front camera preview screen Lock fail"
#                 CameraCommon().unlockScreen()
                CameraCommon().launchUnlockAppToUnlockScreen()
                self.camera.capturePhoto()
                self.logger.debug("front camera capture photo")
                if not CameraCommon().waitForTheFilesAreGenerated(1):
                    self.camera.capturePhoto()
                CameraCommon().waitForTheFilesAreGenerated()
                self.camera.swipeScreen("left")
                CameraCommon().pressPower()
                CameraCommon().getScreenshotAndPullToHost("sc2.png", self.host_path)
                time.sleep(2)
                self.logger.debug(str(self.checkImage.brightness(self.host_path + "/sc2.png")))
                if self.checkImage.brightness(self.host_path + "/sc2.png") > 30.0:
                    assert False, "front camera preview screen Lock fail"
#                 CameraCommon().unlockScreen()
                CameraCommon().launchUnlockAppToUnlockScreen()
                CameraCommon().checkCameraCrash()
                self.logger.debug("switch to back camera")
                CameraCommon().clickScreenCenter()
                self.camera.switchRearOrFront("Back")
                CameraCommon().pressPower()
                CameraCommon().getScreenshotAndPullToHost("sc1.png", self.host_path)
                time.sleep(2)
                if self.checkImage.brightness(self.host_path + "/sc2.png") > 30.0:
                        assert False, "rear camera preview picture screen Lock fail"
#                 CameraCommon().unlockScreen()
                CameraCommon().launchUnlockAppToUnlockScreen()
                CameraCommon().checkCameraCrash()
                self.camera.capturePhoto()
                self.logger.debug("rear camera capture photo")
                self.camera.swipeScreen("left")
                CameraCommon().pressPower()
                CameraCommon().getScreenshotAndPullToHost("sc2.png", self.host_path)
                time.sleep(2)
                if self.checkImage.brightness(self.host_path + "/sc2.png") > 30.0:
                    assert False, "rear camera preview picture screen Lock fail"
#                 CameraCommon().unlockScreen()
                CameraCommon().launchUnlockAppToUnlockScreen()
                CameraCommon().checkCameraCrash()
                time.sleep(2)
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo()
            CameraCommon().lockScreen(False)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            CameraCommon().lockScreen(False)
            self.assertTrue(False, e)

    def cameraCaptureModeVolumeKeyTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Front")
            CameraCommon().pressVolume("up")
            CameraCommon().pressVolume("up")
            CameraCommon().pressVolume("down")
            CameraCommon().pressVolume("down")
            self.camera.capturePhoto()
            self.camera.swipeScreen("left")
            CameraCommon().pressVolume("up")
            CameraCommon().pressVolume("up")
            CameraCommon().pressVolume("down")
            CameraCommon().pressVolume("down")
            CameraCommon().pressBack()
            self.camera.switchRearOrFront("Back")
            self.camera.capturePhoto()
            self.camera.swipeScreen("left")
            CameraCommon().pressVolume("up")
            CameraCommon().pressVolume("up")
            CameraCommon().pressVolume("down")
            CameraCommon().pressVolume("down")
            CameraCommon().pressBack()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraCapturePictureTimerAlarmTest(self):
        try:
            self.appPrepare()
            self.multimedia_setting.install_apk("alarm_apk")
            CameraCommon().launchAlarmApp()
            CameraCommon().setAlarmTime(90)
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            self.camera.setTimer("10s")
            mTime = time.time()
            while time.time() - mTime < 180:
                self.camera.capturePhoto()
                time.sleep(10)
                if CameraCommon().alarmDismissOrSnooze():
                    self.camera.startCameraApp()
                    break
            CameraCommon().checkCameraCrash()
            self.camera.setTimer()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraFunctionAfterChangeSystemLanguageTest(self):
        self.appPrepare()
        self.camera.startCameraApp()
        CameraCommon().pressHome()
        if CameraCommon().getAndroidVersion() == "N":
            CameraCommon().changeLanguageInSettingForN(self.camera.cfg.get("language_input_origin_n"), \
                    self.camera.cfg.get("language_title_orgin_n"), self.camera.cfg.get("language_n"),  \
                    self.camera.cfg.get("language_country_n"))
        else:
            CameraCommon().changeLanguageInSetting(self.camera.cfg.get("language_input_origin"), \
                    self.camera.cfg.get("language_title_orgin"), self.camera.cfg.get("language"))
        try:
            self.camera.startCameraApp()
            self.camera.swipeScreen("right")
            if not (CameraCommon().isWidgetExists(CameraCommon().CommonWidget().text(self.camera.cfg.get("camera_mode_name"))) \
                or self.camera.isShutterBtnExists()):
                assert False, "language did not change"
            CameraCommon().checkCameraCrash()
            if CameraCommon().getAndroidVersion() == "N":
                CameraCommon().changeLanguageInSettingForN(self.camera.cfg.get("language_input_n"), \
                        self.camera.cfg.get("language_title_n"), self.camera.cfg.get("language_origin"))
            else:
                CameraCommon().changeLanguageInSetting(self.camera.cfg.get("language_input"), \
                        self.camera.cfg.get("language_title"), self.camera.cfg.get("language_origin"))
        except Exception as e:
            CameraCommon().checkCameraCrash()
            if CameraCommon().getAndroidVersion() == "N":
                CameraCommon().changeLanguageInSettingForN(self.camera.cfg.get("language_input_n"), \
                        self.camera.cfg.get("language_title_n"), self.camera.cfg.get("language_origin"))
            else:
                CameraCommon().changeLanguageInSetting(self.camera.cfg.get("language_input"), \
                        self.camera.cfg.get("language_title"), self.camera.cfg.get("language_origin"))
            self.assertTrue(False, e)

    def cameraPressShutterKeyQuicklySingleCaptureModeTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            x, y = self.camera.clickShutterBtnArea()
            for i in range(200):
                CameraCommon().clickBtn(x, y)
            CameraCommon().checkCameraCrash()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraRecordModeBackKeyTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            for j in range(9):
                self.logger.debug("===== record mode back key test loop %d =====" % (j + 1))
                self.camera.switchRearOrFront("Front")
                CameraCommon().pressBack()
                CameraCommon().checkCameraCrash()
                self.camera.startCameraApp()
                self.camera.clickRecordBtn()
                time.sleep(5)
                CameraCommon().pressBack()
                CameraCommon().waitForTheFilesAreGenerated(j + 1)
                CameraCommon().checkCameraCrash()
                self.checkFileCorrupt()
                self.camera.switchRearOrFront("Back")
                CameraCommon().pressBack()
                CameraCommon().checkCameraCrash()
                self.camera.startCameraApp()
                self.camera.clickRecordBtn()
                time.sleep(5)
                CameraCommon().pressBack()
                CameraCommon().waitForTheFilesAreGenerated(j + 2)
                CameraCommon().checkCameraCrash()
                self.checkFileCorrupt()
                CameraCommon().removeDeivceFile()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraRecordModeHomeKeyTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Front")
            for i in range(2):
                for j in range(5):
                    self.logger.debug("==== record mode home key loop " + str((i + 1) * (j + 1)) + " ====")
                    CameraCommon().pressHome()
                    CameraCommon().checkCameraCrash()
                    self.camera.startCameraApp()
                    self.camera.clickRecordBtn()
                    time.sleep(5)
                    CameraCommon().pressHome()
                    self.camera.startCameraApp()
                    self.camera.swipeScreen("left")
                    CameraCommon().clickScreenCenter()
                    CameraCommon().pressHome()
                    CameraCommon().checkCameraCrash()
                    self.camera.stopCameraApp()
                    self.camera.startCameraApp()
                    CameraCommon().clickScreenCenter()
                    for n in range(3):
                        if not self.camera.isShutterBtnExists():
                            CameraCommon().pressBack()
                self.camera.switchRearOrFront("Back")
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraRecordVideoLocalTransferTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Back")
            self.camera.recordVideo(1, 5)
            CameraCommon().removeFile(self.host_path + "/*")
            CameraCommon().adbPullFile(self.camera_dir, self.host_path, True)
            CameraCommon().removeDeivceFile()
            self.camera.clickRecordBtn()
            CameraCommon().adbPushFile(self.host_path, self.camera_dir, True)
            time.sleep(3)
            self.camera.clickRecordBtn()
            CameraCommon().waitForTheFilesAreGenerated()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraRecordModeLockAndUnlockTest(self, loop):
        try:
            self.appPrepare()
            self.multimedia_setting.install_apk("unlock_app")
            CameraCommon().lockScreen()
            for i in range(int(loop)):
                CameraCommon().pressHome()
                time.sleep(3)
                self.logger.debug("===== loop %d =====" % (i + 1))
                self.camera.startCameraApp()
                self.camera.selectMode("Video")
                self.camera.switchRearOrFront("Front")
                CameraCommon().clickScreenCenter()
                CameraCommon().pressPower()
                CameraCommon().launchUnlockAppToUnlockScreen()
                self.camera.clickRecordBtn()
                self.logger.debug("video recording start")
                time.sleep(5)
                CameraCommon().pressPower()
                self.logger.debug("video recording end")
                CameraCommon().launchUnlockAppToUnlockScreen()
                self.camera.swipeScreen("left")
                CameraCommon().clickScreenCenter()
                time.sleep(2)
                self.camera.skipPopupButton("got")
                CameraCommon().pressPower()
                CameraCommon().launchUnlockAppToUnlockScreen()
                CameraCommon().checkCameraCrash()
                CameraCommon().pressBack()
                if not self.camera.isShutterBtnExists():
                    CameraCommon().pressBack()
            CameraCommon().lockScreen(False)
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            CameraCommon().lockScreen(False)
            self.assertTrue(False, e)

    def cameraRecordVideoMusicPlayerBackgroundTest(self):
        try:
            self.appPrepare()
            self.audioAppPrepare()
            self.audio.launch_music_am()
            self.audio.enterSongsPage()
            self.audio.playMusic(self.audio_name)
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Back")
            self.camera.clickRecordBtn()
            time.sleep(3)
            CameraCommon().openNotificationAndCheckMusicStatus("Play")
            time.sleep(3)
            self.camera.clickRecordBtn()
            CameraCommon().waitForTheFilesAreGenerated()
            if CameraCommon().isWidgetExists(CameraCommon().CommonWidget().musicPauseBtn()):
                self.assertTrue(False, "google music has stopped, case failed")
            g_common_obj.stop_app_am("com.google.android.music")
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            g_common_obj.stop_app_am("com.google.android.music")
            self.assertTrue(False, e)

    def cameraRecordModeRecentlyApplicationKeyTest(self, loop):
        try:
            self.appPrepare()
            CameraCommon().removeAllAppFromRecentTask()
            for i in range(int(loop)):
                self.logger.debug("=====camera record mode recently app key test loop %d =====" % (i + 1))
                self.camera.startCameraApp()
                self.camera.selectMode("Video")
                self.camera.switchRearOrFront("Front")
                CameraCommon().enterAppFromRecentTask()
                CameraCommon().checkCameraCrash()
                self.camera.clickRecordBtn()
                time.sleep(5)
                CameraCommon().enterAppFromRecentTask()
                time.sleep(2)
                self.camera.swipeScreen("left")
                CameraCommon().clickScreenCenter()
                time.sleep(1)
                CameraCommon().enterAppFromRecentTask()
                if not self.camera.isShutterBtnExists():
                    CameraCommon().pressBack()
                self.camera.startCameraApp()
                self.camera.selectMode("Video")
                self.camera.switchRearOrFront("Back")
                CameraCommon().enterAppFromRecentTask()
                CameraCommon().checkCameraCrash()
                self.camera.clickRecordBtn()
                time.sleep(5)
                CameraCommon().enterAppFromRecentTask()
                time.sleep(2)
                self.camera.swipeScreen("left")
                CameraCommon().clickScreenCenter()
                time.sleep(1)
                CameraCommon().enterAppFromRecentTask()
                if not self.camera.isShutterBtnExists():
                    CameraCommon().pressBack()
                self.checkFileCorrupt()
                CameraCommon().removeDeivceFile()
                CameraCommon().removeFile(self.host_path + "/*")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraViewRecordVideoInformationTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Back")
            self.camera.recordVideo(1, 5)
            info, image_name = self.checkFileCorrupt()
            self.checkVideoInfo(info, image_name)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def reviewVideoTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront(lens)
            self.camera.recordVideo(3, 5)
            self.checkFileCorrupt(3)
            self.camera.reviewPhotoAndVideo(3)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def s016RearReviewPhotoTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront(lens)
            for i in range(3):
                self.camera.capturePhoto()
            self.checkFileCorrupt(3)
            self.camera.reviewPhotoAndVideo(3)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def CameraVideoReviewAndSnapshotTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.capturePhoto()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront(lens)
            mMin = self.camera.getAllVideoResolutions(lens)[2]
            self.camera.snapShotDuringVideo(3, 5, 1)
            self.camera.setVideoResolution(mMin, lens)
            self.camera.snapShotDuringVideo(3, 5, 1)
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo(13)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cleckAllSettingReEnterCameraTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            mDict = self.camera.getAllPhotoResolutions("Back")[0]
            for k in mDict:
                self.logger.debug(mDict[k])
                self.camera.setPhotoResolution(mDict[k], "Back")
                self.camera.capturePhoto()
            CameraCommon().pressBack(2)
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Front")
            mDict = self.camera.getAllVideoResolutions("Front")[0]
            for m in mDict:
                self.logger.debug(mDict[m])
                self.camera.setVideoResolution(mDict[m], "Front")
                self.camera.recordVideo(1, 5)
            self.checkFileCorrupt()
            CameraCommon().removeDeivceFile()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraShutterTimerCaptureScreenLockTest(self):
        try:
            self.appPrepare()
            self.multimedia_setting.install_apk("unlock_app")
            CameraCommon().lockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.setTimer("10s")
            self.camera.capturePhoto(1, False)
            time.sleep(1)
            CameraCommon().pressPower()
            time.sleep(2)
            CameraCommon().launchUnlockAppToUnlockScreen()
            CameraCommon().clickScreenCenter()
            self.camera.capturePhoto(1, False)
            time.sleep(10)
            CameraCommon().waitForTheFilesAreGenerated()
            self.checkFileCorrupt()
            CameraCommon().lockScreen(False)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            CameraCommon().lockScreen(False)
            self.assertTrue(False, e)

    def capturePictureTimer(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront(lens)
            self.camera.setTimer("10s")
            self.camera.capturePhoto(1, False)
            for i in range(5):
                if CameraCommon().checkDeviceFile():
                    self.assertTrue(False, "file exists, check file failed")
            time.sleep(10)
            if not CameraCommon().checkDeviceFile():
                self.assertTrue(False, "file not exists, check file failed")
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def capturePictureTimer_3s(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront(lens)
            self.camera.setTimer("3s")
            self.camera.capturePhoto(1, False)
            if CameraCommon().checkDeviceFile():
                self.assertTrue(False, "file exists, check file failed")
            time.sleep(3)
            CameraCommon().waitForTheFilesAreGenerated()
            if not CameraCommon().checkDeviceFile():
                self.assertTrue(False, "file not exists, check file failed")
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def captureMultiSettingsTest(self):
        try:
            self.appPrepare()
            CameraCommon().launchGpsApp()
            network_longitude, network_latitude = CameraCommon().getGPSLocation()
            self.camera.openLocation()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            mMin = self.camera.getAllPhotoResolutions("Back")[2]
            self.camera.setPhotoResolution(mMin, "Back")
            self.camera.openManualExposure()
            self.camera.openGEO()
            self.camera.setGrid("on")
            self.camera.setTimer("3s")
            self.camera.setExposure("+2")
            self.camera.capturePhoto()
            time.sleep(3)
            info, picName1 = self.checkFileCorrupt()
            if info.get("GPS Latitude"):
                latitude = info.get("GPS Latitude").split(' ')
                longitude = info.get("GPS Longitude").split(' ')
            else:
                os.system("cp " + self.host_path + "/" + picName1 + " " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "Can't get gps information from picture or video")
            self.logger.debug(str(latitude[0]) + ", " + str(longitude[0]) + ", " + str(network_latitude) + ", " + str(network_longitude))
            assert (network_longitude == longitude[0]) and (network_latitude == latitude[0]), "gps location error"
            self.camera.switchRearOrFront("Front")
            self.camera.openManualExposure()
            self.camera.setGrid("off")
            self.camera.setTimer("off")
            self.camera.capturePhoto()
            info, picName2 = self.checkFileCorrupt()
            first_img_back = self.host_path + "/" + picName1
            original_back = self.host_path + "/" + picName2
            if self.checkImage.compare_images_brightness(first_img_back, original_back):
                self.assertTrue("exposure check fail")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def fileSizeTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront(lens)
            self.camera.capturePhoto()
            file_size = CameraCommon().getFileName(self.camera_dir)
            if not file_size:
                assert False, "get file name fail"
#             CameraCommon().adbPullFile(self.camera_dir, self.host_path,True)
            info = self.checkFileCorrupt()[0]
            if not info:
                assert False, "get file exif infomation fail"
            list = info.get("File Size").split(' ')
            self.size = []
            for j in range(len(file_size)):
                cmd = "du -l " + self.host_path + "/" + file_size[j] + "|awk '{print $1}'"
                self.size.append(CameraCommon().getFileSize(cmd))
            result = False
            for m in range(len(file_size)):
                mSize = float(self.size[m])
                if mSize > 1024 * 2:
                    mSize = mSize / 1024
                self.logger.debug("host du -l size=" + str(mSize))
                self.logger.debug("exif size=%s" % str(float(list[0])))
                if abs(float(list[0]) >= (float(mSize) - 5.0)):
                    result = True
                    break
            if not result:
                assert False, "compare file size fail"
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def getExifInfo(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront(lens)
            self.camera.capturePhoto()
            info, fileName = self.checkFileCorrupt()
            return info, fileName
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def gpsTagTest(self, lens):
        try:
            self.appPrepare()
            CameraCommon().launchGpsApp()
            network_longitude, network_latitude = CameraCommon().getGPSLocation()
            self.camera.openLocation()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront(lens)
            self.camera.openGEO()
            self.camera.capturePhoto()
            info = self.checkFileCorrupt()[0]
            if info.get("GPS Latitude"):
                latitude = info.get("GPS Latitude").split(' ')
                longitude = info.get("GPS Longitude").split(' ')
            else:
                self.assertTrue(False, "Can't get gps information from picture or video")
            self.logger.debug(str(latitude[0]) + ", " + str(longitude[0]) + ", " + str(network_latitude) + ", " + str(network_longitude))
            self.assertTrue((network_longitude == longitude[0]) and (network_latitude == latitude[0]), "gps location error")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def exifFileTypeTest(self, lens):
        info = self.getExifInfo(lens)[0]
        self.logger.debug(str(info.get("File Type")))
        self.assertTrue(info.get("File Type") == "JPEG", "file type is null")

    def exifImageSizeTest(self, lens):
        info = self.getExifInfo(lens)[0]
        width = info.get("Image Width")
        height = info.get("Image Height")
        size = info.get("Image Size")
        self.logger.debug(str(width) + ", " + str(height) + ", " + str(size))
        self.assertTrue(size == width + "x" + height, "image size is null")

    def exifImageTitleTest(self, lens):
        info, file_name = self.getExifInfo(lens)
        title = info.get("File Name")
        self.logger.debug(str(title) + ", " + str(file_name))
        self.assertTrue(title == file_name, "image title is null")

    def cameraShutterTimerEnableDisableContinuously(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            self.camera.setTimer("10s")
            x, y = self.camera.clickShutterBtnArea()
            for i in range(30):
                CameraCommon().clickBtn(x, y)
                time.sleep(0.5)
            time.sleep(10)
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraCaptureModePowerOffBySoftwareKeyTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Front")
            g_common_obj2.system_reboot()
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            CameraCommon().clickScreenCenter()
            self.camera.capturePhoto()
            self.camera.enterPreviewPhotos()
            g_common_obj2.system_reboot()
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            CameraCommon().clickScreenCenter()
            self.camera.capturePhoto()
            self.camera.switchRearOrFront("Back")
            g_common_obj2.system_reboot()
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            CameraCommon().clickScreenCenter()
            self.camera.capturePhoto()
            self.camera.enterPreviewPhotos()
            g_common_obj2.system_reboot()
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            CameraCommon().clickScreenCenter()
            self.camera.capturePhoto()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.camera.stopCameraApp()
            self.assertTrue(False, e)

    def cameraReceiveEmailSingleCaptureModeTest(self):
        try:
            self.appPrepare()
            self.multimedia_setting.install_apk("email_notification")
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            CameraCommon().clickScreenCenter()
            CameraCommon().launchWearableNotification()
            CameraCommon().openNotificationAndCheckReceiveEmail()
            self.camera.capturePhoto()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraFirstLaunchCameraDefaultSettingTest(self):
        try:
            self.appPrepare()
            g_common_obj2.system_reboot(90)
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.checkLens()
            CameraCommon().checkCameraCrash()
            self.camera.capturePhoto()
            self.camera.setTimer()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraVideoModePowerOffBySoftwareKeyTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Front")
            g_common_obj2.system_reboot()
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            CameraCommon().clickScreenCenter()
            self.camera.recordVideo(1, 10)
            self.camera.enterPreviewVideos()
            g_common_obj2.system_reboot()
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            CameraCommon().clickScreenCenter()
            self.camera.recordVideo(1, 10)
            self.camera.switchRearOrFront("Back")
            g_common_obj2.system_reboot()
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            CameraCommon().clickScreenCenter()
            self.camera.recordVideo(1, 10)
            self.camera.enterPreviewVideos()
            g_common_obj2.system_reboot()
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            CameraCommon().clickScreenCenter()
            self.camera.recordVideo(1, 10)
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraLaunchFirstLaunchCameraDefaultCameraLensTest(self):
        try:
            self.appPrepare()
            g_common_obj2.system_reboot(90)
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.checkLens()
            CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraDeleteFileCameraFolderTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.capturePhoto(2)
            if not CameraCommon().checkDeviceFile():
                self.assertTrue(False, "file not exists, check file failed")
            CameraCommon().removeDeivceFile()
            if CameraCommon().checkDeviceFile():
                self.assertTrue(False, "file exists, check file failed")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraDeleteRebootTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            self.camera.capturePhoto(2)
            self.camera.selectMode("Video")
            self.camera.recordVideo(2)
            CameraCommon().removeDeivceFile()
            g_common_obj2.system_reboot(90)
            if CameraCommon().checkDeviceFile():
                self.assertTrue(False, "file exists, check file failed")
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Front")
            self.camera.capturePhoto(2)
            self.camera.selectMode("Video")
            self.camera.recordVideo(2)
            CameraCommon().removeDeivceFile()
            g_common_obj2.system_reboot(90)
            if CameraCommon().checkDeviceFile():
                self.assertTrue(False, "file exists, check file failed")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraRenameCameraFolderTest(self):
        try:
            self.appPrepare()
            cmdstr = "adb shell ls /sdcard/DCIM"
            g_common_obj.adb_cmd("rename /sdcard/DCIM /sdcard/DCIM2")
            check_res=os.popen(cmdstr).read()
            if check_res.find("No such file or directory") == -1:
                self.assertTrue(False, "DCIM folder exists,check folder failed")
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            self.camera.capturePhoto(2)
            check_res=os.popen(cmdstr).read()
            if check_res.find("No such file or directory") != -1:
                self.assertTrue(False, "DCIM folder not exists,check folder failed")
            if not CameraCommon().checkDeviceFile():
                self.assertTrue(False, "file not exists, check file failed")
            g_common_obj.adb_cmd("rename /sdcard/DCIM /sdcard/DCIM3")
            check_res=os.popen(cmdstr).read()
            if check_res.find("No such file or directory") == -1:
                self.assertTrue(False, "DCIM folder exists,check folder failed")
            self.camera.selectMode("Video")
            self.camera.recordVideo(2)
            g_common_obj2.system_reboot(90)
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            self.camera.capturePhoto(2)
            check_res=os.popen(cmdstr).read()
            if check_res.find("No such file or directory") != -1:
                self.assertTrue(False, "DCIM folder not exists,check folder failed")
            if not CameraCommon().checkDeviceFile():
                self.assertTrue(False, "file not exists, check file failed")
            g_common_obj.adb_cmd("rm -rf /sdcard/DCIM2")
            g_common_obj.adb_cmd("rm -rf /sdcard/DCIM3")
        except Exception as e:
            g_common_obj.adb_cmd("rm -rf /sdcard/DCIM2")
            g_common_obj.adb_cmd("rm -rf /sdcard/DCIM3")
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraPressShutterButtonQuicklyWhenTransferFilesToPCTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Back")
            self.camera.recordVideo(1, 15)
            self.checkFileCorrupt()
            print CameraCommon().getFileName(self.host_path + "/")
            self.camera.selectMode("Camera")
            self.camera.capturePhoto()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraFITAfterSleep10minsTest(self):
        try:
            self.appPrepare()
            CameraCommon().setKeepAwakeOn(False)
            CameraCommon().setSettingSleepTime("1 minute")
            g_common_obj2.system_reboot(90)
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            CameraCommon().clickScreenCenter()
            time.sleep(10*60)
            CameraCommon().pressPower()
            self.camera.capturePhoto(3)
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Front")
            CameraCommon().clickScreenCenter()
            time.sleep(10*60)
            CameraCommon().pressPower()
            self.camera.recordVideo(2, 5)
            self.camera.reviewPhotoAndVideo(5)
            CameraCommon().setKeepAwakeOn()
        except Exception as e:
            CameraCommon().setKeepAwakeOn()
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraFlashImageOnOffRepeatedly(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            for _ in range(5):
                self.camera.setFlash("on")
                self.camera.setFlash("off")
                CameraCommon().checkCameraCrash()
            self.camera.switchRearOrFront("Front")
            for _ in range(5):
                self.camera.setFlash("on")
                self.camera.setFlash("off")
                CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraTouchFocus20times(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            for i in range(20):
                CameraCommon().clickScreenCenter()
            CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraZoomInZoomOut20timesTest(self):
        try:
            self.appPrepare()
            DeviceControl().pushEventHunterToDevices(self.multimedia_setting)
            mlist = DeviceControl().getZoomCommands()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            for i in range(2):
                CameraCommon().clickScreenCenter()
                for j in range(20):
                    self.logger.debug("== zoom in")
                    CameraCommon().zoomInOrOut(self.camera, mlist[0])
                    self.logger.debug("== zoom out")
                    CameraCommon().zoomInOrOut(self.camera, mlist[1])
                size = self.camera.getAllPhotoResolutions("Back")[1]
                self.camera.setPhotoResolution(size)
                for k in range(20):
                    self.logger.debug("== zoom in")
                    CameraCommon().zoomInOrOut(self.camera, mlist[0])
                    self.logger.debug("== zoom out")
                    CameraCommon().zoomInOrOut(self.camera, mlist[1])
                self.camera.switchRearOrFront("Front")
            self.camera.reviewPhotoAndVideo()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraDoubleClickViewfinderRepeatManyTimesTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            for i in range(20):
                CameraCommon().clickBtn(CameraCommon().x/2, CameraCommon().y/2)
            CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraWhenDUTHighRAMOrCPU(self):
        try:
            self.appPrepare()
            AutodetectImpl().make_device_cpu_overload()
            for i in range(5):
                self.camera.startCameraApp()
                self.camera.selectMode("Camera")
                self.camera.switchRearOrFront("Back")
                self.camera.capturePhoto()
                self.camera.selectMode("Video")
                self.camera.switchRearOrFront("Front")
                self.camera.recordVideo(1,5)
                self.camera.switchRearOrFront("Back")
                self.camera.selectMode("Panorama")
                self.camera.capturePhoto()
                CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)


    def panoramaTest(self):
        try:
            self.appPrepare()
            self.camera.enter_camera_from_home()
            time.sleep(2)
            self.camera.switch_to_camera_options("Panorama")
            if self.camera.GoogleDefaultCamera().camera_page_three_dots(False).exists():
                self.camera.GoogleDefaultCamera().camera_page_three_dots().click()
                self.logger.debug("====wait panoramaModeWidget=====")
                self.camera.waitForWidgetToAppear(self.camera.GoogleDefaultCamera().panoramaModeWidget(False), "Horizontal panorama mode widget")
                self.logger.debug("====click panoramaModeWidget=====")
                self.camera.GoogleDefaultCamera().panoramaModeWidget().click()
            self.logger.debug("====wait shutter button=====")
            self.camera.waitForWidgetToAppear(self.camera.GoogleDefaultCamera().camera_page_shutter(False), "shutter button")
            self.logger.debug("====click shutter button=====")
            self.camera.GoogleDefaultCamera().camera_page_shutter(True).click()
            self.logger.debug("====wait done button====")
            self.camera.waitForWidgetToAppear(self.camera.GoogleDefaultCamera().photoDoneWidget(False), "photo Done Widget")
            
            "moving dut..."
            import testaid.robot
            self.logger.debug("====init====")
            robot = testaid.robot.Robot("/dev/ttyACM0")
            self.logger.debug("====reset====")
            robot.reset()
            self.logger.debug("====rotate====")
            for i in range(18):
                robot.rotate(20, 3)
                time.sleep(3)
            if self.camera.GoogleDefaultCamera().photoDoneWidget(False).exists():
                self.logger.debug("====click done button=====")
                self.camera.GoogleDefaultCamera().photoDoneWidget().click()
            time.sleep(2)
            self.camera.swipe_screen("left")
            self.logger.debug("====wait photo process bar to disappear=====")
            self.camera.waitForWidgetToDisappear(self.camera.GoogleDefaultCamera().photoProcessingBar(False), "photo process bar")
            self.camera.swipe_screen()
            
            "change to the initial angle"
            robot.reset()
            
        except Exception as e:
            self.camera.judge_if_camera_crash()
            self.assertTrue(False, e)

    def sphereTest(self):
        try:
            self.appPrepare()
            self.camera.enter_camera_from_home()
            time.sleep(2)
            self.camera.switch_to_camera_options("Photo Sphere")
            "moving dut..."
            import testaid.robot
            robot = testaid.robot.Robot("/dev/ttyACM1")
            robot.reset()
            robot.swing(-90, 3)
            robot.swing_to(0)
            robot.swing(90, 3)
            
            self.logger.debug("====wait done button====")
            self.camera.waitForWidgetToAppear(self.camera.GoogleDefaultCamera().photoDoneWidget(False), "photo Done Widget")
            self.logger.debug("====click done button=====")
            self.camera.GoogleDefaultCamera().photoDoneWidget().click()
            self.camera.swipe_screen("left")
            self.logger.debug("====wait photo process bar to disappear=====")
            self.camera.waitForWidgetToDisappear(self.camera.GoogleDefaultCamera().photoProcessingBar(False), "photo process bar")
            self.camera.swipe_screen()

            "change to the initial angle"
            # robot.swing_to(0)
        except Exception as e:
            self.camera.judge_if_camera_crash()
            self.assertTrue(False, e)

    def lensblurTest(self):
        try:
            self.appPrepare()
            self.camera.enter_camera_from_home()
            time.sleep(2)
            self.camera.switch_to_camera_options("Lens Blur")
            self.camera.checkNextExists()
            self.camera.check_notification_after_switch_mode()
            self.logger.debug("====wait shutter button=====")
            self.camera.waitForWidgetToAppear(self.camera.GoogleDefaultCamera().camera_page_shutter(False), "shutter button")
            self.logger.debug("====click shutter button=====")
            self.camera.GoogleDefaultCamera().camera_page_shutter(True).click()

            "moving dut..."
            # import testaid.robot
            # robot = testaid.robot.Robot("/dev/ttyACM0")
            # robot.reset()
            # robot.swing(-45, 1)
            self.logger.debug("====wait shutter button=====")
            self.camera.waitForWidgetToAppear(self.camera.GoogleDefaultCamera().camera_page_shutter(False), "shutter button")
            self.camera.swipe_screen("left")
            self.logger.debug("====wait photo process bar to disappear=====")
            self.camera.waitForWidgetToDisappear(self.camera.GoogleDefaultCamera().photoProcessingBar(False), "photo process bar", 30)
            self.camera.swipe_screen()

            "change to the initial angle"
            # robot.swing_to(0)

        except Exception as e:
            self.camera.judge_if_camera_crash()
            self.assertTrue(False, e)

    def markLocationTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.setLocation("ON")
            self.camera.capturePhoto()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def test_Camera_Launch_Close(self):
        self.launchCloseCamera()

    def test_Camera_CloseByReturnKeyToHomeScreen(self):
        self.closeCameraByReturnToHomeScreen()

    def test_Camera_DestoryCameraInTaskManager(self):
        self.destoryCameraInTaskManager()

    def test_Camera_LaunchInTaskManager(self):
        self.launchCameraInTaskManager()

    def test_Camera_VideoMode_MaxSize_SwitchBetween_RearOrFront(self):
        self.cameraRecodingVideoSizeSwitchTest("max")

    def test_Camera_VideoMode_MinSize_SwitchBetween_RearOrFront(self):
        self.cameraRecodingVideoSizeSwitchTest("min")

    def test_Camera_Capture_MaxSize_SwitchBetween_RearOrFront(self):
        self.cameraCaptureSwitchTest("max")

    def test_Camera_Capture_MinSize_SwitchBetween_RearOrFront(self):
        self.cameraCaptureSwitchTest("min")

    def test_Camera_Store_Location(self):
        self.storeLocationTest()

    def test_Camera_DisableAndEnableCamera(self):
        self.disableAndEnableCameraTest()

    def test_Camera_Preview_30mins_CaptureMode_RearCamera(self):
        self.cameraPreviewCaptureModeTest("Camera", "Back", 1800)

    def test_Camera_Preview_30mins_CaptureMode_FrontCamera(self):
        self.cameraPreviewCaptureModeTest("Camera", "Front", 1800)

    def test_Camera_ClearData_ForceClose(self):
        self.cameraClearDataForceCloseTest()

    def test_Camera_Launch_From_LockScreen(self):
        self.launchCameraFromLockScreenTest()

    def test_Camera_VideoMode_Multi_Settings(self):
        self.videoMultiSettingsTest()

    def test_Camera_ExposureBias_Front_Interact_Rear_Front(self):
        self.manualExposureInteractRearFrontTest()

    def test_Camera_ExposureBias_Compare_Exposure_Effect_Increase_Or_Decrease(self):
        self.compareExposureEffectIncreaseOrDecreaseTest()

    def test_Camera_VideoMode_GPSLocation_RecordMode(self):
        self.cameraGPSLocationRecordModeTest()

    def test_Camera_EXIF_Rear_Front_GPSLocation_CaptureMode(self):
        self.cameraGPSLocationCaptureModeTest()

    def test_Camera_VideoMode_Rear_720p_10mins(self):
        self.videoRecording("Video", "Back", "720", 600)

    def test_Camera_VideoMode_Rear_1080p_10mins(self):
        self.videoRecording("Video", "Back", "1080", 600)

    def test_Camera_VideoMode_Rear_480p_10mins(self):
        self.videoRecording("Video", "Back", "480", 600)

    def test_Camera_VideoMode_Front_480p_10mins(self):
        self.videoRecording("Video", "Front", "480", 600)

    def test_Camera_HomeKey_CapturePhoto_WhenRelaunch(self):
        self.homeKeyCapturePhotoWhenRelaunchTest()

    def test_Camera_KillCamera_CapturePhoto_WhenRelaunch(self):
        self.killCameraCapturePhotoWhenRelaunchTest()

    def test_Camera_CaptureMode_LockAndUnlock(self):
        self.cameraCaptureModeLockAndUnlockTest(10)

    def test_Camera_CaptureMode_VolumeKey(self):
        self.cameraCaptureModeVolumeKeyTest()

    def test_Camera_CapturePicture_Timer_Alarm(self):
        self.cameraCapturePictureTimerAlarmTest()

    def test_Camera_Function_After_Change_System_Language(self):
        self.cameraFunctionAfterChangeSystemLanguageTest()

    def test_Camera_PressShutterKeyQuickly_SingleCaptureMode(self):
        self.cameraPressShutterKeyQuicklySingleCaptureModeTest()

    def test_Camera_VideoMode_BackKey(self):
        self.cameraRecordModeBackKeyTest()

    def test_Camera_VideoMode_HomeKey(self):
        self.cameraRecordModeHomeKeyTest()

    def test_Camera_VideoMode_Local_Transfer(self):
        self.cameraRecordVideoLocalTransferTest()

    def test_Camera_VideoMode_LockAndUnlock(self):
        self.cameraRecordModeLockAndUnlockTest(10)

    def test_Camera_VideoMode_MusicPlayer_Background(self):
        self.cameraRecordVideoMusicPlayerBackgroundTest()

    def test_Camera_VideoMode_RecentlyApplicationKey(self):
        self.cameraRecordModeRecentlyApplicationKeyTest(10)

    def test_Camera_VideoMode_View_RecordVideo_Information(self):
        self.cameraViewRecordVideoInformationTest()

    def test_Camera_VideoMode_Front_ReviewVideo(self):
        self.reviewVideoTest("Front")

    def test_Camera_Rear_ReviewPhoto(self):
        self.s016RearReviewPhotoTest("Back")

    def test_Camera_Snapshot_Front_Review(self):
        self.CameraVideoReviewAndSnapshotTest("Front")

    def test_Camera_VideoMode_Rear_ReviewVideo(self):
        self.reviewVideoTest("Back")

    def test_Camera_Snapshot_Rear_Review(self):
        self.CameraVideoReviewAndSnapshotTest("Back")

    def test_Camera_Front_ReviewPhoto(self):
        self.s016RearReviewPhotoTest("Front")

    def test_Camera_CheckAllSetting_ReEnterCamera(self):
        '''
        Verify check all settings after re-enter Camera app
        '''
        self.cleckAllSettingReEnterCameraTest()

    def test_Camera_Timer_Capture_ScreenLock(self):
        self.cameraShutterTimerCaptureScreenLockTest()

    def test_Camera_Timer_Front_10s(self):
        self.capturePictureTimer("Front")

    def test_Camera_Timer_Rear_10s(self):
        self.capturePictureTimer("Back")

    def test_Camera_Capture_Multi_Settings(self):
        self.captureMultiSettingsTest()

    def test_Camera_EXIF_Front_FileSize(self):
        self.fileSizeTest("Front")

    def test_Camera_EXIF_Rear_FileSize(self):
        self.fileSizeTest("Back")

    def test_Camera_EXIF_Front_FileType(self):
        self.exifFileTypeTest("Front")

    def test_Camera_EXIF_Rear_FileType(self):
        self.exifFileTypeTest("Back")

    def test_Camera_EXIF_Front_GPSTag(self):
        self.gpsTagTest("Front")

    def test_Camera_EXIF_Rear_GPSTag(self):
        self.gpsTagTest("Back")

    def test_Camera_EXIF_Front_ImageSize(self):
        self.exifImageSizeTest("Front")

    def test_Camera_EXIF_Rear_ImageSize(self):
        self.exifImageSizeTest("Back")

    def test_Camera_EXIF_Front_ImageTitle(self):
        self.exifImageTitleTest("Front")

    def test_Camera_EXIF_Rear_ImageTitle(self):
        self.exifImageTitleTest("Back")

    def test_Camera_Timer_EnableDisable_Continuously(self):
        self.cameraShutterTimerEnableDisableContinuously()

    def test_Camera_CaptureMode_PowerOffBySoftwareKey(self):
        self.cameraCaptureModePowerOffBySoftwareKeyTest()

    def test_Camera_ReceiveEmail_SingleCaptureMode(self):
        self.cameraReceiveEmailSingleCaptureModeTest()

    def test_Camera_FirstLaunchCamera_DefaultSetting(self):
        self.cameraFirstLaunchCameraDefaultSettingTest()

    def test_Camera_VideoMode_PowerOffBySoftwareKey(self):
        self.cameraVideoModePowerOffBySoftwareKeyTest()

    def test_Camera_Launch_FirstLaunchCamera_Default_Camera_Lens(self):
        self.cameraLaunchFirstLaunchCameraDefaultCameraLensTest()

    def test_Camera_DeleteFile_CameraFolder(self):
        self.cameraDeleteFileCameraFolderTest()

    def test_Camera_Delete_Reboot(self):
        self.cameraDeleteRebootTest()

    def test_Camera_RenameCameraFolder(self):
        self.cameraRenameCameraFolderTest()

    def test_Camera_Press_Shutter_Button_Quickly_When_TransferFiles_To_PC(self):
        self.cameraPressShutterButtonQuicklyWhenTransferFilesToPCTest()

    def test_Camera_FIT_AfterSleep10mins(self):
        self.cameraFITAfterSleep10minsTest()

    def test_Camera_Flash_Image_on_off_repeatedly(self):
        self.cameraFlashImageOnOffRepeatedly()

    def test_Camera_TouchFocus_20times(self):
        self.cameraTouchFocus20times()

    def test_Camera_ZoomIn_ZoomOut_20times(self):
        self.cameraZoomInZoomOut20timesTest()

    def test_Camera_DoubleClickViewfinder_RepeatManyTimes(self):
        self.cameraDoubleClickViewfinderRepeatManyTimesTest()

    def test_Camera_When_DUT_HighRAM_Or_CPU(self):
        self.cameraWhenDUTHighRAMOrCPU()

    def test_Camera_Launch_FirstLaunchCamera_MarkLocation(self):
        self.markLocationTest()

    def test_Camera_Timer_Front_3s(self):
        self.capturePictureTimer_3s("Front")

    def test_Camera_Timer_Rear_3s(self):
        self.capturePictureTimer_3s("Back")

    def test(self):
        self.panoramaTest()
