# coding: utf-8
import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.checkImage import CheckImage
from testlib.camera.checkVideo import CheckVideo
from testlib.camera.CameraCommon import CameraCommon
from testlib.domains.settings_impl import SettingsImpl
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.camera.cameratestbase import CameraTestBase
from testlib.util.repo import Artifactory

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
        self.multimedia_setting = MultiMediaSetting(CameraCommon.DEFAULT_CONFIG_FILE)
        self.checkImage = CheckImage()
        self.video = CheckVideo()
        self.settings = SettingsImpl()
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

    def pictureWallpaperContactIconTest(self):
        try:
            self.appPrepare()
            CameraCommon().pressHome()
            CameraCommon().getScreenshotAndPullToHost("sc1.png", self.host_path)
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.capturePhoto()
            CameraCommon().pressBack(2)
            CameraCommon().setWallpaper("picture")
            CameraCommon().getScreenshotAndPullToHost("sc2.png", self.host_path)
            if self.checkImage.compare_images(self.host_path + "/sc1.png", self.host_path + "/sc2.png"):
                os.system("cp " + self.host_path + "/sc1.png"  +" " + g_common_obj.get_user_log_dir())
                os.system("cp " + self.host_path + "/sc2.png"  +" " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "change wallpaper fail")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)
        finally:
            CameraCommon().setWallpaper()

    def cameraworkairplaneModeOnTest(self):
        try:
            self.appPrepare()
            self.settings.launch_settings()
            self.settings.set_airplane_mode("ON")
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.capturePhoto()
            time.sleep(2)
            file_name= self.checkFileCorrupt()[1]
            self.path = self.host_path + "/" + file_name
            errMsg = self.checkImage.check_image_corrupt(self.path)
            if errMsg!="":
                self.assertTrue(False, errMsg)
            CameraCommon().removeFile(self.path)
            CameraCommon().removeDeivceFile()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)
        finally:
            self.settings.launch_settings()
            self.settings.set_airplane_mode("OFF")

    def push_file(self, cmd, datapath):
        if "http" not in datapath and "ftp" not in datapath:
            datapath = self.multimedia_setting.resource_file_path + datapath
        file_name = cmd.split("/")[-1].replace("\"","")
        push_path = cmd.split("\" \"")[1].replace("\"","")
        if "http" not in datapath and "ftp" not in datapath:
            file_path = os.path.join(datapath, file_name)
            file_path = file_path.replace("%20", " ")
            assert os.path.exists(file_path), "resource file not exist! path=%s" % file_path
            ret_file = file_path
        else:
            ret_file = Artifactory(datapath).get(file_name)
        if os.path.exists(ret_file):
            self.logger.debug("[Download]: Artifactory method")
            g_common_obj.adb_cmd_common("push \"" + ret_file + "\" " + "\"" + push_path + "\"", 1000)
            return ret_file
        else:
            assert 0,"Download filed!"

    def cameraFITAddFilesCameraFolderTest(self):
        try:
            self.appPrepare()
            self.logger.debug("push image file")
            self.image_path = self.push_file(self.camera.cfg.get("fit_image_name"), self.camera.cfg.get("fit_image_url"))
            CameraCommon().refreshCameraDir()
            self.camera.startCameraApp()
            self.camera.reviewPhotoAndVideo()
            CameraCommon().removeDeivceFile()
            CameraCommon().pressBack(2)
            self.camera.stopCameraApp()
            self.logger.debug("push video file")
            self.push_file(self.camera.cfg.get("fit_video_name"), self.camera.cfg.get("fit_video_url"))
            CameraCommon().refreshCameraDir()
            time.sleep(3)
            self.camera.startCameraApp()
            self.camera.reviewPhotoAndVideo()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraFITLaunchFromScreenLockTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            CameraCommon().clickScreenCenter()
            CameraCommon().pressBack(2)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)
            
        platform = CameraCommon().getPlatform()
        try:
            self.camera.startCameraApp()
            CameraCommon().clickScreenCenter()
            CameraCommon().pressBack(2)
            if "3gr" not in platform:
                CameraCommon().lockPIN()
            else:
                CameraCommon().lockScreen()
            CameraCommon().pressPower()
            CameraCommon().pressPower()
            CameraCommon().swipeCorner()
            time.sleep(3)
            CameraCommon().switchCamera(platform)
            time.sleep(2)
            CameraCommon().checkCameraCrash()
            CameraCommon().pressBack(2)
            CameraCommon().unlockPIN()
        except Exception as e:
            CameraCommon().unlockPIN()
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraFITVideoModeAlarmTest(self):
        try:
            self.appPrepare()
            self.multimedia_setting.install_apk("alarm_apk")
            CameraCommon().launchAlarmApp()
            CameraCommon().setAlarmTime(70)
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Back")
            self.camera.clickRecordBtn()
            mTime = time.time()
            while time.time() - mTime < 180:
                time.sleep(5)
                if CameraCommon().alarmDismissOrSnooze():
                    break
            CameraCommon().checkCameraCrash()
            CameraCommon().launchAlarmApp()
            CameraCommon().setAlarmTime(70)
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Front")
            self.camera.clickRecordBtn()
            mTime = time.time()
            while time.time() - mTime < 180:
                time.sleep(5)
                if CameraCommon().alarmDismissOrSnooze():
                    break
            CameraCommon().checkCameraCrash()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraLaunchVia3rdPartAppsTest(self):
        try:
            self.appPrepare()
            self.multimedia_setting.install_apk("thirdpartapk")
            CameraCommon().launchThirdPartApp()
            CameraCommon().captureImageOrRecordingVia3rdPartApp("image")
            CameraCommon().checkGuide()
            self.camera.capturePhoto(wait_time=20, flag=1)
            self.camera.clickDoneBtn()
#             if self.camera.isShutterBtnExists():
#                 CameraCommon().pressBack()
            CameraCommon().launchThirdPartApp()
            CameraCommon().captureImageOrRecordingVia3rdPartApp("video")
            CameraCommon().pressBack(2)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def test_Camera_Picture_Wallpaper_ContactIcon(self):
        self.pictureWallpaperContactIconTest()

    def test_Camera_Work_AirplaneModeOn(self):
        self.cameraworkairplaneModeOnTest()

    def test_Camera_FIT_AddFile_CameraFolder(self):
        self.cameraFITAddFilesCameraFolderTest()

    def test_Camera_FIT_launch_From_Screen_Lock(self):
        self.cameraFITLaunchFromScreenLockTest()

    def test_Camera_FIT_VideoMode_Alarm(self):
        self.cameraFITVideoModeAlarmTest()

    def test_Camera_Launch_Via_3rd_Part_Apps(self):
        self.cameraLaunchVia3rdPartAppsTest()