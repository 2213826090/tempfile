# coding: utf-8
import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.checkImage import CheckImage
from testlib.camera.checkVideo import CheckVideo
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.CameraCommon import CameraCommon
from testlib.common.common import g_common_obj2

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
        self.checkImage = CheckImage()
        self.multimedia_setting = MultiMediaSetting(CameraCommon.DEFAULT_CONFIG_FILE)
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

    def capturePictureTimer(self, lens):
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

    def goThroughImageQualityAndPanoResolution(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Panorama")
            mDict = self.camera.getAllPanoramaResolution()[0]
            for k in mDict:
                self.camera.setPanoramaResolution(mDict[k])
                self.camera.capturePhoto(1,False)
                time.sleep(3)
                self.camera.clickPanoramaDoneBtn()
                time.sleep(3)
                CameraCommon().waitForTheFilesAreGenerated()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraPressShutterKeyQuicklyPanoramaModeTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Panorama")
            x,y = self.camera.clickShutterBtnArea("Panorama")
            CameraCommon().clickBtn(x, y)
            CameraCommon().checkCameraCrash()
            time.sleep(3)
            self.camera.clickPanoramaDoneBtn()
            time.sleep(20)
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraPanoramaModeRecentlyApplicationKeyTest(self,loop):
        try:
            self.appPrepare()
            CameraCommon().removeAllAppFromRecentTask()
            for i in range(int(loop)):
                self.logger.debug("***** loop %d *****" %(i + 1))
                self.camera.startCameraApp()
                self.camera.selectMode("Panorama")
                CameraCommon().enterAppFromRecentTask("Camera")
                self.camera.capturePhoto(1,False)
                CameraCommon().enterAppFromRecentTask("Camera")
                CameraCommon().checkCameraCrash()
                self.camera.capturePhoto(1,False)
                time.sleep(3)
                self.camera.clickPanoramaDoneBtn()
                time.sleep(2)
                CameraCommon().enterAppFromRecentTask("Camera")
                CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def fullStorageBeforeCaptureTest(self, mode):
        """
        make the sdcard full, and to see the camera shutter button behavior
        @param mode: the camera mode, see the config file, tag is 
            low_resource_lensblur|low_resource_panorama|low_resource_photoSphere
        """
        try:
            self.appPrepare()
            self.logger.debug("fill the memory")
            CameraCommon().fillStorage(40)
            self.camera.startCameraApp()
            self.logger.debug("To get the internal memory capacity")
            self.camera.selectMode(mode)
            self.logger.debug("shutter button enabled or not: " + str(self.camera.checkShutterButtonAttribute("clickable")))
            assert self.camera.checkShutterButtonAttribute("clickable") == False, "The storage is full, but the shutterbutton is still workable"
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraPhotoSpherePressFinishButtonQuicklyTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Photo Sphere")
            self.camera.capturePhoto()
            self.camera.clickDoneBtn()
            CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraPanoramaLackStorageTest(self):
        try:
            self.appPrepare()
            self.logger.debug("fill the memory")
            CameraCommon().fillStorage(350)
            self.camera.startCameraApp()
            self.camera.selectMode("Panorama")
            start_time = time.time()
            while True:
                run_time = time.time()
                free_space = CameraCommon().getSdcardMemory()[2]
                print free_space, run_time
                if free_space < 300 or run_time - start_time > 600:
                    break
                self.camera.capturePhoto()
                self.camera.clickDoneBtn()
            for i in range(5):
                CameraCommon().pressBack()
                self.camera.startCameraApp()
            CameraCommon().checkCameraCrash()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraPanoramaLockAndUnlockTest(self,loop):
        try:
            self.appPrepare()
            self.multimedia_setting.install_apk("unlock_app")
            CameraCommon().lockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode("Panorama")
            self.camera.switchRearOrFront("Back")
            for i in range(int(loop)):
                self.logger.debug("=====camera panorama mode lock and unlock test loop %d =====" % (i + 1))
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
                CameraCommon().launchUnlockAppToUnlockScreen()
                CameraCommon().checkCameraCrash()
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo()
            CameraCommon().lockScreen(False)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            CameraCommon().lockScreen(False)
            self.assertTrue(False, e)

    def cameraPanoramaPowerOffBySoftwareKeyTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Panorama")
            g_common_obj2.system_reboot()
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode("Panorama")
            CameraCommon().clickScreenCenter()
            self.camera.capturePhoto()
            g_common_obj2.system_reboot()
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            CameraCommon().clickScreenCenter()
            self.camera.enterPreviewPhotos()
            self.camera.capturePhoto()
            self.camera.enterPreviewPhotos()
            g_common_obj2.system_reboot()
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            CameraCommon().clickScreenCenter()
            self.camera.enterPreviewPhotos()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.camera.stopCameraApp()
            self.assertTrue(False, e)


    def test_Camera_LensBlur_FullStorage(self):
        self.fullStorageBeforeCaptureTest("Lens Blur")

    def test_Camera_Panorama_FullStorage(self):
        self.fullStorageBeforeCaptureTest("Panorama")

    def test_Camera_PhotoSphere_FullStorage(self):
        self.fullStorageBeforeCaptureTest("Photo Sphere")

    def test_Camera_Panorama_RecentlyApplicationKey(self):
        self.cameraPanoramaModeRecentlyApplicationKeyTest(20)

    def test_Camera_Panorama_PressShutterKeyQuickly(self):
        self.cameraPressShutterKeyQuicklyPanoramaModeTest()

    def test_Camera_Panorama_ResolutionSetting(self):
        self.goThroughImageQualityAndPanoResolution()

    def test_Camera_Launch_FirstLaunchCamera_MarkLocation(self):
        self.markLocationTest()

    def test_Camera_Timer_Front_3s(self):
        self.capturePictureTimer("Front")

    def test_Camera_Timer_Rear_3s(self):
        self.capturePictureTimer("Back")

    def test_Camera_PhotoSphere_PressFinishButtonQuickly(self):
        self.cameraPhotoSpherePressFinishButtonQuicklyTest()

    def test_Camera_Panorama_LackStorage(self):
        self.cameraPanoramaLackStorageTest()

    def test_Camera_Panorama_LockAndUnlock(self):
        self.cameraPanoramaLockAndUnlockTest(20)

    def test_Camera_Panorama_PowerOffBySoftwareKey(self):
        self.cameraPanoramaPowerOffBySoftwareKeyTest()