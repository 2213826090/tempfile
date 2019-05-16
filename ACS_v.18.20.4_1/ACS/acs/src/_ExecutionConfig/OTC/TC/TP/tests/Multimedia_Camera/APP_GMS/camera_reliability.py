# coding: utf-8
import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.cameratestbase import CameraTestBase

class CameraReliabilityTest(CameraTestBase):
    """
    @summary: This test used to test camera function
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(CameraReliabilityTest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(CameraReliabilityTest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()
        time.sleep(3)

    def appPrepareWithClearData(self):
        self.camera = CameraCommon().switchPlatform()
        self.host_path = CameraCommon().getTmpDir()
        self.makefileTime = CameraCommon().makefileTime
        self.camera_dir = CameraCommon().camera_dir
        CameraCommon().removeDeivceFile()
        CameraCommon().removeFile(self.host_path + "/*")
        self.camera.cleanMediaFiles()
        CameraCommon().setOrientationToVertical()
        self.logger.debug("app prepare successfully")

    def checkFileCorrupt(self, mediaFileCount=1):
        CameraCommon().checkFileCorrupt(mediaFileCount)

    def quickClickShutterButtonArea(self, loop):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. Quick click shutter button area
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode("Panorama")
            x, y = self.camera.clickShutterBtnArea("Panorama")
            self.logger.debug("width = " + str(x) + ", height = " + str(y))
            for i in range(int(loop)):
                self.logger.debug("***** Loop " + str(i + 1) + " *****")
                CameraCommon().clickBtn(x, y)
                CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def switchAllCameraOptions(self, loop):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        1. Switch to camera options one by one
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            for i in range(int(loop)):
                self.logger.debug("***** Loop " + str(i + 1) + " *****")
                self.camera.selectMode("Video")
                self.camera.selectMode("Lens Blur")
                self.camera.selectMode("Panorama")
                self.camera.selectMode("Photo Sphere")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def switchToRearOrFrontCameraIteratively(self, loop):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. Switch to rear or front camera
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode("Lens Blur")
            for i in range(int(loop)):
                self.logger.debug("***** Loop " + str(i + 1) + " *****")
                CameraCommon().checkCameraCrash()
                self.camera.switchRearOrFront("Front")
                CameraCommon().checkCameraCrash()
                self.camera.switchRearOrFront("Back")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraSwitchFrontAndRearCameraLensBlurMode(self, loop):
        try:
            self.appPrepareWithClearData()
            for i in range(int(loop)):
                self.logger.debug("***** Loop " + str(i + 1) + " *****")
                self.camera.startCameraApp()
                self.camera.selectMode("Lens Blur")
                self.camera.switchRearOrFront("Front")
                self.camera.capturePhoto()
                time.sleep(10)
                self.camera.switchRearOrFront("Back")
                self.camera.capturePhoto(1,False)
                time.sleep(10)
                CameraCommon().waitForTheFilesAreGenerated(1)
            self.checkFileCorrupt(int(loop))
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraPressShutterKeyQuicklyLensBlurModeTest(self):
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode("Lens Blur")
            self.camera.switchRearOrFront("Back")
            x, y = self.camera.clickShutterBtnArea("Lens Blur")
            for i in range(20):
                CameraCommon().clickBtn(x, y)
                CameraCommon().checkCameraCrash()
            time.sleep(5)
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def test_Camera_LensBlur_PressShutterKeyQuickly(self):
        self.cameraPressShutterKeyQuicklyLensBlurModeTest()

    def test_Camera_Panorama_Iteration_QuickShutterPress_500cycles(self):
        self.quickClickShutterButtonArea(500)

    def test_Camera_Iteration_Switch_Mode_500cycles(self):
        self.switchAllCameraOptions(500)

    def test_Camera_LensBlur_SwitchFrontRear_5000cycles(self):
        self.switchToRearOrFrontCameraIteratively(5000)
 
    def test_Camera_LensBlur_SwitchFrontRear_500cycles(self):
        self.switchToRearOrFrontCameraIteratively(500)

    def test_Camera_LensBlur_SwitchFrontAndRear(self):
        self.cameraSwitchFrontAndRearCameraLensBlurMode(20)

