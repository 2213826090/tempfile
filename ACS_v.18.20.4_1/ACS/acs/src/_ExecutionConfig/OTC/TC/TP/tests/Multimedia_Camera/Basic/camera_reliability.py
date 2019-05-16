# coding: utf-8
# import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.CameraCommon import CameraCommon
from testlib.common.common import g_common_obj2


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

    def stillCaptureWithCleanData(self, mode, lens, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. Auto focus then still image capture
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            self.camera.switchRearOrFront(lens)
            self.camera.capturePhoto(num)
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def stillCaptureAndCheckFile(self, mode, lens, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. Auto focus then still image capture
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            self.camera.switchRearOrFront(lens)
            self.camera.capturePhoto(num)
            time.sleep(2)
            self.checkFileCorrupt(num)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def goThroughAllSettings(self, mode, lens, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. change the camera setting
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            self.camera.switchRearOrFront(lens)
            for i in range(int(num)):
                self.logger.debug("******** Loop " + str(i + 1) + " ********")
                self.camera.clickAllSetting()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def terminateCameraWhenRecording(self, mode, lens, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera from app launcher
        2. Switch to video mode
        3. Switch to back/front camera
        4. Start to record video
        5. Teminate camera in Recent Tasks when recording
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            self.camera.switchRearOrFront(lens)
            self.camera.clickRecordBtn()
            CameraCommon().checkCameraCrash()
            time.sleep(2)
            CameraCommon().pressBack(2)
            for i in range(int(num)):
                self.logger.debug("******** Loop " + str(i + 1) + " ********")
                self.camera.startCameraApp()
                self.camera.clickRecordBtn()
                CameraCommon().checkCameraCrash()
                time.sleep(2)
                CameraCommon().removeRecentApp()
            self.checkFileCorrupt(num)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def launchCameraTeminateRecent(self, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera from app launcher
        2. Terminate the camera from recent tasks
        """
        try:
            self.appPrepareWithClearData()
            for i in range(int(num)):
                self.logger.debug("******** Loop " + str(i + 1) + " ********")
                self.camera.startCameraApp()
                CameraCommon().checkCameraCrash()
                CameraCommon().removeRecentApp()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def launchCameraFromHome(self, case_name):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera from am command
        """
        try:
            self.logger.debug("run case is " + str(case_name))
            self.appPrepareWithClearData(case_name)
            for i in range(int(self.camera.cfg.get("loop_num"))):
                self.logger.debug("******** Loop " + str(i) + " ********")
                self.camera.enter_camera_from_home()
                self.camera.judge_if_camera_crash()
        except Exception as e:
            self.camera.judge_if_camera_crash()
            self.assertTrue(False, e)

    def launchCameraFromApp(self, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera from app launcher
        """
        try:
            self.appPrepareWithClearData()
            for i in range(int(num)):
                self.logger.debug("******** Loop " + str(i + 1) + " ********")
                self.camera.startCameraApp()
                CameraCommon().pressBack()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def launchCameraFromRecentTasks(self, mode, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera from recent tasks
        """
        try:
            self.appPrepareWithClearData()
            CameraCommon().removeAllAppFromRecentTask()
            self.camera.startCameraApp()
            CameraCommon().pressBack(2)
            for i in range(int(num)):
                self.logger.debug("******** Loop " + str(i + 1) + " ********")
                CameraCommon().enterAppFromRecentTask()
                CameraCommon().pressBack(2)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def manualAllExposureThenImageCapture(self, mode, lens, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. open manual exposure
        3. set exposure then capture
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            self.camera.switchRearOrFront(lens)
            self.camera.openManualExposure()
            for i in range(int(num)):
                self.logger.debug("******** Loop " + str(i + 1) + " ********")
                self.camera.setExposure("+2")
                self.camera.capturePhoto()
                self.camera.setExposure("+1")
                self.camera.capturePhoto()
                self.camera.setExposure("0")
                self.camera.capturePhoto()
                self.camera.setExposure("-1")
                self.camera.capturePhoto()
                self.camera.setExposure("-2")
                self.camera.capturePhoto()
            self.checkFileCorrupt(num * 5)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

#     def clickScreenAnyAreaThenStillImageCapture(self, case_name):
#         """
#         This test used to test camera
#         The test case spec is following:
#         1. Launch camera
#         2. Manual focus then still image capture
#         """
#         try:
#             self.appPrepareWithClearData(case_name)
#             self.camera.enter_camera_from_home()
#             self.camera.switch_to_camera_options(self.camera.cfg.get("options"))
#             self.camera.change_front_back_camera(self.camera.cfg.get("back_or_front"))
#             for i in range(int(self.camera.cfg.get("loop_num"))):
#                 self.logger.debug("******** Loop " + str(i) + " ********")
#                 self.camera.click_screen_any_area()
#                 time.sleep(2)
#                 self.camera.removeDeivceFile()
#                 self.camera.capture_photo()
#                 time.sleep(2)
#                 self.checkFileCorrupt()
#         except Exception as e:
#             CameraCommon().checkCameraCrash()
#             self.assertTrue(False, e)

    def quickClickShutterButtonArea(self, mode, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. Quick click shutter button area
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            time.sleep(2)
            mx, my = self.camera.clickShutterBtnArea(mode)
            self.logger.debug("width = " + str(mx) + ", height = " + str(my))
            for i in range(int(num)):
                self.logger.debug("******** Loop " + str(i + 1) + " ********")
                CameraCommon().clickBtn(mx, my)
                self.logger.debug("click " + str(mx) + ", " + str(my))
                CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def startCounterAndStopCapture(self, case_name):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. capture image
        """
        try:
            self.logger.debug("run case is " + str(case_name))
            self.appPrepareWithClearData(case_name)
            self.camera.enter_camera_from_home()
            self.camera.switch_module_in_camera(self.camera.cfg.get("options"))
            self.camera.change_front_back_camera(self.camera.cfg.get("back_or_front"))
            self.camera.counter_time()
            for i in range(int(self.camera.cfg.get("loop_num"))):
                self.logger.debug("********Loop " + str(i) + " ********")
                self.camera.capture_photo()
                self.camera.stop_counter_time()
                time.sleep(2)
        except Exception as e:
            self.camera.judge_if_camera_crash()
            self.assertTrue(False, e)

    def changeCameraSizeThenCapture(self, mode, lens, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. capture image
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            self.camera.switchRearOrFront(lens)
            mDict = self.camera.getAllPhotoResolutions(lens)[0]
            for i in range(int(num)):
                self.logger.debug("********Loop " + str(i + 1) + " ********")
                for k in mDict:
                    self.camera.setPhotoResolution(mDict[k], lens)
                    self.camera.capturePhoto()
            self.checkFileCorrupt(num * len(mDict))
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def longLastingImageCaptureMaxResolution(self, lens, wait_time):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. capture image
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront(lens)
            mDict, mMax, mMin = self.camera.getAllPhotoResolutions(lens)
#             print mMax,mMin
            self.camera.setPhotoResolution(mMax, lens)
            timeRecord = time.time()
            i = 0
            while time.time() - timeRecord < float(wait_time):
                i = i + 1
                self.logger.debug("******** Loop " + str(i + 1) + " ********")
                self.camera.capturePhoto()
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo(5)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraSwitchCaptureUntilStorageFull(self, lens):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. capture image
        """
        try:
            self.appPrepareWithClearData()
            CameraCommon().fillStorage(130)
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode(lens)
            mTime = time.time()
            while True:
                self.camera.switchRearOrFront("Front")
                self.camera.capturePhoto()
                self.camera.switchRearOrFront("Back")
                self.camera.capturePhoto()
                time.sleep(1)
                free = CameraCommon().getSdcardMemory()[2]
                if free < 70:
                    self.logger.debug("The memory is full")
                    break
                if time.time() - mTime > 5400:
                    break
        except Exception as e:
            CameraCommon().checkCameraCrash()
            CameraCommon().removeDeivceFile()
            self.assertTrue(False, e)

    def cameraCaptureUntilStorageFull(self, mode, lens):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. capture image
        """
        try:
            self.appPrepareWithClearData()
            CameraCommon().fillStorage(130)
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            self.camera.switchRearOrFront(lens)
            i = 0
            mTime = time.time()
            while True:
                i = i + 1
                self.logger.debug("******** Loop " + str(i + 1) + " ********")
                time.sleep(1)
                self.camera.capturePhoto()
                total, used, free = CameraCommon().getSdcardMemory()
                if free < 70:
                    self.logger.debug("The memory is full")
                    break
                if time.time() - mTime > 5400:
                    break
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def switchToRearOrFrontCameraIteratively(self, mode, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. Switch to rear or front camera
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            for i in range(int(num)):
                self.logger.debug("******** Loop " + str(i + 1) + " ********")
                CameraCommon().checkCameraCrash()
                self.camera.switchRearOrFront("Back")
                time.sleep(1.5)
                CameraCommon().checkCameraCrash()
                self.camera.switchRearOrFront("Front")
            CameraCommon().removeDeivceFile()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def snapshotWhenRecording(self, lens, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. Test video recording start and stop
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront(lens)
            self.camera.snapShotDuringVideo(1, 5, num)
            self.checkFileCorrupt(num + 1)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def changeVideoSizeRecording(self, mode, lens, num):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. Test video recording start and stop
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            self.camera.switchRearOrFront(lens)
            mDict = self.camera.getAllVideoResolutions(lens)[0]
            for i in range(int(num)):
                self.logger.debug("******** Loop " + str(i + 1) + " ********")
                for k in mDict:
                    print mDict[k]
                    self.camera.setVideoResolution(mDict[k], lens)
                    self.camera.recordVideo(1, 10)
                self.checkFileCorrupt()
                CameraCommon().removeDeivceFile()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def longLastingVideoRecording(self, lens, duration):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. Test video recording start and stop
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront(lens)
            self.camera.recordVideo(1, duration)
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def videoRecordingIteration(self, lens, videoNum):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. Test video recording start and stop
        """
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront(lens)
            time.sleep(1)
            for i in range(int(videoNum)):
                self.logger.debug("******** Loop " + str(i + 1) + " ********")
                self.camera.recordVideo(1, 1)
                time.sleep(1)
                CameraCommon().checkCameraCrash()
            self.checkFileCorrupt(videoNum)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraFullStorageRecordModeTest(self):
        try:
            self.appPrepareWithClearData()
            CameraCommon().fillStorage(25)
            CameraCommon().unlockScreen()
            self.camera.startCameraApp()
            CameraCommon().checkCameraCrash()
            total, used, free = CameraCommon().getSdcardMemory()
            if free > 70:
                self.assertTrue(False, "since memory is not full")
            CameraCommon().removeBigFile()
            CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            CameraCommon().removeBigFile()
            self.assertTrue(False, e)

    def cameraLackStorageVideoModeTest(self):
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Back")
            CameraCommon().fillStorage(300)
            CameraCommon().unlockScreen()
            for i in range(5):
                self.camera.startCameraApp()
                CameraCommon().pressBack()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront("Front")
            for i in range(5):
                self.camera.startCameraApp()
                CameraCommon().pressBack()
            CameraCommon().removeBigFile()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            CameraCommon().removeBigFile()
            self.assertTrue(False, e)

    def cameraPressShutterKeyQuicklyVideoModeTest(self):
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode("Video")
            self.camera.switchRearOrFront("Back")
            x, y = self.camera.clickShutterBtnArea()
            for i in range(200):
                CameraCommon().clickBtn(x, y)
                time.sleep(0.2)
                CameraCommon().clickBtn(x, y)
                time.sleep(0.2)
            CameraCommon().checkCameraCrash()
            if self.camera.isRecordTimeExists():
                self.camera.clickRecordBtn()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraRebootDUTLaunchCamera20TimesTest(self):
        try:
            for i in range(20):
                g_common_obj2.system_reboot(90)
                CameraCommon().unlockScreen()
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            self.camera.capturePhoto()
            self.camera.selectMode("Video")
            self.camera.recordVideo()
            if self.camera.checkModeExists("Panorama"):
                self.camera.selectMode("Panorama")
                self.camera.capturePhoto(1,False)
                self.camera.capturePhoto()
            if self.camera.checkModeExists("Lens Blur"):
                self.camera.selectMode("Lens Blur")
                self.camera.capturePhoto()
            if self.camera.checkModeExists("Photo Sphere"):
                self.camera.selectMode("Photo Sphere")
                self.camera.capturePhoto()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraPreviewLockAndUnlock30minsTest(self,sleep_time):
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            CameraCommon().pressPower()
            time.sleep(sleep_time)
            CameraCommon().unlockScreen()
            CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraTorchModeTest(self,sleep_time):
        try:
            self.appPrepareWithClearData()
            CameraCommon().pressHome()
            CameraCommon().flashlightOperation()
            time.sleep(sleep_time)
            CameraCommon().flashlightOperation()
            CameraCommon().pressBack(2)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraSwitchQuickViewAndFOV20timesTest(self):
        try:
            self.appPrepareWithClearData()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            self.camera.capturePhoto(3)
            self.camera.selectMode("Video")
            self.camera.recordVideo(2)
            if self.camera.checkModeExists("Panorama"):
                self.camera.selectMode("Panorama")
                self.camera.capturePhoto(1,False)
                self.camera.capturePhoto()
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo()
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
            if "cht" in CameraCommon().getPlatform():
                cht_skip_flag = 1
            else:
                cht_skip_flag = 0
            for i in range(int(loop)):
                self.logger.debug("***** Loop " + str(i + 1) + " *****")
                self.camera.selectMode("Camera")
                self.camera.selectMode("Video")
                if cht_skip_flag == 1:
                    continue
                self.camera.selectMode("Lens Blur")
                self.camera.selectMode("Panorama")
                self.camera.selectMode("Photo Sphere")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def test_Camera_Rear_Capture_Iteration_5000cycles(self):
        """
        This test used to test auto focus then still image capture
        The test case spec is following:
        1. Launch camera
        2. still image capture
        """
        self.stillCaptureWithCleanData("Camera", "Back", 5000)

    def test_Camera_Rear_Capture_Iteration_500cycles(self):
        """
        This test used to test auto focus then still image capture
        The test case spec is following:
        1. Launch camera
        2. still image capture
        """
        self.stillCaptureAndCheckFile("Camera", "Back", 500)

    def test_Camera_TraverseSetting_Iteration_200cycles(self):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. change the camera setting
        """
        self.goThroughAllSettings("Camera", "Front", 200)

    def test_Camera_VideoMode_Front_TerminateInRecentAppDuringRecord_Iteration_500cycles(self):
        """
        This test used to test launch camera
        1. Launch camera from app launcher
        2. Switch to video mode
        3. Switch to front camera
        4. Start to record video
        5. Teminate camera in Recent Tasks when recording
        """
        self.terminateCameraWhenRecording("Video", "Front", 500)

    def test_Camera_Front_TerminateInRecentAppAfterLaunch_500cycles(self):
        """
        This test used to test launch camera
        The test case spec is following:
        1. Launch camera from app launcher
        2. Terminate the camera from recent tasks
        """
        self.launchCameraTeminateRecent(500)

    def test_Camera_VideoMode_Rear_TerminateInRecentAppDuringRecording_Iteration_500cycles(self):
        """
        This test used to test launch camera
        1. Launch camera from app launcher
        2. Switch to video mode
        3. Switch to back camera
        4. Start to record video
        5. Teminate camera in Recent Tasks when recording
        """
        self.terminateCameraWhenRecording("Video", "Back", 500)

    def test_Camera_LaunchCameraViaAppIcon_Iteration_500cycles(self):
        """
        This test used to test launch camera
        The test case spec is following:
        1. Launch camera from app launcher
        """
        self.launchCameraFromApp(500)

    def test_Camera_LaunchCameraViaTaskManager_Iteration_500cycles(self):
        """
        This test used to test launch camera
        The test case spec is following:
        1. Launch camera from recent tasks
        """
        self.launchCameraFromRecentTasks("Camera", 500)

    def test_Camera_ExposureBias_Front_ExposureSwitchCapture_100cycles(self):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. open manual exposure
        3. set exposure then capture
        """
        self.manualAllExposureThenImageCapture("Camera", "Front", 100)

    def test_Camera_ExposureBias_Rear_ExposureSwitchCapture_100cycles(self):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. capture image
        """
        self.manualAllExposureThenImageCapture("Camera", "Back", 100)

#     def testCameraCapture_Iteration_ClickScreenCaptureImage_500cycles(self):
#         """
#         This test used to test manual focus then still image capture
#         The test case spec is following:
#         1. Launch camera
#         2. Manual focus then still image capture
#         """
#         self.clickScreenAnyAreaThenStillImageCapture('mum_camera_capture_054')

    def test_Camera_QuickPressShutter_CaptureMode_Iteration_500cycles(self):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. Quick click shutter button area
        """
        self.quickClickShutterButtonArea("Camera", 500)

    def test_Camera_VideoMode_QuickShutterPress_Iteration_500cycles(self):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. Quick click shutter button area
        """
        self.quickClickShutterButtonArea("Video", 500)

    def test_Camera_Front_Capture_Iteration_5000cycles(self):
        """
        This test used to test auto focus then still image capture
        The test case spec is following:
        1. Launch camera
        2. still image capture
        """
        self.stillCaptureWithCleanData("Camera", "Front", 5000)

    def test_Camera_Front_Capture_Iteration_500cycles(self):
        """
        This test used to test auto focus then still image capture
        The test case spec is following:
        1. Launch camera
        2. Still image capture
        """
        self.stillCaptureAndCheckFile("Camera", "Front", 500)

    def test_Camera_Front_ResolutionSwitchCapture_Iteration_500cycles(self):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. capture image
        """
        self.changeCameraSizeThenCapture("Camera", "Front", 500)

    def test_Camera_Rear_ResolutionSwitchCapture_Iteration_500cycles(self):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. capture image
        """
        self.changeCameraSizeThenCapture("Camera", "Back", 500)

    def test_Camera_Front_Capture_Longlasting_MaxResolution_8hrs(self):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. capture image
        """
        self.longLastingImageCaptureMaxResolution("Front", 28800)

    def test_Camera_Rear_Capture_Longlasting_MaxResolution_8hrs(self):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. capture image
        """
        self.longLastingImageCaptureMaxResolution("Back", 28800)

    def test_Camera_Front_Rear_SwitchCapture_Iteration_UntilStorageFull(self):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. capture image
        """
        self.cameraSwitchCaptureUntilStorageFull("Camera")

    def test_Camera_Rear_Capture_Iteration_UntilStorageFull(self):
        """
        This test used to test camera
        The test case spec is following:
        1. Launch camera
        2. capture image
        """
        self.cameraCaptureUntilStorageFull("Camera", "Back")

    def test_Camera_SwitchFrontRear_Iteration_5000cycles(self):
        """
        This test used to test switch to rear camera or front camera
        The test case spec is following:
        1. Launch camera
        2. Switch to rear or front camera
        """
        self.switchToRearOrFrontCameraIteratively("Camera", 5000)

    def test_Camera_CaptureMode_SwitchFrontRear_Iteration_500cycles(self):
        """
        This test used to test switch to rear camera or front camera
        The test case spec is following:
        1. Launch camera
        2. Switch to rear or front camera
        """
        self.switchToRearOrFrontCameraIteratively("Camera", 500)

    def test_Camera_VideoMode_SwitchFrontRear_Iteration_5000cycles(self):
        """
        This test used to test switch to rear camera or front camera
        The test case spec is following:
        1. Launch camera
        2. Switch to rear or front camera
        """
        self.switchToRearOrFrontCameraIteratively("Video", 5000)

    def test_Camera_VideoMode_SwitchFrontRear_Iteration_500cycles(self):
        """
        This test used to test switch to rear camera or front camera
        The test case spec is following:
        1. Launch camera
        2. Switch to rear or front camera
        """
        self.switchToRearOrFrontCameraIteratively("Video", 500)

    def test_Camera_Snapshot_Front_Iteration_500cycles(self):
        """
        This test used to test video recording start and stop
        The test case spec is following:
        1. Launch camera
        2. Test video recording start and stop
        """
        self.snapshotWhenRecording("Front", 500)

    def test_Camera_Snapshot_Rear_Iteration_500cycles(self):
        """
        This test used to test video recording start and stop
        The test case spec is following:
        1. Launch camera
        2. Test video recording start and stop
        """
        self.snapshotWhenRecording("Back", 500)

    def test_Camera_VideoMode_Front_ResolutionSwitchRecord_10s_Iteration_500cycles(self):
        """
        This test used to test video recording start and stop
        The test case spec is following:
        1. Launch camera
        2. Test video recording start and stop
        """
        self.changeVideoSizeRecording("Video", "Front", 500)

    def test_Camera_VideoMode_Rear_SwitchResolutionRecord_10s_Iteration_500cycles(self):
        """
        This test used to test video recording start and stop
        The test case spec is following:
        1. Launch camera
        2. Test video recording start and stop
        """
        self.changeVideoSizeRecording("Video", "Back", 500)

    def test_Camera_VideoMode_Front_Longlasting_2hrs(self):
        """
        This test used to test video recording start and stop
        The test case spec is following:
        1. Launch camera
        2. Test video recording start and stop
        """
        self.longLastingVideoRecording("Front", 7200)

    def test_Camera_VideoMode_Rear_Longlasting_2hrs(self):
        """
        This test used to test video recording start and stop
        The test case spec is following:
        1. Launch camera
        2. Test video recording start and stop
        """
        self.longLastingVideoRecording("Back", 7200)

    def test_Camera_VideoMode_Front_ShortVideo_1s_Iteration_100cycles(self):
        """
        This test used to test video recording start and stop
        The test case spec is following:
        1. Launch camera
        2. Test video recording start and stop
        """
        self.videoRecordingIteration("Front", 100)

    def test_Camera_VideoMode_Rear_ShortVideo_1s_Iteration_100cycles(self):
        """
        This test used to test video recording start and stop
        The test case spec is following:
        1. Launch camera
        2. Test video recording start and stop
        """
        self.videoRecordingIteration("Back", 100)

    def test_Camera_VideoMode_FullStorage(self):
        self.cameraFullStorageRecordModeTest()

    def test_Camera_VideoMode_LackStorage_VideoMode(self):
        self.cameraLackStorageVideoModeTest()

    def test_Camera_VideoMode_PressShutterKeyQuickly(self):
        self.cameraPressShutterKeyQuicklyVideoModeTest()

    def test_Camera_RebootDUT_LaunchCamera_20Times(self):
        self.cameraRebootDUTLaunchCamera20TimesTest()

    def test_Camera_Preview_LockAndUnlock_30mins(self):
        self.cameraPreviewLockAndUnlock30minsTest(1800)

    def test_Camera_TorchMode_On10Min(self):
        self.cameraTorchModeTest(60)

    def test_Camera_switch_QuickViewAndFOV_20times(self):
        self.cameraSwitchQuickViewAndFOV20timesTest()

    def test_Camera_Iteration_Switch_Mode_500cycles(self):
        self.switchAllCameraOptions(500)
