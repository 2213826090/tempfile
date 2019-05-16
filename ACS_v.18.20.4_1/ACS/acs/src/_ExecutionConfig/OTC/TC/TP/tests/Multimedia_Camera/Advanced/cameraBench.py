# coding: utf-8
import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.checkImage import CheckImage
from testlib.camera.checkVideo import CheckVideo
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.ArcSoftCamera import ArcSoftCamera
from testlib.camera.RefCam2Camera import RefCam2Camera
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.checkIQ import CheckIQ
from testlib.multimedia.multimedia_setting import MultiMediaSetting

class CameraBenchTest(CameraTestBase):
    """
    @summary: This test used to test camera function
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(CameraBenchTest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(CameraBenchTest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()
        time.sleep(3)

    def appPrepare(self):
        self.camera = CameraCommon().switchPlatform(None,False,True,False,True)
#         self.camera = RefCam2Camera()
        self.common = CameraCommon()
        self.multimedia_setting = MultiMediaSetting(CameraCommon.DEFAULT_CONFIG_FILE)
#         self.camera = self.common.switchPlatform()
        self.checkImage = CheckImage()
        self.video = CheckVideo()
#         self.checkIQ = CheckIQ()
        self.host_path = self.common.getTmpDir()
        self.makefileTime = self.common.makefileTime
        self.camera_dir = self.common.camera_dir
        self.common.removeDeivceFile()
        self.common.removeFile(self.host_path + "/*")
        self.camera.cleanMediaFiles()
#         self.common.setOrientationToVertical()
        self.logger.debug("app prepare successfully")
        self.device = self.common.initDevice()
        self.common.resetDevice(self.device)

    def checkFileCorrupt(self, mediaFileCount=1):
        return self.common.checkFileCorrupt(mediaFileCount)

    def cameraExtendPanoramaRearLeft2RightAutoStopTest(self):
        try:
            self.appPrepare()
#             self.common.controlLightBox(1)
#             self.common.controlLightBox(6)
#             device = self.common.initDevice()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront("Back")
            self.camera.setCaptureMode("Panorama")
            self.camera.capturePhoto(1, False)
            time.sleep(3)
            self.common.rotateDevice(self.device, -180)
            self.common.resetDevice(self.device)
            self.checkFileCorrupt()
#             self.common.controlLightBox(6)
#             self.common.controlLightBox(1)
        except Exception as e:
            self.common.resetDevice(self.device)
            self.common.checkCameraCrash()
            self.assertTrue(False, e)

    def cameraFaceTrackTurnOnAndOffTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            if lens == "Back":
                self.common.rotateDevice(self.device, -90)
            else:
                self.common.rotateDevice(self.device, 90)
            self.camera.faceDetectionOnAndOff()
            time.sleep(5)
            self.common.clickScreenCenter()
            self.common.getScreenshotAndPullToHost("sc_enable_ft1.png", self.host_path)
            self.camera.faceDetectionOnAndOff()
            time.sleep(5)
            self.common.clickScreenCenter()
            self.common.pressBack(2)
            self.camera.startCameraApp()
            self.common.getScreenshotAndPullToHost("sc_disable_ft2.png", self.host_path)
            self.camera.faceDetectionOnAndOff()
            time.sleep(5)
            self.common.clickScreenCenter()
            self.common.getScreenshotAndPullToHost("sc_enable_ft2.png", self.host_path)
            self.common.resetDevice(self.device)
            remotejarFile = self.multimedia_setting.download_file_to_host("Multimedia_Camera/apk/IQCheck")
            print remotejarFile
            iqcheckDir=self.host_path +os.sep + "IQCheck"
            print iqcheckDir
            if os.path.exists(iqcheckDir)==False:
                os.system("cp "+remotejarFile+" "+iqcheckDir)
            os.system("chmod 777 "+iqcheckDir)
            checkIQ = CheckIQ("", self.host_path)
            l = self.camera.getFOVRegion()
            region = CheckIQ.Region(l[0], l[1], l[2], l[3], False)
            enable_number1 = checkIQ.detectRect(self.host_path + "/sc_enable_ft1.png", region)
            disable_number = checkIQ.detectRect(self.host_path +"/sc_disable_ft2.png", region)
            enable_number2 = checkIQ.detectRect(self.host_path +"/sc_enable_ft2.png", region)
            if  enable_number1 > disable_number:
                self.logger.debug("face detect success")
            else:
                os.system("cp " + self.host_path + "/sc_enable_ft1.png " + g_common_obj.get_user_log_dir())
                os.system("cp " + self.host_path + "/sc_disable_ft2.png " + g_common_obj.get_user_log_dir())
                self.logger.debug("enable_number1=" + str(enable_number1))
                self.logger.debug("disable_number=" + str(disable_number))
                self.assertTrue(False, "face detect failure")
            if enable_number2 > disable_number:
                self.logger.debug("face detect success")
            else:
                os.system("cp " + self.host_path + "/sc_enable_ft1.png " + g_common_obj.get_user_log_dir())
                os.system("cp " + self.host_path + "/sc_disable_ft2.png " + g_common_obj.get_user_log_dir())
                self.logger.debug("enable_number2=" + str(enable_number2))
                self.logger.debug("disable_number=" + str(disable_number))
                self.assertTrue(False, "face detect failure")
#             print self.checkIQ.detectRect("/tmp/logs/sc_disable_ft2.png",self.checkIQ.Region(self.camera.getFOVRegion(),False))
#             print self.checkIQ.detectRect("/tmp/logs/sc_enable_ft2.png", self.checkIQ.Region(self.camera.getFOVRegion(), False))
        except Exception as e:
            self.common.resetDevice(self.device)
            self.common.checkCameraCrash()
            self.assertTrue(False, e)

    def cameraSceneDetectPortraitCaptureTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront("Back")
            self.camera.capturePhoto()
            info = self.checkFileCorrupt()[0]
            value = info.get("Scene Capture Type")
            if None != value:
                if value != "Standard":
                    self.assertTrue(False, "scene capture type is not standard")
            else:
                self.assertTrue(False, "scene capture type info is none")
            self.common.removeDeivceFile()
            self.camera.setCaptureMode("Smart")
            self.common.clickScreenCenter()
            self.camera.capturePhoto()
            info = self.checkFileCorrupt()[0]
            value = info.get("Scene Capture Type")
            if None != value:
                if value != "Portrait":
                    self.assertTrue(False, "scene capture type is not portrait")
                else:
                    self.logger.debug("check scene capture type is portrait success")
            else:
                self.assertTrue(False, "scene capture type info is none")
            self.camera.switchRearOrFront("Front")
            self.camera.capturePhoto()
            info = self.checkFileCorrupt()[0]
            value = info.get("Scene Capture Type")
            if None != value:
#                 print "=====================front value3=" + value
                if value != "Standard":
                    self.assertTrue(False, "scene capture type is not standard")
            else:
                self.assertTrue(False, "scene capture type info is none")
            self.common.removeDeivceFile()
            self.camera.setCaptureMode("Smart")
            self.common.clickScreenCenter()
            self.camera.capturePhoto()
            info = self.checkFileCorrupt()[0]
            value = info.get("Scene Capture Type")
            if None != value:
#                 print "====================front value4=" + value
                if value != "Portrait":
                    self.assertTrue(False, "scene capture type is not portrait")
                else:
                    self.logger.debug("check scene capture type is portrait success")
            else:
                self.assertTrue(False, "scene capture type info is none")
            self.common.resetDevice(self.device)
        except Exception as e:
            self.common.resetDevice(self.device)
            self.common.checkCameraCrash()
            self.assertTrue(False, e)

    def cameraSmileShotSmileDetectTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            if lens == "Back":
                self.common.rotateDevice(self.device, -90)
            else:
                self.common.rotateDevice(self.device, 90)
            self.camera.setCaptureMode("Smile")
            time.sleep(10)
            if self.common.checkDeviceFile():
                self.assertTrue(False, "file exists, check file failed")
            self.camera.capturePhoto()
            self.checkFileCorrupt()
            self.common.removeDeivceFile()
            if lens == "Back":
                self.common.rotateDevice(self.device, 90)
            else:
                self.common.rotateDevice(self.device, 90)
            if not self.common.checkDeviceFile():
                self.assertTrue(False, "file not exists, check file failed")
            self.checkFileCorrupt()
            self.common.resetDevice(self.device)
        except Exception as e:
            self.common.resetDevice(self.device)
            self.common.checkCameraCrash()
            self.assertTrue(False, e)

    def test_Camera_ExtendPanorama_Rear_Left2Right_AutoStop(self):
        self.cameraExtendPanoramaRearLeft2RightAutoStopTest()

    def test_Camera_FaceTrack_Front_TurnOnAndOff(self):
        self.cameraFaceTrackTurnOnAndOffTest("Front")

    def test_Camera_FaceTrack_Rear_TurnOnAndOff(self):
        self.cameraFaceTrackTurnOnAndOffTest("Back")

    def test_Camera_SceneDetect_Portrait_Capture(self):
        self.cameraSceneDetectPortraitCaptureTest()

    def test_Camera_SmileShot_Front_SmileDetect(self):
        self.cameraSmileShotSmileDetectTest("Front")

    def test_Camera_SmileShot_Rear_SmileDetect(self):
        self.cameraSmileShotSmileDetectTest("Back")
