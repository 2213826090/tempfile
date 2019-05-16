# coding: utf-8
import os
import time
import threading
from testlib.util.common import g_common_obj
from testlib.camera.checkImage import CheckImage
from testlib.camera.checkVideo import CheckVideo
from testlib.camera.cameratestbase import CameraTestBase
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.camera.ArcSoftCamera import ArcSoftCamera
from testlib.camera.CameraCommon import CameraCommon

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
        self.camera = ArcSoftCamera()
        self.checkImage = CheckImage()
        self.video = CheckVideo()
        self.host_path = CameraCommon().getTmpDir()
        self.makefileTime = CameraCommon().makefileTime
        self.camera_dir = CameraCommon().camera_dir
        CameraCommon().removeDeivceFile()
        CameraCommon().removeFile(self.host_path + "/*")
        self.camera.cleanMediaFiles()
        CameraCommon().setOrientationToVertical()
        self.multimedia_setting = MultiMediaSetting(CameraCommon.DEFAULT_CONFIG_FILE)
        self.multimedia_setting.install_apk("quickpic_apk")
        self.logger.debug("app prepare successfully")

    def checkFileCorrupt(self, mediaFileCount=1):
        return CameraCommon().checkFileCorrupt(mediaFileCount)

    def cameraInformationCameraTest(self, category, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.setCaptureModeValue(category, "low")
            self.camera.capturePhoto()
            info1 = self.checkFileCorrupt()[0]
            low = info1.get(category)
            CameraCommon().removeDeivceFile()
            self.logger.debug(low)
            if low != "Low":
                self.assertTrue(False, "picture %s info is not soft" % category)
            self.camera.setCaptureModeValue(category, "middle")
            self.camera.capturePhoto()
            info2 = self.checkFileCorrupt()[0]
            middle = info2.get(category)
            CameraCommon().removeDeivceFile()
            self.logger.debug(middle)
            if middle != "Normal":
                self.assertTrue(False, "picture %s info is not Normal" % category)
            self.camera.setCaptureModeValue(category, "high")
            self.camera.capturePhoto()
            info3 = self.checkFileCorrupt()[0]
            high = info3.get(category)
            CameraCommon().removeDeivceFile()
            self.logger.debug(high)
            if high != "High":
                self.assertTrue(False, "picture %s info is not high" % category)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraSharpnessInformationCameraTest(self, category, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.setCaptureModeValue(category, "low")
            self.camera.capturePhoto()
            info1 = self.checkFileCorrupt()[0]
            soft = info1.get(category)
            CameraCommon().removeDeivceFile()
            print soft
            if soft != "Soft":
                self.assertTrue(False, "picture %s info is not soft" % category)
            self.camera.setCaptureModeValue(category, "middle")
            self.camera.capturePhoto()
            info2 = self.checkFileCorrupt()[0]
            middle = info2.get(category)
            CameraCommon().removeDeivceFile()
            print middle
            if middle != "Normal":
                self.assertTrue(False, "picture %s info is not normal" % category)
            self.camera.setCaptureModeValue(category, "high")
            self.camera.capturePhoto()
            info3 = self.checkFileCorrupt()[0]
            hard = info3.get(category)
            CameraCommon().removeDeivceFile()
            print hard
            if hard != "Hard":
                self.assertTrue(False, "picture %s info is not hard" % category)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraBestShotRearPreviewTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront("Rear")
            self.camera.setCaptureMode("Best photo")
            self.camera.capturePhoto(1,False)
            self.camera.clickbestPhotoApproveBtn()
            CameraCommon().waitForTheFilesAreGenerated()
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraBestShotRearEXIFTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront("Back")
            self.camera.setCaptureMode("Best photo")
            self.camera.capturePhoto(1,False)
            self.camera.clickbestPhotoApproveBtn()
            CameraCommon().waitForTheFilesAreGenerated()
            info, file_name = self.checkFileCorrupt()
            CameraCommon().checkEXIFInfo(info, file_name)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraSetBrightnessEXIFTest(self, category, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.setCaptureModeValue(category, "low")
            self.camera.capturePhoto()
            info1, file_name1 = self.checkFileCorrupt()
            low = info1.get("Brightness Value")
            CameraCommon().removeDeivceFile()
            error_msg = ""
            if low != "0":
                os.system("cp " + self.host_path + "/" + file_name1 + " " + g_common_obj.get_user_log_dir())
                error_msg += "check low brightness value fail,low brightness value=%s" % low
            self.camera.setCaptureModeValue(category, "middle")
            self.camera.capturePhoto()
            info2, file_name2 = self.checkFileCorrupt()
            middle = info2.get("Brightness Value")
            CameraCommon().removeDeivceFile()
            if middle != "0":
                os.system("cp " + self.host_path + "/" + file_name2 + " " + g_common_obj.get_user_log_dir())
                error_msg += "check normal brightness value fail,normal brightness value=%s" % middle
            self.camera.setCaptureModeValue(category, "high")
            self.camera.capturePhoto()
            info3, file_name3 = self.checkFileCorrupt()
            high = info3.get("Brightness Value")
            CameraCommon().removeDeivceFile()
            if high != "0":
                os.system("cp " + self.host_path + "/" + file_name3 + " " + g_common_obj.get_user_log_dir())
                error_msg += "check high brightness value fail,high brightness value=%s" % high
            if low == middle or low == high or middle == high:
                os.system("cp " + self.host_path + "/" + file_name3 + " " + g_common_obj.get_user_log_dir())
                os.system("cp " + self.host_path + "/" + file_name3 + " " + g_common_obj.get_user_log_dir())
                os.system("cp " + self.host_path + "/" + file_name3 + " " + g_common_obj.get_user_log_dir())
                error_msg += "compare %s camera brightness value fail,low=%s,normal=%s,high=%s;" \
                                % (lens, low, middle, high)
                self.assertTrue(False, error_msg)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraUniPreviewTapShutterWhenRecordingTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.snapShotDuringVideo(1, 5, 1)
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraUniPreviewVideoRecordTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.clickRecordBtn()
            self.camera.checkVideoRecordingButtonStatus()
            time.sleep(5)
            self.camera.clickRecordBtn()
            self.checkFileCorrupt()
            self.camera.reviewPhotoAndVideo()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraUniPreviewVideoRecordOtherModeTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.setCaptureMode("Smart")
            self.camera.clickRecordBtn()
            time.sleep(2)
            self.camera.checkVideoRecordingButtonStatus()
            time.sleep(3)
            self.camera.clickRecordBtn()
            CameraCommon().waitForTheFilesAreGenerated(1)
            self.camera.setCaptureMode("Best photo")
            self.camera.clickRecordBtn()
            time.sleep(2)
            self.camera.checkVideoRecordingButtonStatus()
            time.sleep(3)
            self.camera.clickRecordBtn()
            CameraCommon().waitForTheFilesAreGenerated(2)
            if CameraCommon().isWidgetExists(self.camera.ArcSoftCameraWidget().text("HDR")):
                self.camera.setCaptureMode("HDR")
                self.camera.clickRecordBtn()
                time.sleep(2)
                self.camera.checkVideoRecordingButtonStatus()
                time.sleep(3)
                self.camera.clickRecordBtn()
                CameraCommon().waitForTheFilesAreGenerated(3)
            self.camera.setCaptureMode("Smile")
            self.camera.clickRecordBtn()
            time.sleep(2)
            self.camera.checkVideoRecordingButtonStatus()
            time.sleep(3)
            self.camera.clickRecordBtn()
            CameraCommon().waitForTheFilesAreGenerated(4)
            self.camera.reviewPhotoAndVideo()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraBestShotRearIterativeCaptureTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.setCaptureMode("Best photo")
            for i in range(20):
                self.camera.capturePhoto()
                self.camera.clickbestPhotoApproveBtn()
                CameraCommon().waitForTheFilesAreGenerated(i + 1)
            self.camera.reviewPhotoAndVideo()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraRestoreDefaultTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront("Rear")
            self.camera.setColorEffect("Mono")
            self.camera.capturePhoto()
            file_name = self.checkFileCorrupt()[1]
            self.color_effect = self.checkImage.is_mono(self.host_path + "/" + file_name)
            if not self.color_effect:
                os.system("cp " + self.host_path + "/" + file_name + " " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "check Rear camera color effect to mono fail")
            CameraCommon().removeDeivceFile()
            self.camera.resetCameraSetting()
            self.camera.capturePhoto()
            file_name2 = self.checkFileCorrupt()[1]
            self.color_effect = self.checkImage.is_mono(self.host_path + "/" + file_name2)
            if self.color_effect:
                os.system("cp " + self.host_path + "/" + file_name + " " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "check Rear camera color effect to mono fail")
            self.camera.reviewPhotoAndVideo()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraUniPreviewTapOtherButtonTest(self,lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
 
            def clickRecordingButton(self,rtime=""):
                print time.time()
                ArcSoftCamera().clickShutterBtnWithoutRelease("Video", 4)
                ArcSoftCamera().recordVideo(1, 3)

            def clickCaptureButton(self, rtime=""):
                print time.time()
                time.sleep(2)
                ArcSoftCamera().capturePhoto()
                ArcSoftCamera().clickShutterBtnWithoutRelease("Camera", 3)
 
            threads=[]
            t1 = threading.Thread(target=clickRecordingButton,args=('',))
            threads.append(t1)
            t2 = threading.Thread(target=clickCaptureButton,args=('',))
            threads.append(t2)
 
            for t in threads:
                t.start()
            t.join()
            print time.time()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def test_Camera_BestShot_Rear_Preview(self):
        self.cameraBestShotRearPreviewTest()

    def test_Camera_SetSaturation_Front_EXIF(self):
        self.cameraInformationCameraTest("Saturation", "Front")

    def test_Camera_SetSaturation_Rear_EXIF(self):
        self.cameraInformationCameraTest("Saturation", "Back")

    def test_Camera_SetSharpness_Front_EXIF(self):
        self.cameraSharpnessInformationCameraTest("Sharpness", "Front")

    def test_Camera_SetSharpness_Rear_EXIF(self):
        self.cameraSharpnessInformationCameraTest("Sharpness", "Back")

    def test_Camera_SetContrast_Front_EXIF(self):
        self.cameraInformationCameraTest("Contrast", "Front")

    def test_Camera_SetContrast_Rear_EXIF(self):
        self.cameraInformationCameraTest("Contrast", "Back")

    def test_Camera_BestShot_Rear_EXIF(self):
        self.cameraBestShotRearEXIFTest()

    def test_Camera_SetBrightness_Front_EXIF(self):
        self.cameraSetBrightnessEXIFTest("Brightness", "Front")

    def test_Camera_SetBrightness_Rear_EXIF(self):
        self.cameraSetBrightnessEXIFTest("Brightness", "Back")

    def test_Camera_UniPreview_Front_TapShutterWhenRecording(self):
        self.cameraUniPreviewTapShutterWhenRecordingTest("Front")

    def test_Camera_UniPreview_Rear_TapShutterWhenRecording(self):
        self.cameraUniPreviewTapShutterWhenRecordingTest("Back")

    def test_Camera_UniPreview_Front_VideoRecord_AfterLaunch(self):
        self.cameraUniPreviewVideoRecordTest("Front")

    def test_Camera_UniPreview_Rear_VideoRecord_AfterLaunch(self):
        self.cameraUniPreviewVideoRecordTest("Back")

    def test_Camera_UniPreview_Front_VideoRecord_OtherMode(self):
        self.cameraUniPreviewVideoRecordOtherModeTest("Front")

    def test_Camera_UniPreview_Rear_VideoRecord_OtherMode(self):
        self.cameraUniPreviewVideoRecordOtherModeTest("Back")

    def test_Camera_BestShot_Rear_IterativeCapture(self):
        self.cameraBestShotRearIterativeCaptureTest("Back")

    def test_Camera_RestoreDefault(self):
        self.cameraRestoreDefaultTest()

    def test_Camera_UniPreview_Front_TapOtherButton(self):
        self.cameraUniPreviewTapOtherButtonTest("Front")

