# coding: utf-8
import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.checkImage import CheckImage
from testlib.camera.checkVideo import CheckVideo
from testlib.camera.cameratestbase import CameraTestBase
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

    def appPrepare(self, mode_option=""):
#         self.camera = ArcSoftCamera()
#         self.multimedia_setting = MultiMediaSetting(CameraCommon.DEFAULT_CONFIG_FILE)
        self.camera = CameraCommon().switchPlatform(None, False, True, False, True, mode_option)
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

    def setColorEffectNoneCaptureImageCameraTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.setColorEffect("None")
            self.camera.capturePhoto()
            name1 = self.checkFileCorrupt()[1]
            image1_path = self.host_path + "/" + name1
            CameraCommon().removeDeivceFile()
            self.camera.setColorEffect("Negative")
            self.camera.capturePhoto()
            name2 = self.checkFileCorrupt()[1]
            image2_path = self.host_path + "/" + name2
            CameraCommon().removeDeivceFile()
            self.color_effect = self.checkImage.is_negative(image1_path, image2_path)
            self.camera.setColorEffect("None")
            self.camera.capturePhoto()
            name3 = self.checkFileCorrupt()[1]
            image3_path = self.host_path + "/" + name3
            self.color_effect2 = self.checkImage.is_negative(image2_path, image3_path)
            print self.color_effect, self.color_effect2
            if not (self.color_effect and self.color_effect2):
                os.system("cp " + self.host_path + "/" + name1 + " " + g_common_obj.get_user_log_dir())
                os.system("cp " + self.host_path + "/" + name2 + " " + g_common_obj.get_user_log_dir())
                os.system("cp " + self.host_path + "/" + name3 + " " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "check color effect to none fail")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraContinuousExitRelaunchTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            for i in range(5):
                self.camera.switchRearOrFront(lens)
                self.camera.clickShutterBtnWithoutRelease("Camera", 5)
                CameraCommon().pressHome()
                self.camera.startCameraApp()
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraContinuousMaxoneTimeTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.clickShutterBtnWithoutRelease("Camera")
            CameraCommon().checkCameraCrash()
            self.checkFileCorrupt(99)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def setColorEffectMonoCaptureImageCameraTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.setColorEffect("Mono")
            self.camera.capturePhoto()
            name1 = self.checkFileCorrupt()[1]
            image1_path = self.host_path + "/" + name1
            self.camera.capturePhoto()
            self.is_mono = self.checkImage.is_mono(image1_path)
            if not self.is_mono:
                os.system("cp " + self.host_path + "/" + name1 + " " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "check camera color effect to mono fail")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def setColorEffectNEGATIVECaptureImageCameraTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.capturePhoto()
            name1 = self.checkFileCorrupt()[1]
            image1_path = self.host_path + "/" + name1
            CameraCommon().removeDeivceFile()
            self.camera.setColorEffect("Negative")
            self.camera.capturePhoto()
            name2 = self.checkFileCorrupt()[1]
            image2_path = self.host_path + "/" + name2
            self.color_effect = self.checkImage.is_negative(image1_path, image2_path)
            if not self.color_effect:
                os.system("cp " + self.host_path + "/" + name1 + " " + g_common_obj.get_user_log_dir())
                os.system("cp " + self.host_path + "/" + name2 + " " + g_common_obj.get_user_log_dir())
                self.assertTrue(False, "compare %s camera color effect to negative fail" % lens)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraHDRCaptureTest(self, lens):
        try:
            self.appPrepare("HDR")
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.setCaptureMode("HDR")

            for t_type in ["YUV_420_888", "JPEG"]:
                self.camera.setSettingsButton("Source Format(PP)", t_type)

                for t_type in ["Capture Size (JPEG)", "Capture Size (YUV)"]:
                    self.logger.debug("type=%s" % t_type)
                    size_list, max_size, min_size = self.camera.getAllPhotoResolutions(lens, t_type)
                    if max_size == -1:
                        self.logger.debug("No \"%s\" type! skip!" % t_type)
                        continue
                    for size in size_list[0], size_list[1], min_size:
                        self.logger.debug("size=%s" % size)
                        self.camera.setPhotoResolution(size, lens, t_type)
                        self.camera.capturePhoto()
                        time.sleep(3)
                        info, fileName = self.checkFileCorrupt()
                        self.logger.debug("info: %s" % info)
                        width = info.get("Image Width")
                        height = info.get("Image Height")
                        size = info.get("Image Size")
                        self.logger.debug(str(width) + ", " + str(height) + ", " + str(size))
                        self.assertTrue(size == width + "x" + height, "image size is null")
                        exposure_time = info.get("Exposure Time")
                        self.logger.debug("Exposure Time: %s" % exposure_time)
                        error_msg = ""
                        if exposure_time:
                            value = exposure_time.split('/')
                            mValue = float(value[0]) / float(value[1])
                            if mValue < (1 / 2500) or mValue > 1:
                                error_msg += "check exposure time fail,actual=%s,expected=1/2500<=value<=1; " % mValue
                        else:
                            self.logger.debug("Exposure Time is not found")
                        if error_msg != "":
                            os.system("cp " + self.host_path + "/" + fileName + " " + g_common_obj.get_user_log_dir())
                            self.assertTrue(False, error_msg)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraHDRExifTest(self, lens):
        try:
            self.appPrepare("HDR")
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.setCaptureMode("HDR")
            for i in range(10):
                self.logger.debug("capture time: %s" % i)
                self.camera.capturePhoto()
                time.sleep(3)
                info, fileName = self.checkFileCorrupt()
                width = info.get("Image Width")
                height = info.get("Image Height")
                size = info.get("Image Size")
                self.logger.debug(str(width) + ", " + str(height) + ", " + str(size))
                self.assertTrue(size == width + "x" + height, "image size is null")
                exposure_time = info.get("Exposure Time")
                self.logger.debug("Exposure Time: %s" % exposure_time)
                error_msg = ""
                if exposure_time:
                    value = exposure_time.split('/')
                    mValue = float(value[0]) / float(value[1])
                    if mValue < (1 / 2500) or mValue > 1:
                        error_msg += "check exposure time fail,actual=%s,expected=1/2500<=value<=1; " % mValue
                else:
                    self.logger.debug("Exposure Time is not found")
                if error_msg != "":
                    os.system("cp " + self.host_path + "/" + fileName + " " + g_common_obj.get_user_log_dir())
                    self.assertTrue(False, error_msg)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraContinuousLowStorage(self, lens):
        try:
            self.appPrepare()
            CameraCommon().fillStorage(130)
            self.camera.startCameraApp()
            self.camera.switchRearOrFront(lens)
            self.camera.setFlash("off")
            mTime = time.time()
            while True:
                self.camera.clickShutterBtnWithoutRelease()
                time.sleep(2)
                free = CameraCommon().getSdcardMemory()[2]
                if free < 70:
                    self.logger.debug("The memory is full")
                    break
                if time.time() - mTime > 1800:
                    break
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraHDRIterativeCaptureMaxResolutionLowRAMTest(self, lens):
        try:
            self.appPrepare("HDR")
            type = "Capture Size (YUV)"
            CameraCommon().fillStorage(180)
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront(lens)
            self.camera.setCaptureMode("HDR")
            mResulotion = self.camera.getAllPhotoResolutions(lens, type)[1]
            self.logger.debug("max resolution is " + mResulotion)
            self.camera.setPhotoResolution(mResulotion, lens, type)
            self.camera.capturePhoto(200)
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraHDRIterativeExitTapHome(self, lens):
        try:
            self.appPrepare("HDR")
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront(lens)
            self.camera.setCaptureMode("HDR")
            self.camera.clickShutterBtnArea()
            CameraCommon().pressHome()
            CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraEffectCaptureRepeatedlyTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.switchRearOrFront("Back")
            for i in range(2):
                self.camera.setColorEffect("None")
                self.camera.capturePhoto()
                name1 = self.checkFileCorrupt()[1]
                image1_path = self.host_path + "/" + name1
                CameraCommon().removeDeivceFile()
                self.camera.setColorEffect("Mono")
                self.camera.capturePhoto()
                name2 = self.checkFileCorrupt()[1]
                image2_path = self.host_path + "/" + name2
                CameraCommon().removeDeivceFile()
                self.is_mono = self.checkImage.is_mono(image2_path)
                if not self.is_mono:
                    os.system("cp " + self.host_path + "/" + name2 + " " + g_common_obj.get_user_log_dir())
                    self.assertTrue(False, "check camera color effect to mono fail")
                self.camera.setColorEffect("Negative")
                self.camera.capturePhoto()
                name3 = self.checkFileCorrupt()[1]
                image3_path = self.host_path + "/" + name3
                CameraCommon().removeDeivceFile()
                self.color_effect2 = self.checkImage.is_negative(image1_path, image3_path)
                if not self.color_effect2:
                    os.system("cp " + self.host_path + "/" + name1 + " " + g_common_obj.get_user_log_dir())
                    os.system("cp " + self.host_path + "/" + name3 + " " + g_common_obj.get_user_log_dir())
                    self.assertTrue(False, "check color effect to none fail")
                self.camera.switchRearOrFront("Front")
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraEffectFrontSepiarepeatlyRearFrontSwitchTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            for i in range(5):
                self.camera.switchRearOrFront("Back")
                self.camera.setColorEffect("Sepia")
                self.camera.capturePhoto()
                self.camera.checkColorEffect("Sepia")
                self.camera.switchRearOrFront("Front")
                self.camera.setColorEffect("Negative")
                self.camera.capturePhoto()
                self.camera.checkColorEffect("Negative")
            CameraCommon().checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

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

    def test_Camera_Effect_Front_NEGATIVE_CaptureImage(self):
        self.setColorEffectNEGATIVECaptureImageCameraTest("Front")
 
    def test_Camera_Effect_Rear_NEGATIVE_CaptureImage(self):
        self.setColorEffectNEGATIVECaptureImageCameraTest("Back")

    def test_Camera_Effect_Front_None_CaptureImage(self):
        self.setColorEffectNoneCaptureImageCameraTest("Front")

    def test_Camera_Effect_Rear_None_CaptureImage(self):
        self.setColorEffectNoneCaptureImageCameraTest("Back")

    def test_Camera_Effect_Front_Mono_CaptureImage(self):
        self.setColorEffectMonoCaptureImageCameraTest("Front")

    def test_Camera_Effect_Rear_Mono_CaptureImage(self):
        self.setColorEffectMonoCaptureImageCameraTest("Back")

    def test_Camera_Continuous_Front_MaxoneTime(self):
        self.cameraContinuousMaxoneTimeTest("Front")

    def test_Camera_Continuous_Rear_MaxoneTime(self):
        self.cameraContinuousMaxoneTimeTest("Back")

    def test_Camera_Continuous_Front_ExitRelaunch(self):
        self.cameraContinuousExitRelaunchTest("Front")

    def test_Camera_Continuous_Rear_ExitRelaunch(self):
        self.cameraContinuousExitRelaunchTest("Back")

    def test_Camera_HDR_Rear_Capture(self):
        self.cameraHDRCaptureTest("Back")

    def test_Camera_HDR_Front_Capture(self):
        self.cameraHDRCaptureTest("Front")

    def test_Camera_HDR_Rear_EXIF(self):
        self.cameraHDRExifTest("Back")

    def test_Camera_HDR_Front_EXIF(self):
        self.cameraHDRExifTest("Front")

    def test_Camera_Continuous_Rear_LowStorage(self):
        self.cameraContinuousLowStorage("Back")

    def test_Camera_HDR_Front_IterativeCapture_MaxResolution_LowRAM(self):
        self.cameraHDRIterativeCaptureMaxResolutionLowRAMTest("Front")

    def test_Camera_HDR_Rear_IterativeCapture_MaxResolution_LowRAM(self):
        self.cameraHDRIterativeCaptureMaxResolutionLowRAMTest("Back")

    def test_Camera_HDR_Front_IterativeExit_TapHome(self):
        self.cameraHDRIterativeExitTapHome("Front")

    def test_Camera_HDR_Rear_IterativeExit_TapHome(self):
        self.cameraHDRIterativeExitTapHome("Back")

    def test_Camera_Effect_Rear_Capture_repeatedly(self):
        self.cameraEffectCaptureRepeatedlyTest()

    def test_Camera_Effect_Front_Sepia_repeatly_RearFrontSwitch(self):
        self.cameraEffectFrontSepiarepeatlyRearFrontSwitchTest()

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

    def test_Camera_SetBrightness_Front_EXIF(self):
        self.cameraSetBrightnessEXIFTest("Brightness", "Front")

    def test_Camera_SetBrightness_Rear_EXIF(self):
        self.cameraSetBrightnessEXIFTest("Brightness", "Back")
