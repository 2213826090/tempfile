# coding: utf-8
import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.checkImage import CheckImage
from testlib.camera.checkVideo import CheckVideo
from testlib.camera.RefCam2Camera import RefCam2Camera
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.cameratestbase import CameraTestBase

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

    def appPrepare(self,):
        self.camera = RefCam2Camera()
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

    def defaultSizeRepeatlyTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Burst")
            self.camera.switchRearOrFront(lens)
            self.camera.capturePhoto(100)
            self.checkFileCorrupt()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraBSTScreenLockTest(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Burst")
            for i in range(5):
                CameraCommon().pressPower()
                time.sleep(1)
            CameraCommon().checkCameraCrash()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraExposureBracketEXIF(self, lens):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Burst")
            self.camera.switchRearOrFront(lens)
            self.camera.setExposureBracket()
            self.camera.capturePhoto()
            self.checkFileCorrupt()
            fileNameList = CameraCommon().getFileName(self.camera_dir)
            print fileNameList
            iamge1 = self.host_path + "/" + fileNameList[0]
            iamge2 = self.host_path + "/" + fileNameList[1]
            iamge3 = self.host_path + "/" + fileNameList[2]
            image1_b = self.checkImage.brightness(iamge1)
            image2_b = self.checkImage.brightness(iamge2)
            image3_b = self.checkImage.brightness(iamge3)
            info1 = CameraCommon().getExifInfo(fileNameList[0], self.host_path + "/")
            info2 = CameraCommon().getExifInfo(fileNameList[1], self.host_path + "/")
            info3 = CameraCommon().getExifInfo(fileNameList[2], self.host_path + "/")
            ec1 = info1.get("Exposure Compensation")
            ec2 = info2.get("Exposure Compensation")
            ec3 = info3.get("Exposure Compensation")
            print ec1,ec2,ec3
            self.error_msg = ""
            if ec1 != "-5":
                self.error_msg = self.error_msg + "first picture Exposure Compensation can't match -5;"
                os.system("cp " + iamge1 + " " + g_common_obj.get_user_log_dir())
            if ec2 != "0":
                self.error_msg = self.error_msg + "second picture Exposure Compensation can't match 0;"
                os.system("cp " + iamge2 + " " + g_common_obj.get_user_log_dir())
            if ec3 != "5":
                self.error_msg = self.error_msg + "third picture Exposure Compensation can't match 5;"
                os.system("cp " + iamge3 + " " + g_common_obj.get_user_log_dir())
            if not self.checkImage.compare_images_brightness(iamge1, iamge2):
                self.error_msg = self.error_msg + "-1 compare 0 exposure check fail," + \
                    "-1(" + iamge1 + ") brightness=" + str(image1_b) + ", 0(" + iamge2 + ") brightness=" + str(image2_b) + "; "
                os.system("cp " + iamge1 + " " + g_common_obj.get_user_log_dir())
                os.system("cp " + iamge2 + " " + g_common_obj.get_user_log_dir())
            if not self.checkImage.compare_images_brightness(iamge2, iamge3):
                self.error_msg = self.error_msg + "0 compare 1 exposure check fail," + \
                    "0(" + iamge2 + ") brightness=" + str(image2_b) + ", 1(" + iamge3 + ") brightness=" + str(image3_b) + "; "
                os.system("cp " + iamge2 + " " + g_common_obj.get_user_log_dir())
                os.system("cp " + iamge3 + " " + g_common_obj.get_user_log_dir())
            if self.error_msg == "":
                    self.logger.debug("exposure check successful")
            else:
                self.assertTrue(False, self.error_msg)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def cameraExposureBracketIterativeCaptureMaxResolution(self, lens,run_time):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Burst")
            self.camera.switchRearOrFront(lens)
            self.camera.setExposureBracket()
            resolution = self.camera.getAllPhotoResolutions(lens)[1]
            self.camera.setPhotoResolution(resolution, lens)
            CameraCommon().clickScreenCenter()
            self.error_msg = ""
            for i in range(int(run_time)):
                self.logger.debug("======loop %d ======" %(i+1))
                self.camera.capturePhoto()
                time.sleep(1)
                self.checkFileCorrupt()
                fileNameList = CameraCommon().getFileName(self.camera_dir)
                print fileNameList
                iamge1 = self.host_path + "/" + fileNameList[0]
                iamge2 = self.host_path + "/" + fileNameList[1]
                iamge3 = self.host_path + "/" + fileNameList[2]
                image1_b = self.checkImage.brightness(iamge1)
                image2_b = self.checkImage.brightness(iamge2)
                image3_b = self.checkImage.brightness(iamge3)
                info1 = CameraCommon().getExifInfo(fileNameList[0], self.host_path + "/")
                info2 = CameraCommon().getExifInfo(fileNameList[1], self.host_path + "/")
                info3 = CameraCommon().getExifInfo(fileNameList[2], self.host_path + "/")
                ec1 = info1.get("Exposure Compensation")
                ec2 = info2.get("Exposure Compensation")
                ec3 = info3.get("Exposure Compensation")
                print ec1, ec2, ec3
                self.logger.debug("Exposure Compensation check,pic1 is %s,pic2 is %s, pic3 is %s" %(str(ec1),str(ec2),str(ec3)))
                if ec1 != "-5":
                    self.error_msg = self.error_msg + "first picture Exposure Compensation can't match -5;"
                    os.system("cp " + iamge1 + " " + g_common_obj.get_user_log_dir())
                if ec2 != "0":
                    self.error_msg = self.error_msg + "second picture Exposure Compensation can't match 0;"
                    os.system("cp " + iamge2 + " " + g_common_obj.get_user_log_dir())
                if ec3 != "5":
                    self.error_msg = self.error_msg + "third picture Exposure Compensation can't match 5;"
                    os.system("cp " + iamge3 + " " + g_common_obj.get_user_log_dir())
                if not self.checkImage.compare_images_brightness(iamge1, iamge2):
                    self.error_msg = self.error_msg + "-1 compare 0 exposure check fail," + \
                        "-1(" + iamge1 + ") brightness=" + str(image1_b) + ", 0(" + iamge2 + ") brightness=" + str(image2_b) + "; "
                    os.system("cp " + iamge1 + " " + g_common_obj.get_user_log_dir())
                    os.system("cp " + iamge2 + " " + g_common_obj.get_user_log_dir())
                if not self.checkImage.compare_images_brightness(iamge2, iamge3):
                    self.error_msg = self.error_msg + "0 compare 1 exposure check fail," + \
                        "0(" + iamge2 + ") brightness=" + str(image2_b) + ", 1(" + iamge3 + ") brightness=" + str(image3_b) + "; "
                    os.system("cp " + iamge2 + " " + g_common_obj.get_user_log_dir())
                    os.system("cp " + iamge3 + " " + g_common_obj.get_user_log_dir())
                CameraCommon().removeDeivceFile()
                CameraCommon().clickScreenCenter()
            if self.error_msg == "":
                    self.logger.debug("exposure check successful")
            else:
                self.assertTrue(False, self.error_msg)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def test_BST_Front_defaut_size_repeatly(self):
        self.defaultSizeRepeatlyTest("Front")

    def test_BST_Rear_defaut_size_repeatly(self):
        self.defaultSizeRepeatlyTest("Back")

    def test_Camera_BST_Rear_ScreenLock(self):
        self.cameraBSTScreenLockTest("Back")

    def test_Camera_BST_Front_ScreenLock(self):
        self.cameraBSTScreenLockTest("Front")

    def test_Camera_ExposureBracket_Front_EXIF(self):
        self.cameraExposureBracketEXIF("Front")

    def test_Camera_ExposureBracket_Rear_EXIF(self):
        self.cameraExposureBracketEXIF("Back")

    def test_Camera_ExposureBracket_Front_IterativeCapture_MaxResolution(self):
        self.cameraExposureBracketIterativeCaptureMaxResolution("Front", 20)

    def test_Camera_ExposureBracket_Rear_IterativeCapture_MaxResolution(self):
        self.cameraExposureBracketIterativeCaptureMaxResolution("Back", 20)

