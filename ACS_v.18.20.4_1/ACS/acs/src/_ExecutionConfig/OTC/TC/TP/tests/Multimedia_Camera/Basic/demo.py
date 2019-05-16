# coding: utf-8
import os
import time
# from testlib.util.uiatestbase import UIATestBase
# from testlib.camera.mum_camera_impl import CameraImpl
from testlib.util.common import g_common_obj
from testlib.camera.checkImage import CheckImage
from testlib.camera.checkVideo import CheckVideo
from testlib.camera.AOSPCamera import AOSPCamera
from testlib.camera.ArcSoftCamera import ArcSoftCamera
from testlib.camera.GMSCamera import GMSCamera
from testlib.camera.CameraCommon import CameraCommon
from testlib.audio.audio_impl import AudioImpl
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.checkIQ import CheckIQ
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
        self.light_port = "0"
        self.file_index = 0
        self.test_object = 0
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        if self.light_port!="0":
            self.deviceControl.pressLightBoxButton(self.light_port)
            self.deviceControl.pressLightBoxButton("1")
        if self.test_object!=0:
            #return to the first picture
            pass
        super(CameraTest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()
        time.sleep(3)

    def appPrepare(self,):
#         cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
#             'tests.testplan.camera.conf')
#         current_platform = CameraCommon().getPlatform()
#         self.camera = GMSCamera(self.config.read(cfg_file, "multimedia_camera"))
        #self.camera = ArcSoftCamera(self.config.read(cfg_file, "multimedia_camera"))
        self.camera = CameraCommon().switchPlatform()
        self.multimedia_setting = MultiMediaSetting(CameraCommon().DEFAULT_CONFIG_FILE)
        self.video = CheckVideo()
        self.checkImage = CheckImage()
        self.deviceControl = DeviceControl()
        self.host_path = CameraCommon().getTmpDir()
        self.makefileTime = CameraCommon().makefileTime
        self.camera_dir = CameraCommon().camera_dir
        CameraCommon().removeDeivceFile()
        CameraCommon().removeFile(self.host_path + "/*")
        self.camera.cleanMediaFiles()
        #CameraCommon().setOrientationToVertical()
        self.logger.debug("app prepare successfully")

#     def switchplatform(self, platform, cfg_file):
#         platform_list1 = self.camera.cfg.get("platform1").split(';')
#         platform_list2 = self.camera.cfg.get("platform2").split(';')
#         if platform in platform_list1:
#             self.camera = ArcSoftCamera(self.config.read(cfg_file, "multimedia_camera"))
#             self.logger.debug("the platform is %s" % platform)
#             self.multimedia_setting = MultiMediaSetting(cfg_file)
#             self.multimedia_setting.install_apk("quickpic_apk")
#         if platform in platform_list2:
#             self.camera = AOSPCamera(self.config.read(cfg_file, "multimedia_camera"))
#             self.logger.debug("the platform is %s" % platform)
#         self.multimedia_setting.install_apk("alarm_apk")
#         self.multimedia_setting.install_apk("gps_apk")

    def checkFileCorrupt(self, mediaFileCount=1, checkSim=False):
        file_name_list = CameraCommon().pullFileAndCheckFileNumber(self.camera_dir, self.host_path, mediaFileCount)
        for i in range(len(file_name_list)):
            self.path = self.host_path + "/" + file_name_list[i]
            for j in range(10):
                if not CameraCommon().checkFile(self.path):
                    CameraCommon().adbPullFile(self.camera_dir + file_name_list[i], self.host_path)
                    time.sleep(2)
                else:
                    self.logger.debug(str(file_name_list[i]) + " exists")
                    break
                time.sleep(2)
            self.path = self.host_path + "/" + file_name_list[i]
            os.system("cp "+self.path+" "+g_common_obj.get_user_log_dir())
            self.logger.debug("===picture path="+g_common_obj.get_user_log_dir()+"/"+file_name_list[i]+"===")
            if checkSim:
                panaFile = g_common_obj.get_user_log_dir()+"/"+file_name_list[i]
                self.logger.debug("===compare "+panaFile+" with the pano1.jpg====")
                #os.environ['LD_LIBRARY_PATH'] = "/lib:/usr/lib:/usr/local/lib"
                #os.putenv('LD_LIBRARY_PATH', "/lib:/usr/lib:/usr/local/lib")
                #currentDir = os.path.abspath(os.path.dirname(__file__))
                #cmd = currentDir+os.sep+"GASServer "+currentDir+"/pano1.jpg "+panaFile
                #import subprocess
                #output=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE).communicate()
                #self.logger.debug(output[0][:-1])
                #s=output[0][:-1].split("\n")[2]
                #percent=int(s.split(" ")[2].split("%")[0])
                #if percent < 30:
                #    assert False, "similar checking result < 30% Fail! "+panaFile+" is not similar with pano1.jpg"
                #else:
                #    self.logger.debug("similar checking result > 30% Pass!")
            str_name = file_name_list[i].split('.')
            suffix = str_name[1]
            if not suffix:
                assert False, "file name without the suffix"
            if suffix == "jpg":
                errMsg = self.checkImage.check_image_corrupt(self.path)
                if errMsg != "":
                    assert False, errMsg
                else:
                    self.info = CameraCommon().getExifInfo(file_name_list[i], self.host_path + "/")
                    self.logger.debug("picture validation successful")
                continue
            elif suffix == "mp4" or suffix == "3gp":
                if self.video.check_video_corrupt(self.path):
                    self.info = CameraCommon().getExifInfo(file_name_list[i], self.host_path + "/")
                    self.logger.debug("video validation successful")
                else:
                    self.logger.debug("video validation fails")
#             self.removeFile(self.host_path + "/*")
#         self.removeDeivceFile()
        return self.info, file_name_list[0]

    def flashTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Camera")
            self.camera.switchRearOrFront("Back")
            self.camera.setFlash("on")
            #If light box is ready, here should set flash to auto and set dark light first and then set bright light
            self.camera.capturePhoto(1)
            time.sleep(1)
            ret = self.checkFileCorrupt(1)
            fileName = ret[1]
            flashInfo = ret[0].get("Flash")
            self.logger.info("===fileName=%s, flashInfo=%s" %(fileName, flashInfo))
            CameraCommon().moveFilesFromCameraFolder()
            self.camera.setFlash("off")
            self.camera.capturePhoto(1)
            time.sleep(1)
            ret=self.checkFileCorrupt(1)
            fileName = ret[1]
            flashInfo = ret[0].get("Flash")
            self.logger.info("===fileName=%s, flashInfo=%s" %(fileName, flashInfo))
            CameraCommon().recoverFilesFromCameraTemp()
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)

    def panoramaTest(self):
        try:
            self.appPrepare()
            self.camera.startCameraApp()
            self.camera.selectMode("Panorama")
            self.camera.capturePanorama()
            self.checkFileCorrupt(1, True)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)
            
    def zoomDemo(self):
        #print CameraCommon().getTestEntryName()
        #return 
        self.appPrepare()
        mlist = self.deviceControl.getZoomCommands()
        
        self.camera.startCameraApp()
        self.camera.selectMode("Camera")
        CameraCommon().clickScreenCenter()
        self.camera.capturePhoto()
        time.sleep(2)
        ori_name = self.checkFileCorrupt()[1]
        CameraCommon().removeDeivceFile()
        print ori_name, mlist[0], mlist[1]
        ori_name = self.host_path + "/" + ori_name
        zoom_name = CameraCommon().zoomInOrOut(self.camera, mlist[0])[0]
        print ori_name,zoom_name
#         file1 = "/data/debug/zoom/zoom/3gr/IMG_20160323_125326.jpg"
#         file2 = "/data/debug/zoom/zoom/3gr/IMG_20160323_125403.jpg"
        checkZoom = CheckZoom()
#         self.assertTrue(checkZoom.checkZoomEffect(file1, file2), "Zoom effect is not correct")
        self.assertTrue(checkZoom.checkZoomEffect(ori_name, zoom_name), "Zoom effect is not correct")

    def benchTest(self):
        self.appPrepare()
        localPath = CameraCommon().getTmpDir()
        self.camera.startCameraApp()
        self.camera.selectMode("Camera")
        checkIQ = CheckIQ("", localPath)
        """
        change to the CWF light
        """
        self.light_port="6"
        self.deviceControl.pressLightBoxButton("1")
        self.deviceControl.pressLightBoxButton(self.light_port)
        """
        change the test object
        """
        self.test_object=0
        self.deviceControl.selectTestObject(0)
        """
        initialize robot
        """
        from testaid.robot import Robot
        deviceName = self.deviceControl.getRobotDeviceName()
        robot = Robot(deviceName)
        robot.reset()
        """
        Adjust FOV content
        """
        l = self.camera.getFOVRegion()
        region = CheckIQ.Region(l[0], l[1], l[2], l[3], False)
        self.deviceControl.adjustFOV(localPath, region, robot)
        """
        Main test code, capture picture
        """
        CameraCommon().clickScreenCenter()
        self.camera.capturePhoto()
        time.sleep(2)
        files = CameraCommon().getMediaFilesFromDevice(self.file_index, 1)
        self.file_index += len(files)
        #CameraCommon().moveFilesFromCameraFolder()
        errMsg = ""
        for f in files:
            ret = checkIQ.checkCorrupt(f)
            if ret!="":
                errMsg += "file:"+f+" is abnormal! "+ret
        #CameraCommon().recoverFilesFromCameraTemp()
        robot.reset()
        self.assertTrue(errMsg=="", errMsg) 
        
    def benchTest1(self):
        self.appPrepare()
        localPath = CameraCommon().getTmpDir()
        self.camera.startCameraApp()
        self.camera.selectMode("Camera")
        checkIQ = CheckIQ("", localPath)
        CameraCommon().clickScreenCenter()
        self.camera.capturePhoto()
        time.sleep(2)
        files = CameraCommon().getMediaFilesFromDevice(self.file_index, 1)
        self.file_index += len(files)
        CameraCommon().moveFilesFromCameraFolder()
        errMsg = ""
        #for f in files:
        #    ret = checkIQ.checkCorrupt(f)
        #    if ret!="":
        CameraCommon().clickScreenCenter()
        self.camera.capturePhoto()
        time.sleep(2)
        files = CameraCommon().getMediaFilesFromDevice(self.file_index, 1)
        self.file_index += len(files)
        CameraCommon().recoverFilesFromCameraTemp()
        self.assertTrue(errMsg=="", errMsg) 
    
    def testPanorama(self):
        self.panoramaTest()

    def testFlash(self):
        #self.zoomDemo() 
        self.flashTest()

    def test_Camera_Zoom(self):
        #self.appPrepare()
        #localPath = CameraCommon().getTmpDir()
        #checkIQ = CheckIQ("", localPath)
        #self.benchTest1()
        self.zoomDemo()
