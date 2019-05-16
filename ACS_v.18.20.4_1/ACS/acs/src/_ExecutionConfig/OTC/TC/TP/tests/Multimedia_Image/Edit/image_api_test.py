# coding: utf-8
import os
import time
from testlib.photos.mum_photos_impl import PhotosImpl
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.domains.mum_settings_impl import SettingImpl
from testlib.util.common import g_common_obj as adb
from testlib.systemui.systemui_impl import SystemUI
from testlib.camera.mum_camera_impl import CameraImpl
from testlib.util import make_big_file

class ImageAPITest(UIATestBase):
    """
    @summary: Test Video PlayBack
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(ImageAPITest, self).setUp()
        self.d = g_common_obj.get_device()
        self._test_name = __name__
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]
        print "[Setup]: %s" % self._test_name
        g_common_obj.stop_app_am("com.intel.otc.instrument.otcphotos")
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(ImageAPITest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()
        time.sleep(3)
        g_common_obj.stop_app_am("com.intel.otc.instrument.otcphotos")
        time.sleep(1)
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("remove_video"))

    def setTimeToSec(self, time):
        time = time.split(":")
        i = 1
        temp = 0
        for s in time[::-1]:
            temp += int(s) * i
            i *= 60
        return int(temp)

    def getOrientation(self):
        d = self.get_device()
        width = d.info["displayWidth"]
        height = d.info["displayHeight"]
        if width > height:
            return 1
        else:
            return 0

    def appPrepare(self, case_name, model=1):
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_image.conf')
        self.video = PhotosImpl(\
            self.config.read(cfg_file, case_name))
        self.camera = CameraImpl(\
            self.config.read(cfg_file, case_name))
        
        self.multimedia_setting = MultiMediaSetting(cfg_file)
        self.multimedia_setting.install_apk("photo_apk")
        self.multimedia_setting.install_apk("alarm_apk")
        self.video.set_orientation_n()
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def launchPhotoAPP(self):
        SystemUI().unlock_screen()
        for _ in range(3):
            g_common_obj.launch_app_am("com.intel.otc.instrument.otcphotos", \
                                   "com.intel.otc.instrument.otcphotos.MainActivity")
            time.sleep(3)
            if self.d(textContains="/").exists:
                return
        assert self.d(textContains="/").exists, "launch photo app failed!"

    def launchPhotoAPPWithShortCut(self):
        self.d.press.home()
        self.d(description="Apps").click()
        for _ in range(3):
            if self.d(textContains="Choose some apps").exists:
                self.d(text="OK").click()
        assert not self.d(text="OK").exists, "skip the tips failed!"
        while not self.d(text="otcphoto-app").exists:
            self.d(className="android.view.View").swipe.left()
        self.d(text="otcphoto-app").click()
        time.sleep(3)
        assert self.d(textContains="/").exists, "launch photo app failed!"

    def wait_boot_completed(self, timeout=1000):
        ''' wait Android boot_completed
    
        args: timeout -- optional timeout in second, default 180s
        '''
        count = 0
        sleep_time = 5
        while count < timeout:
            prop_val = adb.adb_cmd_capture_msg('getprop sys.boot_completed')
            if '1' in prop_val:
                print 'boot_completed'
                return
            count += sleep_time
            time.sleep(sleep_time)
        raise Exception('%ds timeout waiting for boot_completed' % timeout)

    def enterPhotoPath(self, path):
        push_str = path.strip("\"").split("/")
        if push_str[0] == "" and push_str[1] == "sdcard":
            push_str = push_str[2:]
        print push_str
        for t_str in push_str:
            if t_str != "":
                self.d(text=t_str).click()
                time.sleep(2)

    def checkPictureExist(self, file_name):
        assert self.d(text=file_name).exists, file_name + " not find!"

    def getPictureSize(self):
        scaleLabel = self.d(resourceId="com.intel.otc.instrument.otcphotos:id/scaleLabel").info["text"]
        return int(scaleLabel.strip("%"))

    def imageViewCheckWithWallpaper(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.multimedia_setting.push_file(self.video.cfg.get("push_picture"), self.video.cfg.get("datapath"))
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        file_name = os.path.split(path)[1]
        self.checkPictureExist(file_name)
        self.d.press.back()
        self.d(text=file_name).long_click()
        self.d(text="Wallpaper").click()
        time.sleep(5)
        assert self.d(text="WallPaper is updated.").exists, "Set wallpaper failed!"
        print "case " + str(case_name) + " is pass"

    def imageViewCheckWithDelete(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.multimedia_setting.push_file(self.video.cfg.get("push_picture"), self.video.cfg.get("datapath"))
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        file_name = os.path.split(path)[1]
        self.checkPictureExist(file_name)
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("remove_video"))
        self.d.press.home()
        self.launchPhotoAPP()
        self.enterPhotoPath(os.path.split(path)[0])
        assert not self.d(text=file_name).exists, "%s file exist!" % file_name
        print "case " + str(case_name) + " is pass"

    def imageViewCheckWithMonoryFull(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        g_common_obj.adb_cmd_capture_msg("rm -rf /storage/sdcard0/DCIM/Camera/*")
        self.camera.clean_up_camera_data()
        make_big_file.fill_no_space_except(int(self.camera.cfg.get("device_storage_limited")))
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.multimedia_setting.push_file(self.video.cfg.get("push_picture"), self.video.cfg.get("datapath"))
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        file_name = os.path.split(path)[1]
        self.checkPictureExist(file_name)
        g_common_obj.adb_cmd_capture_msg("rm -rf /mnt/sdcard/bigfile")
        print "case " + str(case_name) + " is pass"

    def imageViewCheckWithUnlock(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.multimedia_setting.push_file(self.video.cfg.get("push_picture"), self.video.cfg.get("datapath"))
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        self.checkPictureExist(os.path.split(path)[1])
        if 'bxtp_abl' in self.multimedia_setting.get_paltform_hardware():
            #  For BXT use relay card press power to sleep
            self.multimedia_setting.pressPowerKey()
            time.sleep(10)
            self.multimedia_setting.pressPowerKey()
        else:
            self.d.press.power()
            time.sleep(2)
            self.d.press.power()
        self.lock = SystemUI()
        self.lock.unlock_screen()
        time.sleep(1)
        self.checkPictureExist(os.path.split(path)[1])
        print "case " + str(case_name) + " is pass"

    def imageViewTenTimes(self, case_name, t_time=10):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.multimedia_setting.push_file(self.video.cfg.get("push_picture"), self.video.cfg.get("datapath"))
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        file_name = os.path.split(path)[1]
        self.checkPictureExist(file_name)
        for _ in range(10):
            self.d.press.back()
            self.d(text=file_name).click()
            self.checkPictureExist(file_name)
        print "case " + str(case_name) + " is pass"

    def testMultiMedia_Gallery_CropPicture_LockScreen(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckWithUnlock("test_API_image_032")

    def testMultiMedia_Gallery3D_Select_Unselected_Items_20_Times(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewTenTimes("test_API_image_033", 20)

    def testImage_View_Edit_GIF(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckWithWallpaper("test_API_image_034")

    def testImage_View_Edit_JPG(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckWithWallpaper("test_API_image_035")

    def testImage_View_Edit_BMP(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckWithWallpaper("test_API_image_036")

    def testImage_View_Edit_WBMP(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckWithWallpaper("test_API_image_037")

    def testImage_Edit_Save_With_Memory_Full(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckWithMonoryFull("test_API_image_039")

    def testImage_View_Edit_WEBP(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckWithWallpaper("test_API_image_040")

    def testImage_View_Edit_PNG(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckWithWallpaper("test_API_image_041")

    def test_Album_Delete(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckWithDelete("test_API_image_042")