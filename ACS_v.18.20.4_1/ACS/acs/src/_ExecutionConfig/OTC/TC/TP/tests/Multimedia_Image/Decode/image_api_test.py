# coding: utf-8
import os
import time
import re
from testlib.photos.mum_photos_impl import PhotosImpl
from testlib.multimedia.testcasebase import TestCaseBase
from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_setting import MultiMediaSetting, MultiMediaHandle
from testlib.util.common import g_common_obj as adb
from testlib.systemui.systemui_impl import SystemUI
from testlib.multimedia.multimedia_log import MultimediaLogger
logger = MultimediaLogger.instance()

class ImageAPITest(TestCaseBase):
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
        self.dpx = self.d.info["displaySizeDpX"]
        self.dpy = self.d.info["displaySizeDpY"]
        self.tag = "[Image Decode image_api_test] "
        self.serial = self.d.server.adb.device_serial()
        logger.debug(self.tag + "[Setup]: %s" % self._test_name)
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
        
        self.multimedia_handle = MultiMediaHandle()
        self.multimedia_setting = MultiMediaSetting(cfg_file)
        self.multimedia_setting.install_apk("photo_apk")
        self.multimedia_setting.install_apk("alarm_apk")
        self.video.set_orientation_n()
        if self.video.cfg.get("push_picture") is not None:
            self.multimedia_setting.push_file(self.video.cfg.get("push_picture"), self.video.cfg.get("datapath"))
        refresh_sd_cmd = self.config.read(cfg_file, 'common_cmd')['refresh_sd']
        g_common_obj.adb_cmd_capture_msg(refresh_sd_cmd)
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def launchPhotoAPP(self):
        logger.debug(self.tag + "launch otcphotos app")
        SystemUI().unlock_screen()
        for _ in range(3):
            g_common_obj.launch_app_am("com.intel.otc.instrument.otcphotos", \
                                   "com.intel.otc.instrument.otcphotos.MainActivity")
            time.sleep(3)
            if self.d(textContains="/").exists:
                return
        assert self.d(textContains="/").exists, "launch photo app failed!"

    def checkPhotosBackupAndHandle(self):
        """
        check 'back up & sync' exist and handle it
        :return:
        """
        for _ in range(3):
            if self.d(textContains="Update Google Photos").exists:
                logger.debug("Ignore update google photos")
                self.d(textMatches="NOT NOW").click()
            time.sleep(1)
            if self.d(resourceId='com.android.packageinstaller:id/permission_allow_button').exists:
                self.d(resourceId='com.android.packageinstaller:id/permission_allow_button').click()
                time.sleep(1)
            if self.d(textContains='Google Photos uses face').exists and \
                    self.d(textMatches='[O|o][N|n]').exists:
                logger.debug("back up & sync display, try to init photos")
                x = self.d.info['displayHeight'] / 2
                y = self.d.info['displayWidth'] / 2
                self.d.click(x, y)
            if self.d(textContains="Keep backup off").exists:
                self.d(textMatches="Keep off|KEEP OFF").click()
            if self.d(textStartsWith="Never show").exists:
                self.d(textStartsWith="Never show").click()
            if self.d(textMatches="SKIP|Skip").exists:
                self.d(textMatches="SKIP|Skip").click()
            if self.d(textMatches="CANCEL|Cancel").exists:
                self.d(textMatches="CANCEL|Cancel").click()
            if self.d(description="Photos, selected, tab, 2 of 3").exists or\
                                self.d(resourceId="com.google.android.apps.photos:id/toolbar").exists:
                logger.debug("init completed")
                break

    def launchPhotoAPPWithShortCut(self):
        logger.debug(self.tag + "Launch Photos")
        self.d.press.home()
        if self.get_android_version() == "7.1.1":
            logger.debug("DUT android version is 7.1.1")
            self.d(description="Apps list").click()
        elif self.get_android_version() < "7.1.1":
            logger.debug("DUT android version is beloww 7.1.1")
            if self.d(text="GOT IT").exists:
                self.d(text="GOT IT").click()
            self.d(description="Apps").click()
        elif self.multimedia_setting.get_paltform_hardware() == 'androidia_64':
            logger.debug("For AIA, Photos in Google folder")
            self.d(description="Folder: Google").click()
            self.d(description="Photos").click()
            self.checkPhotosBackupAndHandle()
            assert self.d(resourceId="com.google.android.apps.photos:id/toolbar").exists, "launch photo app failed!"
            return True
        #for N build,firstly go into apps list,skip the tips
        for _ in range(3):
            if self.d(textContains="Choose some apps").exists:
                self.d(text="OK").click()
        assert not self.d(text="OK").exists,"skip the tips failed!"
        while not self.d(text="otcphoto-app").exists:
            self.d(className="android.view.View").swipe.left()
        self.d(text="otcphoto-app").click()
        time.sleep(3)
        assert self.d(textContains="/").exists, "launch photo app failed!"

    def launchGalleryAPPWithShortCut(self):
        # for bxtp o car, launch Gallery
        self.d.press.home()
        app_list_cmd = "adb -s %s shell input tap '%d' '%d'" % (self.serial, int(self.dpx*0.78), int(self.dpy*0.97))
        print app_list_cmd
        for _ in range(3):
            os.system(app_list_cmd)
            if self.d(resourceId="com.android.support.car.lenspicker:id/dismiss_area").wait.exists(timeout=3000):
                try:
                    self.d(resourceId="com.android.support.car.lenspicker:id/dismiss_area").scroll.vert.to(text='Gallery')
                except:
                    self.d.swipe(self.x / 2, self.y * 0.75, self.x / 2, self.y * 0.25)
                self.d(text="Gallery").click()
                if self.d(resourceId="com.android.gallery3d:id/gl_root_view").wait.exists(timeout=3000):
                    logger.debug("Launch Gallery success!")
                    return True
        logger.debug("Launch Gallery Failed!")
        return False

    def get_android_version(self):

        prop = "ro.build.version.release"
        buf = g_common_obj.adb_cmd_capture_msg("getprop " + prop)
        v_str = buf.strip()
        return v_str

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
        logger.debug(self.tag + "enter picture path :" + path)
        push_str = path.strip("\"").split("/")
        if push_str[0] == "" and push_str[1] == "sdcard":
            push_str = push_str[2:]
        for t_str in push_str:
            logger.debug(self.tag + "try to click %s"%t_str)
            if t_str != "":
                self.d(text=t_str).click()
                time.sleep(2)

    def checkPictureExist(self, file_name):
        logger.debug(self.tag + "check picture exist, filename:" + file_name)
        if self.d(text=file_name).exists:
            logger.debug(self.tag + file_name + " found!")
            return True
        else:
            logger.debug(self.tag + file_name + " not find!")
            return False

    def openPictureWithGallery(self, file):
        print file
        open_picture_cmd = "shell am start -a android.intent.action.VIEW -d file://%s " \
                   " -t image/* -n com.android.gallery3d/.app.GalleryActivity" % (file)
        for _ in range(3):
            g_common_obj.adb_cmd_common(open_picture_cmd)
            time.sleep(1)
            if self.d(resourceId="com.android.gallery3d:id/photopage_bottom_controls"):
                return True
        logger.debug(self.tag + "open %s failed" % file)
        return False

    def openPictureWithPhotosCmd(self, file):
        logger.debug("Try to open %s with command"%file)
        open_picture_cmd = "shell am start -a android.intent.action.VIEW -d file://%s " \
                           " -t image/* -n com.google.android.apps.photos/.pager.HostPhotoPagerActivity" % (file)
        for _ in range(3):
            g_common_obj.adb_cmd_common(open_picture_cmd)
            time.sleep(1)
            if self.d(resourceId="com.google.android.apps.photos:id/video_player_controller_fragment_container"):
                logger.debug(self.tag + 'Open picture success.')
                return True
        logger.debug(self.tag + "open %s failed" % file)
        return False

    def getPictureSize(self):
        scaleLabel = self.d(resourceId="com.intel.otc.instrument.otcphotos:id/scaleLabel").info["text"]
        return int(scaleLabel.strip("%"))

    def checkVideoPlayBack(self, s=60):
        return self.multimedia_handle.checkVideoPlayBack(s)

    def imageViewRotatePicture(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        self.checkPictureExist(os.path.split(path)[1])
        self.d.orientation = "n"
        time.sleep(2)
        self.checkPictureExist(os.path.split(path)[1])
        self.video.set_orientation_n()
        time.sleep(2)
        self.checkPictureExist(os.path.split(path)[1])
        print "case " + str(case_name) + " is pass"

    def imageViewRotatePictureManyTimes(self, case_name, count):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        self.checkPictureExist(os.path.split(path)[1])
        for _ in range (count):
            self.d.orientation = "n"
            time.sleep(2)
            self.checkPictureExist(os.path.split(path)[1])
            self.video.set_orientation_n()
            time.sleep(2)
            self.checkPictureExist(os.path.split(path)[1])
        print "case " + str(case_name) + " is pass"

    def imageViewCheck(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        self.checkPictureExist(os.path.split(path)[1])
        print "case " + str(case_name) + " is pass"

    def imageViewCheckAfterDeleteFile(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        file_name = os.path.split(path)[1]
        self.checkPictureExist(file_name)
        self.multimedia_setting.clearFolder(self.video.cfg.get("remove_video"))
        time.sleep(2)
        self.d.press.back()
        self.d(text=file_name).click()
        assert not self.d(resourceId='com.intel.otc.instrument.otcphotos:id/sizeLabel').exists,"no warning messsage pop up"
        for _ in range(3):
            self.d.press.back()
        print "case " + str(case_name) + " is pass"

    def imageViewCheckWithUnlock(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
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

    def imageViewCheckWithRotation(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.launchPhotoAPP()
        self.d.orientation = "n"
        self.d.press.home()
        self.multimedia_setting.click_recent_app("otcphoto-app")
        time.sleep(1)
        assert self.d(textContains="/").exists, "launch photo app failed!"
        self.multimedia_setting.launchAlarmAPP()
        self.video.set_orientation_n()
        self.multimedia_setting.click_recent_app("otcphoto-app")
        time.sleep(1)
        assert self.d(textContains="/").exists, "launch photo app failed!"
        self.enterPhotoPath(path)
        self.checkPictureExist(os.path.split(path)[1])
        print "case " + str(case_name) + " is pass"

    def imageViewCheckWithInstallApp(self, case_name, flag=1):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        if flag != 1:
            self.multimedia_setting.push_file(self.video.cfg.get("push_picture_2"), self.video.cfg.get("datapath"))
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        self.multimedia_setting.install_apk("video_apk")
        self.checkPictureExist(os.path.split(path)[1])
        if flag != 1:
            self.d(className="android.widget.ImageView").swipe.left()
            path_2 = self.video.cfg.get("push_picture_2").split(" ")[-1].strip("\"")
            self.checkPictureExist(os.path.split(path_2)[1])
        self.push_path = self.multimedia_setting.push_file(self.video.cfg.get("push_video_2"), self.video.cfg.get("datapath_2"))
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.multimedia_handle.videoPlayBack(self.push_path)
        self.multimedia_setting.install_apk("vpg_apk")
        self.checkVideoPlayBack(30)
        adb.adb_cmd_common('uninstall videoplayer.app.instrument.otc.intel.com.otcvideoplayer')
        adb.adb_cmd_common('uninstall com.intel.vpg.tool')
        print "case " + str(case_name) + " is pass"

    def imageViewCheckWithShortCut(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        if "androidia_64" in self.multimedia_setting.get_paltform_hardware():
            self.launchPhotoAPPWithShortCut()
            assert self.openPictureWithPhotosCmd(path), "Picture not exists!"
        elif self.multimedia_setting.get_android_version() == "O":
            self.launchGalleryAPPWithShortCut()
            assert self.openPictureWithGallery(path), "Picture not exists!"
        else:
            self.launchPhotoAPPWithShortCut()
            self.enterPhotoPath(path)
            assert self.checkPictureExist(os.path.split(path)[1]), "Picture not exists!"
        print "case " + str(case_name) + " is pass"

    def imageViewCheckFolder(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        path_2 = self.video.cfg.get("push_picture_2").split(" ")[-1].strip("\"")
        print path, path_2
        self.multimedia_setting.push_file(self.video.cfg.get("push_picture_2"), self.video.cfg.get("datapath"))
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        file_name = os.path.split(path)[1]
        file_name_2 = os.path.split(path_2)[1]
        self.checkPictureExist(file_name)
        self.d.press.back()
        self.checkPictureExist(file_name)
        self.checkPictureExist(file_name_2)
        print "case " + str(case_name) + " is pass"

    def imageViewCheckSlideShow(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        path_2 = self.video.cfg.get("push_picture_2").split(" ")[-1].strip("\"")
        print path, path_2
        self.multimedia_setting.push_file(self.video.cfg.get("push_picture_2"), self.video.cfg.get("datapath"))
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        file_name = os.path.split(path)[1]
        file_name_2 = os.path.split(path_2)[1]
        self.checkPictureExist(file_name)
        self.d(className="android.widget.ImageView").swipe.left()
        self.checkPictureExist(file_name_2)
        print "case " + str(case_name) + " is pass"

    def imageViewTenTimes(self, case_name, t_time=10):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        file_name = os.path.split(path)[1]
        self.checkPictureExist(file_name)
        for _ in range(10):
            self.d.press.back()
            self.d(text=file_name).click()
            self.checkPictureExist(file_name)
        print "case " + str(case_name) + " is pass"

    def getCpuConsumption(self):
        cmd = "shell top -m 1 -n 1"
        t_pattern = re.compile("User (.*)%, System (.*)%, IOW.*")
        result = g_common_obj.adb_cmd_common(cmd)
        print result
        if t_pattern.findall(result) != []:
            (t_user, t_system) = t_pattern.findall(result)[0]
            return int(t_user) + int(t_system)
        else:
            return 0

    def imageViewWithManyTimes(self, case_name, times):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        cpu_1 = self.getCpuConsumption()
        for iteration in range(times):
            logger.debug("Execute {0} times, total {1} times".format(iteration+1, times))
            self.launchPhotoAPP()
            time.sleep(5)
            g_common_obj.stop_app_am("com.intel.otc.instrument.otcphotos")
            time.sleep(4)
        time.sleep(10)
        cpu_2 = self.getCpuConsumption()
        logger.info("cpu_1={0}, cpu_2={1}".format(cpu_1, cpu_2))
        assert cpu_1 + 10 >= cpu_2, "Cpu Consumption error! cpu_1=%s, cpu_2=%s" % (cpu_1, cpu_2)
        print "case " + str(case_name) + " is pass"

    def imageViewBackHome(self, case_name, flag=1):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        if flag != 1:
            self.multimedia_setting.push_file(self.video.cfg.get("push_picture_2"), self.video.cfg.get("datapath"))
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        self.checkPictureExist(os.path.split(path)[1])
        self.d.press.home()
        self.multimedia_setting.click_recent_app("otcphoto-app")
        time.sleep(1)
        self.checkPictureExist(os.path.split(path)[1])
        if flag != 1:
            self.d(className="android.widget.ImageView").swipe.left()
            path_2 = self.video.cfg.get("push_picture_2").split(" ")[-1].strip("\"")
            self.checkPictureExist(os.path.split(path_2)[1])
        print "case " + str(case_name) + " is pass"

    def imageViewWithAlarm(self, case_name, flag=1):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        if flag != 1:
            self.multimedia_setting.push_file(self.video.cfg.get("push_picture_2"), self.video.cfg.get("datapath"))
        self.multimedia_setting.launchAlarmAPP()
        self.multimedia_setting.setAlarmTime(30)
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        self.checkPictureExist(os.path.split(path)[1])
        self.multimedia_setting.waitAlarmTriiggered(50, "Snooze")
        self.multimedia_setting.setAlarmTime(30)
        self.multimedia_setting.click_recent_app("otcphoto-app")
        time.sleep(1)
        self.checkPictureExist(os.path.split(path)[1])
        self.multimedia_setting.waitAlarmTriiggered(50, "Dismiss")
        self.multimedia_setting.click_recent_app("otcphoto-app")
        time.sleep(1)
        self.checkPictureExist(os.path.split(path)[1])
        if flag != 1:
            self.d(className="android.widget.ImageView").swipe.left()
            path_2 = self.video.cfg.get("push_picture_2").split(" ")[-1].strip("\"")
            self.checkPictureExist(os.path.split(path_2)[1])
        print "case " + str(case_name) + " is pass"

    def imageViewWithReboot(self, case_name, flag=1):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        logger.debug(self.tag + "path:" + path)
        if flag != 1:
            self.multimedia_setting.push_file(self.video.cfg.get("push_picture_2"), self.video.cfg.get("datapath"))
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        self.checkPictureExist(os.path.split(path)[1])
        adb.reboot_device()
        self.wait_boot_completed()
        time.sleep(20)
        if self.d(textContains="Drive safely").exists:
            logger.debug(self.tag + "Drive saftely exist , click owner")
            self.d(text="Owner").click()
            time.sleep(3)
        self.lock = SystemUI()
        self.lock.unlock_screen()
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        self.checkPictureExist(os.path.split(path)[1])
        if flag != 1:
            self.d(className="android.widget.ImageView").swipe.left()
            path_2 = self.video.cfg.get("push_picture_2").split(" ")[-1].strip("\"")
            self.checkPictureExist(os.path.split(path_2)[1])
        print "case " + str(case_name) + " is pass"

    def imageViewWithInvalidFile(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        logger.debug(self.tag + "test file patch:%s"%path)
        self.launchPhotoAPP()
        self.enterPhotoPath(os.path.split(path)[0])
        assert not self.d(text=os.path.split(path)[1]).exists, os.path.split(path)[1] + "Invalid file can be find! error!"
        print "case " + str(case_name) + " is pass"

    def imageViewZoomIn_ZoomOut(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        path = self.video.cfg.get("push_picture").split(" ")[-1].strip("\"")
        print path
        self.launchPhotoAPP()
        self.enterPhotoPath(path)
        self.checkPictureExist(os.path.split(path)[1])
        self.d(className="android.widget.ImageView").gesture((self.x/2, self.y/2-20), (self.x/2, self.y/2)).to((self.x/2, self.y/4), (self.x/2, self.y*3/4))
        self.checkPictureExist(os.path.split(path)[1])
        x1 = self.getPictureSize()
        assert x1 > 100, "ZoomIn error"
        time.sleep(2)
        self.d(className="android.widget.ImageView").gesture((self.x/2, self.y/4), (self.x/2, self.y*3/4)).to((self.x/2, self.y/2-20), (self.x/2, self.y/2))
        self.checkPictureExist(os.path.split(path)[1])
        x2 = self.getPictureSize()
        assert x1 > x2, "ZoomOut error"
        print "case " + str(case_name) + " is pass"

    def testImage_View_RotatePicture(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        2. Rotate picture to landscape mode
        3. Rotate picture to Portrait mode
        """
        self.imageViewRotatePicture("test_API_image_001")

    def testMultiMedia_Gallery_SlideShow_Continue_Idle(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        2. Press home
        """
        self.imageViewBackHome("test_API_image_002", 2)

    def testMultiMedia_Gallery3D_GIF_87a_ZoomIn_ZoomOut(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        2. Press home
        """
        self.imageViewZoomIn_ZoomOut("test_API_image_004")

    def testMultiMedia_Gallery3D_GIF_89a_ZoomIn_ZoomOut(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        2. Press home
        """
        self.imageViewZoomIn_ZoomOut("test_API_image_005")

    def testImage_ViewMode_Full_And_non_FullScreen_Switch(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewTenTimes("test_API_image_006", 10)

    def testMultiMedia_Gallery3D_SlideShow_AlarmReminder(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewWithAlarm("test_API_image_007", 2)

    def testImage_View_InvalidFile(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewWithInvalidFile("test_API_image_008")

    def testMultiMedia_Gallery_Format_UnSupportPicture(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewWithInvalidFile("test_API_image_009")

    def testMultiMedia_Gallery_SlideShow_Then_PowerOff_ON(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewWithReboot("test_API_image_010", 2)

    def testMultiMedia_Gallery_Format_ErrorPicture(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewWithInvalidFile("test_API_image_011")

    def testMultiMedia_Gallery3D_SlideShow_Multiple_Events(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewWithAlarm("test_API_image_012", 2)

    def testMultiMedia_MediaFramework_Display_Picture(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewWithInvalidFile("test_API_image_013")

    def testMultiMedia_Gallery3D_ViewImage_Larger_Than_5_or_8_MB(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheck("test_API_image_014")

    def test_Calendar_Alarm_EmailNote_When_SlideShow(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewWithAlarm("test_API_image_015", 2)

    def testMultiMedia_Gallery3D_Launch_Exit_30_Times(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewWithManyTimes("test_API_image_016", 30)

    def testMultiMedia_Gallery_Launch_Exit_100_Times(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewWithManyTimes("test_API_image_017", 100)

    def testImage_Number_Indicator(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckFolder("test_API_image_018")

    def testCalendar_Interact_When_View_Picture(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewWithAlarm("test_API_image_019")

    def testMultiMedia_Gallery_SlideShow_Long_Time(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckSlideShow("test_API_image_020")

    def testMultiMedia_Gallery_DifferentResolutions_3072x2304(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheck("test_API_image_021")

    def testMultiMedia_MediaFramework_Display_BigSize_Picture(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheck("test_API_image_022")

    def testMultiMedia_MediaFramework_Display_LittleSize_Picture(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheck("test_API_image_023")

    def testMultiMedia_Gallery_Enter_By_ShortCut(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckWithShortCut("test_API_image_024")

    def testJPEG_HW_Decode(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheck("test_API_image_025")

    def testMultiMedia_Gallery3D_Unsupported_Images_Videos(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewWithInvalidFile("test_API_image_026")

    def testMultiMedia_Gallery3D_Gesturing_Screen(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckSlideShow("test_API_image_027")

    def testMultiMedia_Gallery_Panning_Single(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        2. Press home
        """
        self.imageViewZoomIn_ZoomOut("test_API_image_028")

    def testApp_Install_When_View_Picture_Video_SlideShow(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckWithInstallApp("test_API_image_029", 2)

    def testMultiMedia_Gallery_Rotation_Background(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckWithRotation("test_API_image_030")

    def testMultiMedia_Gallery_Image_Review_SD_USB(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheck("test_API_image_031")

    def testImage_View_RotatePicture_ManyTimes(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewRotatePictureManyTimes("test_API_image_043", 20)

    def testVideo_NonExist(self):
        """
        This test used to test Image Decode
        The test case spec is following:
        1. Open a picture
        """
        self.imageViewCheckAfterDeleteFile("test_API_image_044")
