'''
Created on Apr 25, 2017

@author: Li Zixi
'''
import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.IVICamera import IVICamera
from testlib.multimedia.multimedia_canbox_helper import MultiMediaCanboxHelper
from testlib.multimedia.multimedia_checkiq_helper import MultiMediaCheckiqHelper

class IVICameraO(IVICamera):
    HOME_PACKAGE_NAME = "com.android.car.mapsplaceholder"
    HOME_ACTIVITY_NAME = ".MapsPlaceholderActivity"
    CAMERA_CANBOX_0_SENDID = "000"
    CAMERA_CANBOX_0_SENDMESSAGE = "%s#00" % CAMERA_CANBOX_0_SENDID
    CAMERA_CANBOX_C_SENDID = "00C"
    CAMERA_CANBOX_C_SENDMESSAGE = "%s#00" % CAMERA_CANBOX_C_SENDID

    def __init__(self, cfg=None):
        super(IVICameraO, self).__init__(cfg)
        self.camera_common = CameraCommon()
        self.host_path = self.camera_common.getTmpDir()
        self.multimedia_canbox_helper = MultiMediaCanboxHelper()
        self.multimedia_checkiq_helper = MultiMediaCheckiqHelper(self.host_path)
        user_log_dir = g_common_obj.get_user_log_dir()
        self.before_screen_0_file_path = os.path.join(user_log_dir, "before_enter_camera_0_screenshot.png")
        self.after_screen_0_file_path = os.path.join(user_log_dir, "after_enter_camera_0_screenshot.png")
        self.before_screen_c_file_path = os.path.join(user_log_dir, "before_enter_camera_c_screenshot.png")
        self.after_screen_c_file_path = os.path.join(user_log_dir, "after_enter_camera_c_screenshot.png")
        self.now_camera_0_event = 0
        self.camera_0_event_array = [("GEAR_NEUTRAL", 0), ("GEAR_DRIVE", 0), ("GEAR_REVERSE", 1), ("GEAR_PARK", 2), ("GEAR_LOW", 0)]
        self.now_camera_c_event = 0
        self.camera_c_event_array = [("NONE", 0), ("EMERGENCY", 0), ("RIGHT", 1), ("LEFT", 1)]
        self.init_camera_with_canbox()

    class IVICameraWidget:

        def __init__(self):
            self.d = g_common_obj.get_device()

        def homeLayout(self):
            '''
            layout capture
            '''
            return self.d(packageName="com.android.car.dialer")

        def homeLayout_2(self):
            '''
            layout capture
            '''
            return self.d(packageName="com.android.car.overview")

        def homeLayout_3(self):
            '''
            layout capture
            '''
            return self.d(packageName="com.android.car.mapsplaceholder")

        def lockLayout(self):
            '''
            layout capture
            '''
            return self.d(packageName="com.google.android.apps.maps")

        def cameraLayout(self):
            '''
            layout capture
            '''
            return self.d(packageName="com.intel.rvc")

    def isHomeLayoutExists(self):
        if self.d(text="Got it").exists:
            self.d(text="Got it").click.wait()
        self.skipAccountLoginLyout()
        t_result = self.camera_common.isWidgetExists(self.IVICameraWidget().homeLayout())
        if not t_result:
            t_result = self.camera_common.isWidgetExists(self.IVICameraWidget().homeLayout_2())
        if not t_result:
            t_result = self.camera_common.isWidgetExists(self.IVICameraWidget().homeLayout_3())
        return t_result

    def isLockLayoutExists(self):
        return self.camera_common.isWidgetExists(self.IVICameraWidget().lockLayout())

    def change_canbox_0_event(self):
        self.now_camera_0_event += 1
        if self.now_camera_0_event >= len(self.camera_0_event_array):
            self.now_camera_0_event = 0
        self.multimedia_canbox_helper.cansend(IVICameraO.CAMERA_CANBOX_0_SENDMESSAGE)

    def change_canbox_c_event(self):
        self.now_camera_c_event += 1
        if self.now_camera_c_event >= len(self.camera_c_event_array):
            self.now_camera_c_event = 0
        self.multimedia_canbox_helper.cansend(IVICameraO.CAMERA_CANBOX_C_SENDMESSAGE)

    def get_screenshot(self, file_path):
        time.sleep(1)
        file_folder, file_name = os.path.split(file_path)
        self.camera_common.getScreenshotAndPullToHost(file_name, file_folder)

    def get_screenshot_and_compare_picture_similarity(self, expect_file_path, actual_file_path, expect_percent=0.8):
        self.get_screenshot(actual_file_path)
        t_percent = self.multimedia_checkiq_helper.compare_picture_similarity(expect_file_path, actual_file_path)
        self.logger.debug("get_screenshot_and_compare_picture_similarity t_percent=%s" % str(t_percent))
        return t_percent > expect_percent

    def reboot_device(self):
        self.pressPowerKey(10)
        time.sleep(5)
        self.pressPowerKey(2)
        time.sleep(40)
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        os.system("adb shell uiautomator dump /data/local/tmp/uidump.xml")#get layout, 20170822 has bug, can't find layout!
        g_common_obj.adb_cmd("setprop evs.state 0")
        self.backHome()
        self.check_home_or_lock_layout()

    def init_camera_with_canbox(self):
        self.logger.debug("init_camera_with_canbox start")
        self.reboot_device()
#         self.init_camera_c_with_canbox()
#         self.get_screenshot(self.before_screen_file_path)
#         self.camera_event_array[-1]
#         for i in range(self.camera_event_count):
#             self.change_canbox_0_event()
#             time.sleep(1)
#             t_result = self.get_screenshot_and_compare_picture_similarity(self.before_screen_file_path, self.after_screen_file_path)
#             self.camera_event_array[self.now_camera_event] = (0 if t_result else 1)
#         self.logger.debug("camera_event_array=%s" % str(self.camera_event_array))
#         if sum(self.camera_event_array) > self.camera_event_count / 2:
#             for i in range(self.camera_event_count):
#                 self.camera_event_array[i] = (0 if self.camera_event_array[i]==1 else 1)
#         self.logger.debug("camera_event_array=%s" % str(self.camera_event_array))
#         assert sum(self.camera_event_array) <= 2, "fail init_camera_with_canbox!"

    def init_camera_c_with_canbox(self):
        self.logger.debug("init_camera_c_with_canbox start")
        cmd_list = ["stop evs_app", "stop evs_manager", "stop evs-hal-1-0", "start evs-hal-1-0", "start evs_manager", "start evs_app"]
        for cmd in cmd_list:
            g_common_obj.adb_cmd(cmd)
            time.sleep(0.5)

    def change_camera_0_status(self, need_camera_0_status, check_success=1):
        self.logger.debug("change_camera_0_status need_camera_0_status=%s" % need_camera_0_status)
        if check_success == 1:
            before_screen_0_status = self.camera_0_event_array[self.now_camera_0_event][1]
            self.get_screenshot(self.before_screen_0_file_path)
        i = 0
        while need_camera_0_status != self.camera_0_event_array[self.now_camera_0_event][0]:
            i += 1
            if i >= len(self.camera_0_event_array):
                break
            self.change_canbox_0_event()
            time.sleep(1)
        if i == 0:
            self.logger.debug("skip change camera 0 status!")
        if check_success == 1:
            time.sleep(1)
            after_screen_0_status = self.camera_0_event_array[self.now_camera_0_event][1]
            need_status = 1 if before_screen_0_status == after_screen_0_status else 0
            t_result = self.get_screenshot_and_compare_picture_similarity(self.before_screen_0_file_path, self.after_screen_0_file_path)
            assert t_result == need_status, "change_camera_0_status failed!"

    def change_camera_c_status(self, need_camera_c_status, check_success=1):
        self.logger.debug("change_camera_c_status need_camera_c_status=%s" % need_camera_c_status)
        if check_success == 1:
            before_screen_c_status = self.camera_c_event_array[self.now_camera_c_event][1]
            self.get_screenshot(self.before_screen_c_file_path)
        i = 0
        while need_camera_c_status != self.camera_c_event_array[self.now_camera_c_event][0]:
            i += 1
            if i >= len(self.camera_c_event_array):
                break
            self.change_canbox_c_event()
            time.sleep(1)
        if i == 0:
            self.logger.debug("skip change camera c status!")
        if check_success == 1:
            time.sleep(1)
            after_screen_c_status = self.camera_c_event_array[self.now_camera_c_event][1]
            need_status = 1 if before_screen_c_status == after_screen_c_status else 0
            t_result = self.get_screenshot_and_compare_picture_similarity(self.before_screen_c_file_path, self.after_screen_c_file_path)
            assert t_result == need_status, "change_camera_c_status failed!"

    def startCameraApp(self, check_success=1):
        """
        Used to start the camera application
        """
        self.logger.debug("launch ivi camera start")
        self.change_camera_0_status("GEAR_REVERSE", check_success)
        if check_success == 1:
            self.logger.debug("launch ivi camera successfully")
        else:
            self.logger.debug("launch ivi camera, skip check function")

    def stopCameraApp(self):
        """
        Used to stop the camera application
        """
        self.change_camera_0_status("GEAR_NEUTRAL")
        self.logger.debug("stop ivi camera app successfully")

    def skipAccountLoginLyout(self):
        if self.d(textContains="Drive safely").exists:
            self.d(text="Owner").click()
            time.sleep(3)

    def check_home_or_lock_layout(self, check_exist=True):
        assert self.isHomeLayoutExists() or self.isLockLayoutExists(), "Home or Lock layout not exist!"