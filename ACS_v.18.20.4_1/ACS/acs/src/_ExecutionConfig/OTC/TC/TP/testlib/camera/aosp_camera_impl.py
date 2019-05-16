# coding: utf-8
import time
import os
from testlib.util.common import g_common_obj
from testlib.camera.camera_log import CameraLogger
from testlib.camera.mum_camera_impl import CameraImpl

class AOSPCamera(CameraImpl):
    """
        @summary: class for camera application Home UI
    """

    PACKAGE_NAME_AOSPCAMERA = "com.android.camera2"
    ACTIVITY_NAME_AOSPCAMERA = 'com.android.camera.CameraLauncher'

    # --------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

    def __init__(self, cfg=None):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self.dut = self.d
        self.sutter_btn_top = 0
        self.sutter_btn_bottom = 0
        self.sutter_btn_left = 0
        self.sutter_btn_right = 0
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]
        self.logger = CameraLogger.instance()

    def clean_up_camera_data(self):
        """
        @summary: clean Up Camera Data
        """
        g_common_obj.stop_app_am(self.PACKAGE_NAME_AOSPCAMERA)
        g_common_obj.adb_cmd("pm clear %s" % self.ACTIVITY_NAME_AOSPCAMERA)

    #enter camera from home
    def enter_camera_from_home(self):
        """
        @summary: enter camera from home using am command
        """
#         g_common_obj2.unlock()
        self.unlockScreen()
        time.sleep(3)
        self.d.press.home()
        self.logger.debug("launch camera start")
        os.system("adb shell am start -n %s/%s" %(self.PACKAGE_NAME_AOSPCAMERA, self.ACTIVITY_NAME_AOSPCAMERA))
        time_left = 0
        start = time.time()
        success = False
        while time_left < 20:
            if self.isShutterBtnExists() or self.GoogleDefaultCamera().camera_photo_show_page_delete_button().exists():
                success = True
                break
            self.checkCameraAccessDevicesLocation()
            if self.GoogleDefaultCamera().camera_page_text("NEXT").exists():
                self.check_notification_after_switch_mode()
            time_left = time.time() - start
            time.sleep(0.5)
            self.judge_if_camera_crash()
        if (not success) or time_left > 20:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "camera launch fail,launch time is greater than 20 seconds"
        self.logger.debug("launch camera success")

    def chooseCamera(self):
        if self.d(text=self.PACKAGE_NAME_AOSPCAMERA).exists:
            self.d(text=self.PACKAGE_NAME_AOSPCAMERA).click()
        if self.d(text = "Always").exists:
            self.d(text = "Always").click()
