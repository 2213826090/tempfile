# -*- coding:utf-8 -*-

'''
@summary: Android immersive mode test.
@since: 07/06/2016
@author: Lijin Xiong
'''

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.common import UiAutomatorUtils,StatusBar,AdbUtils,DeviceInfo,ScreenSwiper
from testlib.androidframework.fetch_resources import resource
from testlib.androidframework.screenshot_utils import ScreenshotUtils
from testlib.util.log import Logger
from testlib.util.common import g_common_obj

### System OS various strings

SYSTEM_OS_IMMERSIVE_APP_SHORTCUT_NAME = "Immersive UI"
SYSTEM_OS_ENTER_IMMERSIVE_MODE_TXT = "Enter Immersive Mode"
SYSTEM_OS_EXIT_IMMERSIVE_MODE_TXT = "Exit Immersive Mode"
STATUS_BAR_PACKAGE = "com.android.systemui"

LOG = Logger.getlogger(__name__)

class ImmersiveMode(UIATestBase):

    def setUp(self):
        super(ImmersiveMode, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        UiAutomatorUtils.unlock_screen()
        resource.disable_app_verification()
        self.screenshooter = ScreenshotUtils()
        for i in ["temple_run", "immersive_ui"]:
            _apk_path = resource.get_resource_from_atifactory\
            ("tests.tablet.artifactory.conf", "ImmersiveMode", i)
            g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_immersive_mode(self):
        UiAutomatorUtils.launch_app_from_apps_menu(SYSTEM_OS_IMMERSIVE_APP_SHORTCUT_NAME)
        # enter immersive mode
        self.assertTrue(self.d(text=SYSTEM_OS_ENTER_IMMERSIVE_MODE_TXT).wait.exists(timeout=4000))
        self.d(text=SYSTEM_OS_ENTER_IMMERSIVE_MODE_TXT).click()
        # wait for immersive mode
        time.sleep(3)
        self.screenshooter.take_screenshot()
        StatusBar.open_notifications(nr_of_swipes=2)
        time.sleep(1)
        self.screenshooter.take_screenshot()
        # check if status bar is showing after swipe
        self.assertFalse(self.screenshooter.same_screenshots(-1, -2))

    immersive_3d_app_package = "com.imangi.templerun2"

    def test_immersive_mode_3D_game(self):
        AdbUtils.start_activity_with_package_name(ImmersiveMode.immersive_3d_app_package, "--pct-syskeys 0 -v 500")
        time.sleep(20)
        if self.d(resourceId="com.google.android.play.games:id/info").exists:
            self.d(resourceId="android:id/button1").click.wait()
            self.d(resourceId="android:id/button2").click()
        if self.d(resourceId="com.google.android.googlequicksearchbox:id/screen_assist_opt_in_no").exists:
            self.d(resourceId="com.google.android.googlequicksearchbox:id/screen_assist_opt_in_no").click()
        if self.d(resourceId="android:id/button2").exists:
            self.d(resourceId="android:id/button2").click()
        width, height = DeviceInfo.get_device_screen_dimensions()
        # start the game by clicking randomly
        time.sleep(60)
        self.assertTrue(self.d(packageName=ImmersiveMode.immersive_3d_app_package).wait.exists(timeout=20000),
                        "3D app failed to start on DUT")
        for i in range(1, height, height/10):
            AdbUtils.tap(width/2, i)
            time.sleep(2)
        for i in range(5):
            time.sleep(1)
            if i%2 == 0:
                ScreenSwiper.swipe_left()
            else:
                ScreenSwiper.swipe_right()
        StatusBar.open_notifications(1)
#         self.assertTrue(self.d(packageName=STATUS_BAR_PACKAGE).wait.exists(timeout=10000),
#                         "system bar could not be opened")
        _check_status_bar_cmd = "dumpsys SurfaceFlinger | grep '| StatusBar'"
        assert g_common_obj.adb_cmd_capture_msg(_check_status_bar_cmd), "system bar could not be opened"
        self.d.press.back()
        self.d.press.home()
#         self.screenshooter.take_screenshot()
#         # crop a small upper corner area area
#         system_bar_obscured_crop = self.screenshooter.crop_upper_right_corner(25)
#         system_bar_hidden = False
#         for i in range(5):
#             time.sleep(5)
#             self.screenshooter.take_screenshot()
#             system_bar_hidden_crop = self.screenshooter.crop_upper_right_corner(25)
#             if not self.screenshooter.compare_color_lists(system_bar_hidden_crop, system_bar_obscured_crop):
#                 system_bar_hidden = True
#                 break
#         self.assertTrue(system_bar_hidden, "system bar did not dissapear after 25 seconds")