# -*- coding:utf-8 -*-

'''
@summary: Android screenshot test.
@since: 07/05/2016
@author: Lijin Xiong
'''

import time
from testlib.androidframework.screenshot_utils import ScreenshotUtils
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.common import AdbUtils,OrientationChanger,UiAutomatorUtils,SystemUtils
from testlib.androidframework.fetch_resources import resource
from testlib.util.log import Logger

LOG = Logger.getlogger(__name__)

### OS
OS_OPEN_JUST_ONCE_TXT = "Just once"
APP_CRASH_POPUP_BEGIN_TXT = "Unfortunately"
APP_CRASH_POPUP_END_TXT = "has stopped"

### Youtube
YOUTUBE_PACKAGE_NAME = "com.google.android.youtube"

class Screenshot(UIATestBase):

    youtube_sample_video = "http://www.youtube.com/watch?v=YRhFSWz_J3I"

    def setUp(self):
        super(Screenshot, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        resource.disable_app_verification()
        UiAutomatorUtils.unlock_screen()
        self.screenshooter = ScreenshotUtils()
        for i in ["api_test", "system_api"]:
            _apk_path = resource.get_resource_from_atifactory\
            ("tests.tablet.artifactory.conf", "SDK_API", i)
            g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def tearDown(self):
        self.d.freeze_rotation(False)
        ScreenshotUtils().remove_all_screenshots()

    def test_screenshot_app_rotate(self):
        app_start_cmd = "am start -n com.intel.test.apitests/.ApiTests"
        AdbUtils.run_adb_cmd(app_start_cmd)
        time.sleep(5)
        screenshoter = ScreenshotUtils()
        OrientationChanger.change_orientation("n")
        screenshoter.take_screenshot()
        screenshot_width_n = screenshoter.screen_width
        screenshot_height_n = screenshoter.screen_height
        print "natural orientation dimensions: ", screenshot_width_n, screenshot_height_n
        OrientationChanger.change_orientation("l")
        screenshoter.take_screenshot()
        screenshot_width_l = screenshoter.screen_width
        screenshot_height_l = screenshoter.screen_height
        print "left orientation dimensions: ", screenshot_width_l, screenshot_height_l
        self.assertTrue(screenshot_height_n == screenshot_width_l)
        self.assertTrue(screenshot_height_l == screenshot_width_n)

    def test_screenshot_video_playback(self):
        UiAutomatorUtils.launch_activity_with_data_uri(Screenshot.youtube_sample_video)
        if self.d(text=OS_OPEN_JUST_ONCE_TXT).wait.exists(timeout=3000):
            self.d(text=OS_OPEN_JUST_ONCE_TXT).click()
        self.assertTrue(self.d(packageName=YOUTUBE_PACKAGE_NAME).wait.exists(timeout=10000))
        # wait for the video to start
        time.sleep(10)
        self.screenshooter.take_screenshot()
        # wait for misc error messages to appear
        time.sleep(2)
        self.assertTrue(self.d(packageName=YOUTUBE_PACKAGE_NAME).wait.exists(timeout=1000))

    def test_verify_screenshot_width_height(self):
        screen_shooter = ScreenshotUtils()
        try:
            screen_shooter.take_screenshot()
            print "screenshot: ", screen_shooter.screen_width, screen_shooter.screen_height
#             shell_prop_density_info = SystemUtils.get_property("ro.sf.lcd_density_info", "density")
            shell_prop_density_info = g_common_obj.adb_cmd_capture_msg("wm size")
            test_width, test_height = SystemUtils.get_screen_size_from_lcd_density_info(shell_prop_density_info)
            print "shell prop:", test_width, test_height
            assert {int(screen_shooter.screen_width), int(screen_shooter.screen_height)} == {int(test_width), int(test_height)}
        finally:
            pass