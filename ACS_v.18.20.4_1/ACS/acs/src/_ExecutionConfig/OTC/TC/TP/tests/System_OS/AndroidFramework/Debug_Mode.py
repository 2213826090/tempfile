# -*- coding:utf-8 -*-

'''
@summary: Android Debug mode tests.
@since: 07/06/2016
@author: Lijin Xiong
'''

import time
from string import Template
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.cts_test_impl import CTS_Impl
from testlib.androidframework.common import Settings,ScreenSwiper,UiAutomatorUtils
from testlib.androidframework.fetch_resources import resource
from testlib.util.log import Logger
from testlib.androidframework.screenshot_utils import ScreenshotUtils

LOG = Logger.getlogger(__name__)

class DebugMode(UIATestBase):

    def setUp(self):
        super(DebugMode, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self._cts_test = CTS_Impl()
        UiAutomatorUtils.unlock_screen()
        resource.disable_app_verification()
        self.screenshooter=ScreenshotUtils()
        _apk_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "SDK_API", "api_test")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_gpu_overdraw(self):
        try:
            # enable showing of overdraw areas
            g_common_obj.get_device().orientation = 'n'
            Settings.enable_gpu_overdraw_show_overdraw_areas()
            self.screenshooter.take_screenshot()
            total_nr_of_pixels = self.screenshooter.screen_width * self.screenshooter.screen_height
            dark_red_pixels = self.screenshooter.get_dark_red_pixels_from_current_screenshot()
            nr_of_dark_red_pixels = len(dark_red_pixels)
            LOG.info("dark red overdraw pixels nr.: " + str(nr_of_dark_red_pixels))
            percent_of_dark_red_pixels = nr_of_dark_red_pixels * 100.0 / total_nr_of_pixels
            LOG.info("dark red percent: " + str(percent_of_dark_red_pixels))
            self.assertTrue(percent_of_dark_red_pixels < 15, "dark red pixels must be under 15%")
            # enable showing of deuteranomaly
            Settings.enable_gpu_overdraw_show_deuteranomaly()
            self.screenshooter.take_screenshot()
            dark_red_pixels = self.screenshooter.get_dark_red_pixels_from_current_screenshot()
            nr_of_dark_red_pixels = len(dark_red_pixels)
            LOG.info("dark red deuteranomaly pixels nr.: " + str(nr_of_dark_red_pixels))
            percent_of_dark_red_pixels = nr_of_dark_red_pixels * 100.0 / total_nr_of_pixels
            LOG.info("dark red percent: " + str(percent_of_dark_red_pixels))
            self.assertTrue(percent_of_dark_red_pixels < 15, "dark red pixels must be under 15%")
        finally:
            # disable gpu_overdraw
            Settings.disable_gpu_overdraw()

    def test_show_GPU_view_updates(self):

        def get_gpu_update_pixels(x, y, color):
            return color[0] >= 245 and color[1] <= 150 and color[2] <= 150

        Settings.enable_gpu_show_updates()
        time.sleep(3)
        try:
            for i in range(5):
                self.screenshooter.take_screenshot()
                show_update_pixels = self.screenshooter.search_for_pixels(get_gpu_update_pixels)
                pixels_percent = 1.0 * len(show_update_pixels) / (self.screenshooter.screen_width *
                                                                  self.screenshooter.screen_height) * 100
                LOG.info("Percent of update pixels: " + str(pixels_percent))
                if pixels_percent >= 60:
                    return  # Number of update pixels greater than 60%. Test passed.
                ScreenSwiper.swipe_up()
            self.assertTrue(False, "Could not find any update pixels")
        finally:
            UiAutomatorUtils.close_all_tasks()
            Settings.enable_gpu_show_updates()