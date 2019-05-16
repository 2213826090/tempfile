# -*- coding:utf-8 -*-

'''
@summary: Android display metrics test.
@since: 06/30/2016
@author: Lijin Xiong
'''

import re
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.common import SystemUtils,AdbUtils,UiAutomatorUtils
from testlib.util.log import Logger
from testlib.androidframework.platform_catalogue import PlatformCatalogue
from testlib.androidframework.fetch_resources import resource

LOG = Logger.getlogger(__name__)


class Display_Metrics(UIATestBase):

    def setUp(self):
        super(Display_Metrics, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        UiAutomatorUtils.unlock_screen()
        self.d.freeze_rotation(False)
        resource.disable_app_verification()
        _apk_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "SDK_API", "api_test")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_screen_density_reporting(self):
        metrics = SystemUtils.get_display_metrics()
        print metrics
        reported_density = metrics["densityDpi"]
        platform_name = SystemUtils.get_platform_name()
        current_platform = PlatformCatalogue.find_platform(platform_name)
        if current_platform is not None:
            reference_density = current_platform.get_property("ro.sf.lcd_density")
            print "reported density: ", reported_density
            print "reference density: ", reference_density
            self.assertTrue(reported_density in reference_density)
            lcd_density_info = current_platform.get_property("ro.sf.lcd_density_info")
            print "reference density info: ", lcd_density_info
            if lcd_density_info is not None:
                self.assertTrue("density: " + reported_density in lcd_density_info)
        else:
            shell_prop_density = SystemUtils.get_property("ro.sf.lcd_density", "density")
            print shell_prop_density
            print reported_density
            self.assertTrue(int(shell_prop_density) == int(reported_density))

    def get_density_value(self, density_string):
        density = re.findall(u'density:(\s+\d+)', density_string)[0]
        return int(density)

    def test_screen_size_reporting(self):
        metrics = SystemUtils.get_display_metrics()
        print metrics
        reported_density_info = SystemUtils.get_property('ro.sf.lcd_density', 'ro.sf')
        platform_name = SystemUtils.get_platform_name()
        current_platform = PlatformCatalogue.find_platform(platform_name)
        if current_platform is not None:
            lcd_density_info = current_platform.get_property("ro.sf.lcd_density_info")
            print "reported density info: ", reported_density_info
            print "expected density info: ", lcd_density_info
            reported_density = self.get_density_value(reported_density_info)
            lcd_density = self.get_density_value(lcd_density_info)
            if lcd_density_info is not None:
                self.assertTrue(reported_density ==  lcd_density, "reported info: "
                                + reported_density_info + " ; expected info: " + lcd_density_info)
            elif reported_density_info is not None:
                self.assertTrue(str(current_platform.screenWidth) + " x " + str(current_platform.screenHeight)
                                in reported_density_info)
            test_width = None
            test_height = None
            if lcd_density_info is None:
                test_width = current_platform.screenWidth
                test_height = current_platform.screenHeight
            else:
                test_width, test_height = SystemUtils.get_screen_size_from_lcd_density_info(lcd_density_info)
            print "testing values for screen size: " + str(test_width) + " x " + str(test_height)
            screen_size_instrumentation_test = "am instrument -e class com.intel.test.apitests.tests" \
                                               ".DisplayMetricsTestDriver#testDisplayInterfaceSize -e args" \
                                               ' "DisplayWidth:$WIDTH$ DisplayHeight:$HEIGHT$" -w com.intel.test' \
                                               ".apitests/com.intel.test.apitests.runners.GenericArgumentPassingTestRunner"
            test_cmd = screen_size_instrumentation_test.replace('$WIDTH$', str(test_width)) \
                .replace('$HEIGHT$', str(test_height))
            test_output = AdbUtils.run_adb_cmd(test_cmd)
            print test_output
            self.assertTrue('OK' in test_output)
        else:
#             shell_prop_density_info = SystemUtils.get_property("ro.sf.lcd_density", "density")
            shell_prop_density_info = g_common_obj.adb_cmd_capture_msg("wm size")
            print shell_prop_density_info
            test_width, test_height = SystemUtils.get_screen_size_from_lcd_density_info(shell_prop_density_info)
            print test_width, test_height
            reported_height = metrics["heightPixels"]
            reported_width = metrics["widthPixels"]
            print reported_width
            print reported_height
            try:
                self.assertTrue(abs(int(reported_width) - int(test_width)) <= int(test_width) / 15)
                self.assertTrue(abs(int(reported_height) - int(test_height)) <= int(test_height) / 15)
            except:
                self.assertTrue(abs(int(reported_width) - int(test_height)) <= int(test_width) / 15)
                self.assertTrue(abs(int(reported_height) - int(test_width)) <= int(test_height) / 15)
