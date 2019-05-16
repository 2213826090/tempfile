# -*- coding:utf-8 -*-

'''
@summary: Android system time update test.
@since: 07/05/2016
@author: Lijin Xiong
'''

from testlib.androidframework.screenshot_utils import ScreenshotUtils
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.common import SystemApiTestsInterface,UiAutomatorUtils,SystemUtils
from testlib.androidframework.fetch_resources import resource
from testlib.util.log import Logger

LOG = Logger.getlogger(__name__)

class SystemTimeUpdate(UIATestBase):

    youtube_sample_video = "http://www.youtube.com/watch?v=YRhFSWz_J3I"

    def setUp(self):
        super(SystemTimeUpdate, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        UiAutomatorUtils.unlock_screen()
        resource.disable_app_verification()
        self.screenshooter = ScreenshotUtils()
        _apk_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "SDK_API", "system_api")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_system_time_update(self):
        result = SystemApiTestsInterface.run_instrumentation(class_name="CommonTestsDriver",
                                                             method_name="testDisableAutoTime",
                                                             instrumentation_args=None,
                                                             runner_name="GenericArgumentPassingTestRunner")
        self.assertTrue(SystemApiTestsInterface.was_instrumentation_test_successful(result),
                        "Could not disable auto time sync")
        initial_system_time = SystemUtils.get_system_time()
        LOG.info("initial time: " + str(initial_system_time))
        initial_hour_format = SystemUtils.get_hour_format()
        LOG.info("initial hour format: " + str(initial_hour_format))
        new_system_time = initial_system_time.copy()
        new_system_time.min = 0
        new_system_time.day = 2
        new_system_time.time_zone = "Cuba"
        hour_format_toggle = {12: 24, 24: 12}
        new_hour_format = hour_format_toggle[initial_hour_format]

        try:
            self.assertTrue(SystemUtils.set_hour_format(new_hour_format), "could not set new hour format")
            self.assertTrue(SystemUtils.set_system_time(new_system_time), "could not set new System time")
            reboot_time = UiAutomatorUtils.reboot_device()
            LOG.info("reboot time was: " + str(reboot_time))
            UiAutomatorUtils.unlock_screen()
            system_time_after_reboot = SystemUtils.get_system_time()
            LOG.info("after reboot time: " + str(system_time_after_reboot))
            hour_format_after_reboot = SystemUtils.get_hour_format()
            LOG.info("after reboot hour format: " + str(hour_format_after_reboot))
            self.assertTrue(system_time_after_reboot.time_zone == new_system_time.time_zone,
                            "timezone did not persist after reboot")
            self.assertTrue(abs(system_time_after_reboot.day - new_system_time.day) < 2,
                            "system day did not persist after reboot")
            self.assertTrue(system_time_after_reboot.min <= new_system_time.min + 2 * reboot_time / 60 + 5,
                            "system minute was affected by the reboot")
            self.assertTrue(hour_format_after_reboot == new_hour_format,
                            "system hour format was affected by the reboot")
        finally:
            SystemUtils.set_hour_format(initial_hour_format)
            SystemUtils.set_system_time(initial_system_time)