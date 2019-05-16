# -*- coding:utf-8 -*-

'''
@summary: Repeated alarms test.
@since: 07/01/2016
@author: Lijin Xiong
'''

import time,datetime
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.common import Calendar,UiAutomatorUtils,Clock,SystemApiTestsInterface,EnvironmentUtils
from testlib.util.log import Logger
from testlib.androidframework.fetch_resources import resource

#For M
CLOCK_SNOOZE_ALARM_TXT = "Snooze"
CLOCK_DISMISS_ALARM_TXT = "Dismiss"
#For N
CLOCK_SNOOZE_ALARM_TXT_N = "SNOOZE"
CLOCK_DISMISS_ALARM_TXT_N = "DISMISS"

LOG = Logger.getlogger(__name__)


class Clock_Alarm_Reboot(UIATestBase):

    def setUp(self):
        super(Clock_Alarm_Reboot, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        UiAutomatorUtils.unlock_screen()
        resource.disable_app_verification()
        for i in ["api_test", "system_api"]:
            _apk_path = resource.get_resource_from_atifactory\
            ("tests.tablet.artifactory.conf", "SDK_API", i)
            g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_alarm_after_reboot(self):
        hh, mm, ss = Calendar.get_current_dut_time()
        result = SystemApiTestsInterface.run_instrumentation(class_name="CommonTestsDriver",
                                                             method_name="testDisableAutoTime",
                                                             instrumentation_args=None,
                                                             runner_name="GenericArgumentPassingTestRunner")
        self.assertTrue(SystemApiTestsInterface.was_instrumentation_test_successful(result),
                        "Could not disable auto time sync")
        try:
            # set an alarm at least 250 seconds into the future
            alarm_time = Clock.set_new_alarm(hh, mm, offset=120)
            hh, mm, ss = Calendar.get_current_dut_time()
            current_time = datetime.datetime(1, 1, 1, int(hh), int(mm), int(ss))
            time.sleep(5)
            reboot_time = UiAutomatorUtils.reboot_device()
            UiAutomatorUtils.unlock_screen()
            time_delta = alarm_time - current_time
            sleep_time = time_delta.seconds - reboot_time
            if sleep_time > 0:
                LOG.info("sleeping for %s seconds" % time_delta.seconds)
                time.sleep(sleep_time)
            self.d.open.notification()
            self.assertTrue(self.d(text="Alarm").wait.exists(timeout=3000))
            if EnvironmentUtils.get_android_version() == "N":
                self.assertTrue(self.d(text=CLOCK_SNOOZE_ALARM_TXT_N).wait.exists(timeout=3000))
                self.assertTrue(self.d(textContains=CLOCK_DISMISS_ALARM_TXT_N).wait.exists(timeout=3000))
                self.d(text=CLOCK_DISMISS_ALARM_TXT_N).click()
            else:
                self.assertTrue(self.d(text=CLOCK_SNOOZE_ALARM_TXT).wait.exists(timeout=3000))
                self.assertTrue(self.d(textContains=CLOCK_DISMISS_ALARM_TXT).wait.exists(timeout=3000))
                self.d(text=CLOCK_DISMISS_ALARM_TXT).click()
        finally:
            Clock.delete_all_alarms()