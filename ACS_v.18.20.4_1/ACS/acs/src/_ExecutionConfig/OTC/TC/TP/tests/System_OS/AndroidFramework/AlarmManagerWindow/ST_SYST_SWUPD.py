# -*- coding:utf-8 -*-

'''
@summary: Repeated alarms test.
@since: 07/01/2016
@author: Lijin Xiong
'''

import time,datetime
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.common import Calendar,Clock,EnvironmentUtils,UiAutomatorUtils
from testlib.util.log import Logger
from testlib.androidframework.fetch_resources import resource

#For M
CLOCK_SNOOZE_ALARM_TXT = "Snooze"
CLOCK_DISMISS_ALARM_TXT = "Dismiss"
#For N
CLOCK_SNOOZE_ALARM_TXT_N = "SNOOZE"
CLOCK_DISMISS_ALARM_TXT_N = "DISMISS"

LOG = Logger.getlogger(__name__)


class SWUPD_MT(UIATestBase):

    def setUp(self):
        super(SWUPD_MT, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        UiAutomatorUtils.unlock_screen()
        resource.disable_app_verification()
        _apk_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "SDK_API", "api_test")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def tearDown(self):
        self.d.press.home()
        time.sleep(3)
        Clock.delete_all_alarms()

    def test_ST_SYST_SWUPD_MT_034(self):
        hh, mm, ss = Calendar.get_current_dut_time()
        try:
            # Set two alarms
            Clock.set_new_alarm(hh, mm)
            alarm_time = Clock.set_new_alarm(hh, mm)
            hh, mm, ss = Calendar.get_current_dut_time()
            current_time = datetime.datetime(1, 1, 1, int(hh), int(mm), int(ss))
            time_delta = alarm_time - current_time
            time.sleep(time_delta.seconds + 15)
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

    def test_ST_SYST_SWUPD_MT_036(self):
        hh, mm, ss = Calendar.get_current_dut_time()
        try:
            alarm_time = Clock.set_new_alarm(hh, mm)
            hh, mm, ss = Calendar.get_current_dut_time()
            current_time = datetime.datetime(1, 1, 1, int(hh), int(mm), int(ss))
            time_delta = alarm_time - current_time
            time.sleep(time_delta.seconds + 15)
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
            pass