# -*- coding:utf-8 -*-

'''
@summary: Android notification test.
@since: 07/05/2016
@author: Lijin Xiong
'''

import time,datetime
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.common import ShellUtils,Clock,EnvironmentUtils,UiAutomatorUtils
from testlib.util.log import Logger
from testlib.androidframework.fetch_resources import resource

LOG = Logger.getlogger(__name__)

### Clock
CLOCK_PACKAGE_NAME = "com.google.android.deskclock"
CLOCK_SHORTCUT_NAME = "Clock"
CLOCK_ADD_ALARM_BUTTON_DESC = "Add alarm"
CLOCK_AM_MARKER_TXT = "AM"
CLOCK_PM_MARKER_TXT = "PM"
CLOCK_ALARM_DESC = "Alarm"
CLOCK_EXPAND_ALARM_DESC = "Expand alarm"
CLOCK_DELETE_ALARM_DESC = "Delete alarm"
CLOCK_DISMISS_ALARM_TXT = "Dismiss"
CLOCK_SNOOZE_ALARM_TXT = "Snooze"
CLOCK_SNOOZING_STATUS_TXT = "Snoozing"

#For N
CLOCK_SNOOZE_ALARM_TXT_N = "SNOOZE"
CLOCK_DISMISS_ALARM_TXT_N = "DISMISS"


class Notifications(UIATestBase):

    def setUp(self):
        super(Notifications, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        UiAutomatorUtils.unlock_screen()
        resource.disable_app_verification()
        for i in ["api_test", "system_api"]:
            _apk_path = resource.get_resource_from_atifactory\
            ("tests.tablet.artifactory.conf", "SDK_API", i)
            g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_notifications_alarms(self):
        hh, mm, ss = ShellUtils.get_current_dut_time()
        try:
            alarm_time = Clock.set_new_alarm(hh, mm)
            hh, mm, ss = ShellUtils.get_current_dut_time()
            current_time = datetime.datetime(1, 1, 1, int(hh), int(mm), int(ss))
            time_delta = alarm_time - current_time
            time.sleep(time_delta.seconds + 15)
            self.d.open.notification()
            time.sleep(3)
            self.assertTrue(self.d(text="Alarm").wait.exists(timeout=3000))
            if EnvironmentUtils.get_android_version() == "N":
                self.assertTrue(self.d(text=CLOCK_SNOOZE_ALARM_TXT_N).wait.exists(timeout=3000))
                self.d(text=CLOCK_SNOOZE_ALARM_TXT_N).click()
            else:
                self.assertTrue(self.d(text=CLOCK_SNOOZE_ALARM_TXT).wait.exists(timeout=3000))
                self.d(text=CLOCK_SNOOZE_ALARM_TXT).click()
            self.d.press.home()
            time.sleep(5)
            self.d.open.notification()
            time.sleep(3)
            self.assertTrue(self.d(text="Alarm").wait.exists(timeout=3000))
            if EnvironmentUtils.get_android_version() == "N":
                self.assertTrue(self.d(textContains=CLOCK_SNOOZING_STATUS_TXT).wait.exists(timeout=3000))
#                 self.assertTrue(self.d(textContains=CLOCK_DISMISS_ALARM_TXT_N).wait.exists(timeout=3000))
#                 self.d(text=CLOCK_DISMISS_ALARM_TXT_N).click()
            else:
                self.assertTrue(self.d(textContains=CLOCK_SNOOZING_STATUS_TXT).wait.exists(timeout=3000))
                self.assertTrue(self.d(textContains=CLOCK_DISMISS_ALARM_TXT).wait.exists(timeout=3000))
                self.d(textContains=CLOCK_DISMISS_ALARM_TXT).click()
        finally:
            pass

    def tearDown(self):
        Clock.delete_all_alarms()