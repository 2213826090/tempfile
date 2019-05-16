#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl
from testlib.util.common import g_common_obj

class ScreenOffClockAlarm(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.emImpl.clean_clock_data()
        super(ScreenOffClockAlarm, self).setUp()
        self.emImpl.unlock_screen()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.emImpl.clean_clock_data()
        super(ScreenOffClockAlarm, self).tearDown()

    def test_screen_off_clock_alarm(self):
        """
        Screen off Clock Alarm.
        """
        print "[RunTest]: %s" % self.__str__()
        alarm_seconds = 100
        self.emImpl.set_sleep_mode("30 minutes")
        self.emImpl.set_24_hour_format()
        self.emImpl.launch_alarm()
        self.emImpl.add_alarm_by_delay_time(alarm_seconds)
        self.emImpl.set_screen_status("off")
        time.sleep(alarm_seconds)
        time.sleep(5)
        self.emImpl.verify_screen_status("on")

