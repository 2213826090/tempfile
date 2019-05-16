#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0ix import get_s0ix_obj
from testlib.em.settings import DisplaySetting, DateSetting
from testlib.em.apps import Clock

class S0i3ClockAlarm(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.s0ix = get_s0ix_obj()
        self.s0ix.adb_root()
        self.clock = Clock()

        self.clock.clear_data()
        self.clock.set_screen_status("on")
        self.clock.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        DateSetting().set_property(use_24 = True)
        self.clock.testDevice.close_background_apps()
        super(S0i3ClockAlarm, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.clock.clear_data()
        super(S0i3ClockAlarm, self).tearDown()

    def check_s0i3_clock_alarm(self, alarm_seconds):
        self.clock.launch_alarm()
        self.clock.add_alarm_by_delay_time(alarm_seconds)
        stat_inc = self.s0ix.suspend_resume(alarm_seconds)
        assert stat_inc > 0, "Not enter S3"

    def test_s0i3_15m_clock_alarm(self):
        self.check_s0i3_clock_alarm(900)

    def test_s0i3_one_night_clock_alarm(self):
        self.check_s0i3_clock_alarm(20000)

