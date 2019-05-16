#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0ix import get_s0ix_obj
from testlib.em.settings import DisplaySetting

sleep_dict = {"15 seconds": 15, "30 seconds": 30, "1 minute": 60,
    "2 minutes": 120, "5 minutes": 300, "10 minutes": 600, "30 minutes": 1800}

class S0i3SleepMode(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.s0ix = get_s0ix_obj()
        self.s0ix.adb_root()
        self.display = DisplaySetting()
        self.display.set_screen_status("on")
        self.display.unlock_screen()
        self.s0ix.testDevice.close_background_apps()
        super(S0i3SleepMode, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(S0i3SleepMode, self).tearDown()

    def check_s0i3_state_with_sleep_mode(self, sleep_mode):
        self.display.set_sleep_mode(sleep_mode)
        sleep_time = self.s0ix.get_sleep_time_from_conf() + sleep_dict[sleep_mode]
        stat_inc = self.s0ix.suspend_resume_special(sleep_time = sleep_time, sleep_flags = self.s0ix.IDLE_SCREEN_OFF)
        assert stat_inc > 0, "Not enter S3"

    def test_15s_sleep_enter_s0i3_state(self):
        self.check_s0i3_state_with_sleep_mode("15 seconds")

    def test_30s_sleep_enter_s0i3_state(self):
        self.check_s0i3_state_with_sleep_mode("30 seconds")

    def test_1m_sleep_enter_s0i3_state(self):
        self.check_s0i3_state_with_sleep_mode("1 minute")

    def test_2m_sleep_enter_s0i3_state(self):
        self.check_s0i3_state_with_sleep_mode("2 minutes")

    def test_5m_sleep_enter_s0i3_state(self):
        self.check_s0i3_state_with_sleep_mode("5 minutes")

    def test_10m_sleep_enter_s0i3_state(self):
        self.check_s0i3_state_with_sleep_mode("10 minutes")

    def test_30m_sleep_enter_s0i3_state(self):
        self.check_s0i3_state_with_sleep_mode("30 minutes")

