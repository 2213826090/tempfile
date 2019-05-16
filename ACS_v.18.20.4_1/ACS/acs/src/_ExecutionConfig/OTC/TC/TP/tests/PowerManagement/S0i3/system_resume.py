#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0ix import get_s0ix_obj
from testlib.em.settings import DisplaySetting

class SystemResume(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.s0ix = get_s0ix_obj()
        self.s0ix.adb_root()
        self.s0ix.set_screen_status("on")
        self.s0ix.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SystemResume, self).tearDown()

    def sleep_device(self, sleep_time):
        self.s0ix.set_screen_status("off")
        time.sleep(sleep_time)
        self.s0ix.set_screen_status("on")
        self.s0ix.unlock_screen()

    def test_system_resume_wifi_on(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.settings import WifiSetting
        wifi = WifiSetting()
        wifi.switch_wifi("ON")
        stat_inc = self.s0ix.suspend_resume()
        assert stat_inc > 0, "Not enter S3"
        assert wifi.get_wifi_status()

    def test_system_resume_press_power(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.power import get_power_obj
        self.s0ix.set_screen_status("off")
        time.sleep(10)
        get_power_obj().press_power_key(1)
        time.sleep(2)
        self.s0ix.verify_screen_status("on")
        self.s0ix.unlock_screen()

    def test_system_time_sync_up(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import Clock
        clock = Clock()
        sleep_time = 20
        seconds1 = clock.get_device_seconds_since_Epoch()
        self.sleep_device(sleep_time)
        seconds2 = clock.get_device_seconds_since_Epoch()
        assert seconds2 - sleep_time - seconds1 <= 2

