#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0ix import get_s0ix_obj
from testlib.em.settings import DisplaySetting

class S0i3Brightness(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.s0ix = get_s0ix_obj()
        self.s0ix.adb_root()
        self.display = DisplaySetting()
        self.display.set_screen_status("on")
        self.display.unlock_screen()
        self.display.testDevice.close_background_apps()
        self.display.set_sleep_mode("30 minutes")
        super(S0i3Brightness, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(S0i3Brightness, self).tearDown()

    def check_s0i3_brightness_level(self, level):
        if level:
            self.display.set_brightness_auto("OFF")
            self.display.set_brightness_level(level)
        else:
            self.display.set_brightness_auto("ON")
        stat_inc = self.s0ix.suspend_resume()
        assert stat_inc > 0, "Not enter S3"

    def test_s0i3_change_brightness(self):
        self.check_s0i3_brightness_level(80)

    def test_s0i3_max_brightness(self):
        self.check_s0i3_brightness_level(255)

    def test_s0i3_min_brightness(self):
        self.check_s0i3_brightness_level(5)

    def test_s0i3_auto_brightness(self):
        self.check_s0i3_brightness_level(None)

