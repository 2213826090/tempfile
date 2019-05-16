#! /usr/bin/env python
# coding:utf-8

import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.apps import BatteryWidget
from testlib.em.settings import DisplaySetting, BatterySetting
from testlib.em.basic_ui import Notification

class PowerGaugeApp(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.app = BatteryWidget()
        self.app.install()
        self.app.set_screen_status("on")
        self.app.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        super(PowerGaugeApp, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.app.stop()
        super(PowerGaugeApp, self).tearDown()

    def test_power_gauge_app(self):
        """
        power gauge app
        """
        print "[RunTest]: %s" % self.__str__()
        level_widget = self.app.get_battery_level()
        level_noti = Notification().get_battery_level()
        level_settings = BatterySetting().get_battery_level()
        assert abs(level_widget - level_noti) <= 1
        assert abs(level_widget - level_settings) <= 1
